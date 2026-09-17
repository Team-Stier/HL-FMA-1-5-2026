# RDDF State Manager

13개 논리 구간과 19개 RDDF 경로를 연결하는 ROS Noetic 미션 관리자다.
현재 구간, 미션 단계, 좌우 경로, 요청 플래너, 속도 상한과 정지 여부를 결정한다.
규정은 사용자가 제공한 `HL FMA{1_5] 경기규정.pdf` 1–5쪽을 기준으로 읽었다.
PDF 자체는 저장소에 포함하지 않는다. 시험장 현장 설정은 `config/missions.json`과
별도의 landmark 파일로 관리한다.

**이 코드는 미션 결정과 경로 요청의 구현이다.** 경로 추종 Control과
S자 회피용 Object Detection/Local Planner가 연결됐고 신호등 패키지도 선택 실행할 수
있다. 주차 경로 플래너는 아직 통합되지 않았다. 실제 차폭·제동 성능·일부 landmark는
미측정 상태다. `mission.launch`는 Control이
Arduino 명령 토픽을 직접 발행하므로 rosserial을 연결하기 전에 실행 범위를 확인해야
한다.

## 연결 구조

```mermaid
flowchart LR
  L[Localization 승인 위치·유효성·RDDF match] --> S[State Manager]
  R[Localization RDDF Provider] -->|RouteMap| S
  C[Camera] --> T[신호등 인식기]
  T -->|SignalObservation| S
  D[LiDAR] --> O[Object Detection]
  O -->|DBSCAN clusters| P[Local Path Planner]
  O -->|DBSCAN clusters<br/>dynamic RDDF only| S
  L -->|Odometry| P
  R -->|RouteMap| P
  S -->|MissionState| P
  S -->|MissionState<br/>RDDF / LOCAL / PARKING 요청| X[Selector]
  S -->|PlannedPath RDDF| X
  P -->|PlannedPath + decision_id| X
  X -->|nav_msgs/Path| PP[Pure Pursuit Control]
  L -->|Odometry| PP
  S -->|MissionState<br/>속도 · 정지 · 방향 · E-Stop| PP
  E[/vehicle/emergency_stop/] --> PP
  PP -->|/erp42_serial/drive<br/>KPH · 조향 · brake · Gear · EStop| A[Arduino]
  A -->|/erp42_serial/feedback| PP
  S -->|MarkerArray| Z[RViz]
```

RDDF 파일은 `mando_localization`이 읽어 `/route/map`으로 전달한다.
State Manager는 이 토픽을 사용하며 다른 패키지의 RDDF 파일을 직접 읽지 않는다.
활성 미션은 `config/missions.json`의 고정 시작 경로가 아니라
`/molit/localization/rddf/current`의 유효한 `source_route_name`으로 연다. 따라서
어느 RDDF에서 초기화하든 그 RDDF 안의 조건부터 판단할 수 있다. 이후에도 이 토픽의
segment와 fraction을 매 주기 진행거리로 사용한다. State Manager는 Odometry를 RDDF에
다시 투영하거나 다음 활성 RDDF 순서를 판정하지 않는다. Localization match가 다른 원본
RDDF로 바뀌면 새 `decision_id`로 해당 구간의 독립 미션을 시작한다. 유효한 match가 없으면
정지 상태로 대기한다. `next_route`는 미션 진단용 힌트일 뿐 State Manager 내부 추적기를
전환하지 않는다. 분기가 필요한 경우 `/mission/rddf_successor`로 이름만 전달하고,
Localization이 현재 RDDF와의 연결·진입 위치를 확인한 뒤 활성 RDDF를 전환한다.
Selector와 공유하는 순수 Python 검증 코어로 플래너 응답을 동일하게 확인한다.
`decision_id`는 경로·모드·요청 방향이 바뀔 때 변경된다. 플래너는 이 값을 그대로
응답해야 하며 이전 구간의 유효한 경로라도 새 요청에 재사용할 수 없다.
같은 미션 안에서 경로를 다시 계획하면 새 `decision_id`와 경로로 갱신한다.

## 현재 작업 범위와 RViz 점검

1구간 경사로는 확정된 정지구역 앞·뒤 흰 선의 RDDF 중앙에서 정차하도록
설정했다. 2·4·7번 신호 정지선도 확정 좌표를 반영했다.
교차로 출구·주차 확인선·종료선은 사용자가 추후 RDDF에 마킹한다.
실측 치수·제동 성능과 주차 경로 생성은 별도 작업으로 남아 있다. 후진 명령 계약은
Control과 Arduino에 연결됐지만 실차 방향 시험은 아직 필요하다.
당장 마킹이나 차량 파라미터를 채워 넣어 화면을 보기 위한 주행 허가를 만들지 않는다.

경로 구성은 **신호 구간 2·4·7에서 RDDF 추종, 주차 구간에서 좌우 RDDF 선택,
정적 장애물 회피 구간에서 로컬 경로 생성**이다. 다른 구간의 로컬 경로 적용은 추후 정한다.
아래 실행 로직은 현재 구현 기준이며, 3구간은 `LOCAL`을 필수로 요구한다. T자 주차
5·6구간은 RDDF를 직접 사용하고, 평행주차 10·11구간만 `PARKING` 경로/단계 메시지를
요구한다. 평행주차 경로 생성과 추가 로컬 경로 전환은 추후 연결 작업이다.

센서와 Localization을 실차 또는 rosbag으로 먼저 실행한 뒤, 다음 명령으로 한 RViz에서
매니저·RDDF ROI 인식 결과를 점검한다. `run.sh` 대신 점검 launch를 사용한다.

```bash
source devel/setup.bash
roslaunch state_manager inspection.launch
```

이 launch는 매니저, Selector, Control, ROI detector, 읽기 전용 inspection
노드와 RViz를 실행한다. 별도 object_detection 뷰어는 띄우지 않는다. 이미 매니저/인식기가
동작 중이면 중복 실행을 피하도록 아래 옵션을 사용한다. 기존 실행 프로세스의 출력 설정까지
이 launch가 바꾸지는 않으므로 점검은 차량 제어를 시작하지 않은 환경에서 한다.

```bash
roslaunch state_manager inspection.launch start_mission:=false start_detection:=false
```

| RViz 표시 | 확인할 내용 |
|---|---|
| 구간별 색상 RDDF, S01~S13 이름·시작/끝 점 | 전체 코스와 주차 좌우 경로 |
| 흰색 active route와 기존 mission status | 매니저가 진행 중인 구간·미션·단계·정지 이유 |
| 청록색 observed RDDF와 inspection status | Localization `/molit/localization/rddf/current`가 관측한 실제 구간 |
| RDDF/LOCAL/PARK 후보 경로 | 입력된 경로 형상·route·decision ID. 아직 주행이 허용되지 않아도 확인 가능 |
| 노란 `/path/final` | Selector가 내보낸 출력. 미설정 상태에서는 비어 있는 것이 정상 |
| ROI 내부 점, ROI 원, DBSCAN 군집 | 인식 결과 확인. DBSCAN 군집은 객체 종류나 움직임 판정이 아님 |
| Traffic signal | 매니저 입력 메시지의 값·신뢰도. 인식기가 미연결이면 `NO_DATA` |
| Safety / Selector 상태 | 현재 정지 이유, 경로 준비 여부 |

미설정 상태에서는 매니저가 `CALIBRATION_REQUIRED` 등에 머무를 수 있다. 현재
`Observed Sxx`의 원본 RDDF match가 활성 미션과 진행거리의 기준이며, 이전 구간의 완료 여부는
진입 조건이 아니다. 위치가 모호하거나 invalid/stale이면 `UNKNOWN`으로 표시하고
청록색 강조를 지운다. 검사 노드는 제어·미션 토픽을 발행하지 않으며 marker만 발행한다.
위치가 갱신되지 않으면 RViz 카메라를 수동으로 이동해 화면의 상태 문구를 확인한다.

카메라는 신호등 인식에만 사용하고 결과를 `SignalObservation`으로 전달한다.
카메라 기반 차로 제어는 사용하지 않는다. DBSCAN 군집은 3구간 정적장애물 회피용
Local Planner와 State Manager에 함께 연결한다. State Manager는 이름에 `dynamic`이
포함된 RDDF에서만 군집과 전방 RDDF 주행 폭의 겹침을 E-Stop 조건으로 사용한다.

검증: 순수 marker 테스트 및 ROS transport 테스트에서 관측 RDDF와 활성 미션 구간의
일치, 미수신 입력, stale 강조 제거를 검사한다. CI의 Localization은 실제 메시지 정의를
사용하는 message-only fixture이며, Localization 전체/인식기/실제 RViz 렌더링 검증은 아니다.
실차 또는 bag 점검에서는 RViz Displays의 TF 오류, 구간 강조 위치, ROI와 스캔 정합을 확인해야 한다.

## 구간별 처리

이 표의 완료·전환은 전체 코스를 연속 주행할 때의 연결이다. 구간별 시험에서는 현재
RDDF match로 어느 행이든 직접 시작하며 앞 행의 완료 기록을 요구하지 않는다.

| 구간 | 미션과 처리 | 완료·전환 기준 |
|---|---|---|
| 1 left/right | 앞·뒤 정지구역 마커 중앙 접근, 연속 3초 이상 정차, 재출발 | 경사로 정상 통과 후 2로 연결. 0.5m 이상 밀림은 fault |
| 2 | 비허용 신호에서 정지선 가상 벽, fresh `GREEN`에서 RDDF 직진 | 정차 20초 후 강제 출발, 허가 후 교차로를 빠져나갈 때까지 진입 상태 유지 |
| 3 | S자 정적 장애물 회피 | `LOCAL` 경로 필수. 없거나 충돌/미관측이면 정지 |
| 4 | 2와 같은 신호 처리 | 교차로 통과 후 `parking_branches.t`에 설정된 T자 주차 경로로 연결 |
| 5/6 | 선택한 T 주차 `in` RDDF를 후진, `out` RDDF를 전진 추종 | 5 끝에서 정지 요청, 6 시작에서 실제 정차 2초 확인 후 전진; 6 끝에서 7 연결 |
| 7 | 정지선 가상 벽에서 `LEFT_ARROW` 대기, 허용 시 RDDF 좌회전 | 일반 녹색은 좌회전 허가가 아니며 정차 20초 후에만 강제 출발 |
| 8 | RDDF 추종 + DBSCAN 동적장애물 감시 | 군집이 전방 RDDF 주행 폭과 겹치면 E-Stop, 구간 끝에서 9로 연결 |
| 9 | 기준경로 추종 | 구간 끝에서 `parking_branches.parallel`에 설정된 평행주차 경로로 연결 |
| 10/11 | 선택한 평행주차 진입/출차 쌍 유지 | 후진 주차 확인선·자세 확인, 정지 후 전진 출차 |
| 12 | 설정된 종료 분기 추종 | `finish_branch=left`는 중간 분기점, right는 끝에서 13 진입 |
| 13 left/right | 설정된 종료 경로 주행 | 뒷바퀴가 종료선을 지난 뒤 정지 |

주차 좌우 분기는 `missions.json`의 `parking_branches.t`와
`parking_branches.parallel`로 각각 지정한다. 기본값은 둘 다 `left`다. State Manager는
원본 LiDAR로 주차 공간 후보를 계산하지 않는다. T자 주차는 설정된 분기의 RDDF만으로
동작하며, 평행주차는 설정된 분기의 maneuver 입력이 없으면 시작하지 않는다.

RDDF의 주차 `reverse` 표시는 **시작 차체 방향** 메타데이터다. T 주차는
`5_T-*-in` 전체를 `RDDF/REVERSE_ENTRY(-1)`로 주행하고 5번 RDDF 끝에서 정지한다.
Localization이 6번을 활성화하면 실제 정차를 `t_parking_transition_hold_s`(기본 2초)
동안 확인한 뒤 `6_T-*-out`을 `RDDF/FORWARD_EXIT(+1)`로 주행한다. T 주차에는
`/parking/maneuver`와 `/path/park`를 요구하지 않는다.

평행주차 10 진입에서는 `/parking/maneuver`로 받은 단계별 계획을 승인한다. 현재처럼
전진 준비 후 후진 또는 후진부터 시작하는 계획을 지원하며 이번 변경 대상이 아니다.
11은 `FORWARD_EXIT(+1)`다. 시작점과 이전 단계의 목표점에서 차량의 실제 정지를
확인해야 새 단계를 받아들인다. 마지막 후진 목표점은 측정한 주차 확인선과 같아야 하며,
완료 플래그로 이 검사를 대신할 수 없다. 단계가 바뀌면 방향이 같아도 decision_id를
갱신하고 새 계획 경로를 기다린다.

RDDF 접선과 차체 방향이 다른 주차 접근에서는 RDDF의 시작 방향 플래그 대신 실제
플래너 경로의 차체 자세를 검사한다. 이 때문에 10-right의 전진 접근을 처음부터
후진으로 해석하던 문제가 해소된다. 실제 곡률·기어 피드백·주행 가능한 궤적 생성은
Parking Planner와 차량 제어의 책임이다. `DriveCmd`는 속도 절댓값과 `Gear`를 분리하며
Control은 `MissionState.direction=-1`에서 후진 명령을 만든다. Arduino는 실측 0속도를
연속 확인한 뒤에만 전진/후진 방향을 바꾼다.

## 신호 정지선 가상 벽과 RDDF 추종

비전은 `/perception/traffic_signal`의 `SignalObservation`을 State Manager에 보낸다.
`route_name`은 현재 신호가 속한 `2`, `4`, `7`, `header.stamp`는 실제 관측 시각,
`value`는 `RED`/`YELLOW`/`GREEN`/`LEFT_ARROW`/`UNKNOWN`을 사용한다.
기본 신뢰도 하한은 0.8, 유효 시간은 0.5초다. 다른 구간·미래 시각·오래된 관측은
진입을 허용하지 않는다. 일반 초록불과 좌회전 화살표는 구분해야 한다.
`junction_id`는 전달되지만 현재 허가 판단은 `route_name`으로 구분하므로,
비전이 다른 교차로의 신호를 현재 구간 이름으로 잘못 붙이지 않아야 한다.

- 2·4구간: fresh `GREEN`에서 벽 해제, 기존 RDDF를 따라 직진한다.
- 7구간: fresh `LEFT_ARROW`에서 벽 해제, 기존 좌회전 RDDF를 따라간다.
- 적색·황색·불명·신호 단절: 진입 전에는 벽을 유지한다. 정지선까지 접근할 경로는 남기고,
  정지선에서 실제 정차가 연속 `traffic_force_departure_s`(기본 20초) 유지되면 벽을 해제한다.
- 통과 허가 후 앞범퍼가 정지선을 넘어 진입한 상태에서는 신호 변경만으로 벽을 다시 세우지 않는다.
  Localization 또는 경로 입력이 invalid인 경우의 정지는 계속 적용된다.

`/path/rddf`의 끝을 `stop_line_s - vehicle.front_m - stop_buffer_m`까지만 생성한다.
현재 관측을 매 tick에 반영하므로 초록→빨강에서는 다시 잘리고, 허용 신호에서는 전체
lookahead가 복원된다. 후보 경로가 제한을 넘으면 매니저 안전 검사도
`PATH_CROSSES_VIRTUAL_STOP`으로 거부한다. 정지선 미설정 시 신호 구간 경로는 비어 있으며,
보정·위치·경로 검사를 통과해야 움직일 수 있다.

RViz `/mission/markers`에 정지선의 붉은 수직 벽과 `STOP WALL`/`RDDF OPEN` 문구가 나온다.
벽 폭은 차폭+1m(차폭 미설정 시 표시용 3m)이며 실제 차로 폭의 측정값이 아니다.
플래너 제한은 벽 그림의 옆을 돌아가는 우회를 허용하지 않는 **해당 RDDF 진행거리 제한**이다.
마커는 0.5초 뒤 만료되므로 갱신이 끊긴 화면을 현재 상태로 오인하지 않아야 한다.

`/mission/traffic_constraint` (`planning_interfaces/TrafficConstraint`)는 정지선 상태를
진단·시각화하고 State Manager 내부 RDDF 제한과 같은 판단인지 검증하기 위해 발행한다.
현재 2·4·7 신호 구간은 모두 RDDF 모드이고 LOCAL/Frenet 모드와 겹치지 않으므로
`path_planner`에는 연결하지 않는다. 향후 한 구간에서 신호 제약과 지역 회피를 동시에
허용하는 요구가 생길 때만 planner 계약으로 다시 검토한다.

## 경사로 앞·뒤 마커와 3초 정차

확정된 경사로 RDDF 마킹은 경로별 `hill_zone_start_s`와 `hill_zone_end_s`로 연결했다.
값은 해당 RDDF의 시작부터 계산한 누적거리(m)다. 인덱스로 주면 그 지점의 누적거리로
변환해 보정 파일에 기록할 수 있으며 RViz 클릭 편집기도 두 키를 지원한다.
원본 마커 CSV와 전역 RDDF의 투영 결과는
[`yongin_mission_landmarks.json`](../localization/rddf/yongin_mission_landmarks.json)에 기록했다.
경사로 왼쪽 정차 목표는 s=31.028786m, 오른쪽은 s=30.877562m다.

두 값은 **정차가 허용된 영역의 앞·뒤 경계**이며 경사로 전체 시작/정상과 구분한다.
이미 정지구역 경계이므로 각각에 1m를 다시 더하거나 빼지 않는다.
후륜축 `base_link` 목표는 `(hill_zone_start_s + hill_zone_end_s) / 2`다.
곡선에서도 두 좌표를 직선으로 평균내지 않고 RDDF를 따라 중앙점을 구한다.
쌍이 있으면 기존 `hill_stop_s`보다 우선하고, 한쪽만 있거나 순서·범위가 틀리면 미션을 막는다.
기존 `hill_start_s`/`hill_stop_s`/`hill_top_s` 설정은 두 새 마커가 모두 미설정일 때 지원한다.

규정상 정차는 **3초 이상**이다. 중앙 목표의 정지 허용오차 안에서 속도와 위치가
연속적으로 안정된 시간만 누적한다. 음의 속도, 속도 임계치 초과, 기준 위치에서의
변위 초과, 위치/센서 유효성 상실은 누적 시간을 초기화한다.
위치 비교에는 단조 증가 진행률 대신 `raw_s`와 실제 map상의 x/y를 사용한다.
`hill_hold_position_tolerance_m` 기본값 0.02m는 위치 잡음을 구분하기 위한 임시 설정으로,
센서 정밀도와 정차 제어를 실측해 검증해야 한다. `stop_tolerance_m` 기본값은 0.2m다.
RViz에는 앞·뒤 마커와 중앙 목표, 현재 정차 누적 시간이 표시되고 diagnostics에도 기록된다.

매니저는 정지 요청을 유지하고 정차 완료를 판단한다. 브레이크 유지·재출발 순간의
밀림 방지는 추후 병합할 제어기의 역할이며 아직 실차에서 검증하지 않았다.
경사로 두 경계와 2·4·7번 정지선은 설정됐다. 교차로 출구와 차량
파라미터는 미설정 상태를 유지한다.

![전역 RDDF와 확정된 경사로·신호 마커](docs/yongin-mission-landmarks.png)

## 규정 처리와 적용 범위

- 경사로: 시작 1m 이후부터 정상 1m 전까지의 정지 구역에서 3초. 최초 해당 정지부터
  정지구역 정상 쪽 경계 통과까지 30초를 감시한다. 정지 여부는 부호 있는 실제 속도와
  정차 시작 위치 대비 변화를 함께 확인한다. 기존 ramp/target 세 필드 방식도 지원한다.
- 신호: 2/4는 GREEN, 7은 LEFT_ARROW가 정상 진입을 허용한다. 요구 신호가 없어도 정지선에서
  실제 정차가 연속 `traffic_force_departure_s`(기본 20초) 유지되면 해당 교차로만 강제 출발한다. 이미 허가받고 진입한
  교차로에서는 신호가 바뀌었다고 신호 조건만으로 급정지하지 않는다. 교차로 정지
  3초·20초, 통과 30초 초과는 진단에 기록한다.
- 8구간은 RDDF를 추종한다. fresh DBSCAN 군집이 현재 위치부터 설정된 lookahead 안의
  RDDF 주행 폭과 겹치면 State Manager가 `emergency_stop_requested=true`를 유지한다.
  객체 종류나 속도를 분류하지 않고 해당 동적장애물 구간의 경로 점유만 판단한다.
  이 구간에서 DBSCAN heartbeat가 없거나 stale/invalid면 E-Stop을 새로 만들지는 않지만
  유효한 빈 관측이 올 때까지 일반 `stop_requested`로 주행을 막는다.
- 유효한 위치로 실제 출발한 시점부터 전체 8분 및 장시간 무동작을 진단한다. 정지선 전 1m 이내에서 fresh RED에 따라
  멈춘 연속 시간은 주행 시간에서 뺀다. 이 경기시간 진단과 20초 신호 강제 출발 타이머는 서로 독립적이다.
- 차선·중앙선 준수, 범퍼/뒷바퀴와 실제 선의 접촉, 미세 연석의 관측은 센서 정밀도와
  현장 검증에 의존한다. 이 코드가 심판 판정을 대신하거나 모든 감점 조건을 자동
  판정하는 것은 아니다. 경로 추종/플래너가 주행 가능 영역도 검증해야 한다.

## 설정과 RViz에서 위치 지정

RDDF CSV에는 각 점의 좌표·인덱스·누적거리·접선 방향이 있다. 하지만
`정지선`, `경사로 정상`, `주차 확인선`이라는 의미 필드는 없다. 구간 경계는
기존 RDDF 그대로 사용하고, 구간 내부의 규정상 위치만 한 번 지정한다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch state_manager mission.launch calibration_mode:=true start_rviz:=true
```

이 모드는 차량 출력을 끄며 State Manager의 주행 유효성을 false로 유지한다.
Localization과 센서 드라이버는 별도로 실행한다. State Manager의 landmark 편집은
원본 LiDAR나 `laser_link` TF를 요구하지 않으며 RDDF는 위치 입력 없이도 표시된다.
RViz의 `Publish Point` 도구로 지도 위 위치를 클릭하기 전에 대상 의미를 선택한다.

```bash
rosservice call /landmark_editor/set_target "route_name: '2'
landmark: 'stop_line_s'"
```

클릭 결과를 해당 RDDF 점에 맞춰 누적거리와 원본 인덱스를
`~/.ros/stier/landmarks.json`에 기록한다. RViz에 찍은 지점이 실제 선인지는 현장에서
확인해야 한다. `calibration_file:=/원하는/경로.json`으로 저장 위치를 바꿀 수 있다.
기록할 때마다 검증 플래그를 해제하며, 파일 교체는 원자적으로 처리한다.

| 의미 | 입력해야 하는 기준 |
|---|---|
| `hill_zone_start_s`, `hill_zone_end_s` | 새 RDDF의 허용 정지구역 앞·뒤 경계. 중앙 목표 자동 계산 |
| `hill_start_s`, `hill_top_s` | 실제 경사로 시작/정상에 대응하는 RDDF 누적거리 |
| `hill_stop_s` | 그 사이 규정상 정지구역에 차량 기준점이 서야 할 누적거리 |
| `stop_line_s` | 실제 정지선의 누적거리. 런타임이 측정된 전방 범퍼 오프셋을 적용 |
| `parking_confirm_s` | 뒷바퀴가 확인선에 닿았을 때의 기준점 위치. 다음 출차 경로 시작점과 연결되어야 함 |
| `parking_yaw_rad` | 주차 완료 차체 방향. 클릭으로 생성하지 않고 측정한 rad 값을 JSON에 입력 |
| `parking_exit_s` | 주차 출차 확인선을 넘었을 때의 기준점 위치 |

12→13 left 분기점은 `13_left` 시작점을 12번 RDDF에 투영해 자동 계산한다.
종료선 landmark도 입력하지 않는다. 13번 원본 RDDF 끝에서 마지막 진행 방향으로
`finish_runout_m`(기본 3 m)만큼 런타임 경로를 연장하고 그 끝에서 종료한다.

현재 Localization의 `base_link` 기준점은 뒷차축이다. 다른 기준점을 사용한다면
범퍼·차축 오프셋과 모든 landmark 의미를 함께 다시 맞춰야 한다. 실제 주차 확인선이
RDDF 연결점과 다르면 경로도 수정해야 한다.

모든 좌/우 경로에 필요한 항목을 기록한 뒤 검증한다.

```bash
rosservice call /landmark_editor/validate
```

또한 `config/missions.json`을 작업용 파일로 복사하고 다음 차량 값을 측정해 입력한다.
`width_m`, `front_m`, `rear_m`는 `base_link` 기준 차체 외곽이다.
`deceleration_mps2`와 `reaction_s`는 제어·통신·센서 지연을 포함해 실측한다.
`max_speed_mps`는 미션 속도 상한이다. `vehicle.validated`는 전방 범퍼 위치·제동값·
속도 상한을 측정하고 검증한 뒤에만 true로 설정한다. 폭과 뒤쪽 길이는 RViz 표시와
향후 주차 플래너 연결용이며 State Manager가 원본 scan을 검사하는 데 사용하지 않는다.
실제 차량값은 기본 파일에 넣어 두지 않았다. 1 KPH 명령의 제동거리와
정지 여유가 정지 허용 오차보다 크면 현재 정수 속도 인터페이스로 정밀 정지가
불가능하므로 런타임이 차량 설정을 거부한다.

검증 시 RDDF 형상·이름·방향의 digest를 저장한다. RDDF를 수정하면 이전 검증을
자동으로 거부하며, 좌표가 바뀐 landmark는 다시 지정해야 한다.

설정 파일과 landmark는 시작 시 읽는다. 주행 중 편집/자동 적용하지 않는다.
설정 후 노드를 재시작하고 `/mission/state`, `/path/final`, `/control/state`를 확인한다.

```bash
roslaunch state_manager mission.launch config_file:=/측정완료/vehicle-missions.json start_rviz:=true
rostopic echo /mission/state
rostopic echo /mission/safety
rostopic echo /erp42_serial/drive
```

RViz에는 모든 RDDF, 활성 경로, 구간 이름, 현재 위치, 미션·단계·진행률과 정지 사유가
표시된다. 유효하지 않은 위치는 주행 판단에 사용하지 않는다. State Manager는
Odometry의 freshness·좌표계·유한성을 확인하지만 위치 품질과 공분산 허용 여부는
Localization Supervisor가 발행한 `/molit/localization/valid`를 단일 기준으로 따른다.

GPS를 의도적으로 끈 Localization은 `/molit/localization/state=DEAD_RECKONING`과
`valid=true`를 함께 발행한다. State Manager는 위치·yaw 공분산을 다시 제한하지 않고
Localization의 `/valid` 판단을 따른다.

## 인지·플래너 인터페이스

메시지는 `planning_interfaces`에 정의되어 있다. 모든 길이는 m, 시간은 ROS 시각,
경로 프레임은 `map`, `PlannedPath`의 자세는 **차체 방향**이다.

| 토픽 | 타입 | 생산자 책임 |
|---|---|---|
| `/route/map` | `RouteMap` | Localization 소유 RDDF 전체와 origin 제공 |
| `/perception/traffic_signal` | `SignalObservation` | route_name=2/4/7, 신호값·confidence·실제 측정 시각 |
| `/dbscan_clusters` | `visualization_msgs/MarkerArray` | Object Detection의 stamped `map` 군집. dynamic RDDF E-Stop과 3구간 Local Planner가 공유 |
| `/parking/maneuver` | `ParkingManeuver` | 평행주차의 현재 요청 ID를 반영한 단계·방향·RDDF 시작/목표 거리 |
| `/mission/state` | `MissionState` | 활성 요청, decision_id, 속도·정지·전용 E-Stop 제약 |
| `/mission/rddf_successor` | `std_msgs/String` | 주차·종료 분기에서 원하는 다음 RDDF 이름. Localization이 연결·진입 위치를 검증 |
| `/mission/traffic_constraint` | `TrafficConstraint` | 신호 정지선 벽 상태·진행거리 제한·선 위치를 진단·검증용으로 제공 |
| `/path/rddf`, `/path/local`, `/path/park` | `PlannedPath` | 요청 id/route/direction 일치, 유효한 자세·연속 경로 |
| `/path/selector_status` | `PathStatus` | 외부 Selector의 유일한 경로 승인 결과. State Manager는 후보 경로를 재검사하지 않음 |
| `/path/final` | `nav_msgs/Path` | Control용 최종 경로. invalid이면 비움 |
| `/mission/safety` | `SafetyStatus` | 위치·경로·미션 상태의 정지 판단 |
| `/mission/diagnostics` | `std_msgs/String` JSON | 경기 시간·제외 신호 대기·완료 미션·규정 진단 |
| `/mission/markers` | `visualization_msgs/MarkerArray` | RViz 표시 |
| `/erp42_serial/drive` | `erp42_msgs/DriveCmd` | Control의 최종 Arduino 명령; 속도·조향·brake·Gear·EStop |
| `/vehicle/emergency_stop` | `std_msgs/Bool` | 정상 정지·기어 전환과 분리된 Control 비상정지 입력 |

평행주차 플래너는 새 미션의 `decision_id`, `route_name`을 받아 `ParkingManeuver`를
계속 발행한다. 필드는 `leg_index`(0부터), `phase`, `direction`, `start_s`, `target_s`,
`final_leg`이며 `header.frame_id=map`과 실제 계획 시각을 넣는다. 첫 단계 시작은
해당 RDDF의 s=0이며 목표는 시작보다 커야 한다. 역방향 주행도 RDDF 점의 나열
순서에 따른 누적거리는 증가한다. 승인 후 MissionState의 `parking_leg_*`를 확인하고,
갱신된 decision_id로 동일한 단계와 `/path/park`를 다시 발행해야 한다. 승인된 단계의
시작/목표/방향을 몰래 바꾸거나 움직이는 동안 다음 단계로 전환하면 정지한다.

LOCAL은 연결된 `path_planner`의 로컬 경로다. 정적 장애물 구간에서는 RDDF로 자동
fallback하지 않는다. 평행주차 10·11의 PARKING 역시 전용 플래너 응답이 없으면 진행하지 않는다.
미래 시각·낡은 관측·다른 구간의 신호·낮은 confidence는 허가로 쓰지 않는다.
ROS를 정지했다 다시 시작한 경우 새 경기 실행을 위해 State Manager도 재시작한다.

## LiDAR 입력 소유권

State Manager는 원본 `LaserScan`이나 LiDAR TF를 구독하지 않는다. Object Detection이
원본 scan을 처리하고, State Manager는 `dynamic` RDDF의 E-Stop 판단에 필요한 stamped
`/dbscan_clusters`만 받는다. 3구간의 같은 군집은 Path Planner가 직접 사용한다.
RViz inspection도 원본 scan 대신 ROI·DBSCAN 결과만 표시한다.

다음은 ROS 없이 실행 가능한 회귀 테스트다.

```bash
python3 -m unittest discover -s src/state_manager/test -v
python3 -m unittest discover -s src/selector/test -v
```

전체 13개 미션을 차량 연결 없이 논리 입력으로 재생할 수 있다.

```bash
python3 src/state_manager/examples/replay_missions.py --parking-branch left --finish-branch right --output /tmp/stier-replay.json
```

이 재생기는 실제 MissionEngine·Selector를 사용하지만 위치·신호·주차
계획은 **합성 시험 입력**이다. 물리적인 차량 시뮬레이터나 현장 측정 결과가 아니다.
JSON에는 구간·단계·방향·요청 ID와
정지 이유를 남긴다. `test_pipeline.py`는 위치 추적을 포함한 Runtime 연결과 센서
단절, 경로 갱신 경쟁 조건을 별도로 재현한다.

ROS Noetic 호스트 또는 브랜치의 GitHub Actions `State manager` 작업은 다음 검증을 실행한다.

```bash
bash src/state_manager/scripts/verify_noetic.sh
```

임시 Catkin 작업공간에서 공유 메시지와 State Manager·Selector·Control을 빌드한 뒤,
ROS 테스트로 실제 토픽 통신·RViz marker 발행과 기본 보정 상태를 확인한다. 이 빌드에는
센서 드라이버와 Localization C++ 스택을 포함하지 않으며 현장 통합 시험과 구분한다.

실차 적용 전 Ubuntu/Noetic에서 catkin 빌드, rosbag 재생, 센서 단절·위치 도약·신호
변경·막힌 주차칸·정적 장애물 시나리오를 확인해야 한다. macOS 개발 환경에서는
Python 회귀 테스트와 소스/launch 정적 검증만 실행할 수 있다.
