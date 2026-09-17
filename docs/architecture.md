# 시스템 아키텍처와 패키지 사용 현황

이 문서는 2026-09-17 현재 소스와 `stier_bringup/full_vehicle.launch`의 실제 연결을 기준으로
작성했다. 첫 번째 그림은 센서부터 차량까지 실제 전체 흐름을 생략 없이 표시한다.
카메라 차로 제어처럼 제거한 기능은 그림에 넣지 않는다.

## 현재 결론

- 신호등 코드는 `traffic_light` 패키지로 들어왔고 카메라의 일반 신호
  `RED/YELLOW/GREEN/LEFT_ARROW`만 `/perception/traffic_signal`로 발행한다.
- Object Detection의 DBSCAN 출력은 이제 `path_planner` 입력으로 연결된다.
- `path_planner`는 ROS 노드가 되어 `/path/local`을 발행하고 Selector가 이를 선택한다.
- 카메라 기반 차로 제어는 제거했다. `dynamic` 이름의 RDDF에서는 DBSCAN 군집이
  진행 경로 위에 있을 때 State Manager가 전용 E-Stop을 요청한다.
- 12구간의 종료 분기는 카메라가 아니라
  `missions.json`의 고정 `finish_branch`로 정한다.
- Parking Planner는 이번 범위에서 구현하거나 연결하지 않았다.
- Control 기본값은 `pure_pursuit`이며 `/path/final`부터 Arduino 명령까지 연결되어 있다.

## 전체 아키텍처

```mermaid
flowchart TB
    classDef sensor fill:#ddf4ff,stroke:#0969da,color:#1f2328
    classDef active fill:#dafbe1,stroke:#1a7f37,color:#1f2328
    classDef gated fill:#fff8c5,stroke:#9a6700,color:#1f2328
    classDef control fill:#fbefff,stroke:#8250df,color:#1f2328
    classDef later fill:#ffebe9,stroke:#cf222e,color:#1f2328

    subgraph INPUT[1. 센서 · 경로 입력과 1차 처리]
        direction LR
        CAMERA[USB Camera]:::sensor --> TL[Traffic Light<br/>launch 기본 OFF]:::gated
        MOTION[GPS · IMU · Encoder]:::sensor --> LOC[Localization<br/>Odometry · valid · TF<br/>RDDF match]:::active
        LIDAR[2D LiDAR]:::sensor --> OD[Object Detection<br/>RDDF ROI + DBSCAN]:::active
        FILES[RDDF CSV]:::sensor --> LOADER[RDDF 파일 로더<br/>rddf_route_provider]:::active
    end

    subgraph PLAN[2. 미션 판단 · 경로 후보 생성]
        direction LR
        SM[State Manager<br/>Mission FSM · route 선택<br/>RDDF 구간 절단]:::active
        LP[Path Planner<br/>Frenet 정적장애물 회피<br/>실측 보정 전 출력 잠금]:::gated
        PARK[Parking Planner<br/>아직 미구현]:::later
    end

    SEL[3. Selector<br/>요청 모드에 맞는 경로 하나 선택]:::active
    PP[4. control_node<br/>Pure Pursuit PP]:::control
    ESTOP[Emergency Stop]:::sensor
    CAR[Arduino / T870<br/>차량 구동]:::active

    TL -->|traffic signal| SM
    LOC -->|Odometry · valid · current RDDF match| SM
    LOADER -->|/route/map| SM

    OD -->|/dbscan_clusters| LP
    OD -->|dynamic RDDF 장애물| SM
    SM -->|/mission/state · LOCAL 요청| LP

    SM -->|/path/rddf + 선택 요청| SEL
    LP -->|/path/local| SEL
    PARK -.->|/path/park| SEL

    SEL -->|/path/final · 유일한 경로 입력| PP
    SM -.->|속도 · 정지 · 방향 · E-Stop<br/>경로 아님| PP
    ESTOP --> PP
    CAR -->|feedback| PP
    PP -->|/erp42_serial/drive| CAR
```

선 교차를 줄이기 위해 여러 노드가 공유하는 위치·지도 입력과 검증용 역방향 선은
그림에서 생략하고 아래 입력·출력 표에 기록했다. 생략된 선은 Localization에서
Object Detection·Path Planner·PP로 가는 위치 정보, RDDF 파일 로더에서 Path Planner로
가는 `/route/map`, LiDAR와 `/path/local`에서 State Manager로 가는 안전 검증 입력이다.
이 선들은 새로운 경로를 생성하거나 Selector를 우회하지 않는다.

**PP는 State Manager의 `/path/rddf`를 직접 받지 않는다.** State Manager가 잘라서 만든
`/path/rddf`는 반드시 Selector를 통과하고, PP는 Selector가 내보낸 `/path/final`만
추종한다. State Manager에서 PP로 직접 들어가는 `/mission/state`는 경로가 아니라
목표 속도, 정지 요청, 전진·후진 방향 같은 제어 조건이다.

`rddf_route_provider`는 경로를 판단하는 노드가 아니다. RDDF CSV를 읽어 전체 경로
목록인 `/route/map`으로 바꾸는 파일 로더다. 실제 route 선택과 RDDF 절단은
State Manager가 담당한다. 활성 route는 설정 파일의 1구간부터 강제로 시작하지 않고,
Localization의 `/molit/localization/rddf/current`가 확정한 원본 RDDF 이름으로 정한다.
따라서 어느 구간에서 초기위치를 잡아도 이전 구간 완료 이력 없이 해당 RDDF의 미션만
독립적으로 시작한다. 다른 RDDF가 확정되면 새 `decision_id`로 해당 미션을 다시 연다.

## 경로 선택 규칙

- 일반 구간: `State Manager → /path/rddf → Selector → /path/final → PP`
- 정적 장애물 구간: `Path Planner → /path/local → Selector → /path/final → PP`
- 주차 구간: 향후 `Parking Planner → /path/park → Selector → /path/final → PP`

Selector는 경로를 새로 만들거나 RDDF를 자르지 않는다. State Manager가 요청한
`path_mode`, `decision_id`, `route_name`, `direction`, timestamp가 정확히 맞는 후보만
`/path/final`로 전달한다. 현재 일반 구간은 RDDF, 3번 정적장애물 구간만 LOCAL이다.

## Object Detection과 Path Planner 연결 상세

`object_detection`은 각 스캔의 원래 timestamp와 `map` frame을 DBSCAN MarkerArray에
유지한다. 군집이 0개여도 stamped `DELETEALL` heartbeat를 내므로 Planner가
“유효한 빈 관측”과 센서/TF 실패로 인한 header 없는 clear를 구분한다.

`path_planner` 입력과 출력은 다음과 같다.

| 방향 | 토픽 | 타입 | 용도 |
|---|---|---|---|
| 입력 | `/route/map` | `planning_interfaces/RouteMap` | 현재 RDDF 기준선 |
| 입력 | `/mission/state` | `planning_interfaces/MissionState` | LOCAL 요청, route, decision ID |
| 입력 | `/molit/localization/odometry` | `nav_msgs/Odometry` | 후륜축 위치·자세 |
| 입력 | `/dbscan_clusters` | `visualization_msgs/MarkerArray` | 정적 장애물 군집 |
| 출력 | `/path/local` | `planning_interfaces/PlannedPath` | Selector용 지역 회피 경로 |
| 출력 | `/path_planner/status` | `planning_interfaces/PathStatus` | 준비 여부와 거부 이유 |

실측 차량 길이·폭·축거·후륜축 기준점과 RDDF 좌우 주행 가능 폭은 저장소에 없다.
따라서 `path_planner/config/path_planner.yaml`의 `calibration_required` 기본값은 `true`다.
이 상태에서도 모든 ROS 연결과 진단은 동작하지만 `/path/local`은 발행하지 않는다.
실측값을 채우고 검증한 뒤 `local_planner_calibration_required:=false` launch 인자를
명시해야 한다.

PP 연결은 다음 코드·설정으로 확인된다.

- `mission.launch`의 기본 `lateral_controller`는 `pure_pursuit`다.
- Selector 출력은 `/path/final`이다.
- `control/config/t870_path_tracking.yaml`의 `path_topic`도 `/path/final`이다.
- Control 최종 출력은 `/erp42_serial/drive`이며 별도 Vehicle Safety Gate는 없다.
- Stanley 구현은 남아 있지만 launch 인자를 바꾸지 않는 한 사용하지 않는다.

## State Manager의 현재 구간 정책

아래 행은 순차 해제 조건이 아니라 **현재 매치된 RDDF 안에서 적용할 동작**이다.
각 구간은 직접 진입해 독립 시험할 수 있으며, 이전 번호 구간을 먼저 완료할 필요가 없다.

| 구간 | path mode | 현재 처리 |
|---|---|---|
| 1 | RDDF | 경사로 정지구역 3초 정차 |
| 2, 4 | RDDF | GREEN과 정지선으로 직진 허가 결정 |
| 3 | LOCAL | DBSCAN 장애물을 사용한 Frenet 정적 회피 경로 필수 |
| 5, 6 | PARKING | T 주차; Planner는 아직 없음 |
| 7 | RDDF | LEFT_ARROW와 정지선으로 좌회전 허가 결정 |
| 8 | RDDF | DBSCAN이 stale/invalid면 일반 정지, 군집이 설정한 lookahead·경로 반폭 안에 있으면 E-Stop 요청(차량 치수 불필요) |
| 9 | RDDF | 평행주차 접근 및 주차 공간 선택 |
| 10, 11 | PARKING | 평행주차; Planner는 아직 없음 |
| 12 | RDDF | `finish_branch` 고정 설정에 따라 13 left/right 연결 |
| 13 | RDDF | 원본 RDDF 끝에서 마지막 방향으로 3 m 연장한 경로 끝에서 정지 |

신호 정지선은 Frenet Planner 입력이 아니다. 2·4·7구간은 RDDF 모드이며 State Manager가
신호 상태와 정지선으로 경로 길이·정지 요청을 정한다. Selector는 그 RDDF 후보를 고르고,
Control이 속도 상한과 정지 요청을 함께 적용한다.

신호 허가 후에는 별도 출구 landmark 없이 활성 RDDF 끝에서 구간 완료로 판단한다.
12→13 left 분기 위치도 JSON landmark가 아니라 두 RDDF의 연결 형상에서 자동 계산한다.

## Launch 상태

전체 실차 실행은 `roslaunch stier_bringup full_vehicle.launch` 하나로 센서·Arduino,
Localization과 아래 미션 구성을 함께 시작한다. `run.sh`도 이 launch만 호출한다.
Localization의 Encoder/IMU/GPS 내부 드라이버는 꺼서 `sensor_bringup`과 장치를 중복으로
열지 않는다.

`state_manager/mission.launch` 기본 실행 구성은 다음과 같다.

| 구성 | 기본값 | 비고 |
|---|---:|---|
| RDDF 파일 로더 / State Manager / Selector | 켜짐 | 미션 기본 축 |
| Object Detection | 켜짐 | `/dbscan_clusters` 생산 |
| Local Path Planner | 켜짐 | 실측 보정 전에는 출력 억제 |
| Pure Pursuit Control | 켜짐 | `lateral_controller:=pure_pursuit` |
| Traffic Light | 꺼짐 | 모델 의존성·카메라 준비 후 `start_traffic_light:=true` |
| Parking Planner | 없음 | 추후 작업 |

신호등 실행 예:

```bash
python3 -m pip install -r src/traffic_light/requirements.txt
roslaunch state_manager mission.launch start_traffic_light:=true traffic_light_device:=0
```

## 현재 미사용·추후 작업

| 패키지/기능 | 상태 | 이유 |
|---|---|---|
| `parking_path_planning` | 빈 패키지 | `/path/park`, `/parking/maneuver` 생산 코드 없음 |
| `perception_interfaces/ObjectInfo` | 미사용 계약 | Planner는 stamped DBSCAN MarkerArray를 직접 변환 |
| `perception_interfaces/TLLabel` | 미사용 계약 | 신호등은 `planning_interfaces/SignalObservation` 사용 |
| Stanley | 대체 구현 | 현재 Control 기본 선택은 PP |

카메라 기반 차로 제어는 현재 범위에서 제거했다. 동적장애물 E-Stop은 RDDF 이름에
`dynamic`이 포함된 구간에서만 활성화되며, 다른 구간의 DBSCAN 군집은 이 E-Stop을
발생시키지 않는다.

## 남은 실차 작업

1. `path_planner.yaml`에 실측 차량 치수와 3구간 좌우 주행 가능 폭을 기록한다.
2. rosbag 또는 정지 차량에서 빈 관측·단일 장애물·전폭 차단 시나리오를 검증한다.
3. 계산 시간이 `planning_deadline_ms` 안에 들어오는지 실차 PC에서 확인한다.
4. Traffic Light 의존성을 설치하고 카메라 노출·GPU/CPU 지연·confidence를 측정한다.
5. Parking Planner는 전진/후진 leg 계약을 확정한 뒤 별도 브랜치에서 구현한다.
