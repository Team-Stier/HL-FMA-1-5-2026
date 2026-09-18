# 학교 실차 시험 준비 — 하얀차 / 검은차

전체 시험 패키지의 브랜치는 `fix/integration-review-20260918`이다.
Arduino 수정본도 이 저장소의 `src/sensor_drivers/arduino/firmware/`에 포함한다.
별도 Mando clone 없이 [처음 내려받는 순서](branch_test_quickstart.md)를 따라 준비할 수 있다.
실제 보드 업로드·차량 구동은 하지 않았다.

추가 교차 검토와 전체 시험 반복 기록: [5차 검토 기록](five_pass_review_20260918.md).

## 1. 이번 최종 점검에서 고친 불필요한 정지

### 학교 평행주차 경로 끝 방향 오류

학교 CSV는 이동·회전 후 XY를 소수점 여섯 자리로 저장하지만 기어 전환 station은
용인 원본 정밀도를 유지한다. 이 차이로 주차 경로 절단점이 cusp를 아주 조금 넘어가,
끝에 1 µm 미만의 역방향 조각이 붙었다. State Manager가 이 조각으로 마지막 pose의
방향을 계산하면서 Selector가 `PATH_BODY_DIRECTION_MISMATCH`로 경로를 거절했다.
대기 중 차가 움직여야 경로 승인이 바뀌는 상태라 정상 출발할 수 없었다.

`MissionRuntime.rddf_points()`에서 인접 XY 차이가 1 µm 이하인 수치적 중복점만 합친다.
요청한 끝점 XY는 유지하고 기어 station·실제 경로 모양·Selector 방향 검사 기준은
바꾸지 않았다. 실제 역방향 구간은 지우지 않는다.

수정 전 학교 평행주차 ROS 시험 및 회전된 합성 cusp 단위시험이 실패했고, 수정 후
용인/홍익 일반/홍익 S × T자 좌·우/평행 좌·우 **12가지가 모두 통과**했다.
실제 RDDF Provider → Tracker → State Manager → Selector → Control을 연결했다.
각 전·후진 leg의 구동 명령과 주차 뒤 7/12번 RDDF의 전진 명령까지 확인한다.
위치는 CSV로 합성하고 피드백도 모의 공급하므로 실차 주차 궤적 검증은 아니다.

### 유효 Frenet 경로의 계산 시간 초과 폐기

기존 코드는 유효한 경로여도 계산 시간이 `planning_deadline_ms`(80 ms)를 넘으면
폐기하고 planner 상태까지 reset했다. 이제 이 수치는 경고 기준이다.
계획 시작 시각을 유지하므로 실제로 오래된 경로는 Selector의 기존 age 검사에서
여전히 제외된다. 충돌 실패·입력 만료·빈 경로를 성공으로 바꾸지는 않는다.

시험 기준을 0.000001 ms로 낮추면 수정 전 정상 빈 도로도 실패했다. 수정 후에는
clear → 전폭 차단 → clear → 남은 0.75 m → 입력 중단 순서에서 정상/실패가 구분된다.
이는 계약 회귀시험이지, 학교에서 실제 80 ms를 초과했다는 로그 증거는 아니다.

새 정지 gate·승인 절차·timeout을 추가하지 않았다. 기존 EStop, 실제 경로 차단,
입력 만료, 주차 기어 변경 정차, 미션 정지는 유지한다.
**“어떤 상황에서도 안 멈춘다”거나 “실차에서 완벽하다”는 보장은 아니다.**

## 2. 홍익 일반과 홍익 S의 차이

| 실행 인자 | 경로 폴더 | 배치 의미 |
|---|---|---|
| `hongik_test:=true` | `test_data/hongik_rddf` | 1_left 첫 점을 학교 관측 위치로 옮기고 직진 관측 방향에 맞춘 사본 |
| `hongik_s_test:=true` | `test_data/hongik_s_rddf` | 3번 S 구간 시작점을 기준으로 별도로 이동·회전한 사본 |
| 둘 다 생략 | `rddf` | 용인 원본; 학교용 아님 |

알고리즘이나 미션 종류의 차이가 아니라 **지도 위 위치·방향 배치** 차이다.
둘 다 같은 원래 코스 전체를 변환한 자료이고 운동장 경계에 맞춰 새로 설계하지 않았다.
학교 원점은 37.5511496, 126.9250846이며, 일반 배치의 기준 GPS는 RTK FIXED가
아니었다. 경로가 운동장 안에 놓이는지, 차량 실제 진행 방향과 맞는지 RViz에서 확인한다.
S 배치는 `3_s-static-obstacle` 시작을 시험하려고 만든 자료이나 내일 차량을 다른 위치에
놓는 것만으로 CSV가 자동 이동하지는 않는다. 두 학교 인자를 동시에 켜지 않는다.

## 3. 실행 PC 빌드

이 점검은 누락 의존성을 별도 임시 디렉터리에 풀어 전체 빌드를 검증했다.
시스템 설치나 실차 workspace의 영구 환경 설정까지 완료한 것은 아니다.
`/tmp` 시험 workspace를 실차 설치로 사용하지 말고 이 저장소의
[의존성 설치 절차](../README.md#설치-및-최초-설정)를 먼저 적용한다.
검증용 Python에서는 `pyserial`, `torch`, `ultralytics`도 확인되지 않았다.
Arduino의 `python3-serial`과 신호등의
[Python 의존성](../src/traffic_light/README.md)을 빠뜨리지 않는다.

```bash
cd ~/HL-FMA2026-0917
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCATKIN_WHITELIST_PACKAGES='' -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`MissionState.parking_leg_start_s`가 추가되었으므로 관련 노드를 전부 다시 빌드/재시작한다.
옛 메시지를 사용한 다른 workspace를 동시에 source하지 않는다.

## 4. 두 차량의 Arduino

차량마다 아래 **해당 차종 스케치**를 사용한다. 핀·조향 ADC 보정값이 달라 서로 바꿔
올리면 안 된다. generic `BROON_T870_Uno_Controller`는 이번 후진 수정 대상이 아니다.

- 하얀차: `src/sensor_drivers/arduino/firmware/BROON_T870_White_Car/`.
- 검은차: `src/sensor_drivers/arduino/firmware/BROON_T870_Black_Car/`.

두 차 모두 ROS용으로 빌드하려면 `BROON_ENABLE_ROS=1`,
`BROON_ENABLE_HUMAN_SERIAL=0`이어야 한다. **하얀차 저장 기본값은 반대**이므로
Arduino IDE에서 그냥 업로드하면 ROS 통신용으로 빌드되지 않는다. 제공한 AVR 검증
스크립트는 두 차량 모두 ROS용 define을 명시하여 빌드한다. 스크립트는 업로드하지 않는다.

통합 저장소의 `erp42_msgs`로 생성한 `ros_lib`를 사용한다.
DriveCmd MD5: `518982e31d00755722fd5fb8c3000c77`.
생성·빌드 상세는 [ROS 기어 통합 안내](../src/sensor_drivers/arduino/firmware/ROS_GEAR_INTEGRATION.md)에 있다.

현장에서는 휠을 띄운 상태에서 두 차 각각 전진/후진·조향 부호·정지 해제를 먼저 확인한다.
ROS 모드 feedback은 `MorA=1`, 실제 후진 speed는 음수여야 한다.
구동 명령은 속도 크기 `KPH>0`와 `Gear=0` 전진 / `Gear=2` 후진의 조합이다.
주차 속도 제한 0.5 m/s는 정수 KPH 명령으로 **1 km/h**이다. 이 속도에서 실제 모터가
출발하는지는 소프트웨어 시험만으로 알 수 없다. 임의로 최소 속도를 올리지 않았다.
USB 포트 기본값은 `/dev/ttyACM0`, baud 57600이다. 두 차를 같은 ROS master에
동시에 연결하는 다중 차량 구성은 아니므로 **한 대씩** 시험한다.

## 5. 학교 실행 예시

다음 명령은 **실차 센서와 구동 명령을 실행**한다. 실제 배치·선택한 주차 좌우·차량
포트에 맞춰 사용한다. 아래 예시는 T자 왼쪽, 평행 오른쪽이며 네 경우의 정답을 자동
판별하는 기능은 아니다. 두 차의 geometry가 다르면 `vehicle_config`도 실측값에 맞춘다.

```bash
roslaunch stier_bringup full_vehicle.launch hongik_s_test:=true \
  lookahead_m:=2 t_parking_side:=left parallel_parking_side:=right \
  arduino_port:=/dev/ttyACM0
```

일반 배치면 `hongik_s_test:=true` 대신 `hongik_test:=true`를 쓴다.
LD 1 비교 시험은 `lookahead_m:=1`로 바꾼다. 이 인자가 min/max를 함께 고정하므로
YAML의 base만 바꿨는데 min/max=2에 묶이는 문제를 피한다.

기존 초기화 UI의 **시작 위치 선택**은 실제 놓인 경로·진행 방향에 맞춰 선택한다.
라바콘 회피는 기본적으로 **3번 `3_s-static-obstacle` / `path_mode=LOCAL`에서만**
동작한다. 1/2/4번 등 RDDF 전용 구간에 라바콘을 놓아도 Frenet으로 자동 전환하지 않는다.
주차는 접근 경로 4번(T자) 또는 9번(평행)부터 시험하면 진입→기어 변경→탈출을 확인할 수 있다.
평행주차 중간의 겹친 전·후진 점에서 초기화하면 전체 route 방향만으로 모호할 수 있다.

사용자 요청대로 `max_cluster_extent_m: 1.5` 초과 군집은 통째로 버린다.
연석 기반 차선 선택은 넣지 않았다. 라바콘까지 바닥 군집에 합쳐지면 함께 제거될 수 있으므로
회피 시험 전에 `/dbscan_clusters`에 해당 라바콘이 실제로 남는지 확인한다.

## 6. 멈췄을 때는 이 값부터 확인

새 자동 정지 장치를 추가하는 절차가 아니라, 기존 정지 이유를 읽는 방법이다.

```bash
rostopic echo /control/state
rostopic echo /mission/state
rostopic echo /path/selector_status
rostopic echo /path_planner/status
rostopic echo /erp42_serial/drive
rostopic echo /erp42_serial/feedback
```

- `/control/state=ACTIVE_pure_pursuit` 여부와 drive의 `KPH/Gear/brake`를 같이 본다.
- `MISSION_STOP_REQUESTED`면 mission의 `reason`으로 주차 정차인지 경로 대기인지 구분한다.
- `NOT_IN_ROS_MODE`면 해당 차량 모드와 feedback `MorA`를 확인한다.
- `STALE_FEEDBACK`면 포트/baud/ros_lib/펌웨어 ROS 옵션을 확인한다.
- `PATH_BODY_DIRECTION_MISMATCH`면 현재 로컬 수정본이 실제 실행 중인지와 주차 경로 절단을 확인한다.
- `NO_VALID_PATH`면 충돌/경계/회피 공간 문제다. 검사값을 끄는 것으로 해결하지 않는다.
- 정상 후진 명령(`Gear=2, KPH>0, brake=0`)인데 바퀴가 안 움직이면 firmware/mode,
  모터 출발 속도·encoder·하위 fault를 확인한다. PP의 LD만 바꾸면 해결되는 증상이 아니다.

시험별 bag은 `/control/state`, `/mission/state`, `/mission/diagnostics`,
`/path/selector_status`, `/path_planner/status`, `/path/final`, `/path/local`,
`/erp42_serial/drive`, `/erp42_serial/feedback`, `/molit/localization/odometry`,
`/molit/localization/rddf/current`, `/dbscan_clusters`, `/scan`, `/tf`, `/tf_static`을
함께 남긴다. RGB 영상이 필요하지 않은 시험이면 전체 `-a` 녹화보다 이 토픽 선택이 작다.

## 7. 검증의 범위

- 최종 전체 workspace 등록 시험: 5회 연속 각각 `405 tests, 0 errors, 0 failures, 0 skipped`.
  catkin XML 집계이며 wrapper 중복이 포함될 수 있어 독립 시험 개수와 합산하지 않는다.
- 별도 격리 검증 스크립트 전체 통과: Python 단위시험, 실제 Control/Planner ROS 시험,
  LD 1/2 × 직선 및 세 S 배치의 Frenet+PP 단순 차량 모형 8회 포함.
- 12가지 실제 노드 주차 통합 시험: 합성 pose/feedback, 실제 주행이나 타이어 모형 아님.
- 정상/아주 작은 CPU 시간 기준의 Planner 시험: 충돌·stale 실패가 유지되는지 포함.
- State Manager 단위시험: 194 methods, 수치적 cusp 중복점 회귀 포함.
- 하얀차·검은차 실제 스케치 호스트 모의 시험 및 ROS 출력 허용 Uno AVR 빌드 통과.
  보드 업로드·USB 협상·실제 PWM/조향 반응·메모리 최저 여유는 별도 현장 확인.
- 기존 전체 소프트웨어 검증/센서·차량 관련 한계:
  [통합 점검 기록](integration_review_20260918.md).

최종 시험 로그(임시 파일, 재부팅/청소 정책에 따라 사라질 수 있음):

- `/tmp/stier-final-five-N-tests.log`, `-summary.log`, `-xml/` (`N=1..5`): 최종 전체 시험 5회.
- `/tmp/stier-finalreview-all-tests-final.log`: 이전 393 집계 시점의 전체 시험.
- `/tmp/stier-finalreview-isolated.log`: 격리 검증 및 8회 모형 시험.
- `/tmp/stier-parking-matrix-final.log`: 12개 주차 통합 시험.
- `/tmp/stier-finalreview-firmware.log`: 두 차 실제 스케치 호스트 모의 시험.
- `/tmp/stier-finalreview-avr.log`: 두 차 ROS 출력 허용 AVR 컴파일/링크.

전체 workspace에서 주차 시험만 다시 실행하려면 `catkin_make run_tests_stier_bringup`,
전체 등록 시험은 `catkin_make run_tests` 후 `catkin_test_results build/test_results`를 사용한다.
이 주차 시험 launch는 실차 drive 대신 `/parking_contract/drive`로 출력하며 센서/Arduino
드라이버를 실행하지 않는다. 시험은 실차 ROS 세션과 분리해서 실행한다.
