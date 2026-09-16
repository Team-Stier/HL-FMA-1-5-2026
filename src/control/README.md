# HENES T870 path tracking control

HENES T870 실차용 ROS Noetic 경로 추종 패키지다. Pure Pursuit(PP)와 Stanley를 각각
독립적으로 제공하며, 현재는 `lateral_controller`로 선택한 하나만 실행한다. IMM이나
PP–Stanley 하이브리드는 아직 구현하지 않았다.

## 현재 차량·속도 기준

기본 설정은 `config/t870_path_tracking.yaml`에 있다.

| 항목 | 현재값 |
|---|---:|
| wheelbase | `0.75 m` |
| 실제 road-wheel 조향 범위 | `-25 ~ +25 deg` |
| 상위 기본 목표속도 | `5 km/h` |
| 상위 운용 상한 | `10 km/h` |
| Arduino ROS 목표/실측 ceiling | `15 km/h` |

wheelbase와 조향 범위는 현장 실측값을 사용하므로 `calibration_required: false`가
기본값이다. 내부 조향각은 REP-103에 따라 좌회전이 양수지만 Uno `DriveCmd.Deg`는
좌회전이 음수이므로 `steering_command_sign: -1`을 사용한다.

상위 control은 정상 운용 명령을 10 km/h 이하로 제한한다. Arduino는 ROS 목표를
15 km/h로 클램프하고 실측속도 15 km/h까지 PI 추종한다. 15 km/h를 초과한 주기만
구동 PWM을 0으로 제한하며 fault를 래치하지 않는다. RC 모드는 이 속도 정책과 무관하다.

## 입출력과 안전 정지

입력:

- `/path/final` (`nav_msgs/Path`, `map`)
- `/molit/localization/odometry` (`nav_msgs/Odometry`, `map` → `base_link`)
- `/erp42_serial/feedback` (`erp42_msgs/SerialFeedBack`, speed는 m/s)

출력:

- `/pure_pursuit/raw_drive` (`erp42_msgs/DriveCmd`, Safety Gate 입력)
- `/control/state`
- `/control/lookahead_point`, `/control/stanley_projection_point`
- `/control/cross_track_error`, `/control/heading_error`
- `/control/steering_angle_rad`

path·odometry·feedback timeout, frame 불일치, 잘못된 수치, E-stop, RC 모드에서는
`KPH=0`, `Deg=0`, `brake=1`을 raw 출력으로 발행한다. 신호등 정지와 미션 속도 제한은
State Manager와 Safety Gate가 단일하게 적용한다. `control_node`는 TF를 조회하지 않으므로
path와 Odometry가 같은 `expected_frame_id`이고 child frame이 `vehicle_frame_id`여야 한다.
Control은 `/erp42_serial/drive`를 직접 발행하지 않는다.

## 코드 위치

```text
config/t870_path_tracking.yaml             전체 런타임 파라미터
launch/control.launch                      실행 및 제어기 선택
include/control/lateral/                   PP, Stanley 인터페이스
src/lateral/                               PP, Stanley 계산 코드
src/longitudinal/                          고정 목표속도/상한 정책
src/vehicle/                               road-wheel rad → Uno Deg 변환
src/node/control_node.cpp                  ROS 연결, 안전 검사, 제어기 선택
test/unit/                                 알고리즘·속도·명령 변환 테스트
```

ROS 연결과 안전 조건은 `node/`, 횡제어 수식은 `lateral/`, 속도 정책은
`longitudinal/`, T870 명령 부호와 단위는 `vehicle/`에서 수정한다. 서로 다른 책임을 한
파일에 섞지 않는다.

## 파라미터 수정 가이드

기본 YAML을 바로 덮어쓰기보다 시험용 파일을 복사해 launch의 `config`로 지정한다.
`lateral_controller`와 `calibration_required` launch 인자는 YAML 값을 덮어쓴다.

### 공통

| 파라미터 | 수정할 때 |
|---|---|
| `wheelbase_m` | 앞·뒤 차축 중심 거리 재측정 시에만 변경한다. PP와 Stanley 모두에 영향을 준다. |
| `rear_axle_to_pose_reference_m` | Localization `base_link` 기준점이 rear axle보다 앞이면 양수로 실측 입력한다. |
| `maximum_road_wheel_steering_deg` | 반복 사용 가능한 실제 바퀴 조향 한계를 측정해 입력한다. |
| `road_wheel_angle_at_command_limit_deg` | Uno 최대 명령과 실제 road-wheel 각도의 대응값이다. |
| `steering_command_limit_deg`, `steering_command_sign` | Arduino 명령 범위·부호와 반드시 함께 맞춘다. |
| `maximum_steering_rate_deg_per_sec` | 낮추면 부드럽고 느려지며, 높이면 곡선 반응과 기구 충격이 함께 커진다. |
| `target_speed_kph`, `maximum_speed_kph` | target이 maximum을 넘으면 노드가 시작을 거부한다. 상한은 기본 10을 유지한다. |
| `control_rate_hz`, `maximum_control_dt_sec` | 상위 제어 주기와 허용할 최대 timer 지연이다. CPU 지연과 조향 변화율을 함께 확인한다. |
| `path_timeout_sec`, `odometry_timeout_sec`, `feedback_timeout_sec` | 각 topic의 실제 주기와 지연을 rosbag으로 확인한 뒤 변경한다. |
| `expected_frame_id`, `vehicle_frame_id` | 전자는 입력 검증, 후자는 디버그 점의 frame 표시에 사용한다. |
| `require_ros_mode` | feedback의 `MorA==1`을 요구한다. Arduino가 실제 선택 모드를 feedback에 싣기 전까지 fail-closed다. |
| `calibration_required` | `true`면 주행하지 않고 정지 명령만 발행한다. 현재 실측값 승인으로 기본값은 `false`다. |

YAML 위쪽의 `..._topic` 값은 입출력 topic 이름이다. 이름을 변경하면 연결되는 패키지와
rosbag/launch 문서도 함께 수정한다.

### Pure Pursuit

- `lookahead_base_m`: 키우면 부드러워지고 작게 하면 곡선 반응이 빨라진다.
- `lookahead_speed_gain_sec`: 속도가 높을 때 lookahead를 늘리는 정도다.
- `lookahead_curvature_gain_m`: 앞쪽 곡률이 클 때 lookahead를 줄이는 정도다.
- `lookahead_min_m`, `lookahead_max_m`: 최종 lookahead 범위다.
- `minimum_target_distance_m`: 원 교차점을 못 찾았을 때 사용할 전방 점의 최소 거리다.
- `curvature_preview_distance_m`: PP의 곡률 적응 lookahead가 확인할 전방 거리다.

직선 진동은 base/speed gain을 먼저 보고, 곡선 진입이 늦을 때 curvature gain을 조정한다.
한 번에 하나의 값만 바꾸고 같은 경로·속도로 비교한다.

### Stanley

- `stanley_gain`: cross-track error 복귀 강도다. 너무 크면 좌우 진동이 커진다.
- `stanley_softening_speed_mps`: 저속에서 CTE 조향이 과해지지 않게 한다.
- `stanley_minimum_control_speed_mps`: 정지 부근에서 CTE 항이 과해지지 않게 하는 계산 속도 하한이다.
- `stanley_heading_window_m`: 경로 heading을 계산하는 구간이다. 길수록 부드럽지만 반응이 늦다.
- `stanley_heading_error_gain`: heading error 보정 비율이다.
- `stanley_curvature_feedforward_gain`: 경로 곡률 선행 조향이다. 현재 기본값은 0이다.
- `stanley_curvature_preview_distance_m`: Stanley 기준 곡률을 계산할 전방 구간이다.
- `stanley_yaw_rate_damping_gain_sec`: 현재 측정 yaw-rate를 연결하지 않았으므로 0을 유지한다.

Stanley는 gain/softening을 먼저 맞추고 heading gain, 곡률 feedforward 순서로 시험한다.
yaw-rate 센서를 연결하기 전에 damping gain을 켜면 안 된다.

## 빌드와 실행

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg erp42_msgs control
source devel/setup.bash

roslaunch control control.launch lateral_controller:=pure_pursuit
roslaunch control control.launch lateral_controller:=stanley
```

전체 미션 실행에서는 `roslaunch state_manager mission.launch`가 기본적으로 Pure Pursuit
Control을 포함한다. 단독 시험이 필요하면 `start_control:=false`로 통합 Control을 끈다.
Safety Gate의 차량 출력은 별도의 `enable_vehicle_output` opt-in 전까지 preview에만 남는다.

별도 시험 YAML은 다음처럼 지정한다.

```bash
roslaunch control control.launch \
  config:=/absolute/path/to/t870_path_tracking_test.yaml \
  lateral_controller:=pure_pursuit
```

## 테스트 순서

먼저 단위 테스트를 실행한다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg erp42_msgs control
catkin_make run_tests_control
catkin_test_results --all build/test_results/control
```

실차 시험은 다음 순서를 지킨다.

1. Arduino 출력이 잠긴 상태에서 topic, frame, 조향 부호와 timeout 정지를 확인한다.
2. 리프트 상태에서 좌·우 조향, 전·후륜 방향, E-stop과 RC/ROS 전환을 확인한다.
3. 시험 YAML의 목표속도를 `1~2 km/h`로 낮추고 PP와 Stanley를 각각 시험한다.
4. 직선, 완만한 곡선, S자 순서로 진행하고 문제가 없을 때 기본 `5 km/h`로 올린다.
5. 동일 경로에서 CTE, heading error, 조향각, overshoot와 timeout 정지 시간을 비교한다.

최소 기록 topic:

```bash
rosbag record \
  /path/final /molit/localization/odometry /erp42_serial/feedback \
  /pure_pursuit/raw_drive /vehicle_safety/preview_drive /erp42_serial/drive \
  /control/state /control/cross_track_error /control/heading_error \
  /control/steering_angle_rad
```

## 향후 PP–Stanley 하이브리드

첫 하이브리드는 두 조향 출력을 섞는 방식보다, **현재 주행 조건에 따라 PP 또는 Stanley
하나를 선택하는 selector**로 만든다. 기존 두 제어기는 독립 상태로 유지하고 새로운
`lateral/hybrid/` 모듈이 선택만 담당한다.

곡률 기반 selector 예시:

```text
낮은 곡률 구간  → 직선 시험에서 더 안정적인 제어기
높은 곡률 구간  → 곡선 시험에서 오차가 작은 제어기
경계 구간       → 이전 선택 유지(hysteresis)
```

즉 “곡률이 크면 무조건 PP”처럼 미리 고정하지 않는다. PP와 Stanley 단독 rosbag 결과로
직선용/곡선용 제어기를 정한 뒤 `straight_controller`, `curve_controller`, 곡률 진입·해제
threshold를 config에 둔다. 필요하면 이후 속도, CTE, heading error를 선택 조건에 하나씩
추가할 수 있다.

구현 시 확인할 사항:

- `lateral_controller: hybrid`를 추가해 기존 두 모드를 그대로 보존한다.
- selector 뒤에서 공통 조향 포화와 rate limit을 한 번 적용한다.
- threshold 진입/해제 값을 분리해 경계에서 PP/Stanley가 반복 전환되지 않게 한다.
- 선택된 제어기가 invalid면 다른 제어기로 fallback하고 둘 다 invalid면 정지한다.
- 현재 선택 제어기와 곡률을 debug topic으로 남긴다.
- 직선/곡선 선택, hysteresis, fallback, 좌·우 부호를 단위 테스트한다.

IMM은 신뢰할 yaw/yaw-rate와 모델 오차 통계가 준비된 뒤 다시 검토한다. 현재는 사용하지 않는다.

## README 갱신 규칙

control 코드나 설정을 올릴 때는 같은 PR에서 이 README도 갱신한다. 파라미터 기본값,
topic/frame/단위/부호, 안전 제한, 실행법, 테스트법 또는 향후 확장 방식이 바뀌면 해당 내용을
반드시 반영한다. Arduino 명령 계약이 바뀌면 Mando의 Arduino README와 이 문서를 함께
수정한다.

## 출처

PP와 Stanley는 [Team-Stier/Morai_SIM_2026_Team1_with_controller의 morai_path_tracking](https://github.com/Team-Stier/Morai_SIM_2026_Team1_with_controller/tree/main/src/morai_path_tracking)
(MIT)을 T870 ROS 인터페이스에 맞게 옮겼다. MORAI UDP 입출력과 기존 IMM 계층은 포함하지
않는다.

차량 자료는 [BROON T870 매뉴얼](https://www.toyrider.com/sites/default/files/pdf/review/195/henes-broon-t870-kids-electric-ride-on-car-manual.pdf),
[초기형 제품 소개](https://www.cnx-software.com/2016/02/11/henes-broon-t870-is-a-kids-electric-car-controlled-by-an-android-tablet/),
[T870 Sports 판매 사양](https://prod.danawa.com/info/?pcode=21681665)을 참고했다. 공개 자료에
wheelbase와 실제 road-wheel 조향각이 없어 현장 실측값을 우선한다.
