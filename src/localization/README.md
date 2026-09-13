# Localization

이 패키지는 저장소 루트 README의 Localization 계약을 구현한다. u-blox GNSS 위치와
상태, Xsens 자세가 모두 유효할 때만 로컬 ENU `map` 좌표의 `/current_pos`를 발행한다.

## ROS 계약

| 방향 | 토픽 | 타입 | frame |
|---|---|---|---|
| 입력 | `/ublox_position_receiver/fix` | `sensor_msgs/NavSatFix` | `gps_link` |
| 입력 | `/gps/status` | `sensor_interfaces/GpsStatus` | `gps_link` |
| 입력 | `/imu/data` | `sensor_msgs/Imu` | `imu_link` |
| 출력 | `/current_pos` | `geometry_msgs/PoseStamped` | `map` |

센서 드라이버는 이 패키지가 시작하지 않는다. 먼저 `gps_bringup`과 `imu_bringup`을
별도로 실행해야 한다. 입력이 없거나 stale이고, GNSS fix/solution·spoofing·위성 수·예상
정확도가 기준을 통과하지 못하거나 IMU quaternion이 잘못되면 새 출력을 발행하지 않는다.
마지막 pose에 현재 timestamp만 다시 붙여 내보내지 않는다.

입력 장애 뒤 복구할 때도 마지막으로 발행한 위치에서 `max_step_distance_m`보다 멀리
떨어진 fix 묶음을 자동 재기준점으로 받아들이지 않는다. 실제 차량이 장기 GNSS 단절 동안
그 거리보다 멀리 이동했다면 현재 구현은 계속 fail-closed 상태이며, 측량 기준의 명시적
relocalization 절차가 추가되기 전에는 운영 중 자동 위치 점프를 하지 않는다.

## 좌표 기준

`test/localization_test.yaml`의 `reference.mode=first_fix`는 시험용이다. 연속해서 품질
gate를 통과한 세 번째 fix의 차량 위치를 실행 원점 `(0, 0, 0)`으로 설정한다. 그 뒤 WGS84 차이를 원점 주변의 ENU
좌표로 투영하고 IMU yaw를 결합한다. 따라서 재시작하면 원점도 달라지며 대회 지도상의
절대 위치가 아니다.

운영 설정은 기본부터 `manual_datum`이며 측량 metadata가 비어 있어 fail-closed로
종료한다. 실제 지도와 연결할 때는 측량한 datum과 map 대응 좌표를
`config/localization.yaml`에 기록하고 `reference.measured=true`로 변경한다.
`gps_translation_base_m`과
`imu_yaw_offset_rad` 역시 실차에서 측정하기 전까지 0 자리표시자다. 설정의 accuracy
경계는 입력 거부 기준이며 실제 위치 오차를 측정한 결과가 아니다.

현재 구현은 planar 장착만 지원한다. 각도 합성식은
`yaw_map = yaw_imu + imu_yaw_offset_rad + reference.yaw_offset_rad`이고,
`imu_yaw_offset_rad = yaw_base - yaw_imu`로 정의한다. IMU가 roll/pitch로 기울어 장착된
경우 필요한 full `base_link <-> imu_link` 회전은 아직 구현하지 않았으므로 실차 heading이
검증됐다고 볼 수 없다.

IMU orientation covariance의 대각값 세 개가 유한한 양수이고 설정 상한 이하여야 한다.
Xsens가 0 또는 `-1`을 내면 정확도를 알 수 없는 입력으로 보고 `/current_pos`를 발행하지
않는다. 임의의 작은 값을 넣지 말고 실차 정지·진동·방향 기준 시험으로 설정해야 한다.

Arduino의 `/erp42_serial/feedback.encoder`는 누적 카운터가 아니라 최근 100 ms의
`deltaCount`다. 실센서 설정은 `encoder_value_mode: window_delta`로 이 값을 다시
차분하지 않는다. 합성 publisher처럼 누적 카운터를 쓰는 입력만
`encoder_value_mode: cumulative_count`로 설정한다. 2026-09-02 한 방향 1회전에서
`+45 ticks`를 기록했고 사용자가 이 값을 승인해, 설정은 둘레 `0.84823 m` 기준
`0.0188495555556 m/tick`을 사용한다. 실제 노면 rollout으로 타이어 유효 둘레를 다시
측정하기 전까지 거리 정확도는 provisional이다.

## 빌드와 실행

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
catkin_make --pkg sensor_interfaces localization \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

roslaunch gps_bringup gps.launch
roslaunch imu_bringup xsens_mti.launch
./src/localization/launch.sh
```

이 PC에서는 실센서 연결 여부 확인, 연결된 드라이버 시작, 테스트 전용
Localization과 RViz 실행을 다음 명령 하나로 수행할 수 있다. 연결되지 않은 GPS, IMU,
Encoder는 RViz에 `입력 없음`으로 표시된다.

```bash
localization
```

`localization` 명령은 `/dev/arduino`가 없더라도 `/dev/serial/by-id/*Arduino*`를
먼저 찾아 Arduino가 한 대면 Encoder 드라이버까지 자동 실행한다. Arduino가 여러 대인
경우에만 안정적인 by-id 경로를 명시한다.

```bash
STIER_ENCODER_PORT=/dev/serial/by-id/<Arduino-ID> localization
```

RViz에서 공개 pose만 확인하려면 다음처럼 실행한다.

```bash
roslaunch localization localization.launch start_rviz:=true
```

Local Odometry는 실행 후 첫 유효 IMU/엔코더 샘플을 `(x, y, yaw) = (0, 0, 0)`으로
잡는다. `/localization/vehicle_marker`는 이 위치를 중심으로 길이 `1.35 m`, 폭
`0.85 m`인 차량 외곽선을 표시한다. 실센서 격리 시험에서는 동일한 표시가
`/localization/live/vehicle_marker`로 발행된다. 제공 RViz 설정의 Fixed Frame도
로컬 `odom`으로 두므로 GPS가 아직 준비되지 않아도 시작 위치가 화면의 `(0, 0)`이다.

## 센서 없는 합성 디버그

`test/localization_debug.launch`는 테스트 전용 GNSS/IMU와 drift가 들어간 가상 Local
Odometry를 발행한다. 실제 센서, 운영 datum 또는 운영 TF는 바꾸지 않는다. 처음 승인된
Global pose와 Local test pose를 각각 정확히 `(0, 0)`으로 맞춰 RViz에서 궤적 차이를
볼 수 있다.

```bash
roslaunch localization localization_debug.launch
```

표시 토픽은 `/localization/test/local_odometry`,
`/localization/test/global_odometry`, `/localization/test/local_path`,
`/localization/test/global_path`다. 이 네 토픽은 모두 합성 테스트 결과다.
