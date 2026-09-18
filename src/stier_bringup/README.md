# Stier system bringup

실차 센서와 Arduino, Localization, State Manager, Object Detection,
Path Planner, Selector와 Control을 하나의 `roslaunch` 프로세스로 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch stier_bringup full_vehicle.launch
```

워크스페이스 루트의 `./run.sh`도 같은 launch를 실행한다. `roslaunch`가 ROS master와
자식 노드의 종료를 관리하므로 개별 launch를 백그라운드 프로세스로 직접 조합하지 않는다.

## 주요 인자

| 인자 | 기본값 | 설명 |
|---|---:|---|
| `enable_lidar` | `true` | LiDAR 드라이버 |
| `enable_camera` | `true` | USB Camera 드라이버 |
| `enable_gps` | `true` | GPS/NTRIP 드라이버 |
| `enable_imu` | `true` | Xsens IMU 드라이버 |
| `enable_arduino` | `true` | T870 Arduino rosserial |
| `arduino_port` | 빈 값 | 비어 있으면 vehicle interface YAML 값 사용 |
| `start_traffic_light` | `true` | 신호등 인식 노드 |
| `start_rviz` | `false` | State Manager RViz |
| `lookahead_m` | 빈 값 | 지정 시 PP의 min/max를 같은 거리(m)로 설정; 생략 시 YAML |
| `t_parking_side` | 빈 값 | `left/right`; 생략 시 미션 설정 |
| `parallel_parking_side` | 빈 값 | `left/right`; 생략 시 미션 설정 |

학교 S 배치 예시(실차 드라이버와 구동 명령 포함):

```bash
roslaunch stier_bringup full_vehicle.launch hongik_s_test:=true \
  lookahead_m:=2 t_parking_side:=left parallel_parking_side:=right
```

일반 홍익 배치는 `hongik_test:=true`, 용인은 두 학교 인자를 생략한다.
새 `MissionState.parking_leg_start_s`가 추가됐으므로 관련 패키지 전체를 재빌드/재시작한다.
실차 전제조건과 검증 범위는 [통합 점검 기록](../../docs/integration_review_20260918.md)을 참고한다.
두 학교 배치의 차이와 하얀차/검은차 실행 준비는
[학교 시험 안내](../../docs/school_test_20260918.md)에 정리했다.

Arduino 포트를 실행 시 지정하는 예시는 다음과 같다.

```bash
roslaunch stier_bringup full_vehicle.launch \
  arduino_port:=/dev/ttyACM0 start_rviz:=true
```

GPS 없이 수동 초기위치와 IMU·Encoder로 시험할 때는 GPS 드라이버와 융합을 함께 끈다.

```bash
roslaunch stier_bringup full_vehicle.launch \
  enable_gps:=false enable_gps_fusion:=false
```

`sensor_bringup`이 실제 장치를 전부 소유한다. 따라서 이 launch는 Localization 내부의
Encoder rosserial, IMU 및 GPS 드라이버를 항상 비활성화하여 같은 포트를 두 노드가 여는
문제를 방지한다.
