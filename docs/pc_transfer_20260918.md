# 새 PC 인계: 학교 S 시험 / 2026-09-18

브랜치 `fix/school-test-ready-20260918`, 작성자 `choialsgh08-hash`.
기존 검토본 `227a16a` 위에 이 노트북에서 수정한 미션·속도·펌웨어·시험 배치를 포함한다.
다른 팀원의 최신 main을 자동 병합한 브랜치는 아니다. 기존 브랜치는 변경하지 않는다.

## 바로 준비하기

Ubuntu 20.04 + ROS Noetic PC에서 새 폴더에 clone한다.

```bash
git clone --single-branch --branch fix/school-test-ready-20260918 \
  https://github.com/Team-Stier/HL-FMA2026-0917.git HL-FMA2026-school-ready
cd HL-FMA2026-school-ready
source /opt/ros/noetic/setup.bash
```

[README](../README.md)의 시스템 패키지 설치와 rosdep 절차를 적용한다. 이미 ROS가 있어도
`robot_localization`, `rplidar_ros`, `rosserial_python`, `rtcm_msgs`, `nmea_msgs`,
`mavros_msgs`, `libgeographic-dev` 등이 필요하다. 카메라를 끄는 이번 실행에는
torch/ultralytics 설치가 필요 없다. NumPy, PyYAML, rospy, PyQt5 등 선언 의존성은 필요하다.

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
bash src/sensor_drivers/lidar/scripts/install_udev_rules.sh
bash src/sensor_drivers/gps/scripts/install_udev_rules.sh
bash src/sensor_drivers/imu/scripts/install_udev_rules.sh
```

udev 설치 후 센서를 다시 연결한다. 포트 접근 권한이 없으면 새 PC 계정의 dialout 그룹을
설정하고 다시 로그인한다. build/devel이나 노트북의 절대 경로는 복사하지 않는다.

```bash
ls -l /dev/lidar /dev/gps /dev/imu
ls -l /dev/serial/by-id/
```

오늘 연결한 하얀차 Arduino 고정 경로는 `usb-1a86_USB_Serial-if00-port0`이다.
같은 보드를 옮기면 업로드한 펌웨어는 남아 있으므로 **PC 변경만으로 다시 업로드할 필요는 없다.**
다른 보드/검은차는 그 차량 펌웨어와 보정을 사용한다. 펌웨어 소스와 빌드 절차는
[전체 패키지 안내](branch_test_quickstart.md)에 있다.

## 먼저 LiDAR 단독 확인

```bash
roslaunch lidar_bringup rplidar_s2.launch
```

다른 터미널에서 같은 워크스페이스를 source하고 확인한다.

```bash
rostopic hz /scan
rostopic echo -n 1 /scan/header
```

드라이버를 Ctrl+C로 종료한 뒤 통합 실행한다. 단독/통합 드라이버로 같은 포트를 동시에 열지 않는다.

## 오늘 측정한 위치에서 S 구간 시작

```bash
bash run_school_s_current.sh
```

- 이번 실행 목표 5km/h, 설정상의 절대 상한은 15km/h. 곡률·조향·미션 감속은 유지된다.
- 카메라와 신호 인식은 끈다. 1.5m 초과 군집 제거 설정은 유지된다.
- Localization 화면에서 **`3_s-static-obstacle` 첫 점(index 0)**을 선택해 초기화한다.
- 원본의 옛 시작점을 선택하지 않는다. 아래 측정 위치·방향에 차량을 놓은 경우에만 이 배치를 사용한다.
- RC 수동 상태에서 `/molit/localization/valid`, `/path_planner/status`, `/path/final`,
  `/control/state`를 먼저 확인한다. RC ROS 모드 전환 시 실제 구동될 수 있다.

`run_school_s_current.sh`의 “current”는 **매번 현재 GPS를 읽는다는 뜻이 아니다.**
아래 2026-09-18 측정값으로 만든 고정 사본을 사용한다. PC를 바꾼 뒤 차를 다른 장소나
방향으로 옮겼다면 다시 배치해야 한다. S 구간이 끝나면 기존 미션 흐름으로 다음 구간에
이어질 수 있으며, S 끝 전용 자동 정지 기능을 추가한 프리셋은 아니다.

| 측정 항목 | 값 |
|---|---|
| 직진 전 GPS 안테나 | 37.5510243, 126.9246429 |
| 직진 후 GPS 안테나(새 시작점) | 37.5510307, 126.9247017 |
| 직진 변위 | 약 5.244m |
| ENU 진입 yaw | 약 7.7846° (동=0°, 북=90°) |
| 안테나→후륜축 보정 | 기존 설정 x=0.65m, y=0m 사용 |

전체 RDDF/편집 JSON/미션 마커를 한 번에 강체 이동·회전했으며 거리·주차 기어 구간·
분기·GPS datum은 유지한다. 원본 홍익/홍익 S/용인 자료는 변경하지 않았다.
이것은 **5.24m 직진으로 추정한 임시 배치**이며, 운동장 경계에 맞춘 재설계나
실제 주행 검증이 아니다. GPS 측정 오차와 직진 중 작은 방향 변화가 남을 수 있다.

별도 시작 위치를 만들 때는 측정한 GPS 안테나 위치와 진입 방향으로 새 디렉터리를 생성한다.
옛 RDDF에 맞춰 초기화된 IMU yaw를 새 방향 측정값으로 재사용하지 않는다.

```bash
python3 src/localization/scripts/prepare_test_rddf.py --help
```

## 전체 학교 배치 실행

```bash
bash run_school_full.sh
```

이 명령은 **기존 홍익 일반 배치**를 사용하며 위 임시 S 배치와 시작 위치·방향이 다르다.
일반 배치의 1_left 시작점에서 초기화해야 한다. 별도 전체 코스 현장 확인은 아직 안 했다.
경사로 구간 연속 정차 3.5초, 신호 미인식 연속 정차 20초 뒤 출발, 기본 좌측 골인 분기다.
적색을 정상 인식한 상태까지 무시하는 로직은 아니다. 두 실행을 동시에 켜지 않는다.

## 발견된 고장과 검증 범위

1. LiDAR USB: `/dev/lidar` open이 EIO로 실패했고 커널은
   `cp210x_open - Unable to enable UART`, `failed set request ... -32`를 기록했다.
   재연결 후 S2M1 장치 정보·health OK·DenseBoost 시작 응답은 확인했다.
   하지만 해당 단독 구독의 12초 확인창에서 LaserScan 수신은 확인하지 못했다.
   허브/케이블/전원/USB 호스트 중 어느 부품이 원인인지는 미확정이다.
2. 지도 배치: 실제 GPS와 옛 수동 RDDF 초기화 위치가 약 62m 달랐다.
   Global EKF 위치 분산이 약 90~100m²로 커져 기존 25m² 제한에 걸렸고,
   승인 odometry가 끊겨 `ODOMETRY_STALE → MISSION_STOP_REQUESTED`가 발생했다.
   임계값을 올리거나 가짜 odometry로 통과시키지 않고 시험 지도 배치를 수정했다.
3. 오늘 하얀차 펌웨어 20,734 bytes 쓰기/재읽기와 실제 rosserial feedback은 검증했다.
   이번 PC 이전 작업에서 추가 업로드나 주행 명령을 보내지 않았다.
4. 경사로 PWM=0은 능동 위치 유지 토크가 아니다. 물리적인 밀림 방지 성능과
   15km/h 추종·제동 성능은 아직 실차 검증이 필요하다.

오프라인/격리 시험은 실행 문법·통합 계약·회귀 오류를 확인하는 것이며 실차 성공 보장이 아니다.
새로운 주행 정지 조건이나 허위 신호/장애물 입력은 추가하지 않았다.

## 업로드 전 확인 결과

- 실제 전체 워크스페이스 `catkin_make -j4` 성공(Localization·센서 드라이버 포함).
- `bash src/state_manager/scripts/verify_noetic.sh` 성공. 격리 ROS 결과 집계
  70 tests, 0 errors, 0 failures. 별도 Python 회귀시험과 Planner 핵심 시나리오도 통과했다.
- 새 시험 지도 변환 6개, 전체 코스 설정/카메라 없는 미션 5개, launch 계약 4개 통과.
- 하얀차·검은차 속도 코어 시험 통과: 가속 램프, 낮아진 목표의 즉시 적용, 부호 속도, 0 리셋.
- 위 격리 시험은 실차 Arduino 포트를 열거나 실차에 주행 명령을 보내지 않는다.
- 노트북의 진단용 GPS/IMU/LiDAR 프로세스와 별도 ROS master는 종료했다.
