# 전체 시험 패키지 — 내려받기부터 학교 실행까지

15km/h 적응형 속도·3.5초 정차 변경과 당일 GPS 배치는 새 인계 브랜치에 포함된다.
카메라 없는 이번 시험은 [새 PC 인계 안내](pc_transfer_20260918.md)를 먼저 확인한다.
아래는 카메라·양 차량 펌웨어까지 포함한 일반 설치 안내다.

브랜치: `fix/school-test-ready-20260918` / 작성자: `choialsgh08-hash`.
기준 main은 `62ee675f396a81bebbc98f5d4b0917dc27118ea3`이다.
이 브랜치만 공유하며 main·다른 브랜치·Mando 저장소는 변경하지 않는다.

## 포함 범위

- ROS 센서 드라이버, Localization, 검출기, RDDF/Frenet/State Manager/Selector/Control.
- 용인 원본과 학교 두 배치 RDDF, 신호등 추론 모델·설정.
- `src/sensor_drivers/arduino/firmware/`: 하얀차·검은차의 후진 지원 펌웨어,
  차종별 설정·기존 문서, 호스트 시험 및 Uno AVR 빌드 스크립트.
- 실행 안내와 회귀시험. ROS bag, Polycam 자료, 개인 계정, 기존 PC의 build/devel,
  ROS·Python·AVR 도구 자체는 배포물에 넣지 않는다.

**다른 팀원 로컬 폴더를 복사할 필요는 없지만, 새 PC의 의존성 설치·빌드와 해당 차량의
펌웨어 업로드는 필요하다. 다운로드만으로 OS 설정이나 Arduino 탑재본이 바뀌지는 않는다.**
주행 알고리즘/정지 규칙은 최종 검토본 그대로다. 이번 패키징으로 새 정지 조건을 추가하지 않았다.

## 1. 다른 작업 폴더를 덮어쓰지 않고 받기

Ubuntu 20.04 + ROS Noetic 설치 PC가 기준이다. 새 터미널에서 사용한다.
아래 목적지 이름이 이미 있으면 기존 폴더를 지우지 말고 새 이름을 지정한다.

```bash
git clone --branch fix/school-test-ready-20260918 --single-branch \
  https://github.com/Team-Stier/HL-FMA2026-0917.git HL-FMA2026-school-test
cd HL-FMA2026-school-test
```

이후 모든 상대 경로 명령은 이 저장소 루트 기준이다.
GitHub ZIP으로 받아도 소스는 동일하지만 실행 비트가 유지되지 않는 환경에서는
`./run.sh` 대신 `bash run.sh`를 사용한다.

## 2. 의존성과 전체 ROS 빌드

[README 설치 절차](../README.md#설치-및-최초-설정)의 apt 목록을 설치한다.
아래 추가 도구도 사용한다. ROS apt 저장소는 이미 등록된 환경을 전제로 한다.

```bash
sudo apt install build-essential python3-pip python3-nose \
  gcc-avr avr-libc binutils-avr avrdude ros-noetic-rosserial-client
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install --user -r src/traffic_light/requirements.txt
catkin_make -j4 -DCATKIN_WHITELIST_PACKAGES='' -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`rosdep` 최초 설치 PC는 README의 `sudo rosdep init`도 한 번 수행한다.
기존 GPU PyTorch가 있는 PC는 [신호등 안내](../src/traffic_light/README.md)의 CUDA 환경을
유지한다. 학습 버전은 기존 `ultralytics==8.4.131`이며 이번 브랜치에서 바꾸지 않았다.
[공식 패키지 안내](https://pypi.org/project/ultralytics/8.4.131/)와 실행 PC의 Python을 맞춘다.
설치 후 **실제 노드가 사용하는 Python**에서 다음을 확인한다.

```bash
python3 -c 'import serial, rospy, cv_bridge, torch, ultralytics; print("runtime imports OK")'
rosmsg md5 erp42_msgs/DriveCmd
python3 -c 'from ultralytics import YOLO; import numpy as np; m=YOLO("src/traffic_light/models/traffic_light_7class_best.pt"); m.predict(np.zeros((480,640,3), dtype=np.uint8), device="cpu", verbose=False); print(m.names)'
```

DriveCmd MD5는 `518982e31d00755722fd5fb8c3000c77`이다.
검은 영상 추론은 로딩/라이브러리 검사이며 실제 신호등 인식 성능 검증은 아니다.
`MissionState`가 바뀌었으므로 일부 패키지만 재빌드하거나 옛 devel을 함께 source하지 않는다.

## 3. 센서 연결 준비

README의 네 udev 설치 스크립트를 적용한 뒤 장치를 다시 연결한다.
LiDAR S2, GPS, IMU, 카메라, 해당 차량 Arduino를 연결한다.

```bash
ls -l /dev/lidar /dev/gps /dev/imu /dev/cam
ls -l /dev/serial/by-id/
```

Arduino 기본값은 `/dev/ttyACM0`, 57600 baud다. 실제 포트는 실행 인자로 지정할 수 있다.
USB 접근 권한이 없는 계정은 시스템의 dialout/video 그룹 설정 후 다시 로그인한다.
GPS RTK 보정용 네트워크·팀 NTRIP 설정, 센서 장착 TF와 차량 치수는 실행 PC/차량에 맞춰
확인한다. 계정 값을 문서나 로그에 새로 복사해 공개하지 않는다.
하얀차·검은차는 같은 ROS master에 동시에 연결하지 않고 한 대씩 시험한다.

## 4. 두 차량 후진 펌웨어 만들기

별도 Mando clone은 필요 없다. 통합 메시지로 라이브러리를 생성하고,
검증에 사용한 공식 Arduino AVR core 1.8.6을 별도 폴더에 받는다.

```bash
STIER_GENERATED_LIBS=$(mktemp -d /tmp/stier-ros-libraries.XXXXXX)
rosrun rosserial_arduino make_libraries.py "$STIER_GENERATED_LIBS" erp42_msgs std_msgs
STIER_AVR_TOOLS=$(mktemp -d /tmp/stier-avr-tools.XXXXXX)
git clone --depth 1 --branch 1.8.6 https://github.com/arduino/ArduinoCore-avr.git \
  "$STIER_AVR_TOOLS/ArduinoCore-avr"
export STIER_ARDUINO_CORE="$STIER_AVR_TOOLS/ArduinoCore-avr/cores/arduino"
export STIER_ARDUINO_VARIANT="$STIER_AVR_TOOLS/ArduinoCore-avr/variants/standard"
bash src/sensor_drivers/arduino/firmware/tests/verify_ros_drive.sh "$STIER_GENERATED_LIBS/ros_lib"
STIER_ACTUATOR_OUTPUTS=1 STIER_ROS_ENABLED=1 \
  bash src/sensor_drivers/arduino/firmware/tests/build_avr.sh "$STIER_GENERATED_LIBS/ros_lib"
```

마지막 스크립트가 출력한 `AVR artifacts: /tmp/stier-avr-build.XXXXXX` 안에
`White/firmware.hex`, `Black/firmware.hex`가 만들어진다. 위 명령은 **업로드하지 않는다**.
두 HEX 모두 ROS=1/human serial=0/출력 허용=1로 빌드된다. 하얀차 소스의 기존 기본값은
ROS=0이므로 Arduino IDE에서 옵션 확인 없이 그냥 올리는 대신 위 빌드 절차를 사용한다.

업로드할 때에는 상위 ROS/rosserial/시리얼 모니터를 종료하고 실제 차종·포트를 확인한다.
모터 전원을 끄고 해당 Uno를 USB로 연결한 상태에서 아래처럼 사용한다.
`STIER_HEX_DIR`은 방금 출력된 **실제 폴더명으로 교체**한다.
이 명령은 해당 보드의 기존 프로그램을 덮어쓴다. 기존 탑재본이 필요하면 미리 보관한다.

```bash
STIER_HEX_DIR=/tmp/stier-avr-build.XXXXXX
# 하얀차일 때만 실행. 검은차에서는 경로의 White를 Black으로 바꾼다.
avrdude -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 -D \
  -U "flash:w:$STIER_HEX_DIR/White/firmware.hex:i"
```

차량별 핀·조향 보정값이 다르므로 두 HEX를 서로 바꿔 올리지 않는다.
탑재 후 휠을 띄우고 RC의 ROS 모드, 전·후진 방향, 조향 부호를 확인한다.
실제 1 km/h 명령에서의 모터 출발과 주행 응답은 모의 시험으로 보증할 수 없다.
상세 계약·기존 펌웨어와의 차이는 [펌웨어 안내](../src/sensor_drivers/arduino/firmware/ROS_GEAR_INTEGRATION.md)에 있다.

## 5. 학교 주행 실행

아래는 **실차 센서와 구동 명령을 실행**한다. 실제 RDDF 배치와 포트를 확인한 뒤 사용한다.
예시는 학교 S 배치 / LD 2 / T자 왼쪽 / 평행 오른쪽이다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash run.sh hongik_s_test:=true lookahead_m:=2 \
  t_parking_side:=left parallel_parking_side:=right arduino_port:=/dev/ttyACM0
```

일반 학교 배치라면 `hongik_s_test:=true` 대신 `hongik_test:=true` 하나만 사용한다.
초기화 UI에서 실제 위치·진행 방향에 맞는 시작점을 고른다.
라바콘 회피는 **3번 S 정적장애물 구간**에서 동작한다. 다른 RDDF 전용 구간에
라바콘을 놓는 것만으로 Frenet이 켜지지 않는다. 1.5 m 초과 군집 통째 제거는 유지한다.
주차 좌우는 현장에 맞게 지정하며 빈 자리 자동 선택 기능이 아니다.
배치 설명, 토픽 점검 및 녹화 목록은 [학교 시험 안내](school_test_20260918.md)에 있다.

## 6. 차량 없이 소프트웨어 재검증

실차 ROS 세션과 분리된 터미널/ROS master에서 실행한다. 시험 launch는 센서나
Arduino 드라이버를 띄우지 않으며 제어 출력을 시험 전용 토픽으로 바꾼다.

```bash
catkin_make -j4 run_tests
catkin_test_results build/test_results
```

최종 검토 당시 전체 등록 시험은 매회 405 집계, 오류·실패 0으로 5회 연속 통과했다.
[검토 기록](five_pass_review_20260918.md)은 시험 범위와 미검증 실차 항목을 구분한다.
새 PC의 의존성 설치·카메라 추론·보드 업로드까지 대신 완료했다는 의미는 아니다.
