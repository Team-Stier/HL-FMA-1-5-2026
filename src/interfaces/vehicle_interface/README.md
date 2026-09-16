# Vehicle interface

ROS 상위 제어 명령과 BROON T870 Arduino Uno 하위제어기 사이의 통신 구성을 모아 둔다.
이 디렉터리에는 Arduino 펌웨어와 동일한 메시지 정의를 제공하는 `erp42_msgs`를 둔다.
PC 측 rosserial 실행 패키지 `vehicle_interface_bringup`은 장치 드라이버 구조에 맞춰
`src/sensor_drivers/arduino/ros`에 둔다. 별도의 명령 변환 노드는 아직 구현하지 않았다.

## 통신 구조

```text
ROS controller
  -> /erp42_serial/drive (erp42_msgs/DriveCmd)
  -> rosserial_python
  -> Arduino Uno USB serial
  -> /erp42_serial/feedback (erp42_msgs/SerialFeedBack)
```

`DriveCmd`의 필드는 다음과 같다.

```text
uint16 KPH
int16 Deg
uint8 brake
uint8 Gear
uint8 EStop
```

- `KPH`: 목표 속도의 절댓값(km/h)이다.
- `Deg`: 목표 조향각. Arduino 기본 제한은 `-25`~`25`도다.
- `brake`: 정상 정지 요청이다. `0`이면 주행, 0이 아니면 정지한다.
- `Gear`: `0=전진`, `1=중립`, `2=후진`이다.
- `EStop`: 기어와 무관한 별도 비상정지 요청이다. `0=해제`, `1=정지`다.

전진↔후진 요청이 바뀌면 Arduino는 즉시 반대 방향을 출력하지 않는다. 기존 PWM을
0으로 내린 뒤 엔코더가 3회 연속 `deltaCount=0`, 실측 `0.0 km/h`, 앞·뒤 PWM 0을
보고한 경우에만 새 방향을 적용한다. 정상 기어 전환은 `brake + Gear=중립`을 사용하며
`EStop`은 별도 비상정지에만 사용한다.

Arduino는 ROS 명령을 한 번만 받고 계속 유지하지 않는다. 현재 명령 타임아웃은 1초이므로
제어기는 `/erp42_serial/drive`를 10 Hz 정도로 계속 발행해야 한다.

## rosserial 설치

Ubuntu 20.04와 ROS Noetic 기준으로 다음 패키지가 필요하다.

```bash
sudo apt update
sudo apt install ros-noetic-rosserial-python ros-noetic-rosserial-arduino
```

설치 확인:

```bash
source /opt/ros/noetic/setup.bash
rospack find rosserial_python
rospack find rosserial_arduino
```

## Arduino용 ROS 헤더 생성

`erp42_msgs`를 빌드하고 워크스페이스 환경을 불러온 뒤 Arduino 라이브러리를 생성한다.

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
catkin_make --pkg erp42_msgs
source devel/setup.bash
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries erp42_msgs std_msgs
```

다음 파일이 있으면 메시지 헤더 생성이 완료된 것이다.

```bash
test -f ~/Arduino/libraries/ros_lib/ros.h && echo "OK ros.h"
test -f ~/Arduino/libraries/ros_lib/erp42_msgs/DriveCmd.h && echo "OK DriveCmd.h"
test -f ~/Arduino/libraries/ros_lib/erp42_msgs/SerialFeedBack.h && echo "OK SerialFeedBack.h"
```

메시지 정의를 수정하면 `catkin_make` 후 `make_libraries.py`를 다시 실행해야 한다. PC와
Arduino 헤더의 메시지 MD5가 다르면 rosserial 연결 후 메시지를 주고받을 수 없다.

## Arduino ROS 빌드 설정

ROS 통신만 먼저 확인할 때는 Arduino의 `BuildOptions.h`를 다음과 같이 설정한다.

```cpp
#define BROON_ENABLE_ACTUATOR_OUTPUTS 0
#define BROON_ENABLE_ROS 1
#define BROON_ENABLE_HUMAN_SERIAL 0
```

ROS와 사람이 읽는 `Serial.print` 출력은 Uno의 같은 USB 직렬 포트를 함께 사용할 수 없다.
ROS 시험 중에는 Arduino IDE의 시리얼 모니터도 닫는다.

참고 Arduino 저장소의 현재 생산 스케치는 `BroonT870Core.h`를 요구하지만 해당 라이브러리를
같이 제공하지 않는다. 별도로 검증된 `BroonT870Core` 라이브러리가 Arduino 라이브러리
경로에 있어야 컴파일할 수 있다.

## Arduino 연결과 rosserial 실행

현재 Arduino용 udev 고정 이름은 아직 등록하지 않았다. Uno를 연결한 뒤 실제 포트를 찾는다.

```bash
ls -l /dev/serial/by-id/
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
```

기본 포트와 baud는 `src/sensor_drivers/arduino/ros/config/serial.yaml`에서 지정한다.
`port`에는 `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/serial/by-id/*` 경로를 사용할 수 있다.
Arduino `ros_lib`와 PC 측 baud를 모두 같은 값으로 맞춘다. `roslaunch`가 실행 중인 ROS
Master가 없으면 Master도 함께 시작한다.

```yaml
port: /dev/ttyACM0
baud: 57600
```

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch vehicle_interface_bringup arduino.launch
```

센서 드라이버와 함께 실행할 때는 통합 launch에서 Arduino를 명시적으로 활성화한다.

```bash
roslaunch sensor_bringup sensors.launch enable_arduino:=true
```

다른 장치 경로나 baud를 이번 실행에만 사용할 때는 인자를 덮어쓴다.

```bash
roslaunch vehicle_interface_bringup arduino.launch \
  port:=/dev/ttyUSB0 baud:=57600

roslaunch sensor_bringup sensors.launch \
  enable_arduino:=true arduino_port:=/dev/serial/by-id/usb-DEVICE_ID
```

연결되면 다른 터미널에서 토픽과 피드백을 확인한다.

```bash
rostopic list | grep erp42_serial
rostopic info /erp42_serial/drive
rostopic echo /erp42_serial/feedback
```

일반 사용자가 직렬 포트를 열 수 없다면 `dialout` 그룹에 추가한 뒤 로그아웃하고 다시
로그인한다.

```bash
sudo usermod -aG dialout "$USER"
```

## 무출력 통신 시험

처음에는 차량 24 V 모터 전원을 차단하고 `BROON_ENABLE_ACTUATOR_OUTPUTS=0`을 유지한다.
정지 명령을 10 Hz로 발행해 명령 구독과 피드백만 확인한다.

```bash
rostopic pub -r 10 /erp42_serial/drive erp42_msgs/DriveCmd \
  "{KPH: 0, Deg: 0, brake: 1, Gear: 1, EStop: 0}"
```

ROS 모드는 ROS 토픽으로 선택하지 않는다. RC 수신기의 AUX 신호가 유효하면서 현재 설정의
ROS 임계값인 1200 us 이하여야 한다. AUX 신호가 사라지거나 ROS 명령이 1초 이상 끊기면
Arduino는 명령을 무효화하고 출력을 차단한다.

## 실제 출력 전 확인사항

실제 모터 출력은 ROS 연결과 별개의 물리 검증 단계다. 다음 항목을 실차에서 측정하고
검증하기 전에는 출력 잠금을 해제하지 않는다.

- 조향 포텐셔미터의 좌측 기계 끝, 좌측 안전 끝, 중앙, 우측 안전 끝, 우측 기계 끝
- 조향 ADC 증가 방향과 조향 모터 방향
- 전륜 엔코더 count/rev와 타이어 둘레
- 전륜·후륜 구동 방향과 안전한 최대 PWM
- E-stop 활성·해제 극성
- RC 조향·스로틀·AUX 범위
- ROS 명령 timeout 및 RC 수신기 손실 시 PWM 0 동작

교정값이 현재 `ControllerConfig.h`의 0 또는 임시값으로 남아 있는 상태에서
`BROON_ENABLE_ACTUATOR_OUTPUTS`, `BROON_STEERING_CALIBRATION_CONFIRMED`,
`BROON_ESTOP_POLARITY_CONFIRMED`를 임의로 활성화하지 않는다.

## 제어 노드 연결

현재 Arduino의 최종 명령 토픽은 `/erp42_serial/drive`다. 미션 주행에서는 제어기가
`/mission/state`의 정지·속도·방향과 `/path/final`을 반영해 이 토픽으로 직접 발행한다.
별도 Vehicle Safety Gate는 제거했다. `/vehicle/emergency_stop`은 Control의 독립 비상정지
입력이고, 물리 RC 정지와 ROS timeout은 Arduino 내부에서도 계속 적용된다. 자세한 연결은
[State Manager 안내](../../state_manager/README.md)를 참고한다.

## rosserial 문제 해결

`Unable to sync with device`가 반복되면 다음을 순서대로 확인한다.

```bash
ls -l /dev/serial/by-id/
fuser /dev/ttyACM0
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

- Arduino IDE의 Serial Monitor를 닫는다.
- Arduino 펌웨어와 PC의 baud를 모두 `57600`으로 맞춘다.
- `BROON_ENABLE_ROS=1`, `BROON_ENABLE_HUMAN_SERIAL=0`인지 확인한다.
- `erp42_msgs` 변경 후 `make_libraries.py`를 다시 실행하고 펌웨어를 재컴파일한다.
- 포트를 점유한 이전 `serial_node.py` 또는 다른 프로그램을 종료한다.

연결 직후 `wrong checksum for topic id` 또는 메시지 MD5 오류가 나오면 Arduino의
`~/Arduino/libraries/ros_lib/erp42_msgs`가 현재 워크스페이스 메시지에서 생성된 것인지
확인하고 헤더를 다시 생성한다.
