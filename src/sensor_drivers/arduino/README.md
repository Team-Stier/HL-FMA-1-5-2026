# Arduino sensor driver

BROON T870 Arduino의 차량별 펌웨어와 분리해, PC에서 실행하는 rosserial bringup만
`ros/`에 둔다. Catkin 패키지 이름은 기존 명령과 호환되도록
`vehicle_interface_bringup`을 유지한다.

이 시험 패키지 브랜치는 별도 Mando clone 없이 사용할 수 있도록 하얀차·검은차 펌웨어의
검증된 수정본을 [`firmware/`](firmware/ROS_GEAR_INTEGRATION.md)에 함께 제공한다.
원래 Mando 저장소는 수정하지 않으며, Catkin 패키지 `ros/`와 펌웨어 소스는 분리한다.
두 펌웨어 모두 동일한 ROS 메시지와 직렬 연결을 사용하며, 한 ROS Master에서는
실제 연결한 차량의 Arduino 하나만 실행한다.

## 포트 설정

기본 연결값은 [`ros/config/serial.yaml`](ros/config/serial.yaml)에서 변경한다.

```yaml
port: /dev/ttyACM0
baud: 57600
```

`port`에는 `/dev/ttyACM0`, `/dev/ttyUSB0` 또는 안정적인
`/dev/serial/by-id/...` 경로를 지정할 수 있다. 실제 장치 경로는 다음 명령으로 확인한다.

```bash
ls -l /dev/serial/by-id/
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
```

설정 파일의 기본값으로 실행한다.

```bash
roslaunch vehicle_interface_bringup arduino.launch
```

파일을 수정하지 않고 이번 실행에만 포트나 baud를 덮어쓸 수도 있다.

```bash
roslaunch vehicle_interface_bringup arduino.launch \
  port:=/dev/ttyUSB0 baud:=57600

roslaunch sensor_bringup sensors.launch \
  enable_arduino:=true arduino_port:=/dev/serial/by-id/usb-DEVICE_ID
```

다른 설정 파일을 사용하려면 `config:=/absolute/path/serial.yaml` 또는 통합 launch의
`arduino_config:=/absolute/path/serial.yaml`을 지정한다.
