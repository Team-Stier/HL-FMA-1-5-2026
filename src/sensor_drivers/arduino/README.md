# Arduino

두 아두이노는 별도 USB 포트를 사용하며 센서 통합 launch에서 둘 다 기본 실행한다.

| 용도 | 포트 설정 파일 | 기본 설정 | ROS 패키지 |
|---|---|---|---|
| ROS 제어·피드백 | `ros/config/serial.yaml` | `/dev/ttyACM1`, 57600 | `vehicle_interface_bringup` |
| CAN CSV 로거 | `logger/config/logger.yaml` | `/dev/ttyUSB3`, 115200 | `can_logger` |

각 YAML의 `port`를 실제 연결된 장치로 수정한다. 기본 번호는 이전 연결에서 확인한
예시이므로 재연결 후 확인해야 한다. 두 파일에 같은 장치를 지정하지 않는다.
`/dev/serial/by-id/...` 같은 경로도 사용할 수 있다.

```bash
source ~/HL-FMA2026-suhyeon/devel/setup.bash
roslaunch sensor_bringup sensors.launch
```

제어 아두이노만 제외하려면 `enable_arduino:=false`, 로거만 제외하려면
`enable_can_logger:=false`를 붙인다. IMU가 없으면 `enable_imu:=false`를 붙인다.

로거 저장 경로와 이름도 `logger/config/logger.yaml`에서 변경한다.
기본 출력은 `~/bags/can/날짜_시간_sensors/`이며 Ctrl+C 시 CSV를 닫는다.
자세한 로깅 동작은 [logger 안내](logger/README.md)를 참고한다.

이전에 만든 `/dev/arduino` 규칙은 등록 당시 CH340 보드와 USB 포트를 가리킨다.
제어용과 로거용 펌웨어를 식별하는 이름이 아니므로 실제 역할을 확인한 뒤 사용한다.
