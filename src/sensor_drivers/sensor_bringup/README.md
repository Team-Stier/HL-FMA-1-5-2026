# Sensor bringup

센서 드라이버와 선택적인 T870 Arduino rosserial을 한 번에 실행하는 launch 전용
패키지다. 개별 드라이버 패키지는 그대로 유지하며 이 패키지는 실행 순서와 토픽
계약만 조합한다.

```bash
roslaunch sensor_bringup sensors.launch
```

기본 실행은 LiDAR, Camera, GPS, IMU를 시작한다. Arduino는 차량별 펌웨어가
`BROON_ENABLE_ROS=1`, `BROON_ENABLE_HUMAN_SERIAL=0`으로 빌드됐는지 확인하기 전에는
연결하지 않도록 기본 비활성이다.

```bash
roslaunch sensor_bringup sensors.launch \
  enable_arduino:=true
```

Arduino 기본 포트와 baud는
`src/sensor_drivers/arduino/ros/config/serial.yaml`에서 설정한다. 실행 시에만 바꾸려면
`arduino_port:=/dev/ttyUSB0 arduino_baud:=57600`을 추가한다. 별도 YAML은
`arduino_config:=/absolute/path/serial.yaml`로 선택할 수 있다.

통합 launch는 다음 Localization 계약으로 센서 출력을 정렬한다.

| 장치 | 통합 출력 |
|---|---|
| LiDAR | `/molit/sensors/lidar/scan`, frame `laser_link` |
| IMU | `/mando_localization/internal/driver/imu` |
| GPS fix | `/mando_localization/internal/driver/gps_fix` |
| GPS NavPVT | `/mando_localization/internal/driver/gps_navpvt` |
| Arduino feedback | `/erp42_serial/feedback` |

Localization이 `base_link → laser_link` 정적 TF를 소유하므로 통합 launch에서는 LiDAR
드라이버의 TF 발행을 기본 비활성화한다. Localization 없이 센서만 시험할 때는
`publish_lidar_static_tf:=true`를 명시한다.

개별 장치를 제외하려면 `enable_lidar`, `enable_cam`, `enable_gps`, `enable_imu`,
`enable_arduino` 중 해당 인자를 `false`로 지정한다. 같은 드라이버의 개별 launch와
이 통합 launch를 동시에 실행하면 안 된다.
