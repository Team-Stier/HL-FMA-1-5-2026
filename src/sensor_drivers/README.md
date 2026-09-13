# sensor_drivers

실차 센서 드라이버를 종류별로 모아 둔 디렉터리다. 각 센서의 bringup과 외부 드라이버가
섞이지 않도록 다음 구조를 사용한다.

```text
sensor_drivers/
├── arduino/
│   ├── ros/
│   └── logger/
├── sensor_bringup/
│   └── launch/
├── lidar/
│   ├── config/
│   ├── launch/
│   ├── scripts/
│   └── udev/
├── cam/
│   ├── cam_bringup/
│   ├── scripts/
│   └── udev/
├── gps/
│   ├── gps_bringup/
│   ├── ublox/
│   ├── ublox_utils/
│   ├── ntrip_client-ros/
│   ├── ROS-UTM-LLA/
│   ├── scripts/
│   └── udev/
└── imu/
    ├── imu_bringup/
    ├── xsens_ros_mti_driver/
    ├── scripts/
    └── udev/
```

## 센서 통합 실행

`sensor_bringup`은 센서별 launch를 조합하는 실행 전용 패키지다.
워크스페이스를 빌드한 뒤 다음 명령으로 LiDAR, 카메라, GPS, IMU, ROS 제어 아두이노, CAN CSV 로거를 함께 실행한다.
두 아두이노는 기본 활성화되며 포트는 [Arduino 안내](arduino/README.md)의 YAML 파일에서 각각 설정한다.
각 센서의 기존 설정과 고정 장치 경로를 사용하며, 카메라는 기본적으로 Razer Kiyo Pro다.

```bash
source ~/HL-FMA2026-suhyeon/devel/setup.bash
roslaunch sensor_bringup sensors.launch
```

연결하지 않은 장치는 `enable_lidar`, `enable_cam`, `enable_gps`, `enable_imu`, `enable_arduino`를
`false`로 지정해 제외한다. 예를 들어 LiDAR와 IMU만 실행하려면:

```bash
roslaunch sensor_bringup sensors.launch enable_cam:=false enable_gps:=false enable_arduino:=false enable_can_logger:=false
```

Logitech C922를 사용하는 경우:

```bash
roslaunch sensor_bringup sensors.launch camera_launch:=cam_logi.launch
```

CAN 로거의 포트·baud·저장 폴더는 `arduino/logger/config/logger.yaml`에서 수정한다.
로거는 기본 활성화되며 `enable_can_logger:=false`로 제외한다.
CSV는 기본 `~/bags/can/` 아래 실행별 폴더에 저장된다.
[CAN 로거 안내](arduino/logger/README.md)를 참고한다.

제어용 Arduino rosserial은 `arduino/ros/config/serial.yaml`에서 포트와 baud를 설정한다.
기본값은 `/dev/ttyACM1`, 57600 baud다. 현재 CH340 보드의 udev 규칙은
장치 ID와 이 PC의 등록된 USB 포트 위치를 함께 검사하므로 같은 USB 포트를 사용한다.
새 PC에서는 Arduino udev 설치 스크립트를 실행하고 연결 위치를 확인해야 한다.
등록 전에는 확인한 `/dev/serial/by-id/...` 경로를
`arduino_port`에 지정하거나 `enable_arduino:=false`로 제외한다.
통신속도는 `arduino_baud`로 변경할 수 있으며 펌웨어 설정과 일치해야 한다.
자세한 포트 식별 방법은 [Arduino 연결 안내](../interfaces/vehicle_interface/README.md#arduino-연결과-rosserial-실행)를 참고한다.

GPS는 NTRIP 클라이언트도 함께 실행하므로 RTK 보정 수신에는 인터넷 연결이 필요하다.
통합 실행 중에는 같은 센서의 개별 launch를 중복 실행하지 않는다.
`Ctrl+C`로 함께 실행한 센서 노드를 종료한다.

## 장치 이름 고정

`/dev/ttyUSB*` 번호는 연결 순서에 따라 바뀌므로 사용하지 않는다. udev 규칙에서 센서의
USB 고유 시리얼을 확인해 역할별 고정 이름을 만든다.

- LiDAR: `/dev/lidar`
- GPS: `/dev/gps`
- IMU: `/dev/imu`
- Camera: `/dev/cam`

같은 역할의 예비 센서가 여러 대면 각 고유 시리얼에 대해 같은 고정 이름을 부여한다. 정상
운용 시 같은 역할의 센서는 한 대만 연결한다.

## RPLIDAR S2

S2M1 두 대의 USB 고유 시리얼은 `lidar/udev/99-stier-rplidar.rules`에 등록되어 있으며 둘
중 어느 것을 연결해도 `/dev/lidar`가 된다. 드라이버는 1,000,000 baud, 10 Hz로 실행해
`sensor_msgs/LaserScan` 형식의 `/scan`을 발행한다.

차량 좌표계는 `base_link`의 `+x`가 전방, `+y`가 왼쪽, `+z`가 위쪽이다. 현재 확인한
RPLIDAR ROS 출력은 차량 기준으로 전후와 좌우가 모두 반전되어 있으므로 launch 파일이
`base_link`에서 `laser`로 yaw `pi`인 정적 TF를 발행한다. RViz의 Fixed Frame은
`base_link`로 설정한다.

```bash
./src/sensor_drivers/lidar/scripts/install_udev_rules.sh
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lidar_bringup rplidar_s2.launch
```

LiDAR의 실제 장착 위치를 측정한 뒤 `laser_x`, `laser_y`, `laser_z`에 `base_link` 원점
기준 좌표를 m 단위로 지정한다. 장착 방향이 바뀌면 `laser_yaw`, `laser_pitch`,
`laser_roll`도 rad 단위로 조정한다.

```bash
roslaunch lidar_bringup rplidar_s2.launch \
  laser_x:=0.0 laser_y:=0.0 laser_z:=0.0 \
  laser_yaw:=3.141592653589793
```

## u-blox GPS

ZED-F9P는 udev 고정 경로 `/dev/gps`를 사용한다. 참고 프로젝트와 같은 `ublox_gps`,
`ublox_utils`, NTRIP 설정 및 UTM Zone 52 North 변환을 함께 실행한다.

```bash
./src/sensor_drivers/gps/scripts/install_udev_rules.sh
roslaunch gps_bringup gps.launch
```

## 현재 구현 상태

| 센서 | 상태 | 고정 장치 경로 | 실행 패키지 | 주요 토픽 |
|---|---|---|---|---|
| RPLIDAR S2M1 | 드라이버·udev·TF 구성 완료 | `/dev/lidar` | `lidar_bringup` | `/scan` |
| u-blox ZED-F9P | 드라이버·NTRIP·UTM·상태·udev 구성 완료 | `/dev/gps` | `gps_bringup` | `/ublox_position_receiver/fix`, `/gps/status`, `/utm` |
| Xsens MTi-3-0L-DK | 드라이버·udev 구성 완료 | `/dev/imu` | `imu_bringup` | `/imu/data` |
| USB Camera | bringup·udev 구성 완료 | `/dev/cam` | `cam_bringup` | `/usb_cam/image_raw` |

상세 장치 식별 방법은 [LiDAR README](lidar/README.md), [Camera README](cam/README.md),
[GPS README](gps/README.md), [IMU README](imu/README.md)를 참고한다.

## 필요한 패키지

루트 README의 일괄 설치 명령을 사용하는 것이 가장 간단하다. 센서 부분만 설치하려면
Ubuntu 20.04와 ROS Noetic에서 다음 명령을 실행한다.

```bash
sudo apt update
sudo apt install \
  python3-serial libgeographic-dev v4l-utils \
  ros-noetic-diagnostic-updater \
  ros-noetic-mavros-msgs \
  ros-noetic-nmea-msgs \
  ros-noetic-rplidar-ros \
  ros-noetic-rqt-image-view \
  ros-noetic-rqt-runtime-monitor \
  ros-noetic-rtcm-msgs \
  ros-noetic-rviz \
  ros-noetic-tf2-ros \
  ros-noetic-topic-tools \
  ros-noetic-usb-cam
```

Xsens SDK와 ROS 드라이버, u-blox 드라이버, NTRIP 클라이언트와 UTM 변환기는 저장소 안에
포함되어 있다. RPLIDAR와 USB camera 실행 파일은 위 apt 패키지를 사용한다.

## 공통 준비

모든 명령은 워크스페이스 루트에서 실행한다.

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

새 터미널을 열 때마다 최소한 다음 환경 설정은 다시 실행한다.

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## udev 규칙 최초 설치

각 규칙은 PC마다 한 번 설치한다. 설치 후 해당 USB 장치를 뺐다가 다시 연결한다.

```bash
./src/sensor_drivers/lidar/scripts/install_udev_rules.sh
./src/sensor_drivers/cam/scripts/install_udev_rules.sh
./src/sensor_drivers/gps/scripts/install_udev_rules.sh
./src/sensor_drivers/imu/scripts/install_udev_rules.sh

ls -l /dev/lidar /dev/cam /dev/gps /dev/imu
```

LiDAR는 등록된 두 제품의 USB 고유 시리얼 중 어느 장치를 연결해도 `/dev/lidar`가 된다.
GPS 규칙은 참고 차량과 동일하게 u-blox USB ID `1546:01a9`를 `/dev/gps`로 연결한다.
Camera는 등록된 Kiyo Pro 또는 Logitech C922를 `/dev/cam`으로 연결하고, IMU는 USB serial
`DB8GG04M`을 `/dev/imu`로 연결한다. 같은 역할의 장치를 동시에 두 대 연결하지 않는다.

## 좌표계 규약

차량 기준 좌표계는 ROS REP-103의 차량 좌표 관례에 맞춰 다음과 같이 사용한다.

- `base_link`: 차량 기준 프레임
- `+x`: 차량 전방
- `+y`: 차량 왼쪽
- `+z`: 차량 위쪽

RPLIDAR 출력은 실차 장착 방향에서 전후·좌우가 반전되어 확인되었기 때문에 기본 launch가
`base_link -> laser`에 yaw `pi`인 정적 TF를 발행한다. 센서 장착 위치를 측정한 뒤
`laser_x`, `laser_y`, `laser_z`를 m 단위로 수정한다. GPS, IMU, Camera는 현재 메시지의
`frame_id`만 각각 `gps_link`, `imu_link`, `camera_link`로 설정되어 있다. 해당 센서에서
`base_link`로 이어지는 장착 TF는 실제 위치와 축 방향을 측정한 뒤 추가한다.

## LiDAR 실행과 확인

```bash
roslaunch lidar_bringup rplidar_s2.launch
```

다른 터미널에서 다음을 확인한다.

```bash
rostopic hz /scan
rostopic echo -n 1 /scan/header
rosrun tf tf_echo base_link laser
```

RViz에서는 `Fixed Frame`을 `base_link`로 설정하고 `LaserScan` 표시의 Topic을 `/scan`으로
지정한다. 빨간색 축은 `+x`, 녹색 축은 `+y`, 파란색 축은 `+z`다.

## Camera 실행과 확인

Razer Kiyo Pro는 기본 launch, Logitech C922는 전용 하드웨어 설정 launch를 사용한다.

```bash
roslaunch cam_bringup cam.launch
# 또는
roslaunch cam_bringup cam_logi.launch
```

다른 터미널에서 영상 토픽과 실제 화면을 확인한다.

```bash
rostopic hz /usb_cam/image_raw
rostopic echo -n 1 /usb_cam/image_raw/header
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw
```

## GPS 실행과 확인

```bash
roslaunch gps_bringup gps.launch
```

다른 터미널에서 GNSS Fix, 수신 빈도 및 UTM 변환 결과를 확인한다.

```bash
rostopic echo -n 1 /ublox_position_receiver/fix
rostopic hz /ublox_position_receiver/fix
rostopic echo -n 1 /ublox_position_receiver/navstatus
rostopic echo -n 1 /gps/status
rostopic echo -n 1 /utm
```

GPS bringup은 u-blox 수신기, NMEA 생성, NTRIP 클라이언트, RTCM 전달 및 UTM Zone 52 North
변환을 함께 실행한다. NTRIP 최초 연결 실패로 노드가 종료될 경우 ROS 재실행 대기시간은
기존 30초에서 3초로 설정되어 있다. 연결 이후 NTRIP 내부 재접속 간격은 5초이고 RTCM
수신 타임아웃은 4초다.

GPS가 연결되어 있지 않으면 `/dev/gps`가 생성되지 않으며 실제 수신 시험도 할 수 없다.
먼저 다음 명령으로 장치 연결 상태를 확인한다.

```bash
ls -l /dev/gps
udevadm info --query=property --name=/dev/gps \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
```

## IMU 실행과 확인

```bash
roslaunch imu_bringup xsens_mti.launch
```

다른 터미널에서 Xsens MTi의 `sensor_msgs/Imu` 출력을 확인한다.

```bash
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
```

현재 드라이버는 USB 고유 시리얼 `DB8GG04M`으로 생성되는 `/dev/imu`를 115200 baud로
열고 메시지의 `frame_id`를 `imu_link`로 발행한다. 센서 장착 방향은 아직 확정하지 않았으며
차량 좌표계로 회전하는 TF는 실차 축 방향을 확인한 뒤 추가한다.

## 공통 문제 해결

`Resource not found` 또는 `package not found`가 나오면 현재 터미널에서 환경을 다시 불러온다.

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

고정 장치 경로가 없으면 해당 udev 설치 스크립트를 다시 실행하고 센서를 물리적으로 뺐다가
연결한다. 실제 포트와 USB 속성은 다음 명령으로 확인한다.

```bash
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' -o -name 'video*' \) -print
ls -l /dev/serial/by-id/ /dev/v4l/by-id/ 2>/dev/null
udevadm monitor --udev --property
```

토픽이 보이지 않으면 먼저 노드가 살아 있는지와 토픽 연결 상태를 확인한다.

```bash
rosnode list
rostopic list
rostopic info /scan
rostopic info /imu/data
rostopic info /ublox_position_receiver/fix
rostopic info /usb_cam/image_raw
```

동일 역할의 예비 센서를 동시에 연결하면 같은 `/dev/lidar`, `/dev/cam`, `/dev/gps` 또는
`/dev/imu` 이름이 충돌할 수 있다. 정상 운용에서는 역할별 한 대만 연결한다.
