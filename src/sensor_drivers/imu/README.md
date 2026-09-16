# IMU

Xsens 원본 드라이버는 `xsens_ros_mti_driver/`, 차량 실행 설정은 `imu_bringup/`에 분리해
둔다.

## 장치 식별

Xsens 등 USB/시리얼 IMU의 고정 경로와 현재 포트를 확인한다.

```bash
ls -l /dev/serial/by-id/
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
```

후보 포트의 제조사, 모델, 고유 시리얼을 확인한다.

```bash
udevadm info --query=property --name=/dev/ttyUSB0 \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
```

포트를 특정하기 어려우면 다음 명령을 실행하고 IMU USB를 뺐다가 다시 연결한다.

```bash
udevadm monitor --udev --property
```

IMU 드라이버를 추가할 때는 각 IMU의 고유 시리얼을 udev 규칙에 등록하고 모두 같은
`/dev/imu` 이름을 부여한다. `/dev/ttyUSB*` 번호는 사용하지 않는다. Xsens 본체의 제품
코드나 장치 ID가 USB 시리얼과 별개로 제공되는 경우에는 드라이버 시작 로그도 함께
기록한다.

## 설치된 드라이버

- 장치: Xsens MTi-3-0L-DK
- ROS 패키지: `xsens_mti_driver` 1.1.0
- 원본: `xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client`
- 가져온 원본 revision: `71c487c95d883359d38e7bce6b3d0ef8dee9daad`
- 고정 장치 경로: `/dev/imu`
- baud rate: `115200`
- frame: `imu_link`
- 주요 토픽: `/imu/data`
- 실기 확인 결과: 약 100 Hz

Xsens 원본 `/imu/data_raw`의 세 covariance 배열이 모두 0이면 `imu_bringup`의
`imu_covariance_override_node`가 2026-08-31 정지·모터 OFF 측정값을 분산으로 변환해
`/imu/data`에 넣는다. 설정과 측정 출처는
`imu_bringup/config/imu_covariance.yaml`에서 관리한다. 이 값은 정지 noise floor이며
주행 진동이나 절대 yaw 정확도를 보증하지 않는다. 장치가 향후 0이 아닌 covariance를
제공하면 기본 설정은 그 값을 보존한다.

원본 저장소의 NTRIP 패키지는 u-blox GPS 구성을 중복하므로 가져오지 않았고,
`src/xsens_ros_mti_driver` 패키지만 이 폴더 아래에 포함했다. 원본 드라이버는 자동 장치
검색이 기본값이지만 `imu_bringup`에서 자동 검색을 끄고 `/dev/imu`를 명시한다.
또한 드라이버가 임의의 `world -> imu_link` TF를 발행하지 않도록
`pub_transform=false`로 덮어쓴다. 저장소의 기준 TF 트리는 실측한 정적 장착 TF와
Localization이 별도로 구성해야 한다.

## 등록된 장치

현재 확인된 Xsens USB 정보는 다음과 같다.

- Vendor ID: `2639`
- Product ID: `0300`
- USB serial: `DB8GG04M`
- 현재 연결 시 모델명: `Xsens Motion Tracker Dev. Board`
- 드라이버가 보고한 장치: `MTi-3-8A7G6`, device ID `03889250`

추가 등록 장치:

- USB serial: `DBBE3VMC`
- Vendor/Product ID: `2639:0300`
- 현재 연결 시 모델명: `Xsens Motion Tracker Dev. Board`
- 드라이버가 보고한 장치: `MTi-3-8A7G6`, device ID `0388BD48`

udev 규칙은 `udev/99-stier-xsens.rules`에 있다. 다른 MTi 제품도 예비 장치로 사용할 경우
장치를 하나씩 연결해 `ID_SERIAL_SHORT`를 확인한 뒤 같은 `/dev/imu` 이름을 사용하는 규칙을
한 줄씩 추가한다. 정상 운용 중에는 IMU를 한 대만 연결한다.

## 의존성 설치

Xsens SDK와 ROS 노드 소스는 저장소 안에 포함되어 있다. 드라이버가 사용하는 ROS 메시지
패키지를 설치한다.

```bash
sudo apt update
sudo apt install \
  ros-noetic-mavros-msgs \
  ros-noetic-nmea-msgs \
  ros-noetic-tf2-ros
```

## udev 규칙 설치

워크스페이스 루트에서 한 번 실행한 뒤 IMU를 다시 연결한다.

```bash
./src/sensor_drivers/imu/scripts/install_udev_rules.sh
ls -l /dev/imu
```

## 빌드와 실행

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
catkin_make --pkg xsens_mti_driver imu_bringup
source devel/setup.bash
roslaunch imu_bringup xsens_mti.launch
```

다른 터미널에서 다음을 확인한다.

```bash
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
```

정상 연결 시 시작 로그에 `/dev/imu`, 115200 baud, device ID와 제품명이 나오고
`Measuring ..`로 진입한다. 주요 출력은 다음과 같다.

- `/imu/data`: 자세 quaternion, 각속도, 선형 가속도
- `/imu/data_raw`: Xsens 원본 메시지(현재 covariance 배열은 모두 0)
- `/imu/angular_velocity`: 각속도
- `/imu/acceleration`: 선형 가속도
- `/imu/mag`: 자기장
- `/filter/euler`: Euler angle
- `/filter/free_acceleration`: 중력 성분을 제거한 가속도

장치 연결에 실패하면 먼저 고정 경로와 실제 USB 속성을 확인한다.

```bash
ls -l /dev/imu
udevadm info --query=property --name=/dev/imu \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
```

udev 규칙을 설치하지 않고 임시로 실제 포트를 시험할 때만 launch 인자를 덮어쓴다.

```bash
roslaunch imu_bringup xsens_mti.launch device:=/dev/ttyUSB0
```

포트를 다른 프로세스가 사용 중이면 열 수 없다. 다음 명령으로 점유 프로세스를 확인한다.

```bash
fuser /dev/imu
```

센서 축과 차량 축의 회전 및 `base_link -> imu_link` 장착 TF는 실차 데이터로 방향을 확인한
뒤 설정한다. 드라이버 설치 단계에서는 임의의 축 회전을 적용하지 않는다.
현재 적용한 covariance는 정지·모터 OFF noise 측정치다. 실제 차량 진동과 자기장 환경에서
방향 및 noise를 다시 측정하기 전에는 localization 정확도가 검증된 상태가 아니다.
