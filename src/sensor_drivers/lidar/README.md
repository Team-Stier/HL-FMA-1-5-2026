# LiDAR

LiDAR 드라이버 설정은 `config/`, 실행 파일은 `launch/`에 둔다. 실차에는 LiDAR 한 대만
사용하며, 같은 모델의 두 제품 중 어느 것을 연결해도 별도 설정 변경 없이 동작해야 한다.
두 제품의 USB 고유 시리얼을 udev 규칙에 등록해 둘 중 어느 것을 연결해도 `/dev/lidar`로
접근한다. `/dev/ttyUSB*` 번호는 사용하지 않는다.

## 장치 식별

LiDAR를 하나만 연결한 상태에서 고정 경로를 확인한다.

```bash
ls -l /dev/serial/by-id/
```

현재 포트가 `/dev/ttyUSB0` 또는 `/dev/ttyUSB1`이라면 udev 속성에서 USB 인터페이스의
고유 시리얼을 확인할 수 있다.

```bash
udevadm info --query=property --name=/dev/ttyUSB0 \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
```

어느 포트인지 모르면 다음 명령을 실행한 상태에서 LiDAR USB를 뺐다가 다시 연결한다.

```bash
udevadm monitor --udev --property
```

현재 등록된 두 S2의 USB 시리얼은 다음과 같다.

- `de67a055ea82eb1193cb9d8d9693f7bc`
- `fc5924fee413ec119e0cf2ef7a109228`

등록 파일은 `udev/99-stier-rplidar.rules`다. LiDAR가 추가되면 기존 규칙 한 줄을 복사해
`ATTRS{serial}` 값만 새 장치의 `ID_SERIAL_SHORT`로 바꿔 추가한다.

## 의존성 설치

RPLIDAR ROS 드라이버와 RViz를 설치한다.

```bash
sudo apt update
sudo apt install ros-noetic-rplidar-ros ros-noetic-rviz
```

## udev 규칙 설치

워크스페이스 루트에서 한 번 실행하고 LiDAR를 다시 연결한다.

```bash
./src/sensor_drivers/lidar/scripts/install_udev_rules.sh
ls -l /dev/lidar
```

## 실행

- 모델: RPLIDAR S2M1
- 드라이버 설정: `config/rplidar_s2.yaml`
- 장치 경로: `/dev/lidar`
- 프레임: `laser`
- 토픽: `/scan`

등록된 S2 중 사용할 한 대를 연결하고 실행한다.

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
catkin_make --pkg lidar_bringup
source devel/setup.bash
roslaunch lidar_bringup rplidar_s2.launch
```

기본 설정은 1,000,000 baud, 10 Hz, DenseBoost scan이며 `base_link -> laser`에 yaw `pi`인
정적 TF를 함께 발행한다. 장착 위치는 m, 회전은 rad 단위 launch 인자로 지정한다.

```bash
roslaunch lidar_bringup rplidar_s2.launch \
  laser_x:=0.0 laser_y:=0.0 laser_z:=0.0 \
  laser_yaw:=3.141592653589793
```

## 출력 확인

```bash
rostopic hz /scan
rostopic echo -n 1 /scan/header
rosrun tf tf_echo base_link laser
rviz
```

RViz에서 `Fixed Frame`은 `base_link`, `LaserScan` Topic은 `/scan`으로 설정한다. Axes 표시의
빨간색은 `+x`, 녹색은 `+y`, 파란색은 `+z`다.

## 문제 해결

장치가 열리지 않으면 고정 경로와 권한을 확인한다.

```bash
ls -l /dev/lidar
udevadm info --query=property --name=/dev/lidar \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
```

시작 직후 reset 과정에서 `RESULT_OPERATION_TIMEOUT`이 한 번 나타나더라도 이후 로그에
`RPLidar reset successfully`, `health status : OK`, scan mode와 frequency가 차례로 나오고
`/scan`이 발행되면 정상 동작이다. 해당 로그 없이 노드가 종료되거나 `/scan`이 나오지 않으면
USB 케이블, 전원, 다른 프로세스의 포트 점유 여부를 확인한다.
