# Camera

카메라 launch와 하드웨어 설정 스크립트는 `cam_bringup/`, 고정 장치 이름 규칙은 `udev/`와
`scripts/`에 분리해 둔다.

## 장치 식별

연결된 V4L2 카메라와 `/dev/video*` 번호를 확인한다.

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/ /dev/v4l/by-path/
```

특정 영상 장치의 제조사, 모델, 고유 시리얼을 확인한다.

```bash
udevadm info --query=property --name=/dev/video0 \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
v4l2-ctl --device=/dev/video0 --all
```

하나의 USB 카메라가 영상·메타데이터용 `/dev/video*`를 여러 개 만들 수 있으므로 지원
포맷도 확인한다.

```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

카메라 드라이버를 추가할 때는 각 카메라의 고유 시리얼을 udev 규칙에 등록하고 같은
`/dev/cam` 이름을 부여한다. 영상 캡처 인터페이스만 매칭하도록 V4L2 capability도 함께
확인한다. `v4l2-ctl`이 없으면 `sudo apt install v4l-utils`로 설치한다.

## 가져온 카메라 구성

- 참고 원본: `Team-Stier/JEJU-ROS-autonomous-vehicle/src/drivers/cam`
- 참고 revision: `937e52a259ab9afc6d87b8f7e3f2ffc9aa0cd21c`
- 주 카메라: Razer Kiyo Pro
- 예비 카메라: Logitech C922
- 실행 패키지: `cam_bringup`
- 고정 장치 경로: `/dev/cam`
- 출력: `/usb_cam/image_raw`, 640x360, YUYV, 30 FPS
- frame: `camera_link`

참고 저장소의 세 launch와 Kiyo/Logitech 하드웨어 설정을 가져왔다. 이 워크스페이스에서는
센서 종류별 폴더와 bringup 패키지를 분리하고, 정상 운용 중 카메라 한 대만 연결한다.

## 등록된 장치

udev 규칙은 `udev/99-stier-camera.rules`에 있다.

- Razer Kiyo Pro: Vendor `1532`, Product `0e05`
- Logitech C922: Vendor `046d`, Product `085c`, Serial `8F25D8AF`

참고 저장소의 Kiyo Pro 규칙에는 고유 시리얼 조건이 없어 vendor/product와 영상 capture
capability 조건을 그대로 사용했다. 실제 Kiyo를 연결했을 때 `ID_SERIAL_SHORT`가 제공되면
그 값을 규칙에 추가한다. 두 규칙 모두 `/dev/cam`을 생성하므로 두 카메라를 동시에 연결하지
않는다.

```bash
./src/sensor_drivers/cam/scripts/install_udev_rules.sh
ls -l /dev/cam
```

## 의존성

Ubuntu 20.04와 ROS Noetic 기준으로 다음 패키지가 필요하다.

```bash
sudo apt update
sudo apt install ros-noetic-usb-cam ros-noetic-rqt-image-view v4l-utils
```

패키지를 빌드하고 현재 터미널에 반영한다.

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
catkin_make --pkg cam_bringup
source devel/setup.bash
```

## 실행

Razer Kiyo Pro:

```bash
roslaunch cam_bringup cam.launch
```

Logitech C922:

```bash
roslaunch cam_bringup cam_logi.launch
```

참고 저장소의 저가형 카메라 launch도 보관했다. PC마다 by-path가 다를 수 있으므로 실행할
때 실제 영상 capture 경로를 지정한다.

```bash
roslaunch cam_bringup cam_daiso.launch device:=/dev/video0
```

영상 출력은 다음 명령으로 확인한다.

```bash
rostopic hz /usb_cam/image_raw
rostopic echo -n 1 /usb_cam/image_raw/header
rostopic echo -n 1 /usb_cam/camera_info
rosrun rqt_image_view rqt_image_view /usb_cam/image_raw
```

카메라의 실제 장착 위치와 `base_link -> camera_link` TF, 내부 파라미터 calibration은 실차
장착 후 별도로 설정한다.

## 문제 해결

고정 경로가 없거나 잘못된 영상 인터페이스를 가리키면 다음 순서로 확인한다.

```bash
ls -l /dev/cam
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/cam --list-formats-ext
udevadm info --query=property --name=/dev/cam \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT|ID_V4L_CAPABILITIES)='
```

`cannot open device`가 나오면 udev 규칙을 다시 설치한 뒤 카메라를 뺐다가 연결한다. Kiyo나
Logitech에서 일부 `v4l2-ctl` control 이름을 지원하지 않는다는 오류가 나지만 영상은 정상인
경우, 하드웨어 설정 노드만 끄고 실행할 수 있다.

```bash
roslaunch cam_bringup cam.launch apply_hardware_setup:=false
```

프레임이 나오지 않으면 실제 지원 포맷에 맞춰 launch 인자를 바꾼다.

```bash
roslaunch cam_bringup cam.launch \
  image_width:=640 image_height:=360 pixel_format:=yuyv framerate:=30
```
