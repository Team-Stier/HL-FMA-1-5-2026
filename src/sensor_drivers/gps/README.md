# GPS (u-blox ZED-F9P)

`ublox_gps 1.5.0` 드라이버와 `ublox_utils 1.0.0`을 사용한다. udev 규칙으로 수신기를
`/dev/gps`에 고정하고 `gps_bringup/launch/gps.launch`에서 Network-RTK 보정과 UTM 변환을
함께 실행한다.

## 장치 식별

u-blox 등 USB/시리얼 GNSS의 고정 경로와 현재 포트를 확인한다.

```bash
ls -l /dev/serial/by-id/
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
```

후보 포트의 제조사, 모델, 고유 시리얼을 확인한다.

```bash
udevadm info --query=property --name=/dev/ttyACM0 \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
```

포트를 특정하기 어려우면 다음 명령을 실행하고 GNSS USB를 뺐다가 다시 연결한다.

```bash
udevadm monitor --udev --property
```

현재 udev 규칙은 Team Stier에서 사용하는 u-blox 장치의 USB ID `1546:01a9`를
`/dev/gps`로 연결한다. 설치 후 수신기를 다시 연결한다.

```bash
./src/sensor_drivers/gps/scripts/install_udev_rules.sh
ls -l /dev/gps
```

참고 차량의 세 수신기가 같은 USB ID를 사용하므로 고유 serial 대신 vendor/product로
등록되어 있다. 두 u-blox 수신기를 동시에 연결하면 `/dev/gps`가 충돌할 수 있으므로 정상
운용에서는 한 대만 연결한다.

## 의존성 설치와 빌드

u-blox, NTRIP과 UTM 변환 소스는 저장소 안에 포함되어 있다. 다음 시스템/ROS 의존성을
설치한다.

```bash
sudo apt update
sudo apt install \
  python3-serial libgeographic-dev \
  ros-noetic-diagnostic-updater \
  ros-noetic-mavros-msgs \
  ros-noetic-nmea-msgs \
  ros-noetic-rqt-runtime-monitor \
  ros-noetic-rtcm-msgs \
  ros-noetic-topic-tools
```

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 실행

```bash
roslaunch gps_bringup gps.launch
```

주요 토픽:

- `/ublox_position_receiver/fix`: `sensor_msgs/NavSatFix`
- `/ublox_position_receiver/navpvt`: u-blox 위치·속도 정보
- `/ublox_position_receiver/navstatus`: GNSS fix 상태
- `/gps/status`: localization에서 사용할 GNSS 상태와 예상 정확도
- `/utm`: UTM Zone 52 North 좌표 `geometry_msgs/PoseStamped`

수신 상태와 RTCM 입력도 함께 확인할 수 있다.

```bash
rostopic hz /ublox_position_receiver/fix
rostopic hz /ublox_position_receiver/navpvt
rostopic echo -n 1 /ublox_position_receiver/rxmrtcm
rostopic echo -n 1 /diagnostics
rosrun rqt_runtime_monitor rqt_runtime_monitor
```

## GPS 상태와 정확도

`gps.launch`는 `gps_status` 노드를 함께 실행한다. 이 노드는 u-blox의 `NavPVT`와
`NavSTATUS`를 읽어 다음 정보를 `sensor_interfaces/GpsStatus` 형식의 `/gps/status`로
발행한다.

- `status_text`: `NO_FIX`, `2D_FIX`, `3D_FIX`, `DGNSS`, `RTK_FLOAT`, `RTK_FIXED`
- `fix_ok`: DOP 및 accuracy mask를 통과한 유효 위치인지 여부
- `carrier_solution`: RTK 없음, Float, Fixed
- `spoofing_state`: 감지 정보 없음, 이상 없음, 감지됨, 여러 번 감지됨
- `satellites_used`: 위치 계산에 사용한 위성 수
- `horizontal_accuracy_m`, `vertical_accuracy_m`: 수신기가 추정한 위치 정확도 [m]
- `position_dop`: PDOP
- `speed_accuracy_mps`: 속도 추정 정확도 [m/s]
- `heading_accuracy_deg`: 진행 방향 추정 정확도 [deg]
- `time_to_first_fix_ms`: 수신기 시작 후 최초 fix까지 걸린 시간 [ms]

`spoofing_state`는 최근에 도착했다는 이유만으로 재사용하지 않는다. `NavPVT`와
`NavSTATUS`의 GPS time-of-week가 기본 500 ms 이내이고, `NavSTATUS` 수신 시각도
0.5초 이내일 때만 `nav_status_available=true`로 발행한다. 조건을 벗어나면 상태를
`SPOOF_UNKNOWN`으로 내보내 Localization이 fail-closed로 거부한다.

확인 명령은 다음과 같다.

```bash
rostopic echo /gps/status
rostopic hz /gps/status
rosmsg show sensor_interfaces/GpsStatus
```

Localization에서는 `fix_ok`가 참인지 먼저 검사하고, `solution`과
`horizontal_accuracy_m`을 함께 사용한다. `RTK_FIXED`여도 추정 정확도가 허용 범위를
벗어나면 위치를 무조건 신뢰하지 않는다. `horizontal_accuracy_m`과
`vertical_accuracy_m`은 실제 기준점과 비교한 오차가 아니라 수신기가 계산한 예상 정확도다.

`solution` 값은 다음 enum을 사용한다.

| 값 | 상태 |
|---:|---|
| 0 | `NO_FIX` |
| 1 | `2D_FIX` |
| 2 | `3D_FIX` |
| 3 | `DGNSS` |
| 4 | `RTK_FLOAT` |
| 5 | `RTK_FIXED` |
| 6 | Dead reckoning only |
| 7 | GNSS + dead reckoning |
| 8 | Time only |

기존 `/ublox_position_receiver/navstatus`에서 `gpsFix`와 `flags2`를 직접 볼 경우 자주 보이는
조합은 다음과 같다. 전체 숫자를 직접 비교하기보다 `/gps/status`를 사용하는 것을 권장한다.

| `gpsFix`, `flags2` | 의미 |
|---|---|
| `2`, `8` | 2D Fix, RTK 없음 |
| `3`, `72` | 3D Fix, RTK Float |
| `3`, `136` | 3D Fix, RTK Fixed, spoofing 감지 없음 |
| `3`, `144` | 3D Fix, RTK Fixed, 해당 epoch에서 spoofing 이상 감지 |

참고 프로젝트와 동일하게 다음 NTRIP 값이 `gps_bringup/launch/gps.launch`에 평문으로
들어 있다.

- Host: `RTS1.ngii.go.kr`
- Mountpoint: `VRS-RTCM31`
- Username: `suhyeon351`
- Password: `ngii`

NTRIP 노드가 실패해 종료되면 roslaunch가 3초 뒤 다시 시작한다. 연결된 NTRIP 클라이언트의
내부 재접속 간격은 5초이고 RTCM 수신 timeout은 4초다. RTK Float 또는 Fixed로 가지 않으면
인터넷 연결, 계정, mountpoint, GGA 위치 전송과 RTCM 토픽을 차례로 확인한다.

UTM 변환은 `src/sensor_drivers/gps/ROS-UTM-LLA`의 `utm_lla` 패키지를 사용한다. 입력은
`/ublox_position_receiver/fix`, 출력은 `/utm`이며 한국 기준 Zone 52 North로 설정되어
있다. `/utm`은 로컬 원점을 뺀 좌표가 아니라 절대 UTM Easting/Northing이며 orientation은
설정하지 않는다. 현재 변환기가 `/fix`의 header를 그대로 복사하므로 `/utm`의
`header.frame_id`도 `gps_link`다. Localization에서는 이 값을 센서 로컬 좌표로 해석하지
말고 전역 `map`/UTM 프레임 정책을 정한 뒤 사용한다.

## 문제 해결

장치와 드라이버가 잡히지 않으면 다음을 확인한다.

```bash
ls -l /dev/gps
udevadm info --query=property --name=/dev/gps \
  | grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_SERIAL_SHORT)='
rosnode list | grep -E 'ublox|ntrip|utm|gps_status'
```

일반 GNSS 위치는 나오지만 RTK가 잡히지 않는 경우 다음 값들을 동시에 확인한다.

```bash
rostopic echo -n 1 /ublox_position_receiver/navpvt \
  | grep -E 'fixType|flags|numSV|hAcc|vAcc|pDOP'
rostopic echo -n 1 /gps/status
rostopic echo -n 1 /ublox_position_receiver/rxmrtcm
```

실내나 안테나 시야가 막힌 환경에서는 정상 구성이어도 Fix가 오래 걸리거나 RTK가 잡히지
않는다. 먼저 하늘이 열린 실외에서 안테나와 수신기를 시험한다.
