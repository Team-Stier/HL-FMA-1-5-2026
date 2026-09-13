# 기록한 GPS·LiDAR와 수동 경로 비교

ROS 1 Noetic에서 저장된 bag을 재생하는 절차다. 센서 드라이버나 차량 제어 노드를 실행할 필요가 없다. 아래 bag과 경로 파일은 저장소 밖에 두고 사용한다. 현재 이 PC의 원본 bag 폴더는
`/media/stier/Data/Ubuntu/rosbag_0906`이다. 저장장치가 마운트된 상태에서 실행한다.

## 1. 빌드와 터미널 준비

처음 한 번 워크스페이스를 빌드한다.

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

**이후 사용하는 모든 터미널**에서 다음을 실행한다. 재생용 ROS master는 `11411` 포트를 사용한다.

```bash
source /opt/ros/noetic/setup.bash
source ~/HL-FMA2026-suhyeon/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
```

첫 번째 터미널에서 master를 켠다.

```bash
roscore -p 11411
```

## 2. 수동 경로와 GPS를 함께 보기

다른 터미널에서 HTML 편집기의 **작업 저장 JSON**과 bag을 지정한다. JSON을 사용하면 편집기에 저장된 경로와 좌표 원점을 함께 읽을 수 있다.

```bash
roslaunch localization bag_route_compare.launch \
  bag:=/media/stier/Data/Ubuntu/rosbag_0906/1.bag \
  route_file:=/home/stier/다운로드/yongin_route_project.json \
  rate:=0.5 paused:=true \
  start_rviz:=true start_lidar_rviz:=true
```

두 RViz 창의 용도가 다르다.

- `route_map`: 수동 경로와 기록된 GPS를 동일한 원점의 동쪽·북쪽 미터 좌표로 비교한다. 기존 용인 편집기 원점은 위도 `37.288731`, 경도 `127.1072336`이다.
- `laser`: 차량에 붙어 있는 LiDAR 기준으로 `/scan`을 본다. GPS 지도 위에 LiDAR를 배치하는 창은 아니다.

위 실행은 일시정지 상태로 시작한다. 환경을 설정한 다른 터미널에서 재생·일시정지를 바꾼다.

```bash
rosservice call /route_compare_play/pause_playback 'data: false'
rosservice call /route_compare_play/pause_playback 'data: true'
```

첫 줄은 재생, 둘째 줄은 일시정지다. `roslaunch`에서는 터미널 Space 대신 이 서비스를 사용한다. 시작부터 재생하려면 `paused:=false`로 실행한다. 속도는 `rate:=0.5`가 절반, `rate:=1.0`이 원래 속도다. 비교 실행과 아래의 수동 `rosbag play`를 동시에 켜지 않는다.

`start:=30`으로 30초 지점부터 재생하고, `loop:=true`로 반복할 수 있다. 반복으로 시간이
되돌아가면 이전 GPS 궤적을 지운다. `max_points:=20000`은 보관할 GPS 점 개수이며,
재생이 끝나도 마지막 비교 화면은 유지된다. `Ctrl+C`로 비교 launch를 종료한다.

`route_file`에는 편집기에서 내보낸 개별 CSV도 지정할 수 있다. JSON은 프로젝트에 저장된
원점을 사용하고, CSV는 위 용인 원점을 기본으로 사용한다. 다른 원점의 CSV라면
`origin_latitude`와 `origin_longitude`를 함께 지정한다. 헤더 없는 UTM RDDF TXT는 이
도구의 입력 형식이 아니다. 경로 이름을 보려면 RViz의 Manual Routes → Namespaces에서
`manual_labels`를 켠다.

GPS 품질은 위치와 같은 `NavPVT` 메시지의 fix·carrier flags로 구분한다. 이 기록의 `/gps/status`에는 위치 메시지와 약 0.2~0.4초 차이가 있는 경우가 있어, 비교 도구는 이 상태를 별도 메시지와 시간으로 맞춰 붙이지 않는다.

FIX는 초록, FLOAT는 주황, DGNSS는 파랑이다. 유효한 일반 GPS·2D 위치도 표시하며,
유효한 위치가 없는 메시지는 상태만 갱신한다. 점 간격이 크게 벌어지거나 수신이 끊기면
GPS 선을 끊어서 표시한다.

## 3. bag 내용과 원시 센서 확인

```bash
rosbag info /media/stier/Data/Ubuntu/rosbag_0906/1.bag
```

확인한 원본 기록의 예시는 다음과 같다.

| bag | 길이 | GPS 품질·센서 |
|---|---:|---|
| `1.bag` | 약 112.8초 | GPS 862점: RTK fixed 844, DGNSS 18. LiDAR·카메라 있음, IMU 없음 |
| `2.bag` | 약 208.7초 | GPS 1,592점: fixed 1,068, float 107, DGNSS 417. 품질 변화 비교용 |
| `sensors_20260906_132019.bag` | 약 16.8초 | GPS·LiDAR·카메라·IMU가 함께 있는 짧은 기록 |

비교 launch를 종료한 뒤, 원시 토픽만 재생하려면 다음을 사용한다.

```bash
rosparam set /use_sim_time true
rosbag play --clock --pause -r 0.5 /media/stier/Data/Ubuntu/rosbag_0906/1.bag --topics \
  /scan /tf_static \
  /ublox_position_receiver/fix /ublox_position_receiver/navpvt \
  /gps/status /usb_cam/image_raw /usb_cam/camera_info
```

직접 실행한 `rosbag play` 터미널에서 **Space**는 재생·일시정지, **s**는 일시정지 중 다음 메시지로 이동이다. `Ctrl+C`로 종료한다. 다시 실행할 때 `-r 1`로 원래 속도를 지정할 수 있고, `--start=30`을 추가하면 30초 지점에서 시작한다.

다른 터미널에서 토픽과 GPS 원문을 확인한다. `echo`와 `hz`는 재생 중에 출력된다.

```bash
rostopic list
rostopic type /scan
rostopic hz /scan
rostopic echo -n 1 /ublox_position_receiver/fix
rostopic echo -n 1 /ublox_position_receiver/navpvt
rostopic echo -n 1 /gps/status
```

`NavSatFix`의 `status`만으로 RTK fixed와 float를 구분할 수는 없다. `/gps/status`의 `status_text`, `carrier_solution` 또는 `NavPVT.flags`를 확인한다. `NavPVT.lat/lon`은 값에 `1e-7`을 곱하면 도 단위, `hAcc`는 `0.001`을 곱하면 미터 단위다.

IMU도 보려면 위 bag 경로를 `sensors_20260906_132019.bag`으로 바꾸고, 재생할 토픽 목록에 `/imu/data`를 추가한다.

```bash
rostopic type /imu/data
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
```

### LiDAR 창

```bash
rosrun rviz rviz -f laser
```

RViz의 **Fixed Frame**을 `laser`로 두고, **Add → LaserScan**을 추가한 다음 **Topic**을 `/scan`으로 선택한다. 기록된 센서는 `PointCloud2`가 아닌 2D `sensor_msgs/LaserScan`이며 약 10Hz, 한 바퀴 약 3,585개 거리값을 보낸다.

### 카메라 창

```bash
rosrun image_view image_view image:=/usb_cam/image_raw
```

또는 `rosrun rqt_image_view rqt_image_view`를 실행하고 `/usb_cam/image_raw`를 선택한다. 확인한 원시 영상은 `640×360`, `rgb8`, frame `camera_link`다. 원본 bag에는 JPEG `/usb_cam/image_raw/compressed`도 있지만, 위 재생 명령은 raw 영상만 골라 재생한다.

## 4. 좌표 차이를 해석할 때

- 기존 `gps_bag_rviz_replay.launch`는 첫 GPS 위치를 `(0,0)`으로 잡는다. 원점이 지정된 수동 경로와 비교할 때는 위 `bag_route_compare.launch`를 사용한다.
- `hAcc`는 수신기가 추정한 위치 정확도다. 위성·항공사진이나 차선 중심에 대한 실제 오차 측정값은 아니다. GPS 위치도 안테나 위치이므로 그 궤적이 반드시 차선 중심과 일치하는 것은 아니다.
- 확인한 28개 원본 bag에는 움직이는 차량의 `/tf`가 없다. `/tf_static`은 `base_link → laser` 한 개이며, 기록값은 이동 0과 yaw 180°다. LiDAR를 지도 위에 누적하려면 같은 시각의 차량 위치·방향과 실제 센서 장착 위치가 추가로 필요하다.
- `/utm`은 절대 UTM 좌표인데 header는 `gps_link`이고 quaternion은 모두 0이다. 이를 그대로 RViz의 유효한 차량 pose로 사용하지 않는다.

원본 bag·다운로드한 경로 JSON·CSV는 로컬 파일로 유지한다. 재생이나 비교는 해당 파일을 변경하지 않는다.

## 확인한 동작

2026-09-14 이 PC의 ROS Noetic에서 전체 catkin 빌드와 localization 패키지 테스트,
비교 도구의 코어 테스트 30개를 통과했다. 실제 `1.bag` 재생에서 수동 경로 19개와
GPS·LiDAR·카메라 토픽, 서비스에 의한 일시정지를 확인했다. 마지막 구간을 반복 재생해
시간 되감기마다 GPS 궤적이 초기화되고, 지정한 보관 점 개수를 넘지 않는 것도 확인했다.
