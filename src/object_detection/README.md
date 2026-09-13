# object_detection

현재 단계는 라이다 전처리: `/scan` (`sensor_msgs/LaserScan`)을 받아 X축 기준
180° 회전한 `/lidar_preprocessed` (`sensor_msgs/PointCloud2`)를 발행한다.
노드 이름은 `lidar_preprocessor`다.

`x = r cos(θ), y = -r sin(θ), z = 0`으로 계산한다. 전후는 유지하고 좌우가 뒤집힌다.
유효 범위 밖의 거리와 NaN/Inf는 제외하며, 대응하는 intensity가 있으면 함께 보존한다.
출력 timestamp는 원본 scan의 첫 빔 시각을 유지한다. 움직임에 의한 scan 왜곡 보정은 하지 않는다.

## 실행

각 터미널에서 먼저:

```bash
source ~/HL-FMA2026-suhyeon/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
```

터미널 1 — 전처리와 RViz (master도 자동 시작):

```bash
roslaunch -p 11411 object_detection lidar_preprocessor.launch rviz:=true
```

터미널 2 — bag의 90초부터 반복 재생:

```bash
rosparam set /use_sim_time true
rosbag play --clock -l -s 90 /media/stier/Data/Ubuntu/rosbag_0906/2.bag
```

터미널 3 — 출력 확인:

```bash
rostopic hz /lidar_preprocessed
rostopic echo -n 1 /lidar_preprocessed/header
```

RViz는 Fixed Frame `lidar_preprocessed`, PointCloud2 Topic `/lidar_preprocessed`로 본다.
출력은 센서 원점을 기준으로 X축 회전만 적용한 별도 좌표계다. 차량 `base_link`로
이동·정렬하거나 TF를 발행하지 않는다. 기존 라이다 장착 설정의 yaw 180°와 이번
roll 180°는 다른 회전이며, 실제 장착 TF는 별도로 맞춰야 한다.

입출력 토픽은 `input_topic:=/scan output_topic:=/lidar_preprocessed`로 바꿀 수 있다.

## 빌드·검사

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
catkin_make run_tests_object_detection
catkin_test_results build/test_results/object_detection
```

좌표 기준: [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst),
[LaserScan 정의](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/LaserScan.msg).
