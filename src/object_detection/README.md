# object_detection

`/scan` (`LaserScan`)을 기록된 TF로 **base_link에 먼저 변환**한 뒤,
**base_link의 Y만 반전**하여 `/lidar_preprocessed` (`PointCloud2`)로 발행한다.
X와 Z는 유지하며 출력 `header.frame_id`는 `base_link`다.

```text
p_base = R_base_scan × p_scan + t_base_scan
p_output = (p_base.x, -p_base.y, p_base.z)
```

scan 시각의 TF를 사용한다. TF가 없으면 경고를 출력하고 해당 scan을 건너뛴다.
범위 밖의 거리·NaN·Inf는 제외하고, 대응하는 intensity와 원본 timestamp는 보존한다.
scan 중 차량 움직임에 대한 왜곡 보정은 하지 않는다.

## 실행

각 터미널에서 워크스페이스를 source한다. 기본 ROS master를 사용한다.
이전에 별도 포트를 설정한 터미널이면 `unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME`으로 해제한다.

```bash
source ~/HL-FMA2026-suhyeon/devel/setup.bash
```

터미널 1 — 전처리만 실행 (RViz는 자동 실행하지 않음):

```bash
roslaunch object_detection lidar_preprocessor.launch
```

터미널 2 — bag의 90초부터 반복 재생. 이미 bag이 재생 중이면 추가로 실행하지 않는다.

```bash
rosparam set /use_sim_time true
rosbag play --clock -l -s 90 /media/stier/Data/Ubuntu/rosbag_0906/2.bag
```

`/scan`만 골라 재생하면 TF가 없어 변환할 수 없다. 토픽을 제한할 때는 `/tf_static`과
기록에 포함된 `/tf`도 함께 재생한다.

기존 RViz에서 **Fixed Frame: base_link**, **PointCloud2 Topic: /lidar_preprocessed**로 설정한다.
같은 base_link 화면에 원본 `/scan`을 추가하면 좌우 반전을 비교할 수 있다.

```bash
rostopic hz /lidar_preprocessed
rostopic echo -n 1 /lidar_preprocessed/header
```

입출력 토픽은 `input_topic:=/scan output_topic:=/lidar_preprocessed`로 바꿀 수 있다.

## DBSCAN 시각화

현재 단계에서는 ROI와 객체 메시지 없이 `/lidar_preprocessed`의 모든 XY 점을 DBSCAN으로
군집화한다. `2.bag`의 angle-compensated scan을 기준으로 기본값은 `eps=0.1 m`,
`min_samples=4`다. 파라미터는 `config/dbscan.yaml`에서 수정하며 RViz는 자동 실행하지 않는다.

```bash
roslaunch object_detection dbscan_visualizer.launch
```

RViz에서 Fixed Frame을 `base_link`로 놓고 **MarkerArray** display에
`/dbscan_clusters`를 지정한다. 군집마다 다른 색이고 noise는 회색이다.

설정 파일의 `eps`, `min_samples`, `point_size`를 저장하고 launch를 다시 실행하면 반영된다.
다른 설정 파일을 시험할 때는 `config_file:=/절대/경로/dbscan.yaml`로 지정할 수 있다.

## 빌드·검사

```bash
cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
catkin_make run_tests_object_detection
catkin_test_results build/test_results/object_detection
```
