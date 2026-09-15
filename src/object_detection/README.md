# object_detection

팀 Localization 코드를 수정하지 않고 현재 RDDF와 연결된 다음 RDDF 주변의 2D LiDAR만
DBSCAN으로 군집화한다. 원본 `/molit/sensors/lidar/scan`을 scan 시각의 팀 TF로 `map`에
변환하므로 별도의 좌우 반전 전처리를 사용하지 않는다.

## RDDF ROI

ROI는 현재 차량 위치의 뒤 2 m부터 진행 방향 앞 30 m까지 사용한다. 그 범위에 포함되는
각 RDDF 점을 중심으로 **반경 2 m 원**을 만들고, 모든 원의 합집합 안에 있는 LiDAR 점만
DBSCAN 입력으로 사용한다. 현재 RDDF 끝에 가까워지면 연결된 다음 번호 RDDF까지 미리
이어 붙인다. 좌우 경로가 갈리는 지점에서는 아직 경로가 확정되지 않은 두 후보를 모두
포함해 전환 순간에 ROI가 끊기지 않게 한다.

`config/rddf_roi.yaml`에서 `corridor_enabled: false`로 바꾸면 RDDF 매칭과 원형 ROI를
사용하지 않는다. 이 모드에서는 LiDAR의 차량 전방 전체에 차체 제거와 DBSCAN을 적용한다.
전방 기준은 아래 `lidar_forward_roi` 설정을 그대로 사용한다.

LiDAR 점은 팀 TF로 `base_link`에 변환한 뒤, LiDAR 장착 위치보다 차량 전방(+X)에 있는
점만 사용한다. `laser_link`가 yaw 180도로 장착되어도 차량 전방을 올바르게 선택한다.
LiDAR 원점 앞에서 제외할 거리와 사용 여부는 `config/rddf_roi.yaml`의
`lidar_forward_roi`에서 바꿀 수 있다. 필요하면 같은 파일의 `angular_roi`도 추가로 켤 수
있으며 기본값은 꺼져 있다.

차량 자체 반사는 DBSCAN 전에 `base_link` 기준 직사각형 영역으로 제거한다. 복사한 뷰어에는
실제 제거 영역이 반투명 빨간 면과 외곽선으로 표시된다. 팀 뷰어의 기존 차량 외곽 및 실제
차체를 보면서 `config/rddf_roi.yaml`의 `self_filter` 값을 조절한다.

RDDF 원 판정과 DBSCAN 이웃 검색은 SciPy가 설치되어 있으면 KD-tree를 사용한다. ROI를
통과한 점은 기본 5 cm XY voxel로 줄인 뒤 DBSCAN에 넣는다. `voxel_size_m: 0`이면
다운샘플링을 끌 수 있다. RViz 표시 구성과 토픽 이름은 바뀌지 않는다.

파라미터와 한글 튜닝 설명은 `config/rddf_roi.yaml`과 `config/dbscan.yaml`에 있다.
기본 출력은 다음과 같다.

- `/object_detection/roi_markers`: RDDF 중심선과 반경 2 m 원
- `/object_detection/roi_points`: ROI 안에 남은 LiDAR 점
- `/dbscan_clusters`: ROI 점을 DBSCAN으로 군집화한 결과와 큰 장애물 중점

## rosbag으로 실행

터미널 1에서 팀 Localization을 실행한다. 팀 뷰어는 끄고 bag을 반복 재생한다.

```bash
cd ~/HL-FMA2026-suhyeon
source devel/setup.bash
roslaunch mando_localization replay.launch \
  bag:=/home/stier/bag/20260906_123929/2026-09-06_12-39-31__00h08m31.193s.bag \
  loop:=true start_rviz:=false
```

터미널 2에서 ROI, DBSCAN, 복사한 뷰어를 함께 실행한다.

```bash
cd ~/HL-FMA2026-suhyeon
source devel/setup.bash
roslaunch object_detection rddf_roi_detection.launch
```

복사한 뷰어에는 팀 뷰어의 표시와 함께 청록색 ROI, ROI 내부 포인트, 클러스터가 추가된다.
뷰어 없이 노드와 토픽만 확인하려면 `start_viewer:=false`를 붙인다.

```bash
rostopic hz /object_detection/roi_points
rostopic echo -n 1 /object_detection/roi_points/header
rostopic hz /dbscan_clusters
```

## 빌드·검사

```bash
sudo apt update
sudo apt install -y python3-scipy

cd ~/HL-FMA2026-suhyeon
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
catkin_make run_tests_object_detection
catkin_test_results build/test_results/object_detection
```
