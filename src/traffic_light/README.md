# traffic_light

`traffic_light_yolo`에서 학습한 YOLO 7클래스 모델을 실차 USB 카메라 영상에 적용하는
ROS Noetic 패키지다. 학습 데이터셋, 학습 영상, Ultralytics 실행 결과와 가상환경은 포함하지
않으며 실차 추론에 필요한 노드, 설정과 최종 `best.pt`만 저장한다.

## 포함 모델

- 파일: `models/traffic_light_7class_best.pt`
- 원본: `seventh_7class_aug/weights/best.pt`
- 크기: 5,464,531 bytes
- SHA-256: `caaf2ab7a3f1aa6500a9907ab1a2044119e3051857d990a54ee0097603e6966f`
- 클래스: `red`, `yellow`, `green`, `left_arrow`, `speed_20`, `down_arrow`, `x_sign`

일반 신호 중 confidence가 가장 높은 검출을 `RED`, `YELLOW`, `GREEN`, `LEFT_ARROW`로
변환한다. 현재 차량은 카메라 기반 차로 제어를 사용하지 않으므로 `speed_20`,
`down_arrow`, `x_sign`은 검출 결과에서 무시한다. 검출이 없거나 추론이 실패하면 항상
`UNKNOWN`을 발행한다.

## 입출력

| 구분 | 토픽 | 타입 |
|---|---|---|
| 입력 | `/usb_cam/image_raw` | `sensor_msgs/Image` |
| 입력 | `/mission/state` | `planning_interfaces/MissionState` |
| 출력 | `/perception/traffic_signal` | `planning_interfaces/SignalObservation` |

출력 header에는 카메라 원본 측정 시각과 frame을 그대로 보존한다. 일반 신호 출력의
`route_name`은 최신 `/mission/state`에서 가져와 다른 구간의 과거 신호가 재사용되지 않게
한다. State Manager 없이 단독 시험할 때만 launch의 `route_name` 인자로 구간을 지정한다.

## 설치와 빌드

GPU를 쓸 경우 PC의 CUDA 환경과 맞는 PyTorch를 먼저 설치한다. 그 다음 학습에 사용한
Ultralytics 버전을 설치한다.

```bash
cd ~/HL-FMA2026-stier
python3 -m pip install -r src/traffic_light/requirements.txt
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make --pkg planning_interfaces traffic_light
source devel/setup.bash
```

## 실차 실행

먼저 카메라를 실행하고 영상이 들어오는지 확인한다.

```bash
roslaunch cam_bringup cam.launch
rostopic hz /usb_cam/image_raw
```

다른 터미널에서 검출 노드를 실행한다. `device:=0`은 첫 CUDA GPU, `device:=cpu`는 CPU를
강제한다. 기본 `auto`는 Ultralytics의 자동 선택을 사용한다.

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch traffic_light traffic_light.launch device:=0
```

State Manager를 실행하지 않은 단독 시험에서는 현재 촬영 중인 구간을 명시한다.

```bash
roslaunch traffic_light traffic_light.launch device:=0 route_name:=2
rostopic echo /perception/traffic_signal
```

실제 주행 허가는 State Manager의 별도 confidence/freshness/route 검사를 통과해야 한다.
현장에서는 카메라 장착 상태, 노출, 신호등 크기, 주야간 조건과 GPU 처리 지연을 반드시
측정하고 `config/detector.yaml`의 추론 threshold를 검증한다.
