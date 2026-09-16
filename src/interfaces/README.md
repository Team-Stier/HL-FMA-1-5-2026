# Interfaces

노드 사이에서 공유하는 프로젝트 전용 ROS 메시지를 모아 둔 디렉터리다. 이 아래 패키지는
메시지만 제공하며 실행 노드를 포함하지 않는다.

```text
interfaces/
├── perception_interfaces/
│   └── ObjectInfo.msg, TLLabel.msg
├── planning_interfaces/
│   └── 미션·경로·인지 관측 메시지 및 위치 지정 서비스
├── sensor_interfaces/
│   └── GpsStatus.msg
└── vehicle_interface/
    ├── README.md
    └── erp42_msgs/
```

## 패키지 역할

- `perception_interfaces`: 장애물과 신호등 인식 결과
- `sensor_interfaces`: localization 등에서 공통으로 사용하는 센서 상태
- `planning_interfaces`: RDDF/미션/계획 경로/안전 상태와 미션 인지 관측. [계약과 토픽](../state_manager/README.md) 참고
- `erp42_msgs`: 상위 제어기와 Arduino 하위제어기 사이의 명령 및 피드백

## 빌드와 확인

```bash
cd ~/HL-FMA2026-stier
source /opt/ros/noetic/setup.bash
catkin_make --pkg perception_interfaces sensor_interfaces planning_interfaces erp42_msgs
source devel/setup.bash

rosmsg show perception_interfaces/ObjectInfo
rosmsg show perception_interfaces/TLLabel
rosmsg show sensor_interfaces/GpsStatus
rosmsg show planning_interfaces/MissionState
rosmsg show planning_interfaces/PlannedPath
rosmsg show erp42_msgs/DriveCmd
```

다른 Catkin 패키지에서 메시지를 사용하려면 해당 `package.xml`에 메시지 패키지를
`<depend>`로 추가하고, `CMakeLists.txt`의 `find_package(catkin REQUIRED COMPONENTS ...)`와
`catkin_package(CATKIN_DEPENDS ...)`에도 같은 이름을 추가한다.

## GPS 상태 메시지

`sensor_interfaces/GpsStatus`는 `/gps/status`에 사용한다. Localization은 위치 좌표를
`/ublox_position_receiver/fix` 또는 `/utm`에서 받고, 다음 필드로 GNSS 입력의 신뢰도를
판단한다.

- `fix_ok`: 수신기의 DOP/accuracy 조건을 통과한 위치인지 여부
- `solution`: No Fix, 2D, 3D, DGNSS, RTK Float, RTK Fixed 등의 enum
- `carrier_solution`: carrier phase 없음, Float 또는 Fixed
- `spoofing_state`: spoofing detector 상태
- `satellites_used`: 위치 계산에 사용한 위성 수
- `horizontal_accuracy_m`, `vertical_accuracy_m`: 수신기 예상 위치 정확도
- `position_dop`: PDOP
- `speed_accuracy_mps`, `heading_accuracy_deg`: 속도 및 진행 방향 예상 정확도

메시지 정의와 상태 값은 [GPS README](../sensor_drivers/gps/README.md)를 참고한다.

## 메시지 변경 시 주의사항

메시지를 변경하면 워크스페이스를 다시 빌드하고 해당 메시지를 사용하는 모든 노드를
재빌드한다.

```bash
catkin_make
source devel/setup.bash
```

`erp42_msgs`를 변경한 경우 PC 쪽 빌드만으로는 충분하지 않다. Arduino용 `ros_lib`도 다시
생성하고 펌웨어를 재컴파일해야 한다. 자세한 절차는
[vehicle interface README](vehicle_interface/README.md)를 참고한다.
