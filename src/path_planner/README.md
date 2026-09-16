# Path Planner

로컬 Frenet C++14 코어와 ROS Noetic wrapper를 함께 제공한다. 공개 C++ namespace는
`path_planner`, 라이브러리 target은 `path_planner_core`, 실행 노드는
`path_planner_node`다. 차량 실측값과 rosbag은 포함하지 않으며 테스트 치수/장애물은
합성 조건이다.

## 포함된 알고리즘

- 기준선 CSV/waypoint 검사·보간과 제한된 s 범위의 차량/장애물 투영.
- 여러 회피 시작 시점 및 횡방향 조합의 5차 다항식 후보 생성.
- 회전된 실제 차체 사각형, 점 사이 보간(swept) 충돌·좌우 경계 검사.
- 곡률/곡률 변화율, 전진 방향, Frenet 특이점 검사.
- 동일 s 위치의 이전 경로와 비교하는 비용 및 기준선 복귀.
- pose·map/odom 불연속 감지와 명시적 `reset()` 함수.

이번 분리는 기존 여섯 C++ 코어와 여덟 header를 그대로 복사한다. 파일별 SHA-256은
배포 스냅샷의 `handoff_manifest.json`에 기록한다. ROS wrapper를 복사하지 않았다는 이유로
wrapper가 하던 안전 검사가 자동으로 이 라이브러리에 들어온 것은 아니다.

## 호출 계약

1. `ReferencePath::initialize(waypoints, &error)` 또는 `loadCsv(...)` 성공을 확인한다.
   각 점의 x/y와 차량 위치는 **같은 고정 좌표계·datum**이어야 한다.
   좌우 bound는 해당 기준선부터 검증된 주행 가능 경계까지 거리다. 차체 반폭을 미리 빼지 않는다.
   RDDF의 east/north만 rename하고 임의 도로 폭을 채워서는 안 된다.
2. `FrenetPlannerConfig.vehicle`에 실측 길이·폭·축거·후륜축→몸체 중심 거리를 넣고
   `isValid(config, &reason)`를 확인한다. 기본 차량 치수는 0이므로 명시적으로 설정해야 한다.
3. 실제 후륜축 pose를 진행도 주변의 제한된 s 창에서 `projectPose`한다. 초기 route/branch
   선택과 경로 전환은 외부에서 결정한다. 가까운 평행/복귀 구간의 전역 최근접점을 쓰지 않는다.
4. 유효한 관측 polygon을 `PlannerInput.obstacles`에 모두 넣는다. bbox 꼭짓점은 finite이며
   둘레 순서의 비퇴화 convex 사각형이어야 한다. `projectObstaclesToFrenet`은 같은 장애물의
   비용용 s/d 표현을 만든다. 투영 범위 밖 장애물을 원래 충돌 검사 벡터에서 제거하지 않는다.
5. `FrenetPlanner::plan(reference, input)`을 호출한다. 유효 결과도 아래 발행 직전 검사를 통과해야 한다.
   실패하면 빈 결과를 소비하고 `reset()`한다. 마지막 성공 경로를 fallback 주행 경로로 쓰지 않는다.

공개 구조체는 저수준 수치 API다. 호출자는 finite/range/frame/stamp/크기/센서 유효성 검사를
해야 한다. ROS wrapper에서 수행하던 ObjectInfo 음수 count, 원래 stamp·receipt timeout,
치수 교정 flag, TF 시각/유효성, 계산 deadline 검사가 코어에 자동 포함돼 있지는 않다.

## 통합자가 보존해야 하는 안전 동작

- 유효성 heartbeat와 pose/관측의 원래 시각 및 monotonic 수신 시각을 함께 검사한다.
  stale/무효 입력에서는 계획하지 않고 정지 요청을 전달한다. 빈 장애물 배열은 유효 관측일 때만 허용한다.
- `PoseJumpGuard` 호출 전 숫자·시각을 검사한다. 점프/경로·datum 변경 시 planner.reset(),
  과거 map/odom 장애물 폐기, 점프 이후 **새 측정** 대기를 수행한다. guard 하나가 캐시를 지우지는 않는다.
- 측정 시각의 장애물을 연속적인 odom에 보관한다. 최종 후보를 **최신 유효 map→odom 변환의
  역변환**으로 odom에 옮기고 `pathHasCollision`으로 최신 관측과 다시 검사한다.
  이 library는 TF를 조회하지 않는다. map 충돌 검사와 이 재검사를 혼동하지 않는다.
- 최종 `sweptPathWithinReferenceBounds`와 시간 초과/출력 finite 검사를 유지한다.
  계산 deadline은 외부에서 측정해 초과 결과를 버린다. 한 번의 `plan`이 스스로 wall-clock deadline에 중단되지는 않는다.
- `valid=false`/빈 경로가 실제 제어기 정지로 이어지는지, 경로 소비자 timeout 및 MCU watchdog/E-stop은 별도 검증한다.
- GPS가 천천히 치우치거나 연석이 가려지면 점프 검사/관측만으로 해결되지 않는다.
  측량·위치·추종·치수 오차를 고려한 주행 가능 경계와 여유거리 관리가 필요하다.

ROS wrapper는 `/route/map`, `/mission/state`, Localization Odometry와 stamped
`/dbscan_clusters`를 검사하고 LOCAL 요청에서만 `/path/local`을 발행한다. 입력이 stale이거나
frame·decision·방향이 다르면 마지막 성공 경로를 재사용하지 않는다. 상태는
`/path_planner/status`로 발행한다.

## 장애물 구간 전용 / 전체 전진 구간

두 방식에서 **코어는 동일**하다. 미션별 호출 여부는 State Manager가 `path_mode`로 결정하고,
Selector는 요청된 경로가 맞는지 검증한다.
좋은 RDDF를 유지하면서 전체 전진 구간에서 호출하면 장애물이 없을 때 기준선 추종 후보를 고른다.
장애물 구간에서만 사용할 때는 진입 준비·s/방향·위치 연속성·복귀 확인을 전환 조건으로 둔다.
GPS 좌표 하나를 경계로 토글하거나 실패 시 막힌 일반 경로로 돌아가는 정책은 구현하지 않는다.

코어를 사용하지 않는 구간의 연석/장애물 감시까지 없어져도 된다는 뜻은 아니다. 전체 적용 시에는
전체 RDDF·분기·끝점·경계의 후보 유효성과 계산 시간을 검증해야 한다. `speed_limit_kph`와
`mission_zone`은 읽는 메타데이터일 뿐 속도 제어나 ON/OFF 명령이 아니다. 후진 주차는 지원하지 않는다.

## 빌드·시험

워크스페이스 루트에서:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg planning_interfaces path_planner
source devel/setup.bash
roslaunch path_planner path_planner.launch
```

기본 설정은 `calibration_required: true`라 경로를 발행하지 않는다. 실측 차량 치수와
RDDF 좌우 주행 가능 경계를 `config/path_planner.yaml`에 넣고 검증한 뒤
`roslaunch path_planner path_planner.launch calibration_required:=false`로 실행한다.

### Localization/RDDF 없이 운동장에서 Frenet 실차 시험

`frenet_field_test.launch`는 Localization, State Manager, RDDF tracker, Selector를 실행하지
않는다. 좌표계는 차량에 붙은 `base_link`이고 테스트 어댑터가 직선 기준선과 원점 pose만
형식에 맞게 공급한다. 장애물은 가상 데이터가 아니라 실제 LiDAR를 RDDF corridor 없이
DBSCAN한 `/dbscan_clusters`만 사용한다. Frenet 결과의 `Path`를 PP 입력으로 변환하고
Arduino rosserial까지 연결한다. 생산 코어와 생산 설정은 변경하지 않으며 모든 전용 코드는
`path_planner/test`에 있다.

테스트의 `base_link → laser_link`는 뒷차축 기준 X=`1.05 m`다. wheelbase `0.75 m`와
앞차축보다 0.30 m 전방인 장착 위치를 합한 값이다. RPLIDAR raw scan 전방을 맞추는 기존
yaw 180°를 유지하고, 물리 장착 방향인 전방 X축 기준 roll 180°를 추가한다.
RViz의 TF 축과 LaserScan은 이 변환을 자동 적용한다. 파란 직선은 Planner 성공 여부와
무관하게 항상 보이는 테스트 기준선이고, 노란 선은 실제 Frenet 성공 출력이다.
흰색 외곽선은 planner가 사용하는 차체, 청록색 선은 후륜축(`base_link`), 초록색 선은
앞차축(X=`0.75 m`)이다. `laser_link` 축은 앞차축보다 정확히 0.30 m 앞에 보여야 한다.

현장 테스트의 차체 `1.40 × 0.775 m`, 후륜축→중심 `0.38 m`는 기존 Frenet 코어 시험용
값이다. 동작 확인용이며 대회 장애물 여유거리로 사용하기 전에는 실측값으로 교체해야 한다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch path_planner frenet_field_test.launch \
  arduino_port:=/dev/ttyACM0
```

시작 직후에는 `DISARMED`라 Control이 `KPH=0, brake=1`을 보낸다. RViz에서 실제 LiDAR,
청록 ROI point, DBSCAN cluster, 회색 직선 기준선, 노란 Frenet 경로와 초록 PP 입력을
확인한다. Arduino feedback의 `MorA=1`까지 확인한 뒤에만 주행을 켠다. 시험 속도는 전용
Control 설정에서 `5 km/h`다.

```bash
rostopic echo /erp42_serial/feedback
rostopic echo /path_planner/status
rosservice call /frenet_test/run "data: true"
```

정지는 `rosservice call /frenet_test/run "data: false"`, 별도 비상정지는
`rostopic pub -1 /vehicle/emergency_stop std_msgs/Bool "data: true"`다. LiDAR나 Arduino를
이미 별도로 실행 중이면 각각 `start_lidar:=false`, `start_arduino:=false`를 준다.

시험은 ROS 설치/GTest 다운로드 없이 빈 도로, 라바콘 회피, 전폭 차단·reset, 잘못된 치수/방향,
동일 s 비교, 위치/변환 점프의 여섯 시나리오 그룹을 검사한다. 현장 주행/제동 검증이 아니다.
더 넓은 기존 회귀시험은 원본 workspace에 보존하며 이 전달본에는 필요한 최소 코어 시험만 넣는다.
