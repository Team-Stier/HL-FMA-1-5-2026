# Path Planner

**기존 로컬 Frenet의 ROS 비의존 C++14 코어만 추린 통합용 라이브러리**다.
공개 namespace와 라이브러리 target 이름은 모두 `path_planner`다.
ROS 노드, launch, selector, localization, 센서, 검출기, 제어기, RRT 비교 백엔드는 포함하지 않는다.
차량 실측 설정·RDDF/rosbag·개인 경로도 포함하지 않는다. 테스트 치수/장애물은 합성 조건이다.
`catkin_make`가 자동 시작하는 패키지가 아니며 통합자가 자신의 빌드에서 연결한다.

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

현재 ROS wrapper는 포함하지 않는다. 통합자는 자신의 노드에서 동일 의미를 구현해야 한다.

## 장애물 구간 전용 / 전체 전진 구간

두 방식에서 **코어는 동일**하다. 미션별 호출 여부는 State Manager가 `path_mode`로 결정하고,
Selector는 요청된 경로가 맞는지 검증한다.
좋은 RDDF를 유지하면서 전체 전진 구간에서 호출하면 장애물이 없을 때 기준선 추종 후보를 고른다.
장애물 구간에서만 사용할 때는 진입 준비·s/방향·위치 연속성·복귀 확인을 전환 조건으로 둔다.
GPS 좌표 하나를 경계로 토글하거나 실패 시 막힌 일반 경로로 돌아가는 정책은 구현하지 않는다.

코어를 사용하지 않는 구간의 연석/장애물 감시까지 없어져도 된다는 뜻은 아니다. 전체 적용 시에는
전체 RDDF·분기·끝점·경계의 후보 유효성과 계산 시간을 검증해야 한다. `speed_limit_kph`와
`mission_zone`은 읽는 메타데이터일 뿐 속도 제어나 ON/OFF 명령이 아니다. 후진 주차는 지원하지 않는다.

## 차선 선택과의 관계

이 코어의 목표는 **현재 기준선 d=0 복귀**다. 연석 기반 차선 선택 코어의 `(s, target_d)`를
자동 소비하지 않는다. 그 기능은 별도 라이브러리이며, 새 목표 기준선/목표 d profile 연결은
통합 후속 작업이다. 탐색 범위 ±1.50 m를 단순히 키워서 차선 변경을 완료했다고 판단하지 않는다.

## 빌드·시험

`src/path_planner` 폴더에서:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
(cd build && ctest --output-on-failure -V)
```

시험은 ROS 설치/GTest 다운로드 없이 빈 도로, 라바콘 회피, 전폭 차단·reset, 잘못된 치수/방향,
동일 s 비교, 위치/변환 점프의 여섯 시나리오 그룹을 검사한다. 현장 주행/제동 검증이 아니다.
더 넓은 기존 회귀시험은 원본 workspace에 보존하며 이 전달본에는 필요한 최소 코어 시험만 넣는다.
