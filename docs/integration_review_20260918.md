# 2026-09-18 통합 점검과 로컬 수정

학교 양쪽 차량 시험을 위한 추가 수정(평행주차 끝 미세 역방향 조각, Planner 계산 예산)과
12가지 실제 노드 주차 회귀 결과는 [최종 학교 시험 안내](school_test_20260918.md)를 참고한다.
아래 최초 점검의 시험 집계와 최종 재검증 집계는 별도로 기록한다.

최종 재검증: 전체 workspace catkin XML 집계 **393 tests / 0 errors / 0 failures**,
격리 검증 스크립트도 전체 통과. 최종 로그는 `/tmp/stier-finalreview-all-tests-final.log`,
`/tmp/stier-finalreview-isolated.log`이며 원래 367 집계 이후 추가된 시험을 포함한다.

이후 사용자 요청으로 수행한 추가 교차 검토에서는 Frenet 실제 노드 연결 시험을 보강해
**매회 405 집계 / 오류·실패 0으로 전체 시험 5회 연속 통과**했다.
실행 스크립트 인자 처리 및 시험 fixture 수정의 근거는
[5차 검토 기록](five_pass_review_20260918.md)에 별도로 남겼다.

## 상태

- 기준: `Team-Stier/HL-FMA2026-0917`, main `62ee675f396a81bebbc98f5d4b0917dc27118ea3`.
- 검토 브랜치: `fix/integration-review-20260918`. 검토 당시 커밋·푸시·차량 구동·펌웨어 업로드 없음.
  이후 사용자 요청으로 같은 이름의 브랜치에 전체 시험 패키지를 공유한다.
  [배포본 안내](branch_test_quickstart.md)의 펌웨어는 이 저장소 안에 포함한다.
- 목표: RDDF 기본 추종, 3번 정적장애물 구간 Frenet, 기록된 RDDF로 T자/평행주차.
- **소프트웨어 회귀시험과 실차 성공은 다르다. 새 후진 펌웨어를 빌드/모의 검증했으나
  보드에 올리지 않았고, 물리적인 후진·조향·제동 성능은 아직 확인하지 않았다.**
- 학교 시험 launch 인자와 당시 bag/log는 제공되지 않았다. LD는 PP lookahead(m)이며,
  사용자가 보고한 증상은 LD 1에서 회피 추종 불량, 2에서 개선, 주차 시 조향/중립 반복이다.

## 확인 범위와 코스 이해

센서 bringup → Localization/현재 RDDF → State Manager → RDDF/Frenet 후보 →
Selector → PP → DriveCmd 흐름과 각 설정·메시지·시험을 점검했다.
LiDAR ROI/DBSCAN, 기어별 주차 경로 절단, 후속 RDDF 요청, 위치 매칭, 신호/정지 상태,
제어기 속도·조향 변환, 관련 launch도 포함한다. 모든 벤더 드라이버 내부나 신호등 모델
정확도를 검증했다는 뜻은 아니다.

RDDF는 용인/홍익/홍익 S 세 디렉터리 모두 19개 CSV이며 S 기준선은 107점, 약 40.765 m다.
용인 원점은 (37.288731, 127.1072336), 학교 원점은 (37.5511496, 126.9250846)이다.
평행주차 CSV 하나에 전·후진 leg가 함께 들어간다. 파일 전체의 `route_directions`만으로
주차 중 body heading이나 진행 구간을 정할 수 없다.

사용자 설명상 도로는 3.27 m × 2차로, 연석 높이 20 cm, base_link는 후륜축 중앙이다.
이 수치가 RDDF 각 점에서 좌우 연석까지 각각 3.27 m라는 뜻은 아니다.
기존 용인 bag은 뒤집힘 + 전방 기울어진 장착으로 기록됐으므로 수평 마운트의 새 실차와
같은 LiDAR 관측이라고 가정하지 않았다. 학교 문제의 재현 bag으로 사용하지도 않았다.
이전 연도의 최종맵 ZIP은 사용자 지시에 따라 현 대회 지도 근거에서 제외한다.

## 코드에서 확인하여 수정한 사항

| 영역 | 기존 문제 | 수정 |
|---|---|---|
| 후진 속도 | Localization은 signed speed를 수용하지만 Control은 speed < 0이면 정지 | 유한한 signed speed 수용, 횡제어 gain에는 절댓값 사용 |
| 후진 RViz | 추종 계산용으로 반전한 X를 그대로 base_link에 표시 | 발행 전에 X를 실제 차체 좌표로 복원 |
| PP 목표점 | 경로 전체에서 첫 원 교점을 선택하여 이미 지나간 prefix를 고를 수 있음 | 차량 최근접 투영점 이후 구간에서 탐색 |
| 평행주차 위치 매칭 | 되짚는 전/후진 구간이 서로 경쟁하고 CSV 전체 방향으로 yaw 판정 | Mission이 지정한 현재 leg의 s 범위와 방향으로 Localization이 투영 |
| S 끝 전환 | Planner는 남은 1 m 미만에서 무조건 실패, Localization의 끝 연결 반경은 기본 0.5 m | 장애물이 없는 짧은 RDDF 꼬리는 계속 발행; 회피 공간 부족은 그대로 실패 |
| Planner 실패 출력 | false status와 빈 RViz 경로만 발행, Selector는 이전 /path/local을 만료 때까지 사용 가능 | 동일 요청 ID의 빈 PlannedPath로 기존 후보 교체 |
| 공통 차량 치수 | /vehicle이 있어도 제어기/Planner는 개별 YAML 사용 | 공통 wheelbase/body geometry 적용, State Manager/self-filter도 연결 |
| 현장 설정 | LD base만 바꿔도 min=max=2이면 실제 LD는 2 | lookahead_m 실행 인자로 min/max 함께 지정 |
| 주차 분기 | JSON을 직접 바꿔야 함 | t_parking_side / parallel_parking_side 실행 인자 추가 |
| Localization 설정 회귀시험 | fe4ef2f의 YAML은 DR 2000초, 시험/설명은 200초 | 런타임 설정은 유지하고 시험/문서를 현재 설정과 일치시킴; 허용 오차 인증 아님 |

전체 시험에서 기존 테스트 계약 두 곳도 정리했다. Startup Gate의 2초 만료 시험은
실차 YAML(2000초)에 기대지 않고 **시험 launch만 2초로 지정**하여 원래의 만료 검사를
유지한다. RDDF 공개 토픽 시험은 취득 전 `TOO_FAR`와 취득 후 기존 `MATCHED_OFF_ROUTE`
구간 유지 정책을 구분한다. 위치가 매우 멀어도 현재 route identity를 유지하는 실제 정책은
이번에 변경하지 않았다. 이 matched 플래그가 경로 추종 가능성/위치 정확도 보증은 아니다.

불필요한 새 timeout·정지 gate·안전 잠금은 추가하지 않았다. 기존 E-Stop, 센서 freshness,
충돌 검사, 조향 한계, 실제 경로가 없는 경우의 정지는 유지한다. Planner 실패 후보를
비우는 변경은 기존 실패를 Selector에 전달하는 계약 수정이며, 실패를 성공으로 감추지 않는다.

### 평행주차 인터페이스 변경

`MissionState.parking_leg_start_s`를 추가했다. 기존 `parking_leg_target_s`와 함께 현재
진행할 CSV station 구간을 나타낸다. 이는 경로 선택 요청이며 위치 관측 자체는 아니다.
Localization은 현재 source route가 일치할 때만 이 구간을 사용한다. 기어 전환 정차/
새 경로 대기 중에도 새 leg를 받아야 하므로 `valid`에 의존하는 순환 대기는 만들지 않는다.

**관련 ROS 노드를 모두 재빌드하고 재시작해야 한다. 이전 MissionState 바이너리와 혼용 금지.**
이번 수정으로 `DriveCmd`/`SerialFeedBack` 정의는 바꾸지 않았다.

### 사용자 확인: 큰 군집 제거 유지

주행 중 LiDAR가 흔들려 바닥을 잡으므로 `max_cluster_extent_m: 1.5`를 넘는 군집은
통째로 제거하는 현재 정책이 필요하다고 사용자가 확인했다. 제안했던 군집 분할은 되돌렸다.
이번 통합은 RDDF 추종을 기본으로 하며 연석 검출/가상 차선 생성은 범위에서 제외한다.
작은 장애물·라바콘 검출과 기존 noise/extent 필터를 유지한다.

## 후진 펌웨어 조사와 별도 로컬 구현

아래는 최초 구현 위치에 대한 기록이다. 배포본에는 같은 소스를
`src/sensor_drivers/arduino/firmware/`에 복사했으며 원래 Mando 저장소는 수정·푸시하지 않는다.

Mando 최신 main `4753c4755b70494a3013f18524c7248f1ba15aa5` 및 조회 가능한 원격 브랜치에서
다음 실제 파일을 확인했다.

- [White Car RosBridge.h](https://github.com/Team-Stier/Mando/blob/4753c4755b70494a3013f18524c7248f1ba15aa5/arduino/BROON_T870_White_Car/RosBridge.h)
- [Black Car RosBridge.h](https://github.com/Team-Stier/Mando/blob/4753c4755b70494a3013f18524c7248f1ba15aa5/arduino/BROON_T870_Black_Car/RosBridge.h)
- [White Car sketch](https://github.com/Team-Stier/Mando/blob/4753c4755b70494a3013f18524c7248f1ba15aa5/arduino/BROON_T870_White_Car/BROON_T870_White_Car.ino)
- [Black Car sketch](https://github.com/Team-Stier/Mando/blob/4753c4755b70494a3013f18524c7248f1ba15aa5/arduino/BROON_T870_Black_Car/BROON_T870_Black_Car.ino)

두 RosBridge의 수신부는 KPH/Deg/brake만 보관하고 Gear/EStop을 읽지 않는다. ROS 속도
제어는 양수 targetSpeedKph로 동작하며 feedback의 MorA도 채우지 않는다.
RC 후진 지원이 ROS 후진 지원을 뜻하지 않는다.
White의 저장된 기본 build option은 ROS 꺼짐/사람용 serial 켜짐, Black은 그 반대다.
보드에 실제 업로드된 설정/코드는 저장소와 다를 수 있다.

통합 저장소 README의 “Arduino가 0속도 3회 확인 후 기어 전환한다”는 설명은 위 코드에서
확인되지 않았다. 사용자 요청에 따라 **별도 로컬 worktree**에 이를 구현했다.

- 폴더: `/home/choiminho/바탕화면/Mando-firmware-integration`
- 브랜치: `fix/ros-gear-integration-20260918`
- White/Black의 `.ino`, `RosBridge.h`, 신규 `RosDrivePolicy.h` 및 `arduino/tests`.
- 설명: 해당 worktree의 `arduino/ROS_GEAR_INTEGRATION.md`.
- 원래 `mando` 작업 파일, 원격 저장소, 실제 보드는 변경하지 않았다.
- generic `BROON_T870_Uno_Controller`는 이번 구현 대상이 아니다.

`KPH`는 unsigned 크기를 유지하고 `Gear=0/1/2`로 전진/중립/후진을 지정한다.
PI는 절댓값 속도를 사용하고 PWM에 방향을 적용한다. 새 encoder 0속도 관측 3회와
PWM 0을 확인한 뒤 방향을 수락한다. 일반 정지 중 확인한 샘플도 사용하며 동일 샘플을
루프마다 다시 세지 않는다. `EStop`은 별도 적용하고 `MorA/Gear`를 실제 상태에 맞춰 발행한다.

기존 일반 ROS brake가 arming 상태를 매번 초기화하여 다음 양수 명령으로 출발하지
못하는 결합도 수정했다. 일반 brake 동안 모든 출력을 억제하되 기존 stationary arming
dwell은 완료할 수 있게 했다. RC stop, 비상정지, timeout, 기존 fault는 유지한다.
PWM 0만으로 경사로 제동 유지 성능까지 보장하는 것은 아니다.

**통합 저장소의 erp42_msgs로 ros_lib를 새로 생성해야 한다.** Mando의 오래된
Localization_pkg 메시지가 아니다. DriveCmd payload 7 bytes,
MD5 `518982e31d00755722fd5fb8c3000c77`를 호스트 시험에서 확인한다.
White 저장 기본 옵션은 ROS가 꺼져 있으므로 실제 ROS 빌드 시 ROS=1/human serial=0이
필요하다. 차종별 핀/보정값과 저장 기본 옵션을 임의로 바꾸지 않았다.

또한 Localization은 실제 전진 + / 후진 -인 encoder speed를 기대한다. 절댓값 speed만
오는 firmware라면 Control에서 받아준다고 localization까지 맞는 것이 아니다. 실제 feedback의
speed/encoder/Gear를 함께 확인해야 한다. 명령 기어만 보고 관측 부호를 추측하지 않았다.

## 검증 방법과 결과

```bash
bash src/state_manager/scripts/verify_noetic.sh
```

별도 임시 catkin workspace에 실제 State Manager/Selector/Control/Planner를 빌드한다.
Localization은 이 시험에서 **메시지 fixture와 실제 Python RDDF matcher**만 사용한다.
이 스크립트 자체는 전체 EKF/센서 드라이버나 Arduino firmware를 빌드/구동하지 않는다.
ROS 노드 시험의 drive 출력은 `/control_contract/drive`로 격리돼 차량 토픽에 발행하지 않는다.

- State Manager Python: 192 tests.
- Selector Python: 20 tests.
- RDDF tracker Python: 20 tests.
- 실제 3종 RDDF 기어별 투영: 2 test methods, 81 위치 subcases 중 종료 handoff도 검사.
  새 leg 선택을 제거한 비교에서는 36 subcase가 실패했다.
- Object Detection: 18 tests, 기존 oversized 군집 제거 검사 포함.
- C++ PP/Stanley/속도/명령 매핑 단위시험 및 Frenet 9 scenario groups.
- 실제 Control ROS 시험: 전진, 일반 정지, signed 후진, magnitude 후진,
  E-Stop, feedback EStop 비되먹임/해제 후 재출발, RC 모드 정지, NaN 속도 정지.
- 실제 Planner ROS 시험: clear → 전폭 차단 → clear → 남은 0.75 m 경로.
- State Manager–Selector ROS smoke test.
- 전체 launch 파라미터 전개: 용인/홍익/홍익 S × LD 1/2 총 6개 설정,
  주차 좌우 override 및 1.5 m 군집 제거 설정 유지 확인. 드라이버 실행 없음.
- 5 Hz Planner + 20 Hz PP 폐루프: LD 1/2 각각 직선 라바콘,
  용인/홍익/홍익 S RDDF의 가상 라바콘 총 8회.

폐루프는 속도 1 m/s, wheelbase 0.75 m, 조향 한계 25도, 조향 rate 45도/s의 간단한
kinematic bicycle이다. 장애물은 출발 station +6 m에 둔 0.36 m 정사각형, 실측 시험 배치가
아니다. 사용한 치수는 현재 통합 설정 1.35 × 0.85 m다. 센서 잡음, GPS jump, latency,
타이어/모터, 정수 조향 양자화/Arduino PI는 완전히 재현하지 않는다.

직선 반환 오차는 LD1 약 -0.00010 m, LD2 약 +0.024 m,
세 S 기준선은 약 +0.0099 / +0.0037 m였다. 모형상 차체-장애물 충돌 없이 종료했다.
**실차 cm급 정확도 주장이나 LD 1이 학교에서도 정상이라는 증거는 아니다.**

별도 펌웨어 검증은 White/Black 실제 스케치 호스트 모의 입출력 시험과 Uno AVR
컴파일/링크를 수행했다. ROS 출력 허용/금지 및 ROS 끈 RC 빌드 총 6개 구성이 통과했다.
ROS 출력 허용 빌드는 Flash 20,772 / 20,790 B, static SRAM 각 1,551 B였다.
정적 할당 후 SRAM 497 B는 실제 stack/heap 최소 여유를 측정한 값이 아니다.
실제 USB rosserial 협상·지속 통신과 하드웨어 입출력 검증은 남아 있다.

### 추가: 전체 workspace 빌드 및 등록 시험

별도 임시 의존성 환경에서 **실제 전체 workspace 빌드와 `run_tests`도 완료**했다.
최종 `catkin_test_results` 출력은 다음과 같다.

```text
Summary: 367 tests, 0 errors, 0 failures, 0 skipped
```

367은 catkin XML 집계값이며 gtest suite/rostest wrapper 중복 집계가 포함될 수 있어,
앞의 독립 시험 개수와 합산하지 않는다. 실제 EKF를 사용한 delayed GPS,
IMU 보정·미래 시각, 초기 heading, encoder yaw hold, GPS gate/reanchor,
RDDF 초기화(GPS/수동)·공개 tracker, startup gate 등 Localization 등록 ROS 시험도 통과했다.
전체 launch 검사 역시 실행 파일/패키지 해석 검사이며 실제 센서를 켠 시험은 아니다.

첫 전체 시험 실패 중 누락 패키지/임시 catkin 환경 캐시 문제는 시험 환경에서 해결했다.
나머지는 위에 명시한 DR 시간 설정 및 취득 전후 RDDF 정책과 기존 시험의 불일치였다.
운용 정책을 몰래 강화/완화해서 통과시킨 것이 아니라 실행 설정을 유지하고 시험 계약을
명시적으로 정리했다. 전체 시험 최종 로그: `/tmp/stier-full-tests-final.log`.

학교에서 LD 2가 개선된 이유는 더 먼 회피 부분을 미리 보는 효과, 경로 갱신/조향 응답,
좌표/시간 지연 등 여러 가능성이 있다. 새 원 교점 회귀시험은 특정 버그를 입증하지만,
그 버그가 학교 증상의 단독 원인이었는지는 당시 `/path/final`, odometry, control state,
DriveCmd 기록 없이는 확정하지 않는다.

## 실행 설정

다음은 **실차 구동이 포함된** 명령이다. 이 점검 중에는 실행하지 않았다.
의존성 설치/전체 빌드, 새 메시지 적용, 실제 탑재 firmware 계약 확인 후 사용한다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch stier_bringup full_vehicle.launch hongik_s_test:=true \
  lookahead_m:=2 t_parking_side:=left parallel_parking_side:=right
```

일반 홍익 배치면 `hongik_test:=true`, 용인이면 둘 다 생략한다. 두 학교 인자를 동시에 켜지
않는다. `lookahead_m` 생략 시 기존 YAML 적용(현재 min=max=2 m). 분기 인자 생략 시
mission JSON/overlay 설정을 따른다. `left/right`만 유효하고 parking profile 자체는
현재 RDDF 누적거리 기준으로 유지한다. 다른 주차 CSV로 바꾸면 cusp station도 다시 정해야 한다.
임의 새 `rddf_directory`를 쓸 때 Localization initialization YAML의 디렉터리/원점도 함께 맞춘다.

현재 노트북에는 robot_localization, GeographicLib, rtcm/nmea/mavros/geographic/uuid
메시지 등 일부 의존성이 설치되지 않았다. 첫 기본 빌드는 이 환경 의존성에서 막혔다.
이후 공식 ROS/Ubuntu DEB를 별도 임시 폴더에 풀고 **실제 전체 소스 빌드에 성공했다**.
시스템 패키지 설치, `/opt/ros` 변경은 하지 않았다. binary CMake 설정의 GeographicLib
절대 경로 한 곳만 임시 추출 디렉터리에 맞춰 바꾸고 include/library 경로를 전달했다.
이는 Localization 메시지 fixture를 사용한 앞의 회귀 workspace와는 별도 결과다.

- 전체 빌드: `/tmp/stier-full-build.MgIgkX`, 로그 `/tmp/stier-full-build.log`.
- 추출 의존성: `/tmp/stier-localization-deps.bV66zF/extracted`.
- 통합 회귀 로그: `/tmp/stier-verify-final-v2.log`.
- 전체 등록 시험 로그: `/tmp/stier-full-tests-final.log` (0 errors / 0 failures).

위 `/tmp` 환경을 영구 실차 설치로 사용하면 안 된다. 정상 실차 workspace에서는 README의
의존성 설치/rosdep 절차 후 새 메시지를 포함하여 전체 빌드한다. 루트 build에는 점검용
whitelist가 남을 수 있으므로 `catkin_make -DCATKIN_WHITELIST_PACKAGES=''`로 해제한다.
rosserial/camera/LiDAR 등의 런타임 의존성과 실제 장치 연결은 별도로 필요하다.

## 아직 필요한 실차 확인 / 변경하지 않은 사항

1. **실제 업로드 firmware**: Gear/EStop 수신, 후진 출력, signed feedback, rosserial 메시지 MD5.
2. 차량 치수: 예전 사용자 실측 1.40 × 0.775 m와 팀 통합본 1.35 × 0.85 m가 다르다.
   임의로 새 실측값을 단정하지 않고 현재 팀 설정을 유지했다. 확인 후 vehicle.yaml 한 곳 수정.
3. RDDF 좌우 폭 2 m는 일괄 설정이다. 실제 curb 경계가 아닐 수 있다. 도로 총폭 6.54 m만으로
   좌/우 잔여 폭을 대칭으로 넣으면 안 된다.
4. Planner의 RDDF_CLEAR는 기준선이 비었을 때 원 RDDF로 바로 바뀐다. 단순 회피/복귀 회귀는
   통과했지만, 추종 오차가 큰 상태의 복귀 접속 곡선까지 독립적으로 보장하는 기능은 아니다.
5. 현재 경로 충돌 검사는 map 좌표이고, 최신 LiDAR를 odom/base_link로 옮긴 별도 최종 재검사는
   이번 통합본에 없다. pose jump guard만으로 모든 global correction을 보장하지 않는다.
6. `standstill_speed_mps=0.5`는 1.8 km/h까지 정지로 간주한다. Arduino 실제 기어 전환 계약과
   함께 재검토할 항목이다. 현재 threshold/기어 dwell은 임의 변경하지 않았다.
7. 주차 제한 0.5 m/s는 uint16 KPH 내림 때문에 실제 상위 명령 1 km/h이다. 1 km/h에서
   모터가 출발하는지는 물리 시험이 필요하다. 최소 속도를 임의로 올려 상한을 넘기지 않았다.
8. 신호 20초 후 출발/종료 차선 fallback 등 기존 미션 정책은 유지했다. 이번 변경은 경기
   규정 준수 인증이 아니다. 주최 측 현재 규정과 팀 정책을 별도로 확인해야 한다.
9. 후방 LiDAR ROI는 제공하지 않는다. 후진 주차 주변의 사람/장애물 회피까지 갖췄다는 뜻은 아니다.
10. 팀 커밋 `fe4ef2f`는 GPS 소실 시 dead reckoning 상한을 2,000초 / 1,000 m,
    GPS/전역 위치 차이 허용치를 30 m로 완화했다. 이 값은 이번에 변경하지 않았다.
    정지가 덜 발생하는 설정과 위치 정확도는 별개다. 실제 운용 의도와 GPS 소실 후
    허용 누적 오차는 Localization 담당자와 확인해야 한다.

다음 시험에서는 `/control/state`, `/mission/diagnostics`, `/mission/state`,
`/path/selector_status`, `/path_planner/status`, `/path/final`, `/path/local`,
`/erp42_serial/drive`, `/erp42_serial/feedback`, `/molit/localization/odometry`,
`/molit/localization/rddf/current`, `/dbscan_clusters`, `/scan`, `/tf`, `/tf_static`을
동시에 기록한다. 조향→중립이 반복될 때 stop reason과 기어/속도/경로를 같은 시각으로
대조하면 경로 승인 문제와 하위제어 문제를 구분할 수 있다.
