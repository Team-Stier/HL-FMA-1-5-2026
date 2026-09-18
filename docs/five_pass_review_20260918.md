# 5차 교차 검토와 전체 시험 5회 반복

요청: 마지막 학교 시험 전 최소 5번 검토. 새 안전 규칙·정지 조건을 추가하지 않는다.
기준은 이전 통합 수정이 반영된 로컬 `fix/integration-review-20260918`이다.
이번 검토에서는 **실차 주행 로직, timeout, EStop, 차량 파라미터를 변경하지 않았다.**
추가/수정한 것은 실행 스크립트의 인자 전달, 시험 코드/등록/의존성, 안내 문서다.
검토 당시에는 커밋·푸시·보드 업로드를 하지 않았다. 이후 사용자 요청으로 이 검토본을
같은 이름의 공유 브랜치에 배포하며, 준비 순서는 [전체 시험 패키지 안내](branch_test_quickstart.md)에 있다.
보드 업로드·실차 주행 검증을 추가로 수행했다는 뜻은 아니다.

## 1차 — 실행 구성과 메시지 연결

- 용인/홍익 일반/홍익 S × LD 1/2의 6개 launch 파라미터 전개 확인.
- RDDF Provider/Tracker/검출기의 경로 디렉터리, 주차 좌우 선택,
  LD min/max, 공통 wheelbase, 1.5 m 군집 제거 설정을 대조했다.
- launch/test/package XML 81개 파싱 통과.
- 전체 workspace를 source한 상태에서는 테스트가 동일한 소스 폴더를
  `ROS_PACKAGE_PATH`에 다시 넣어 `full_vehicle.launch`를 중복 탐색했다.
  **시험의 roslaunch 호출만 절대 파일 경로로 변경**했다. 실차 launch는 변경하지 않았다.
- 신호등 모델 파일 SHA-256은 설정값과 일치했다. 실제 카메라 추론 정확도 검증은 아니다.
- README가 0917 저장소를 clone한 뒤 예전 `HL-FMA2026-stier` 폴더로 이동하도록
  안내하던 다섯 곳을 현재 저장소 이름으로 통일했다. 다른 workspace의 옛 실행 파일을
  source하는 혼동을 줄이는 문서 수정이다.
- `./run.sh --help`가 catkin setup의 인자로 섞여 도움말 문자열을 shell 코드로 읽다가
  실패하는 문제를 재현했다. `run.sh`가 원래 인자를 배열에 보관하고, ROS 환경을
  인자 없이 source한 뒤 roslaunch에 그대로 전달하도록 수정했다. `--help`와
  `--dump-params hongik_s_test:=true lookahead_m:=2`를 노드 구동 없이 확인했다.

## 2차 — RDDF → Frenet → RDDF 연결

새 `src/stier_bringup/test/frenet_pipeline.test` / `frenet_pipeline_ros.py`는 실제
Provider, Tracker, State Manager, Frenet Planner, Selector, Control을 연결한다.
별도 시험 ROS master를 사용하고 구동 출력은 `/parking_contract/drive`로 바꾼다.
센서/Arduino 드라이버는 실행하지 않는다.

1. 2번 경로 끝 3 m 전에서 RDDF 구동 명령 확인.
2. 3번 S 경로에서 `LOCAL` 선택과 실제 구동 명령 확인.
3. 6 m 앞 합성 라바콘으로 `/path/local`뿐 아니라 `/path/final`도 기준선에서
   0.3 m 이상 벗어나는 회피 경로인지 확인.
4. 라바콘 관측 제거 후 `RDDF_CLEAR` 복귀 확인.
5. 끝에서 0.9 m 남은 곳에서 유효 Frenet 경로/구동 확인.
6. 4번 경로로 전환하여 RDDF 선택과 전진 명령 확인.

용인/학교 두 배치 × LD 1/2의 6가지로 등록했다. 처음에는 학교 S의 2번 경로 전체
107.57 m를 재생해 성공했고, 반복 시험은 연결 경계에 집중하도록 접근 3 m부터 시작한다.
위치·피드백·신호등·군집은 합성 입력이다. 차가 출력 조향을 따라 움직이는 시험이 아니므로
별도의 Frenet+PP 차량 모형 8가지와 구분한다.

예비 반복에서 시험 조건 오류를 발견했다. 기존 미션은 끝 0.8 m 이내에서 다음 경로를
요청할 수 있는데 시험이 0.75 m에서 여전히 LOCAL이어야 한다고 요구했다.
학교 4개 경우에서 정상적인 4번 경로 전진을 실패로 판정했다. 시험 지점을 0.9 m로
옮겨 기존 1 m Planner 제한의 재발은 검사하면서 정상 handoff는 방해하지 않게 했다.
**이 과정에서 실차 완료/전환 임계값을 수정하지 않았다.**

## 3차 — 주차 네 경우와 주차 뒤 복귀

- 용인/홍익 일반/홍익 S × T자 좌·우/평행 좌·우의 12가지 실제 노드 시험.
- 전·후진 leg마다 pose를 이전 정차 지점에 둔 채 다음 경로 승인을 기다린다.
  억지로 다음 경로로 위치를 옮겨 승인을 통과시키지 않는다.
- 각 leg 및 7/12번 RDDF 복귀에서 네 번 연속 정상 구동 명령을 확인한다.
- 실제 3종 RDDF의 현재 leg 투영 81개 위치 subcase와, 미세 cusp 중복점 회귀도 확인.
- State Manager 194 methods, Selector 20 methods 통과.

정상 미션/기어 변경의 정차, 시작 시 경로 대기, 비동기 전환 중 잠깐의 정지 명령은
기존 동작이다. 시험 로그에 관련 경고가 있다는 사실을 숨기지 않는다. 검증한 것은
정상 입력 아래 다음 경로 승인을 못 받아 계속 멈추는 문제가 재발하는지이며,
모든 제어 tick에서 한 번도 brake가 나오지 않는다는 주장이 아니다.

## 4차 — 두 차량 하위제어와 센서 입력

- White/Black 실제 스케치·Core·RosBridge에 생성된 실제 `ros_lib` 메시지를 연결한
  호스트 모의 시험 통과. 메시지 MD5/직렬화, signed 속도, Gear, MorA, 정지 후 출발 확인.
- 시험을 확장해 각 차량에서 **1 km/h의 전·후진 주차 leg를 20회 교대**했다.
  기존 arming/기어/PI 상태가 앞 leg에서 남아 출발을 막는지, PWM과 피드백의 방향을 확인했다.
  실제 모터의 breakaway torque나 노면 주행은 모의 핀 출력으로 판단할 수 없다.
- 검출기 실제 함수로 roll 180° + yaw 180°의 좌표 변환, 비유한 LaserScan 빔 제거,
  timestamp가 있는 정상 빈 군집과 무효 clear의 구분을 확인했다.
- DBSCAN/ROI 18개 단위시험 통과. 1.5 m 초과 군집 제거 설정/알고리즘은 그대로다.
- LiDAR TF는 Localization 한 곳이 발행하고 드라이버에서 중복 발행하지 않는 구성을 유지한다.

## 5차 — 메모리 검사와 반복 실행

AddressSanitizer + UndefinedBehaviorSanitizer를 켜서 다음을 실행했고, 실행한 범위에서는
메모리/정의되지 않은 연산 오류가 보고되지 않았다.

- Frenet 코어 9개 시나리오 그룹.
- 실제 Frenet+PP, LD 1/2 × 직선/용인 S/학교 두 S 배치의 차량 모형 8회.
- 두 차량 실제 스케치의 호스트 모의 시험 및 추가한 20회 1 km/h 교대 주차.

이는 호스트 CPU의 검사다. Uno의 실제 stack/heap 최저 여유나 USB rosserial 통신을
측정한 것으로 해석하지 않는다. 격리 검증 스크립트도 새 시험 호출 방식으로 재실행했다.

첫 반복 묶음의 2회차에서는 초기 heading ROS 시험 하나가 실패했다. 시험이 t=4 s에
엔코더 이동 시작과 IMU 선회 시작을 동시에 발행하면서 모든 표본의 각속도 보존을
요구했다. Header 없는 엔코더 수신과 IMU가 서로 다른 토픽이므로 첫 선회 표본에
기존 정지 관측이 적용되면 정상 yaw-hold가 gyro를 0으로 만든다.
합성 직진 시작을 t=3 s, 선회를 기존 t=4 s로 분리했다. 각속도 등식 검사, 생산
`encoder_yaw_hold=true`, 정지/재출발 전용 ROS 시험은 유지한다. 런타임 timeout이나
허용 오차를 바꾸지 않았다. fixture 수정 후 별도 초기 heading 5회가 모두 통과했고,
전체 시험도 5회 연속 통과했다. 결과는 아래와 같다.

### 전체 등록 시험 반복 결과

최종 시험 조건을 고정하고, 각 회차마다 임시 빌드의 이전 XML 결과를 비운 뒤
전체 `run_tests`와 `catkin_test_results`를 실행한다. 통과한 회차의 XML도 별도 보관한다.
최종 5회 모두 통과했으며 회차별 결과는 다음과 같다.

| 회차 | 집계 결과 |
|---|---|
| 1 | 405 tests, 0 errors, 0 failures, 0 skipped |
| 2 | 405 tests, 0 errors, 0 failures, 0 skipped |
| 3 | 405 tests, 0 errors, 0 failures, 0 skipped |
| 4 | 405 tests, 0 errors, 0 failures, 0 skipped |
| 5 | 405 tests, 0 errors, 0 failures, 0 skipped |

405는 catkin XML 집계이며 suite/rostest wrapper 중복이 포함될 수 있다.
독립 시험 개수로 보거나 5배를 서로 다른 시나리오 수라고 부르지 않는다.
새 6개 Frenet 노드 연결 시험으로 이전 집계 393에서 405가 되었다.
최종 반복에는 주차 12종 × 5회 = 60회 실행, Frenet 구간 연결 6종 × 5회 = 30회
실행이 포함된다. 각각 서로 다른 60/30개의 시나리오라는 뜻은 아니다.

회차별 로그/결과(임시 파일):

- `/tmp/stier-final-five-N-tests.log` / `-summary.log` / `-xml/` (`N=1..5`).
- `/tmp/stier-five-review-isolated.log`: 격리 빌드/검증.
- `/tmp/stier-five-review-state.log`, `/tmp/stier-five-review-selector.log`.
- `/tmp/stier-five-review-firmware.log`: 두 차 1 km/h 반복을 포함한 호스트 시험.
- `/tmp/stier-five-review-core-sanitizer.log`, `-white-sanitizer.log`, `-black-sanitizer.log`.
- `/tmp/stier-review-1-tests.log`: 시험의 0.75 m 기대값 오류가 드러난 예비 실행.
- `/tmp/stier-reviewed-pass-2-tests.log`: 초기 heading의 합성 입력 순서 문제가 드러난 실행.
- `/tmp/stier-initial-heading-N.log` (`N=1..5`): fixture 수정 후 초기 heading 반복.

## 내일 실행 전 남은 사항

1. 실제 보드 업로드 여부는 확인되지 않았고 이번 작업에서 업로드하지 않았다.
   해당 차종의 후진 지원 펌웨어, ROS 모드, signed feedback을 확인한다.
2. 검증용 `/usr/bin/python3` 환경에는 `serial`(pyserial), `torch`, `ultralytics`가 없다.
   ROS/GeographicLib 등 일부 패키지는 `/tmp`의 별도 의존성으로만 전체 빌드를 검증했다.
   실제 실행 PC에는 README의 시스템/ROS 의존성 및 `traffic_light/README.md`의
   Python 의존성을 설치해야 한다. `rosdep`만으로 pip의 YOLO 의존성이 설치되지는 않는다.
3. 올바른 학교 배치·실제 위치/방향·차량 치수·센서 장착은 실차 확인 사항이다.
   이번 시험은 실제 GPS 오차/센서 흔들림/카메라 인식률/마찰/조향 지연을 모두 재현하지 않는다.

실행 명령과 점검 토픽은 [학교 시험 안내](school_test_20260918.md)를 사용한다.
소프트웨어 반복 통과는 실차 성공 보장이 아니며, 이를 이유로 관측되지 않은 장애물이
없다고 가정하거나 기존 EStop을 제거하지 않았다.
