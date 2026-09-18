# 로컬 미션·속도 수정 — 상한 15km/h

개발 기준은 기존 시험 패키지 `227a16a`이며, 인계 브랜치는 `fix/school-test-ready-20260918`이다.
처음에는 `fix/local-mission-speed-tuning`에서 로컬로 개발했고, 이후 사용자 요청으로
공유 패키지에 포함했다. 하얀차 Arduino 업로드·재읽기 검증도 완료했다.
다른 PC의 최신 실행 순서는 [인계 안내](pc_transfer_20260918.md)를 따른다.
기존 배포 안내의 기본 속도·정차 시간 설명보다 이 문서가 우선한다.

## 동작

| 항목 | 현재 설정/동작 |
|---|---|
| 제어 명령 절대 상한 | 15km/h. 기본 목표도 15이지만 계속 15를 보내는 정속 제어는 아님 |
| 일반 RDDF 미션 상한 | 15km/h, 곡률·조향·경로 길이·정지선에 따라 낮아짐 |
| 정적 장애물 / 교차로 | 각각 5km/h 상한 |
| 경사로 / 주차 | 기존 1.0 / 0.5m/s 상한 유지. 정수 명령으로 최대 3 / 1km/h |
| 경사로 정차 | 허용 정지구역 안에서 연속 3.5초, 바퀴 이동 시 누적 시간 재시작 |
| 신호 미인식 | 정지선 0.5m 이내에서 정지 요청 유지, 미인식 상태로 연속 정차 20초 후 출발 |
| 정상적으로 인식한 신호 | 2·4는 GREEN, 7은 LEFT_ARROW. 적색/황색/잘못된 화살표를 20초 뒤 무시하지 않음 |
| Pure Pursuit LD | 기본 속도·곡률 적응형 2~4m. `lookahead_m:=2`로 고정 가능 |
| 경로 전방 확인 | RDDF 40m 발행, 속도 계획도 최대 40m 확인 |

속도 정책은 **기존 주행 허가 안에서 속도만** 정한다. 새 전역 정지 게이트나 새 장치
타임아웃을 추가하지 않았다. 기존 비상정지, 실제 경로 충돌, 유효하지 않은 위치/경로,
후진 전 정차 조건은 삭제하지 않았다. 미인식 20초 정책도 이 기존 조건을 우회하지 않는다.

## 속도 계산

`control/src/longitudinal/path_speed_profile.cpp`:

- 진행방향으로 정렬한 후륜축 경로를 차량의 투영점부터 0.5m 간격으로 확인한다.
- 곡률 상한은 `sqrt(a_lat / |curvature|)`이며, 현재 요구 조향각과 이번 주기의
  변화율 제한 후 출력각도 반영한다. 조향이 풀리는 동안 속도가 먼저 올라가지 않게 한다.
- 전방 곡선 진입 전 감속은 `v² <= v_curve² + 2*a_decel*distance`로 계획한다.
- 정지선 접근은 잔여 거리와 반응 거리를 반영한다. 실제 정지는 기존 미션이 요청한다.
- 짧은 경로를 긴 직선으로 오인하지 않도록 확인 가능한 길이도 속도에 반영한다.
- Arduino 명령 해상도는 정수 km/h다. 이미 주행이 허가된 상태에서 계산값이 1km/h
  미만이면 접근이 영원히 끝나지 않도록 1km/h까지 접근하며, 미션의 더 낮은 상한을 올리지는 않는다.
- `MissionState.speed_limit_mps`의 float32 반올림으로 15가 14km/h로 잘리는 오류를 보정했다.
- feedback `steer`는 조향 ADC 값이다. 물리적 바퀴 각도라고 가정하지 않았다.

기본 횡가속도 1.0m/s², 계획 감속도 0.25m/s², 반응시간 0.5초는 **초기 계획 가정**이다.
실측 접지력·브레이크 성능 인증값이 아니다. 예를 들어 조향 25도, 축거 0.75m에서
이 가정으로 계산한 상한은 약 4.6km/h이며 정수 명령은 더 낮다. 15km/h는 충분히 긴
직선 등에서만 도달 가능한 설정값이지, 모든 RDDF에서 검증한 안전 최고속도가 아니다.

White/Black `BroonT870Core.cpp`는 가속 시 1km/h/s 램프를 유지하되 낮아진 목표 속도는
즉시 PI 목표에 반영한다. 이전에는 15→3 요청에도 내부 목표 하강에 12초가 걸렸다.
이는 PWM·속도 목표 응답 수정이며 능동 브레이크나 역토크를 추가한 것이 아니다.
실제 감속 성능은 탑재한 드라이버와 관성에 따라 달라진다.

## 3.5초 정차와 밀림의 한계

기존 미션의 정지구역 중앙점을 유지한다. 중앙을 조금 지나도 허용 구역 안이면 정차
완료를 인정하고, GPS가 약간 뒤로 보정되어도 정지/재접근을 반복하지 않도록 정지 요청을
유지한다. 구역 밖 정차로 미션을 완료시키지는 않는다.

Arduino의 새 `alive` 관측에서 `abs(speed) <= 0.05m/s`와 `encoder delta == 0`을 확인한다.
바퀴가 움직이거나 그 관측이 오래되면 정차 누적 시간이 초기화된다. 신선한 바퀴 관측이
있으면 작은 GPS 위치 보정만으로 타이머를 초기화하지 않는다. 기존 재생 자료처럼 바퀴
입력이 전혀 없는 경우 기존 위치 변위 판단을 사용한다. 전역 localization 검사와 기존
0.5m rollback 판정은 유지하므로 GPS 보정 영향이 모두 사라진다는 의미는 아니다.

**아직 물리적인 경사로 밀림 방지 기능은 완성되지 않았다.** 확인한 팀 펌웨어에서
`brake=1`은 PWM=0이며 경사 유지 토크/위치 제어는 없다. 3.5초 정지 요청과 이동 감시가
능동 제동을 대신하지 않는다. 실제 모터 드라이버의 브레이크 입력·배선·극성을 모르는
상태에서 임의 역토크를 구현하지 않았다. 경사로에서는 실제 밀림과 재출발을 확인해야 한다.

## 팀 코드 비교에서 확인한 사항

읽어 비교한 원격 main은 `ae430d72d46a86accf6f95b9af73bfbb7a044e32`이다.
통째 병합하지 않았으며 다른 팀원의 브랜치나 main은 변경하지 않았다.

- 팀 버전의 신호 정지선 접근 0.5m 기준은 반영했다.
- 팀 버전의 경사로 정지 완료는 구역 밖에서도 가능한 부분이 있어 그대로 복사하지 않았다.
- 팀 버전의 일반/주차 속도와 학교 설정이 일관되지 않았다. 양쪽 mission config를 맞췄고
  주차를 바로 6km/h로 올리는 설정은 사용하지 않았다.
- 새 원격 신호 모델의 SHA256은 `b59bafce683edab9e5f52a28ac0ec0398017689b556db61c3c4415f4056cc540`,
  설정에 남은 기대값은 `caaf2ab7a3f1aa6500a9907ab1a2044119e3051857d990a54ee0097603e6966f`였다.
  그 조합으로는 노드의 해시 검사에서 시작이 막힌다. 실제 학교 실패 원인이 이것이었다고
  단정할 로그는 없다. 이 로컬 브랜치는 기존 모델과 일치하는 해시 조합을 유지했다.

## 빌드와 실행

ROS C++ 코드가 바뀌었으므로 YAML만 복사하거나 예전 devel을 사용하는 것으로는 적용되지
않는다. 해당 워크스페이스에서 의존성 설치 후 다시 빌드한다. 자세한 설치/장치 연결은
[전체 시험 패키지 안내](branch_test_quickstart.md)를 따른다.

```bash
cd /home/choiminho/바탕화면/HL-FMA2026-0917
source /opt/ros/noetic/setup.bash
catkin_make -j4 -DCATKIN_WHITELIST_PACKAGES='' -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

아래 명령은 **실차 장치와 제어 명령을 실행**한다. 실제 사용하는 학교 RDDF 배치와
주차 좌우, Arduino 포트를 맞춘 뒤 사용한다. 두 학교 배치를 동시에 켜지 않는다.
기본값 자체가 상한 15이고, 다음 예시는 이를 명시한 것이다.

```bash
bash run.sh hongik_s_test:=true target_speed_kph:=15 \
  t_parking_side:=left parallel_parking_side:=right arduino_port:=/dev/ttyACM0
```

저속 확인 시 `target_speed_kph:=5`, 기존 LD 비교 시 `lookahead_m:=2`를 덧붙인다.
미션별 상한은 `normal_speed_kph`, `hill_speed_kph`, `static_speed_kph`,
`intersection_speed_kph`, `parking_speed_kph` 인자로 조절할 수 있다. 전체 제어 상한 15는
유지된다. `adaptive_speed_enabled:=false`는 비교 시험용이며 곡률 감속이 사라진다.
차량별 수정 펌웨어도 다시 빌드·업로드해야 하위 목표 감속 변경이 적용된다.
코드 수정만으로 현재 보드에 탑재된 프로그램은 바뀌지 않는다.

## 로컬 검증

```bash
# 센서/Arduino를 열지 않는 분리 워크스페이스 시험
STIER_KEEP_TEST_WORKSPACE=1 bash src/state_manager/scripts/verify_noetic.sh
bash src/sensor_drivers/arduino/firmware/tests/verify_speed_profile.sh
```

확인한 결과:

- State Manager Python 202개, Selector 20개, RDDF tracker/parking 22개,
  launch 계약 2개, 검출기 18개 통과.
- 속도 코어 12개 및 기존 PP/Stanley/속도·조향 단위시험 통과.
- 실제 `control_node` + 합성 입력: 15km/h 상한, float32 미션 속도, 곡선 감속,
  정지선 저속 접근, 정지/재출발, 주차 1km/h, 전후진/기존 EStop 계약 통과.
- State Manager/Selector와 Planner 메시지 전송 시험 통과.
- Frenet 9개 시나리오, 합성·용인·학교 두 RDDF의 LD1/2 기하 모델 회귀시험 통과.
- 하얀차/검은차 실제 PI 코어를 각각 UBSan으로 검사: 가속 램프 유지, 즉시 목표 감속,
  부호 있는 속도, 0속도 초기화 통과.
- 양쪽 실제 `.ino` + RosBridge 호스트 회귀시험 통과: 메시지 직렬화, 전후진, 정차 후
  기어 변경, 브레이크 해제 재출발, 기존 RC/EStop, 1km/h 주차 방향 20회 반복.
- Arduino AVR core 1.8.6, ROS=1, actuator=1 설정의 Uno 펌웨어 빌드 통과.
  하얀차 Flash 20,734B / 검은차 20,760B, 각각 정적 SRAM 1,551B. 실제 스택 여유나
  차량 작동 성능을 이 수치만으로 보장하지 않는다. 생성물: `/tmp/stier-avr-build.t6HGys`.

검증 로그: `/tmp/stier-mission-speed-verified.log`, 격리 빌드:
`/tmp/stier-noetic.NvirRF`. 이 경로는 임시 파일이라 재부팅 후 사라질 수 있다.
해당 ROS 빌드는 localization 메시지용 fixture를 사용하며 **실제 localization C++/센서
드라이버 전체 빌드가 아니다.** 모델 회귀시험도 15km/h 실차 동역학·마찰·밀림을 재현하지 않는다.
하드웨어 실행이나 업로드는 하지 않았다. 실차 15km/h 추종, 실제 제동거리, 경사 유지 토크는
별도 현장 검증 대상이다.
