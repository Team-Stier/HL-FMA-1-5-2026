# HL-FMA2026-stier 이름 정렬 결과

이 작업은 Mando `5c6c753`의 `mando_localization`을 HL-FMA2026-stier
`1f59303`의 `src/localization`으로 옮긴 **이름·경로 정렬 단계**다.
추정 알고리즘, 상태 전이, 품질 gate, 센서 보정값, EKF 설정 수치와 융합 입력
선택은 변경하지 않았다. 대상 저장소의 완전한 실행 계약을 충족한 상태는 아니다.
기존 대상 패키지는 Git의 기준 커밋에 보존되어 있고 Mando 원본도 수정하지 않았다.

## 변경한 이름과 경로

| 구분 | 원본 | 적용 |
|---|---|---|
| 패키지·C++ namespace·생성 메시지 namespace | `mando_localization` | `localization` |
| GNSS 위치 | `/molit/sensors/gps/fix` | `/ublox_position_receiver/fix` |
| GNSS 상세 입력 | `/molit/sensors/gps/navpvt` | `/ublox_position_receiver/navpvt` |
| IMU 입력 | `/molit/sensors/imu/data` | `/imu/data` |
| LiDAR 입력 | `/molit/sensors/lidar/scan` | `/scan` |
| 변환된 엔코더 속도 | `/molit/vehicle/twist` | `/localization/encoder/twist` |
| 추정 결과·상태 토픽 접두사 | `/molit/localization` | `/localization` |
| 내부 토픽·파라미터 namespace | `/mando_localization` | `/localization` |
| LiDAR frame | `laser_link` | `laser` |
| 센서 노드 이름 | `ublox_gps_node`, `mando_encoder_serial`, `mando_rplidar_s2` | `ublox_position_receiver`, `arduino_serial`, `rplidar_s2` |
| 실행 진입점 | `roslaunch mando_localization bringup.launch` | `./src/localization/launch.sh` |
| 로컬 실행 스크립트 workspace | 현장 PC의 고정 절대 경로 | 현재 스크립트 위치에서 계산, `LOCALIZATION_WS`로 지정 가능 |
| RDDF 파일 | 별도 workspace `rddf/` | 패키지 내부 `rddf/`, 파일 내용 동일 |

`launch/localization.launch`는 `bringup.launch`의 별칭이다. 루트 `run.sh`가
호출하는 `launch.sh`는 센서·차량 드라이버와 RViz를 별도로 시작하지 않는다.
원본의 단독 센서 실행·기록용 스크립트는 보존되어 있으므로 통합 진입점과 구분한다.
공개 출력은 여전히 `/localization/odometry` (`nav_msgs/Odometry`)다.
이번 단계에서 `/current_pos`를 잘못된 타입으로 발행하지 않는다.

## 이름 변경으로 끝나지 않는 부분

### 알고리즘은 그대로 두고 별도 연결 코드가 필요한 부분

1. **`/current_pos` 메시지 변환**: 대상은 `geometry_msgs/PoseStamped`를 요구한다.
   승인된 최종 `/localization/odometry`의 `header`와 `pose.pose`를 복사하는
   출력 어댑터가 필요하다. 이 변환은 EKF 알고리즘 변경이 아니지만, 이번 요청의
   이름 변경 범위를 넘으므로 추가하지 않았다. Raw Global EKF에서 직접 변환하면
   최종 출력 gate를 우회하므로 반드시 최종 승인 출력을 사용해야 한다.
2. **실행 단위**: 대상 README는 패키지당 노드 1개를 요구하나 기존 대상 launch도
   localization·odometry·status 3개 노드를 실행했다. 원본의 다중 노드 구성은
   유지했다. 프로세스 구조 재편 또는 아키텍처 문서 정리가 필요하며 이름만
   하나로 바꾸면 여러 노드가 서로 종료시킨다.
3. **센서 드라이버 의존성**: 원본의 단독 bringup과 manifest에는 `ublox_gps`,
   `xsens_mti_driver` 의존성이 남아 있다. 대상의 내부 실행 패키지 간 직접
   의존 금지에 완전히 맞추려면 이 실행 도구를 센서 bringup 소유로 분리해야 한다.
   기존 `erp42_msgs`, u-blox 드라이버를 중복 복사하지 않았다.

### 품질 판단 또는 시간 처리 동작을 바꿔야 하는 부분

1. **GPS 상태 gate**: 대상은 `/gps/status` (`sensor_interfaces/GpsStatus`)의
   solution, spoofing, 위성 수, 예상 정확도를 위치 승인에 사용한다. 원본에는
   이 메시지 구독이 없다. NavPVT를 사용하는 yaw 보정은 이 위치 승인 검사를
   대체하지 않는다. 입력 추가와 승인 조건 연결은 품질 판단 변경이므로 보류했다.
2. **GPS 장애 중 출력 정책**: 원본은 최대 2초 또는 10m까지 제한된 추측항법을
   유효한 출력으로 허용한다. 대상 기존 `/current_pos`는 GPS 품질·freshness가
   나빠지면 발행하지 않는다. 대상 정책을 그대로 따르려면 공개 출력의 승인
   조건을 바꿔야 한다. 추측항법 알고리즘 자체를 없앨 필요는 없다.
3. **GPS 측정 시각 계약**: 원본 timing monitor는 드라이버의
   `stamp_source=gnss_utc` 진단을 확인해야 `clock_ready`를 허용한다.
   대상 u-blox 드라이버는 유효한 NavPVT UTC를 사용할 수 있지만, UTC가 유효하지
   않으면 PC 현재 시각으로 대체하며 원본 전용 timing 진단을 제공하지 않는다.
   따라서 현재 연결만으로는 GPS gate가 승인되지 않을 수 있다. 원본의 측정 시각
   검증·진단을 대상 센서 드라이버에 이식하거나 시간 계약을 검토해야 한다.
   이번 작업에서는 `require_clock_ready`를 끄거나 timestamp를 덮어쓰지 않았다.

### 알고리즘 변경보다 현장 설정·TF 소유자 합의가 필요한 부분

- `first_fix`, `measured: false`, 시작 yaw 약 161.47°와 레버암·장착값은 원본
  그대로다. 운영 지도의 측량 datum, 시작 방향과 실제 장착에 맞춰 설정해야 한다.
- 원본 LiDAR 장착은 `[1.05, 0, 0] m`, roll/yaw 각각 180°다. 대상 LiDAR
  bringup은 다른 장착값의 `base_link -> laser` 정적 TF를 기본 발행한다.
  frame 이름을 맞춘 뒤에도 두 발행자를 동시에 켜면 충돌한다. 실측값과 단일
  발행자를 정해야 하므로 이번 작업에서 어느 보정값도 임의로 선택하지 않았다.
- 원본 기록 스크립트의 외부 Arduino CAN 로거 경로와 특정 센서 장치 ID는
  현장 장치 의존성이다. 해당 외부 도구는 이 패키지로 복사하지 않았다.

아직 `/current_pos`가 없으므로 planner·selector에 위치 입력이 연결된 통합
완료 상태로 실행하면 안 된다. 위 항목은 이름 정렬 완료와 별도로 남아 있다.

## 이번 작업의 검증

- C++·Python 구현 파일 51개를 Mando 원본에 `name_alignment_map.json`의 이름
  치환만 적용한 결과와 비교해 전부 일치함을 확인했다.
- 모든 YAML 설정을 원본과 비교했다. 이름과 RDDF 파일 경로 외에 EKF 입력 선택,
  수치, 보정값, gate와 상태 정책 변경은 없다. RDDF 파일 내용도 동일하다.
- 기존 설정·보정 IMU·엔코더 yaw hold·센서 시간·LiDAR 표시·뷰어 테스트 총 92개 통과.
- Catkin 패키지 검색, launch/test XML, YAML 파싱 및 셸 구문 검사 통과.
- `catkin_make --only-pkg-with-deps localization -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2`는
  이 PC의 `robot_localization` 미설치로 CMake 단계에서 중단됐다. C++ 컴파일과
  ROS 통합 테스트를 통과한 것으로 보지 않는다. 실센서와 차량 제어는 실행하지 않았다.
