# ROS 전·후진 통합 — 2026-09-18 시험 패키지

기준 Mando main: `4753c4755b70494a3013f18524c7248f1ba15aa5`.
최초 구현은 별도 `Mando-firmware-integration` worktree의
`fix/ros-gear-integration-20260918`에서 수행했다. 사용자 요청으로 이 수정본을
HL-FMA2026-0917의 `fix/integration-review-20260918` 브랜치에 함께 제공한다.
원래 Mando 저장소와 작업 폴더는 변경하지 않았으며 **보드 업로드·실차 검증은 하지 않았다.**
이하 명령은 HL-FMA2026-0917 저장소 루트에서 실행한다.
펌웨어 두 폴더와 `tests/`는 원래 검증한 소스 그대로이며, 과거 차종별 안내서보다
이 문서와 [전체 시험 패키지 안내](../../../../docs/branch_test_quickstart.md)의 실행 절차를 우선한다.

적용 대상은 `BROON_T870_White_Car`, `BROON_T870_Black_Car` 두 스케치다.
`BROON_T870_Uno_Controller`는 변경하지 않았다. 차종별 핀·조향 ADC·모터 방향 보정값은
그대로 유지했다. 다른 색상의 스케치를 서로 바꿔 올리는 방식으로 사용하면 안 된다.

## 발견한 불일치와 수정

기존 두 RosBridge는 `KPH/Deg/brake`만 읽고 ROS `Gear/EStop`을 무시했다.
ROS PI는 양수 목표만 처리했고 feedback에 `MorA`와 실제 구동 방향을 넣지 않았다.
또한 일반 ROS brake가 매번 disarm시키는 반면 출발 시에는 0목표로 arming을 요구하여,
상위 Control의 brake 정지 → 양수 출발 명령과 결합하면 출발 대기가 계속될 수 있었다.

| 항목 | 현재 로컬 수정본 |
|---|---|
| `DriveCmd.KPH` | unsigned 속도 크기, 전진 최대 15 / 후진 최대 5 km/h; 기존 상한 적용 |
| `Gear` | 0 전진 / 1 중립 / 2 후진; 음수 KPH 전송은 하지 않음 |
| `brake` | 일반 정지: 모든 모터 출력 억제, 정지 중 기존 arming dwell은 진행 가능 |
| `EStop` | RC stop과 별도로 ROS 비상정지 적용; 명시적인 새 EStop=0 수신 시 해제 |
| 방향 변경 | PWM=0이고 새로운 encoder delta=0, speed=0 관측이 3회 연속일 때 수락 |
| PI | 속도 절댓값으로 기존 PI, 결과 PWM에 수락한 방향 부호 적용 |
| feedback `MorA` | 실제 선택 MODE_ROS=1 / RC=0 |
| feedback `Gear` | 구동 허용된 방향; 일반 정지/방향 변경 대기 중에는 중립 |
| feedback speed/encoder | 원래 encoder의 signed 관측을 보존; 명령 기어로 관측 부호를 만들지 않음 |

기어용 정지 관측은 루프 횟수가 아니라 **새 encoder 샘플**만 센다. 이전 정지 관측을
반복해서 3회로 만들지 않으며, 일반 정지 중 이미 확인한 정지 관측은 전환에 사용할 수 있다.
mode 변경 시 방향 상태와 PI를 초기화한다. 출력 금지 중 PI 적분을 쌓지 않는다.
기존 RC 정지·RC 방향 interlock·통신 timeout·fault 정책과 차종 보정값은 유지한다.
정상 brake와 EStop은 구분한다. 정상 brake는 출력 금지를 유지하지만 매번 arming 상태까지
지우지는 않는다. PWM 0은 기계식 제동력이나 경사로 정지 유지의 보장이 아니다.

## 반드시 통합 저장소의 메시지로 ros_lib 생성

이번 상대 PC는 `HL-FMA2026-0917/src/interfaces/vehicle_interface/erp42_msgs`다.
Mando의 오래된 `Localization_pkg` 메시지로 생성하면 안 된다.
현재 `DriveCmd` payload는 7 bytes이고 MD5는 `518982e31d00755722fd5fb8c3000c77`이다.

통합 workspace를 빌드하고 source한 터미널에서(rosserial_arduino/client 설치 필요):

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find erp42_msgs
rosmsg md5 erp42_msgs/DriveCmd
STIER_GENERATED_LIBS=$(mktemp -d /tmp/stier-ros-libraries.XXXXXX)
rosrun rosserial_arduino make_libraries.py "$STIER_GENERATED_LIBS" erp42_msgs std_msgs
```

기존 Arduino library를 위 명령으로 덮어쓰지 않는다. 생성 위치는 새 임시 폴더다.
실제 IDE에서 사용할 ros_lib 선택은 기존 라이브러리와 MD5를 확인한 뒤 진행해야 한다.

## 출력 없는 검증

호스트 시험은 실제 두 `.ino`, RosBridge, 각 차량 Core와 실제 생성 메시지 클래스를
컴파일하고, 시간·핀·ROS transport만 모의로 제공한다. Arduino 포트에 접근하지 않는다.

```bash
bash src/sensor_drivers/arduino/firmware/tests/verify_ros_drive.sh "$STIER_GENERATED_LIBS/ros_lib"
```

다음은 Uno ATmega328P 실제 ELF/HEX를 생성할 뿐 업로드하지 않는다.
Arduino AVR core 1.8.6과 AVR gcc/g++/binutils로 검증했다. 이 PC의 오래된 시스템 core는
`digitalPinToInterrupt`가 없어 사용하지 않았으며 별도 공식 core로 빌드했다.

```bash
export STIER_ARDUINO_CORE=/ArduinoCore-avr/cores/arduino
export STIER_ARDUINO_VARIANT=/ArduinoCore-avr/variants/standard
STIER_ACTUATOR_OUTPUTS=0 bash src/sensor_drivers/arduino/firmware/tests/build_avr.sh "$STIER_GENERATED_LIBS/ros_lib"
STIER_ACTUATOR_OUTPUTS=1 bash src/sensor_drivers/arduino/firmware/tests/build_avr.sh "$STIER_GENERATED_LIBS/ros_lib"
STIER_ROS_ENABLED=0 STIER_ACTUATOR_OUTPUTS=1 \
  bash src/sensor_drivers/arduino/firmware/tests/build_avr.sh "$STIER_GENERATED_LIBS/ros_lib"
```

`STIER_ACTUATOR_OUTPUTS=1`은 생성 바이너리에 출력 허용 설정을 넣는다. 이 시험 스크립트는
`avrdude`나 업로드 명령을 호출하지 않는다. 생성 HEX는 자동 설치하지 않는다.
현 저장 파일의 기본값은 White ROS=0/human serial=1, Black ROS=1/human serial=0이다.
실제 ROS용 빌드에서는 양쪽 모두 ROS=1, human serial=0이어야 한다. 기본값을 몰래 바꾸지 않았다.

### 검증 결과

- 실제 스케치 호스트 시험 White/Black 통과: 메시지 직렬화/MD5, 정상 제동 후 출발,
  전진→감속→정지 확인→후진, 후진→정지→전진, 중립, 0 KPH, 속도 상한,
  signed feedback/MorA/Gear, EStop·해제, timeout, 잘못된 Gear, RC stop/interlock.
- Uno ROS 출력 금지: White Flash 18,658 B / Black 18,676 B; static SRAM 각 1,544 B.
- Uno ROS 출력 허용: White Flash 20,772 B / Black 20,790 B; static SRAM 각 1,551 B.
- SRAM 2,048 B 중 출력 허용 빌드의 정적 할당 후 잔여는 497 B다. 이는 **stack/heap
  여유의 실측 최저값이 아니다**. 실제 rosserial 협상·지속 통신 중 메모리 안정성 검증은 남아 있다.

## 차량에서 남은 확인

실제 학교 탑재본이 저장소와 동일했는지는 확인되지 않았다. 따라서 원인을 확정한 것이
아니라, 확인한 소스 불일치를 수정하고 재현 가능한 시험을 붙인 것이다.

휠을 지면에서 분리한 상태에서 적절한 차량 스케치/핀/ADC 설정, ROS mode 및 RC stop,
전진 시 speed/encoder 양수·후진 시 음수, 앞뒤 모터 방향, 조향 부호, EStop/통신 끊김을
확인해야 한다. 특히 **1 km/h에서 실제 모터가 출발하는지**, 제동 시 얼마나 굴러가는지,
기어 전환 후 바퀴 반응은 모의 시험으로 판정할 수 없다.
상위 주차 0.5 m/s 제한은 정수 KPH 내림으로 1 km/h 명령이 되며 이를 임의로 올리지 않았다.

하위 feedback EStop을 상위 명령 EStop으로 그대로 되먹이면 해제되지 않는 루프가 된다.
현재 통합 Control은 feedback EStop 동안 일반 brake를 보내고 외부 EStop 요청만 재전송한다.
오래된 상위 제어기와 혼용하지 않는다.
