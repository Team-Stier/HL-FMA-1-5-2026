# 하얀차 현장 실행 — 2026-09-18

이 문서는 업로드 직후 기록이다. 이후 S 시험에서 USB 장애와 지도 배치 불일치를 확인했다.
최신 수정본과 다른 PC 실행은 [인계 안내](pc_transfer_20260918.md)를 사용한다.

## 완료 상태

- 사용자 확인 후 구동 전원 OFF 상태에서 하얀차 Arduino 업로드 완료.
- ATmega328P, 20,734 bytes 쓰기/재읽기 검증 완료. ROS 통신 57600 baud.
- 기존 flash 백업과 올린 HEX:
  `/home/choiminho/.local/share/stier/firmware-backups/white-20260918.i0csVZ/`
- 실제 피드백 수신 확인: speed=0, encoder=0, alive 증가. 당시 MorA=0(수동 모드).
- 실제 주행 워크스페이스 전체 빌드 완료. 이전의 메시지 fixture 빌드가 아님.
- 내가 실차 주행 명령을 보내지는 않았다. 시험용 serial 연결/master는 종료했다.

## 확인한 연결

| 장치 | 현재 포트 |
|---|---|
| 하얀차 Arduino | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` → ttyUSB0 |
| RPLIDAR S2 | `/dev/lidar` → ttyUSB1 |
| Xsens IMU | `/dev/imu` → ttyUSB2 |
| u-blox GPS | `/dev/gps` → ttyACM0 |

**Arduino에 ttyACM0을 지정하면 GPS 포트를 열게 된다.** 아래의 Arduino 고정 경로를 쓴다.
USB 포트를 재연결하면 tty 번호가 바뀔 수 있으나 by-id 경로는 장치 기준이다.

## 학교 라바콘/RDDF·주차 시험

다음은 센서와 Control을 시작해 실제 구동 명령을 발행하는 명령이다.
차량 방향·시험 공간·RC 조작 준비를 확인한 뒤 사용한다. 보드가 수동 모드인 동안은
기존 Control이 주행을 허가하지 않는다. RC를 ROS 모드로 전환하면 조건 충족 시 움직일 수 있다.

학교 S 배치, 주차 좌우는 설정 파일 기본값(둘 다 left)을 사용한다. 실제 배치에 따라
`t_parking_side:=right` 또는 `parallel_parking_side:=right`를 덧붙인다.

```bash
cd /home/choiminho/바탕화면/HL-FMA2026-0917
bash run.sh hongik_s_test:=true \
  arduino_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  enable_camera:=false start_traffic_light:=false \
  target_speed_kph:=5
```

설정 파일의 절대 상한은 요청한 **15km/h** 그대로다. 위 명령은 업로드 후 첫 동작 확인을
위해 이번 실행 목표만 5km/h로 낮춘 예시다. 추종·감속·방향을 확인한 뒤 목표를 높이는
시험에서는 마지막 인자를 `target_speed_kph:=15`로 바꾼다. 곡률/미션 감속은 계속 적용된다.
기존 고정 LD2 비교는 `lookahead_m:=2`를 추가한다. 생략 시 기본 적응형 LD2~4를 쓴다.

일반 학교 배치라면 `hongik_s_test:=true` 대신 `hongik_test:=true` 하나만 사용한다.
두 배치는 경로의 지도상 위치·방향이 다르다. RViz의 RDDF와 실제 시험 배치가 일치하는지
확인하고 초기화 UI에서 차량 위치·진행 방향에 맞는 시작점을 선택한다.
라바콘 회피는 3번 `3_s-static-obstacle`/`LOCAL` 구간에서 시험한다.

현재 `/dev/cam`이 없고 `torch`/`ultralytics`도 설치되어 있지 않아 위 명령은 카메라와
신호 인식을 끈다. **실제 신호 인식/골인 차선 판독 검증용이 아니다.** 신호 구간에 들어가면
미인식 연속 정차 20초 후 출발 정책이 적용된다. 카메라·모델 실행환경을 갖추기 전에는
신호에 반응하는 시험이라고 간주하지 않는다. 검출기는 기존 NumPy 대체 구현으로 실행
가능하며, SciPy 가속을 쓰려면 `sudo apt install python3-scipy`를 설치한다.
GPS RTK 보정은 NTRIP 네트워크 연결 상태도 확인한다.

## 별도 터미널에서 확인

```bash
cd /home/choiminho/바탕화면/HL-FMA2026-0917
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo -n 1 /erp42_serial/feedback
```

ROS 모드에서는 `MorA: 1`이어야 한다. 전진·후진은 `Gear`와 속도를 함께 확인한다.
필요한 상태를 각각 다른 터미널에서 확인한다.

```bash
rostopic echo /control/state
rostopic echo /mission/state
rostopic echo /path/selector_status
```

중단 시 RC 정지/수동 전환을 먼저 사용하고, 실행 터미널에서 Ctrl+C로 종료한다.
현재 PWM=0 정지 명령만으로 물리적인 경사로 밀림 방지까지 보장하지 않는다.
이 작업에서 커밋·푸시는 하지 않았다.
