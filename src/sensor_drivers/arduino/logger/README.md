# CAN CSV logger

T870CanCsvLogger 아두이노의 USB 텍스트 출력(115200 baud)을 저장한다.
ROS 토픽은 발행하지 않으며 roslaunch가 실행과 종료만 관리한다.

`config/logger.yaml`에서 `port`, `baud`, `output_root`, `name`을 수정한다.
기본 포트 `/dev/ttyUSB3`는 마지막으로 확인한 CH340 번호이며 현재 연결 상태를
보장하지 않는다. `/dev/serial/by-id/...` 경로도 지정할 수 있다.
동일 포트를 rosserial, 시리얼 모니터, 다른 캡처 프로그램과 함께 열지 않는다.

```bash
source ~/HL-FMA2026-suhyeon/devel/setup.bash
roslaunch sensor_bringup sensors.launch
# 로거만 실행
roslaunch can_logger logger.launch
# 로거 제외
roslaunch sensor_bringup sensors.launch enable_can_logger:=false
```

`~/bags/can/날짜_시간_sensors/`에 다음 파일을 저장한다.

- `can_frames.csv`: 수신 시각, CAN ID, DLC, 원본 데이터
- `raw_can.csv`: 속도, 엔코더, 조향, 고장 등의 텔레메트리
- `run_metadata.json`: 종료 시 수신 개수와 오류 기록

CSV는 행마다 flush한다. Ctrl+C 시 파일을 닫고 메타데이터를 저장한다.
장치가 없거나 분리되면 오류를 출력하고 종료하며 자동 재시작하지 않는다.
일부 CAN 프레임만 있거나 텔레메트리가 없으면 받은 원본은 보존하되 종료 코드는 실패다.
자동 진단 및 DBC 처리는 포함하지 않는다. 데이터는 rosbag과 별도 CSV로 저장된다.

## 원본 및 변경

원본: Team-Stier/Mando, `feature/t870-can-integration`, 커밋
`533d704b87f3fcb2ccb0494adb3db2cafb61330a`의
`arduino/BROON_T870_Uno_Controller/logger/capture_and_diagnose.py`.

원본 캡처·파싱·메타데이터 로직을 가져와 Python 3.8의 with 문법에 맞췄다.
별도 진단 도구 호출을 제거하고, 실행 폴더명에 마이크로초를 추가했으며,
시리얼 exclusive 옵션을 켰다. DTR/RTS 비활성화 및 시작 헤더를 놓쳤을 때의
행 복원 처리를 유지한다. 저장소 밖의 Mando checkout에는 의존하지 않는다.
