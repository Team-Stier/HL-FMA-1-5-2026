# Vehicle Safety Gate

제어기의 `/pure_pursuit/raw_drive`와 실제 차량 입력 `/erp42_serial/drive` 사이에서
매 0.05초 명령을 검사한다. 기본 실행은 `/vehicle_safety/preview_drive`에만 결과를
발행하며 **실차 토픽에는 publisher를 만들지 않는다**.

```bash
roslaunch vehicle_safety vehicle_safety.launch
```

실차 연결은 `enable_vehicle_output:=true`를 명시해야 한다. 이 경우 Controller 및 다른
노드는 `/erp42_serial/drive`에 직접 발행하지 않고 Gate만 실제 차량 명령을 발행해야 한다.
차량 토픽에 다른 publisher가 있으면 이 소프트웨어만으로 그 명령을 차단할 수 없다.

| 입력 | 정상 조건 |
| --- | --- |
| `/mission/state` | 유효한 미션, 정지/종료 아님, 전진, 유효한 속도 제한 |
| `/mission/safety` | LiDAR 유효, 정지 요청 없음 |
| `/path/selector_status` | ready, decision/route/source/direction 모두 현재 미션과 일치 |
| `/molit/localization/valid` | 계속 수신되는 true heartbeat |
| `/pure_pursuit/raw_drive` | 최근 0.25초 내 받은 유효한 DriveCmd |

나머지 입력은 기본 0.5초 내 stamp와 수신 시각이 필요하다. 미래 stamp 허용은 0.05초다.
선택 경로와 LiDAR 안전 승인의 `path_fingerprint`도 비어 있지 않고 정확히 같아야 한다.
같은 미션 요청에서 경로 A를 B로 다시 계획하면 B를 실제 검사한 승인 및 새 raw 명령이 필요하다.
어떤 입력이라도 없거나 오래되면 `KPH=0, Deg=0, brake=1`을 계속 발행한다. 상태/경로 변경,
일시적인 invalid, 센서 끊김, 시각 역행 이후에는 모든 입력이 정상인 시점에 **새로 수신한**
raw 명령이 있어야 재개한다. 정지 전 캐시된 raw 명령이 복구와 동시에 살아나지 않는다.

속도는 `floor(speed_limit_mps * 3.6)` km/h 이하, 조향은 펌웨어 범위인 ±25도로 제한한다.
현재 DriveCmd는 `uint16 KPH`만 갖고 기어가 없으므로, 후진 방향 `-1`은 항상
`REVERSE_INTERFACE_UNAVAILABLE`로 정지시킨다. T자/평행주차 후진 RDDF는 계획 및 RViz 확인용으로
표현할 수 있지만, 실제 후진 주행에는 차량 메시지·펌웨어·피드백의 별도 확장이 필요하다.
1 km/h 미만 속도 제한도 현재 정수 인터페이스로 표현할 수 없어 정지한다.

`/vehicle_safety/status`는 allowed, reason, 출력 허용 설정, 최종 명령을 담은 JSON String이다.
이 Gate는 인지된 SafetyStatus를 사용하며, LiDAR로 보이지 않는 연석이나 물리적 제동 성능까지
보장하지 않는다. ROS 시간이 멈추거나 프로세스가 종료되면 Arduino의 명령 타임아웃도 필요하다.

```bash
python3 -m unittest discover -s src/vehicle_safety/test -v
```
