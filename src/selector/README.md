# State-aware Selector

State Manager가 요청한 `path_mode`의 경로만 검증해서 `/path/final`로 보낸다.
미션 판단이나 주차 분기 선택은 하지 않는다.

| mode | 입력 (`planning_interfaces/PlannedPath`) |
| --- | --- |
| `RDDF` | `/path/rddf` |
| `LOCAL` | `/path/local` |
| `PARKING` | `/path/park` |

`/mission/state`와 경로의 `decision_id`, `route_name`, `direction`이 모두 일치해야 한다.
메시지/수신 시각은 기본 0.5초 이내, 미래 허용치는 0.05초다. Wrapper와 Path의
header stamp가 필요하며, 두 header와 모든 PoseStamped의 frame은 `map`이어야 한다.
경로는 2개 이상의 유한 좌표, 정규화 quaternion, 기본 최대 2m 간격을 요구한다.
경로 Pose의 yaw와 진행 방향은 Selector에서 중복 검사하지 않는다.
조건 미충족 시 빈 Path로 이전 경로를 지우며 `/path/selector_status`의 `ready=false`를
발행한다. LOCAL/PARKING 경로가 없으면 RDDF로 자동 복귀하지 않는다.

`PathStatus.ready`는 경로 형상의 준비 여부다. 미션이 준비 대기 중 invalid/stop 상태여도
요청이 일치하는 경로를 평가한다. 이렇게 해야 State Manager의 WAIT_PATH_READY가
준비 완료를 확인할 수 있다. `valid=false`인 동안 실제 `/path/final`은 비운다.
`valid=true, stop_requested=true`이면 경로는 유지하되 최종 Safety Gate가 차량을 정지시킨다.

`PathStatus.path_fingerprint`는 검증한 경로의 모든 좌표·자세·frame과 요청 정보를
묶은 SHA256이다. SafetyStatus의 승인 fingerprint와 일치해야 Gate가
주행을 허용한다. 같은 decision_id로 경로를 다시 계산해도 이전 경로의 안전 승인을
다른 형상의 경로에 재사용하지 않는다. Header와 수신 시각은 별도로 freshness를 검증하며
fingerprint에서는 제외한다. 같은 형상에 새 stamp만 붙였을 때 아직 유효한 안전 승인을
불필요하게 무효화하지 않기 위한 것이다.

새 요청을 받으면 Planner는 새 `decision_id`로 새 경로를 발행해야 한다. 숫자 ID만 바꾸어
과거 경로를 재포장하는 것은 피해야 하며 현재 route/direction에 맞게 계산해야 한다.
`legacy_mode`는 기본 false이고, true로 명시한 경우에만 기존 `/path/lidar` Path 단순
중계가 동작한다. 이 모드는 PathStatus를 제공하지 않아 새 Safety Gate의 주행을 허용하지 않는다.

```bash
roslaunch selector selector.launch
python3 -m unittest discover -s src/selector/test -v
```
