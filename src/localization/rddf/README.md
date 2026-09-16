# Yongin RDDF catalogue

`yongin_<route>.csv` 19개는 Localization과 경로 추종이 사용하는 전역 RDDF다.
신호등 정지선과 경사로 흰 선은 새 주행 경로가 아니라 기존 RDDF 위의 의미
마커다. 따라서 단일 점 정지선 CSV를 Route Provider가 경로로 읽지 않도록
`yongin_mission_landmarks.json`에 투영 결과를 분리해 두었다.

## 확정된 마커

| 전역 RDDF | 마커 | RDDF 누적거리 s (m) | 원본과 RDDF 투영 오차 |
|---|---|---:|---:|
| `1_left` | 경사로 앞 흰 선 | 25.929471 | 0 m |
| `1_left` | 경사로 뒤 흰 선 | 36.128101 | 0 m |
| `1_left` | 정차 목표(두 선의 RDDF 중앙) | 31.028786 | 파생값 |
| `1_right` | 경사로 앞 흰 선 | 25.768230 | 0 m |
| `1_right` | 경사로 뒤 흰 선 | 35.986894 | 0 m |
| `1_right` | 정차 목표(두 선의 RDDF 중앙) | 30.877562 | 파생값 |
| `2` | 신호등 1 정지선 | 58.029721 | 0.001411 m |
| `4` | 신호등 2 정지선 | 33.898126 | 0 m |
| `7` | 신호등 3 정지선 | 93.436841 | 0 m |

경사로 중앙은 양끝 ENU 좌표의 단순 평균이 아니라 해당 RDDF를 따른 누적거리
중앙이다. `state_manager/config/missions.json`에는 확정된 수치가 동일하게 반영돼
있다. 출처 좌표·GPS·전역 RDDF 인덱스·투영 오차는
`yongin_mission_landmarks.json`이 기록한다.
경사로 원본 22점 전체와 전역 RDDF의 최대 이격은 왼쪽 0.0318m, 오른쪽
0.0272m이며 양끝 흰 선 점은 전역 RDDF 위에 일치한다.

2·4·7번의 `intersection_exit_s`는 제공된 파일에 없어 설정하지 않았다. 임의로
교차로 출구를 추정하지 않으며, 출구 마커가 추가되기 전까지 신호 구간의 실차
주행 검증은 fail-closed 상태를 유지한다.

## 시각화 재생성

![용인 전역 RDDF와 확정 미션 마커](../../state_manager/docs/yongin-mission-landmarks.png)

정적 그림은 전역 RDDF CSV와 실행 설정을 직접 읽어 생성한다.

```bash
python3 src/state_manager/tools/render_yongin_mission_landmarks.py
```

실차 RViz에서는 `/mission/markers`의 빨간 횟선이 신호 정지선, 흰색 횟선이
경사로 앞·뒤 선, 주황색 구가 경사로 정차 목표다.
