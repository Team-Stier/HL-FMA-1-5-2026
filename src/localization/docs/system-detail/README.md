# 전체 시스템 아키텍처 탐색기

[탐색기 열기](../system-architecture.html). 기존 [localization-five-stage.html](../localization-five-stage.html) 진입점도 같은 탐색기를 연다.

전체 시스템 → Localization 또는 다른 영역 → 내부 처리·판정 → 함수/설정 → 소스 원문으로 탐색한다. 노드 클릭, 구성요소 설명, 전체 검색, 브라우저 뒤로가기, breadcrumb, 깊은 링크, 밝은/어두운 테마를 제공한다. 25개 Archify 아키텍처, 127개 구성요소, 90개 소스 스냅샷을 포함한다.

본문과 별도 탐색 UI는 한국어다. Archify 원본의 고정 Viewer UI와 `<html lang>`는 스킬이 지원하는 English fallback을 사용한다. 탐색기는 Archify가 검증·생성한 SVG를 포함하는 별도 UI이며 자체 브라우저 검사로 확인한다. Archify의 원본 HTML·spec·SHA-256 receipt는 별도로 유지한다. 탐색기에서 글자와 관계 마스크의 표시 크기는 조절하지만 노드 배치·경로의 의미와 연결은 원본을 따른다. PNG/SVG 등의 표준 내보내기는 각 화면의 ‘Archify 원본’ 링크에서 이용한다.

README의 설계, 현재 파일에 구현된 연결, 미구현 뼈대를 구분한다. 실제 ROS 노드 실행·실차 동작을 측정한 문서가 아니다. GPS timestamp, RDDF 시작 조건, 두 EKF의 관측 소유권, Supervisor의 프로세스 구성, Output Gate의 초기화 준비 조건을 포함한다. 소스의 인증 설정 줄은 원문 스냅샷에서 생략한다. SHA-256은 생략 전 실제 파일의 해시이며 스냅샷 문서 자체의 해시와 구분한다.

- [Qwen 사용량과 하드웨어 튜닝](usage/REPORT.md)
- [소스 해시](source-manifest.json)
- [Archify delivery receipts](delivery-summary.json)
- [Archify 자동 브라우저 검사](browser-summary.json)
- [탐색기 기능·화면 검사](explorer-browser-check.json)
- [통합 검증 기록](validation-summary.json)

`tools/build_model.py`가 근거와 스펙을 생성한다. `geometry-repairs.json`/`viewport-repairs.json`에는 검사에서 필요했던 좌표 보정만 저장한다. 각 spec을 Archify `validate` 및 `deliver`로 검증한 후 `tools/build_usage.py`, `tools/build_explorer.py`를 실행한다. `build_explorer.py`는 소스와 SVG를 인라인으로 묶어 파일 하나로 오프라인 탐색을 지원한다. 원본 Archify HTML을 여는 링크와 별도 검증 보고서까지 공유하려면 `system-detail/` 폴더도 같이 전달한다.

이전 5단계·코드 상세 진입점은 현재 탐색기로 연결합니다. 위치 추정은 IMU·엔코더·GPS를 사용하며 라이다는 스캔 수집·표시 기능입니다.
