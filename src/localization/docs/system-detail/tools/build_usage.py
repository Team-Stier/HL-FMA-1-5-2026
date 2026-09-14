#!/usr/bin/env python3
import pathlib,json
OUT=pathlib.Path(__file__).resolve().parents[1]
rows=[json.loads(s) for s in (OUT/'usage/qwen-calls.jsonl').read_text().splitlines()]
reviewed={('scripts/rddf_initializer_node.py',200),('scripts/sensor_timing_monitor.py',171),('config/time_sync.yaml',1),('config/tf_configuration.yaml',1),('config/imu_heading_calibration.yaml',1),('scripts/timing_core.py',1)}
reviews=[]
for r in rows:
 key=(r['source'].split('/src/localization/')[-1],r['start_line'])
 if key in reviewed:reviews.append({'source':r['source'],'start_line':r['start_line'],'end_line':r['end_line'],'reread_tokens':r['input_proxy'].get('source_tokens',0)})
gross=sum(r['input_proxy'].get('gross_reduction_tokens',0) for r in rows)
usage={'calls':len(rows),'local_prompt_tokens':sum(r['local'].get('prompt_tokens',0) for r in rows),'local_generated_tokens':sum(r['local'].get('generated_tokens',0) for r in rows),'gross_proxy':gross,'review_adjusted_proxy':gross-sum(r['reread_tokens'] for r in reviews),'reviewed_source_ranges':reviews,'description':'원문 대신 Qwen 요약 JSON을 읽었을 때의 입력량 차이를 o200k_base로 계산했습니다. 원문 재검토 6건을 추가 차감했습니다. 음수는 이 비교에서 입력이 오히려 늘었다는 뜻이며, 전체 OpenAI 사용량이나 청구 비용 절감을 측정한 값이 아닙니다.','rejected_claims':['초기화 서비스 실패 시 atomicity 보장: rollback 코드 없음','모든 센서가 PC 수신 stamp를 사용한다: GPS stamp_semantics는 별도','GPS 정적 TF enabled=false이므로 GPS 데이터 미사용: TF와 fusion 활성화는 별개','encoder yaw가 GNSS 보정의 입력이다: 원문은 엔코더 움직임 정보를 사용','timing_core 상단 import만으로 측정 이력 처리를 설명한 부분: 해당 범위에서 증명되지 않음']}
(OUT/'usage/summary.json').write_text(json.dumps(usage,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(usage,ensure_ascii=False,indent=2))
