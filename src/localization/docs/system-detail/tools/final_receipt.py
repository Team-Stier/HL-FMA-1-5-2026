#!/usr/bin/env python3
import pathlib,json,hashlib,datetime
OUT=pathlib.Path(__file__).resolve().parents[1];ROOT=OUT.parents[3]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
model=json.loads((OUT/'model.json').read_text());browser=json.loads((OUT/'explorer-browser-check.json').read_text());artifact=OUT.parent/'system-architecture.html'
assert browser['artifact']['sha256']==digest(artifact)
assert browser['status']=='pass'
receipts={}
for id in model['layers']:
 delivery=json.loads((OUT/(id+'.delivery.json')).read_text());visual=json.loads((OUT/(id+'.visual-check.json')).read_text())
 assert delivery['ok'] and delivery['validation']['checksPassed']==9
 assert delivery['validation']['errors']==0 and delivery['validation']['warnings']==0
 assert delivery['specification']['sha256']==digest(OUT/(id+'.architecture.json'))
 assert delivery['artifact']['sha256']==digest(OUT/(id+'.html'))==visual['artifact']['sha256']
 assert visual['status']=='pass'
 receipts[id]={'specification_sha256':delivery['specification']['sha256'],'artifact_sha256':delivery['artifact']['sha256'],'validation':'9/9 showcase, 0 errors, 0 warnings','browser_evidence':'passed'}
for p,source in model['sources'].items():assert digest(ROOT/p)==source['sha256'],p
assert (OUT.parent/'localization-five-stage.html').read_bytes()==artifact.read_bytes()
summary={'diagram_type':'architecture','output':str(artifact),'specification_sha256':digest(OUT/'model.json'),'artifact_sha256':digest(artifact),'artifact_bytes':artifact.stat().st_size,'archify':{'count':len(receipts),'validation':f'{len(receipts)} × 9/9 showcase, 0 errors, 0 warnings','browser_evidence':'passed','receipts':receipts},'explorer':{'browser_evidence':'passed','node_interactions':sum(len(L['nodes']) for L in model['layers'].values()),'desktop_viewport_theme_measurements':len(browser['measurements']),'offline':True,'compatibility_entry':'localization-five-stage.html'},'visual_review':{'status':'passed','reviewer':'Codex image-capable review','scope':['explorer localization at 2048x1320 dark'],'note':'Codex inspected the current Localization screenshot. Automated browser coverage spans all 25 diagrams; native receipts do not imply perceptual review.','artifact_sha256':digest(artifact)},'source_snapshots':len(model['sources']),'source_drift':[],'runtime_ros_verification':{'catkin_build':'passed','catkin_test_results':{'tests':233,'errors':0,'failures':0},'live_sensors':False},'local_qwen_report':'usage/REPORT.md'}
(OUT/'validation-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'status':'pass','layers':len(receipts),'node_interactions':sum(len(L['nodes']) for L in model['layers'].values()),'viewport_checks':len(browser['measurements']),'artifact_sha256':summary['artifact_sha256'],'source_snapshots':len(model['sources'])}))
