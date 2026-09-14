#!/usr/bin/env python3
import json,pathlib,re,hashlib,shutil
OUT=pathlib.Path(__file__).resolve().parents[1];TOOLS=OUT/'tools';DOCS=OUT.parent
model=json.loads((OUT/'model.json').read_text());model['usage']=json.loads((OUT/'usage/summary.json').read_text());model['diagrams']={}
for id in model['layers']:
 html=(OUT/(id+'.html')).read_text();receipt=json.loads((OUT/(id+'.delivery.json')).read_text())
 model['diagrams'][id]=re.search(r'<svg\b[\s\S]*?</svg>',html).group()
# Runtime navigation wraps the extracted Archify SVGs; original delivered files remain byte-identical.
html=(TOOLS/'explorer.template.html').read_text().replace('/*__CSS__*/',(TOOLS/'explorer.css').read_text()).replace('/*__JS__*/',(TOOLS/'explorer.js').read_text()).replace('/*__DATA__*/',json.dumps(model,ensure_ascii=False,separators=(',',':')).replace('<','\\u003c'))
(DOCS/'system-architecture.html').write_text(html)
(DOCS/'localization-five-stage.html').write_text(html)
(OUT/'explorer-build.json').write_text(json.dumps({'artifact':'../system-architecture.html','sha256':hashlib.sha256(html.encode()).hexdigest(),'bytes':len(html.encode()),'diagram_type':'architecture','archify_diagrams':len(model['diagrams']),'navigation':'custom wrapper with unchanged Archify source SVG geometry','compatibility_entry':'../localization-five-stage.html'},indent=2)+'\n')
print('Built standalone explorer:',len(html.encode()),'bytes')
