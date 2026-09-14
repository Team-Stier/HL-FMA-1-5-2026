#!/usr/bin/env python3
"""Offline browser verification for navigation, sources, and viewport containment."""
import hashlib,json,pathlib
from playwright.sync_api import sync_playwright
OUT=pathlib.Path(__file__).resolve().parents[1];FILE=OUT.parent/'system-architecture.html'
model=json.loads((OUT/'model.json').read_text());checks=[];errors=[];measurements=[];shots=[]
with sync_playwright() as p:
 browser=p.chromium.launch(executable_path='/home/paik/.local/bin/google-chrome',headless=True,args=['--no-sandbox','--disable-gpu'])
 page=browser.new_page(viewport={'width':1440,'height':900})
 page.on('pageerror',lambda e:errors.append(str(e)))
 requests=[];page.on('request',lambda r:requests.append(r.url))
 page.goto(FILE.as_uri());page.wait_for_selector('#diagram [data-node-id="localization"]')
 # The exact interaction requested: click a subsystem to replace the architecture.
 page.locator('#diagram [data-node-id="localization"]').click();page.wait_for_function("document.querySelector('h1').textContent==='Localization 내부'")
 page.locator('#diagram [data-node-id="motion"]').click();page.wait_for_function("document.querySelector('h1').textContent==='IMU·엔코더와 Local EKF'")
 page.locator('#diagram [data-node-id="encoder"]').focus();page.keyboard.press('Enter');page.wait_for_function("document.querySelector('h1').textContent==='엔코더 검사·Twist 변환'")
 page.locator('#diagram [data-node-id="encoder0"]').click();page.wait_for_selector('#modal[open]');assert 'Header가 없는' in page.locator('#modal-body').inner_text()
 page.locator('.file-button').first.click();page.wait_for_selector('.code .highlight');assert 'validate_measurement' in page.locator('.code .highlight').inner_text()
 source_hash=page.evaluate('location.hash');page.reload();page.wait_for_selector('.code .highlight');assert 'validate_measurement' in page.locator('.code .highlight').inner_text();checks.append('offline root → localization → motion → encoder → function source; source deep link reload')
 page.locator('#close').click();page.wait_for_selector('#modal',state='hidden');page.locator('#breadcrumbs [data-crumb="system"]').click();page.wait_for_function("document.querySelector('h1').textContent==='전체 시스템'")
 page.locator('#search').fill('validate_measurement');page.wait_for_selector('#search-results button');page.locator('#search-results button').last.click();page.wait_for_selector('#modal[open]');checks.append('global source/function search')
 page.keyboard.press('Escape');page.wait_for_selector('#modal',state='hidden');page.locator('#components').click();page.wait_for_selector('.component-list');page.locator('[data-drill="localization"]').click();page.wait_for_function("document.querySelector('h1').textContent==='Localization 내부'");page.locator('#back').click();page.wait_for_selector('.component-list');checks.append('component inspector, drill-down, browser back, Escape')
 page.locator('#close').click()
 # Every authored node must activate its exact target or open its own inspector.
 for id,L in model['layers'].items():
  for n in L['nodes']:
   page.goto(FILE.as_uri()+'#view='+id);page.wait_for_selector('#diagram [data-node-id="'+n['id']+'"]')
   page.locator('#diagram [data-node-id="'+n['id']+'"]').click()
   if n['target']:page.wait_for_function('(title)=>document.querySelector("h1").textContent===title',arg=model['layers'][n['target']]['title'])
   else:page.wait_for_selector('#modal[open]');assert page.locator('#modal-title').inner_text()==n['label']
 checks.append('all %d diagram nodes activate the authored target or inspector' % sum(len(L['nodes']) for L in model['layers'].values()))
 # At all desktop viewports, all chapters fit in normal document flow in both themes.
 for width,height in [(1440,900),(1600,1000),(1920,1080),(2048,1320)]:
  page.set_viewport_size({'width':width,'height':height})
  for theme in ['dark','light']:
   for id,L in model['layers'].items():
    page.goto(FILE.as_uri()+'#view='+id);page.evaluate('(t)=>setTheme(t)',theme)
    m=page.evaluate('''()=>({iw:innerWidth,ih:innerHeight,sw:document.documentElement.scrollWidth,sh:document.documentElement.scrollHeight})''');m.update(view=id,theme=theme)
    m['pass']=m['sw']<=width and m['sh']<=height;measurements.append(m)
    if width in [1440,2048] and id in ['system','localization','encoder','rddf','gps','safety']:
     name=f'explorer.{id}.{width}x{height}.{theme}.png';page.screenshot(path=str(OUT/name));shots.append(name)
 # Narrow screen allows vertical scrolling but never horizontal clipping.
 page.set_viewport_size({'width':390,'height':844});page.goto(FILE.as_uri());m=page.evaluate('({iw:innerWidth,sw:document.documentElement.scrollWidth})');assert m['sw']<=m['iw'];checks.append('390px mobile horizontal containment')
 page.set_viewport_size({'width':1440,'height':900});page.goto(FILE.as_uri());page.locator('#usage').click();page.wait_for_selector('#modal[open]');assert '2,618' in page.locator('#modal-body').inner_text();assert '-454' in page.locator('#modal-body').inner_text();checks.append('usage record displays measured and proxy quantities distinctly')
 assert not [u for u in requests if u.startswith(('https:','http:'))],requests
 browser.close()
receipt={'evidenceKind':'custom-explorer-automated-browser','artifact':{'path':str(FILE),'sha256':hashlib.sha256(FILE.read_bytes()).hexdigest(),'bytes':FILE.stat().st_size},'checks':checks,'measurements':measurements,'screenshots':shots,'errors':errors,'offline':True,'visualReview':'pending','status':'pass' if not errors and all(m['pass'] for m in measurements) else 'fail'}
(OUT/'explorer-browser-check.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'status':receipt['status'],'checks':len(checks),'measurements':len(measurements),'failures':[m for m in measurements if not m['pass']],'errors':errors},ensure_ascii=False))
raise SystemExit(0 if receipt['status']=='pass' else 1)
