'use strict';
const DATA=JSON.parse(document.getElementById('architecture-data').textContent), $=id=>document.getElementById(id), layers=DATA.layers;
const escapeHTML=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let active='system', restoreFocus=null;
function setTheme(theme){document.documentElement.dataset.theme=theme;$('theme').textContent=theme==='dark'?'☀ 밝게':'☾ 어둡게';$('theme').setAttribute('aria-label',theme==='dark'?'밝은 테마로 전환':'어두운 테마로 전환');try{localStorage.setItem('hl-architecture-theme',theme)}catch{}}
try{setTheme(localStorage.getItem('hl-architecture-theme')||'dark')}catch{setTheme('dark')}
$('theme').onclick=()=>setTheme(document.documentElement.dataset.theme==='dark'?'light':'dark');
function route(view,extra={}){const p=new URLSearchParams({view,...extra});const next='#'+p;if(location.hash===next)renderRoute();else location.hash=next;}
function chain(id){const a=[];while(id){a.unshift(layers[id]);id=layers[id].parent;}return a;}
const ROOT_NAV=['system','sensors','localization','perception','planning','control','interfaces','operations'];
const NAV_LABELS={system:'전체 시스템',sensors:'센서 드라이버',localization:'Localization',perception:'인지',planning:'경로 계획·선택',control:'차량 제어',interfaces:'인터페이스',operations:'TF·실행·관측',motion:'IMU·엔코더 처리',gps:'GPS 위치 승인',lidar:'LiDAR 스캔 수집·표시',filters:'두 EKF',rddf:'RDDF 초기화',timing:'센서 시각',safety:'상태·복구·출력',tf:'TF 프레임',viewer:'Viewer·시각화',launches:'Launch 구성'};
function renderTree(){
 const ancestors=chain(active).map(l=>l.id),ids=[];
 for(const id of ROOT_NAV){ids.push(id);if(id==='localization'&&ancestors.includes(id)&&!ancestors.includes('operations'))ids.push('motion','gps','filters','rddf','timing','safety');if(id==='operations'&&ancestors.includes(id))ids.push('tf','launches','viewer')}
 $('tree').innerHTML=ids.map(id=>`<button data-view="${id}" class="${ROOT_NAV.includes(id)?'':'sub'}">${escapeHTML(NAV_LABELS[id]||layers[id].title)}</button>`).join('');
}
$('tree').addEventListener('click',e=>{const b=e.target.closest('[data-view]');if(b)route(b.dataset.view)});
$('counts').innerHTML=`<strong>${Object.keys(layers).length}</strong>개 아키텍처<br><strong>${Object.values(layers).reduce((s,l)=>s+l.nodes.length,0)}</strong>개 구성요소 · <strong>${Object.keys(DATA.sources).length}</strong>개 소스<br><br>하위 구조 → 함수·조건 → 설정과 원문<br>파일을 이동해도 탐색·소스 확인은 오프라인으로 동작합니다.`;
function renderRoute(){
 const p=new URLSearchParams(location.hash.slice(1));active=layers[p.get('view')]?p.get('view'):'system';
 const L=layers[active];renderTree();if($('modal').open)$('modal').close();
 $('view-title').textContent=L.title;document.title=L.title+' · HL-FMA2026 아키텍처';
 $('breadcrumbs').innerHTML=chain(active).map(l=>`<button data-crumb="${l.id}">${escapeHTML(l.title)}</button>`).join('<span aria-hidden="true">›</span>');
 $('breadcrumbs').querySelectorAll('button').forEach(b=>b.onclick=()=>route(b.dataset.crumb));
 document.querySelectorAll('#tree button').forEach(b=>{b.classList.toggle('active',b.dataset.view===active);if(b.dataset.view===active)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current')});
 $('level').textContent=`LEVEL ${chain(active).length-1} · ${L.nodes.length}개 구성요소 · ${L.edges.length}개 관계`;
 $('original').href=`system-detail/${active}.html`;$('diagram').innerHTML=DATA.diagrams[active];
 const svg=$('diagram').querySelector('svg');svg.setAttribute('lang','ko');svg.setAttribute('aria-label',L.title);svg.removeAttribute('aria-labelledby');
 for(const g of svg.querySelectorAll('g[data-edge-id]')){
  const text=g.querySelector('text'),mask=g.querySelector('rect');if(text&&mask){const box=text.getBBox();mask.setAttribute('x',box.x-5);mask.setAttribute('y',box.y-2);mask.setAttribute('width',box.width+10);mask.setAttribute('height',box.height+4)}
 }
 for(const node of L.nodes){
  const g=svg.querySelector(`[data-node-id="${node.id}"]`);if(!g)throw new Error('Missing rendered node '+node.id);
  g.setAttribute('aria-label',`${node.label}: ${node.target?'내부 아키텍처 열기':'함수·소스 설명 열기'}`);g.removeAttribute('aria-pressed');
  if(/설계|뼈대/.test(node.state))g.classList.add('node-planned');
  const hint=document.createElementNS('http://www.w3.org/2000/svg','text');hint.setAttribute('x',node.pos[0]+125);hint.setAttribute('y',node.pos[1]+79);hint.setAttribute('text-anchor','middle');hint.setAttribute('class','drill-hint');hint.textContent=node.target?'내부 구조 열기  ↗':'함수 · 설정 · 소스  ↗';g.appendChild(hint);
  const go=()=>node.target?route(node.target):route(active,{node:node.id});g.addEventListener('click',go);g.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();go()}});
 }
 $('note').textContent=L.note||'각 노드에서 역할·입출력·설정값과 줄 번호가 있는 소스 원문을 확인할 수 있습니다.';
 if(p.get('file')&&DATA.sources[p.get('file')])showSource(p.get('file'),Number(p.get('line'))||1);
 else if(p.get('node')){const node=L.nodes.find(n=>n.id===p.get('node'));if(node)showNode(node)}
 else if(p.get('panel')==='components')showComponents();else if(p.get('panel')==='usage')showUsage();
}
function openModal(title,kicker,html){restoreFocus=document.activeElement;$('modal-title').textContent=title;$('modal-kicker').textContent=kicker;$('modal-body').innerHTML=html;if(!$('modal').open)$('modal').showModal();$('modal').scrollTop=0;}
function closeModal(){route(active);if(restoreFocus&&restoreFocus.isConnected)restoreFocus.focus()}
$('close').onclick=closeModal;$('modal').addEventListener('cancel',e=>{e.preventDefault();closeModal()});$('modal').addEventListener('click',e=>{if(e.target===$('modal')){const r=$('modal').getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)closeModal()}});
function fileButton(source){const f=DATA.sources[source.path];return `<button class="file-button" data-file="${escapeHTML(source.path)}" data-line="${source.line}">${escapeHTML(source.path)}:${source.line}<small>${f.line_count}줄 · SHA-256 ${f.sha256.slice(0,16)}…${f.redacted_lines.length?' · 인증 설정 줄 생략':''}</small></button>`;}
function bindFiles(){document.querySelectorAll('[data-file]').forEach(b=>b.onclick=()=>route(active,{file:b.dataset.file,line:b.dataset.line||1}));}
function showNode(node){
 const L=layers[active],related=L.edges.filter(e=>e.from===node.id||e.to===node.id);
 let html=`<p class="summary">${escapeHTML(node.summary)}</p><span class="pill">${escapeHTML(node.state)}</span>`;
 if(node.target)html+=` <button id="drill">내부 아키텍처 열기 ↗</button>`;
 if(node.io.length)html+='<h3>공개 입출력</h3><p>'+node.io.map(escapeHTML).join('<br>')+'</p>';
 if(related.length)html+='<h3>이 화면에서 확인한 관계</h3><div class="relations">'+related.map(e=>`<button data-neighbor="${e.from===node.id?e.to:e.from}">${escapeHTML(L.nodes.find(n=>n.id===e.from).label)} → ${escapeHTML(L.nodes.find(n=>n.id===e.to).label)} · ${escapeHTML(e.label)}${e.variant==='dashed'?' [설계]':''}</button>`).join('')+'</div>';
 html+='<h3>소스 근거와 전체 함수 목록</h3><div class="files">'+node.sources.map(fileButton).join('')+'</div>';
 for(const ref of node.sources){const f=DATA.sources[ref.path];if(f.params.length)html+=`<details><summary>${escapeHTML(ref.path.split('/').pop())} · 설정 ${f.params.length}개</summary><div class="table-wrap"><table><thead><tr><th>설정 키</th><th>현재 값</th></tr></thead><tbody>${f.params.map(([k,v])=>`<tr><td>${escapeHTML(k)}</td><td>${escapeHTML(JSON.stringify(v))}</td></tr>`).join('')}</tbody></table></div></details>`;else if(f.functions.length)html+=`<details><summary>${escapeHTML(ref.path.split('/').pop())} · 함수/클래스 ${f.functions.length}개</summary><div class="functions">${f.functions.map(fn=>`<button data-file="${escapeHTML(ref.path)}" data-line="${fn.line}">L${fn.line} ${escapeHTML(fn.label)}</button>`).join('')}</div></details>`;}
 openModal(node.label,L.title+' / '+node.sublabel,html);bindFiles();if($('drill'))$('drill').onclick=()=>route(node.target);document.querySelectorAll('[data-neighbor]').forEach(b=>b.onclick=()=>route(active,{node:b.dataset.neighbor}));
}
function showSource(path,line){const source=DATA.sources[path];line=Math.min(source.line_count,Math.max(1,line));openModal(path.split('/').pop(),'소스 스냅샷 · '+path,`<p class="muted">현재 작업 파일의 정적 스냅샷 · ${source.line_count}줄${source.redacted_lines.length?' · 인증 관련 설정 줄 생략':''}<br>SHA-256 <code>${source.sha256}</code></p><button id="source-back">← 구성요소로 돌아가기</button><div class="code" aria-label="줄 번호가 있는 소스">${source.text.split('\n').map((text,i)=>`<span class="line ${i+1===line?'highlight':''}" id="source-line-${i+1}" data-line="${i+1}">${escapeHTML(text)||' '}</span>`).join('')}</div>`);$('source-back').onclick=()=>history.back();requestAnimationFrame(()=>document.getElementById('source-line-'+line)?.scrollIntoView({block:'center'}));}
function showComponents(){openModal(layers[active].title,'구성요소 목록 · 역할과 하위 구조',`<div class="component-list">${layers[active].nodes.map(n=>`<article><h3>${escapeHTML(n.label)}</h3><span class="pill">${escapeHTML(n.state)}</span><p>${escapeHTML(n.summary)}</p><button data-detail="${n.id}">함수·설정·소스</button>${n.target?`<button data-drill="${n.target}">내부 구조 ↗</button>`:''}</article>`).join('')}</div>`);document.querySelectorAll('[data-detail]').forEach(b=>b.onclick=()=>route(active,{node:b.dataset.detail}));document.querySelectorAll('[data-drill]').forEach(b=>b.onclick=()=>route(b.dataset.drill));}
function showUsage(){const u=DATA.usage;openModal('로컬 Qwen 사용·튜닝 기록','실측과 추정의 구분',`<div class="stat-row"><div class="stat"><b>${u.calls}</b>아키텍처 요약 요청</div><div class="stat"><b>${u.local_prompt_tokens.toLocaleString()}</b>로컬 입력 토큰 · 실측</div><div class="stat"><b>${u.local_generated_tokens.toLocaleString()}</b>로컬 생성 토큰 · 실측</div></div><p>${escapeHTML(u.description)}</p><table><tr><th>측정 항목</th><th>값</th></tr><tr><td>원문 → 요약 JSON 입력 감소량</td><td>${u.gross_proxy.toLocaleString()} 토큰 (o200k_base 대리 지표)</td></tr><tr><td>명시적 재검토 원문을 차감한 값</td><td>${u.review_adjusted_proxy.toLocaleString()} 토큰</td></tr><tr><td>실제 OpenAI 청구 절감량</td><td>측정 불가 · 추론·조율·검증·캐시·최종 작성 비용 미포함</td></tr><tr><td>기본 컨텍스트</td><td>8,192 · 긴 입력은 16,384 명시 선택</td></tr></table><h3>기록 파일</h3><p><a href="system-detail/usage/REPORT.md" target="_blank">사용량·방법·튜닝 보고서 ↗</a> · <a href="system-detail/usage/qwen-calls.jsonl" target="_blank">호출 원장 ↗</a> · <a href="system-detail/usage/context-benchmark.json" target="_blank">메모리·컨텍스트 실측 ↗</a></p><p class="muted">로컬 모델 토큰과 OpenAI 토큰은 같은 단위로 간주하지 않습니다. 설정 비교 시험은 별도 비용으로 기록하며 절감으로 세지 않습니다.</p>`)}
$('components').onclick=()=>route(active,{panel:'components'});$('usage').onclick=()=>route(active,{panel:'usage'});$('back').onclick=()=>{if(history.length>1)history.back();else route(layers[active].parent||'system')};
let searchTimer;
$('search').addEventListener('input',()=>{clearTimeout(searchTimer);searchTimer=setTimeout(()=>{const q=$('search').value.trim().toLowerCase();if(!q){$('search-results').innerHTML='';return}const results=[];for(const L of Object.values(layers)){if(L.title.toLowerCase().includes(q))results.push({view:L.id,title:L.title,sub:'아키텍처 화면'});for(const n of L.nodes){if([n.label,n.sublabel,n.summary,...n.io,...n.sources.map(s=>s.path)].join(' ').toLowerCase().includes(q))results.push({view:L.id,node:n.id,title:n.label,sub:L.title+' / '+n.sublabel})}}for(const [path,s] of Object.entries(DATA.sources)){for(const fn of s.functions){if(fn.label.toLowerCase().includes(q))results.push({view:active,file:path,line:fn.line,title:fn.label,sub:path+':'+fn.line})}}$('search-results').innerHTML=results.length?results.slice(0,24).map((r,i)=>`<button class="result" data-result="${i}">${escapeHTML(r.title)}<small>${escapeHTML(r.sub)}</small></button>`).join(''):'<div class="empty">일치하는 구성요소가 없습니다.</div>';$('search-results').querySelectorAll('button').forEach(b=>b.onclick=()=>{const r=results[Number(b.dataset.result)],extra={};for(const k of ['node','file','line'])if(r[k])extra[k]=r[k];$('search-results').innerHTML='';$('search').value='';route(r.view,extra)});},100)});
$('search').addEventListener('keydown',e=>{if(e.key==='Escape'){$('search-results').innerHTML='';$('search').value=''}if(e.key==='ArrowDown')$('search-results').querySelector('button')?.focus()});
document.addEventListener('keydown',e=>{if(e.key==='/'&&!['INPUT','TEXTAREA'].includes(document.activeElement.tagName)&&!$('modal').open){e.preventDefault();$('search').focus()}});
window.addEventListener('hashchange',renderRoute);renderRoute();
