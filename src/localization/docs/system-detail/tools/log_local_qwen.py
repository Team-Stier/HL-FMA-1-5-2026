#!/usr/bin/env python3
"""Run the authorized local helper and persist observed usage; no billing claims."""
import argparse, datetime, hashlib, json, pathlib, subprocess, sys
p=argparse.ArgumentParser()
p.add_argument('--task',required=True); p.add_argument('--file',required=True)
p.add_argument('--start-line',type=int,default=1); p.add_argument('--end-line',type=int,required=True)
p.add_argument('--ledger',type=pathlib.Path,required=True)
a=p.parse_args(); path=pathlib.Path(a.file).resolve()
source=''.join(path.read_text().splitlines(keepends=True)[a.start_line-1:a.end_line])
cmd=['python3','/home/paik/.codex/tools/local_qwen.py','--task',a.task,'--file',str(path),'--start-line',str(a.start_line),'--end-line',str(a.end_line),'--max-tokens','450']
r=subprocess.run(cmd,text=True,capture_output=True)
try: result=json.loads(r.stdout)
except ValueError: result={'status':'error','error':r.stderr[-500:] or r.stdout[-500:]}
record={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source':str(path),'start_line':a.start_line,'end_line':a.end_line,'source_sha256':hashlib.sha256(source.encode()).hexdigest(),'source_bytes':len(source.encode()),'task':a.task,'exit_code':r.returncode,'local':result}
# This encoding measures a reproducible proxy, not Codex's hidden billing tokenizer.
try:
 import tiktoken
 enc=tiktoken.get_encoding('o200k_base')
 visible=json.dumps(result,ensure_ascii=False)
 record['input_proxy']={'encoding':'o200k_base','source_tokens':len(enc.encode(source)),'returned_json_tokens':len(enc.encode(visible)),'gross_reduction_tokens':len(enc.encode(source))-len(enc.encode(visible)),'scope':'source text replaced by returned JSON only; excludes orchestration, verification, reasoning, caching and final authoring; not actual OpenAI bill savings'}
except ImportError:
 record['input_proxy']={'status':'unavailable','reason':'optional tiktoken not installed; no token estimate fabricated'}
a.ledger.parent.mkdir(parents=True,exist_ok=True)
with a.ledger.open('a') as f: f.write(json.dumps(record,ensure_ascii=False)+'\n')
print(json.dumps(result,ensure_ascii=False))
sys.exit(r.returncode)
