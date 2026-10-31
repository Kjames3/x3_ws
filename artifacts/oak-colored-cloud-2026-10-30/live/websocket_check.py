import asyncio,json,struct,time
from pathlib import Path
import websockets
out=Path('artifacts/oak-colored-cloud-2026-10-30/live')
async def main():
 async with websockets.connect('ws://100.64.52.55:8765',subprotocols=['foxglove.sdk.v1'],max_size=10000000) as ws:
  channels={};ids={};nextid=1
  phases=[('off',15),('on',30),('off_again',15)];results=[]
  for phase,seconds in phases:
   start=time.monotonic();counts={};ages=[];sizes=[];last={};gaps={}
   while time.monotonic()-start<seconds:
    wanted=['/scan','/oak/rgbd/metadata']+(['/surroundings/oak/points','/surroundings/oak/pose'] if phase=='on' else [])
    for topic,ch in channels.items():
     if topic in wanted and topic not in ids.values():
      ids[nextid]=topic;await ws.send(json.dumps({'op':'subscribe','subscriptions':[{'id':nextid,'channelId':ch}]}));nextid+=1
    remove=[i for i,t in ids.items() if t not in wanted]
    if remove:
     await ws.send(json.dumps({'op':'unsubscribe','subscriptionIds':remove}))
     for i in remove:ids.pop(i)
    try:m=await asyncio.wait_for(ws.recv(),1)
    except asyncio.TimeoutError:continue
    if isinstance(m,str):
     d=json.loads(m)
     if d.get('op')=='advertise':
      channels.update({c['topic']:c['id'] for c in d['channels']})
     continue
    if m[0]!=1:continue
    topic=ids.get(struct.unpack_from('<I',m,1)[0]);now=time.monotonic()
    if not topic:continue
    counts[topic]=counts.get(topic,0)+1
    if topic in last:gaps.setdefault(topic,[]).append(now-last[topic])
    last[topic]=now
    if topic=='/surroundings/oak/points':
     sec,ns=struct.unpack_from('<iI',m,17);ages.append((time.time()-sec-ns*1e-9)*1000);sizes.append(len(m))
     (out/'sample.cdr').write_bytes(m[13:])
   elapsed=time.monotonic()-start
   import numpy as np
   r=dict(phase=phase,elapsed=elapsed,counts=counts,rates={k:v/elapsed for k,v in counts.items()},max_gap={k:max(v) for k,v in gaps.items() if v},age_p95_ms=float(np.percentile(ages,95)) if ages else None,bytes_per_second=sum(sizes)/elapsed)
   results.append(r);print(json.dumps(r),flush=True);(out/'websocket-check.json').write_text(json.dumps(results,indent=2))
asyncio.run(main())
