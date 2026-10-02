#!/usr/bin/env python3
"""Capture foot diagnostic telemetry and timing. Sends clock probes, never motion."""
import argparse
import asyncio
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import re
import time
import uuid

import websockets


def distribution(values):
    a=sorted(float(v) for v in values if isinstance(v,(int,float)) and math.isfinite(v))
    if not a:
        return None
    def percentile(q):
        pos=(len(a)-1)*q;lo=int(pos);hi=min(lo+1,len(a)-1)
        return a[lo]+(a[hi]-a[lo])*(pos-lo)
    return dict(n=len(a),min=a[0],median=percentile(.5),p95=percentile(.95),max=a[-1])


def clock_sample(t1,t2,t3,t4):
    """Server-minus-client monotonic offset interval, assuming nonnegative delays.

    t1/t4: client send/receive; t2/t3: server receive/send. Midpoint is
    an estimate, not an assertion that paths are symmetric.
    """
    low=t3-t4;high=t2-t1
    if t4<t1 or t3<t2 or low>high:
        return None
    return dict(offset_ns=(low+high)/2,error_bound_ns=(high-low)/2,
                round_trip_ns=t4-t1,server_handling_ns=t3-t2,
                client_midpoint_ns=(t1+t4)/2)


async def probe_clock(ws, count=5):
    samples=[]
    for _ in range(count):
        nonce=uuid.uuid4().hex
        t1=time.monotonic_ns()
        await ws.send(json.dumps(dict(type='diagnostic_clock_ping',nonce=nonce)))
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            try:
                raw=await asyncio.wait_for(ws.recv(),timeout=max(.001,deadline-time.monotonic()))
            except asyncio.TimeoutError:
                break
            t4=time.monotonic_ns()
            if not isinstance(raw,str):
                continue
            d=json.loads(raw)
            if d.get('type')!='diagnostic_clock_pong' or d.get('nonce')!=nonce:
                continue
            sample=clock_sample(t1,d['server_receive_monotonic_ns'],d['server_send_monotonic_ns'],t4)
            if sample: samples.append(sample)
            break
        if not samples:
            break  # Older server: collect host timings without a transport estimate.
    return dict(samples=samples,best=min(samples,key=lambda s:s['error_bound_ns']) if samples else None)


def delivery_timing(diag, telemetry, receive_ns, calibration):
    if not calibration:
        return {}
    server_receive=receive_ns+calibration['offset_ns']
    result=dict(clock_error_bound_ms=calibration['error_bound_ns']/1e6)
    prepared=telemetry.get('prepare_monotonic_ns')
    if prepared is not None:
        result['prepare_to_client_estimate_ms']=(server_receive-prepared)/1e6
    capture=diag.get('timing',{}).get('capture_monotonic_ns')
    if capture is not None:
        result['capture_to_client_estimate_ms']=(server_receive-capture)/1e6
    # Do not clamp negative estimates: they expose clock/measurement uncertainty.
    return result


def summarize(rows):
    unique={}
    for r in rows:
        d=r['diagnostic']
        unique.setdefault((d.get('session_id'),d.get('seq')),r)
    counters=Counter(); stages=defaultdict(list); snapshots=defaultdict(list)
    transport=defaultdict(list); settings={}; previous_frames={}
    for r in unique.values():
        d=r['diagnostic'];counters[d.get('status','unknown')]+=1
        for k,v in d.get('timing',{}).items():
            if k.endswith('_ms') and k not in ('ready_to_snapshot_ms','capture_to_snapshot_ms'):
                stages[k].append(v)
        config=dict(d.get('settings',{}));config.pop('depth_fps_observed',None)
        if config:settings[json.dumps(config,sort_keys=True)]=config
    for r in rows:
        d=r['diagnostic'];timing=d.get('timing',{})
        for k in ('ready_to_snapshot_ms','capture_to_snapshot_ms'):
            snapshots[k].append(timing.get(k))
        snapshots['depth_fps_observed'].append(d.get('settings',{}).get('depth_fps_observed'))
        for k,v in r.get('client_timing',{}).items():transport[k].append(v)
        tt=r.get('telemetry_timing',{})
        if tt.get('prepare_monotonic_ns') is not None and timing.get('snapshot_monotonic_ns') is not None:
            snapshots['snapshot_to_prepare_ms'].append((tt['prepare_monotonic_ns']-timing['snapshot_monotonic_ns'])/1e6)
        previous=tt.get('previous_frame')
        if previous:
            previous_frames[(d.get('session_id'),previous['frame_seq'])]=previous
    for prev in previous_frames.values():
        for k in ('encode_wall_ms','broadcast_wall_ms'):transport[k].append(prev[k])
    # Repeated snapshots can expire; calculate gaps from every received readout.
    gaps=[];start=None
    for r in rows:
        present=bool(r['diagnostic'].get('feet'))
        if not present and start is None:start=r['elapsed_s']
        if present and start is not None:
            gaps.append(dict(start_s=start,end_s=r['elapsed_s'],duration_s=r['elapsed_s']-start));start=None
    if start is not None:
        gaps.append(dict(start_s=start,end_s=rows[-1]['elapsed_s'],duration_s=rows[-1]['elapsed_s']-start,open_end=True))
    intervals=[b['elapsed_s']-a['elapsed_s'] for a,b in zip(rows,rows[1:])]
    firsts=list(unique.values())
    packet_intervals=[];capture_intervals=[]
    for a,b in zip(firsts,firsts[1:]):
        da,db=a['diagnostic'],b['diagnostic']
        if da.get('session_id')!=db.get('session_id'):continue
        packet_intervals.append((b['elapsed_s']-a['elapsed_s'])*1000)
        ta=da.get('timing',{}).get('capture_monotonic_ns')
        tb=db.get('timing',{}).get('capture_monotonic_ns')
        if ta is not None and tb is not None and tb>ta:capture_intervals.append((tb-ta)/1e6)
    return dict(packet_receive_interval_ms=distribution(packet_intervals),
                source_capture_interval_ms=distribution(capture_intervals),unique_packets=len(unique),packets_with_feet=sum(bool(r['diagnostic'].get('feet')) for r in unique.values()),
                statuses=dict(counters),telemetry_rows=len(rows),
                stages_ms={k:distribution(v) for k,v in stages.items()},
                snapshots={k:distribution(v) for k,v in snapshots.items()},
                delivery_ms={k:distribution(v) for k,v in transport.items()},
                telemetry_interval_s=distribution(intervals),no_foot_intervals=gaps,
                settings=list(settings.values()),motors_commanded=False,
                note='Stages count distinct packets; snapshot ages count all readouts. Delivery estimates have clock-probe uncertainty and exclude browser rendering. Camera delivery combines device processing, queues and transfer; CPU time covers only the current thread.')


async def capture(args):
    label=re.sub(r'[^a-zA-Z0-9_-]','_',args.label)
    directory=Path(args.output)/(time.strftime('%Y%m%d-%H%M%S')+'-'+label)
    directory.mkdir(parents=True,exist_ok=False)
    rows=[];missing=0
    async with websockets.connect(f'ws://{args.host}:{args.port}',max_size=8*1024*1024) as ws:
        print('Connected. Measuring clock-offset bounds (no motion commands).',flush=True)
        before=await probe_clock(ws)
        if before['best']:
            print(f"Clock probe bound: ±{before['best']['error_bound_ns']/1e6:.1f} ms",flush=True)
        else:
            print('No clock reply; delivery estimates unavailable. Server may need updating.',flush=True)
        print('Stand still; capture starts in 3 seconds.',flush=True)
        for n in (3,2,1):
            print(n,flush=True);until=time.monotonic()+1
            while time.monotonic()<until:
                try:await asyncio.wait_for(ws.recv(),timeout=max(.001,until-time.monotonic()))
                except asyncio.TimeoutError:break
        print(f'START — {args.seconds:g} seconds. Keep robot stationary; slowly extend and withdraw one foot.',flush=True)
        start=time.monotonic()
        with (directory/'telemetry.ndjson').open('w') as out:
            while time.monotonic()-start<args.seconds:
                try:raw=await asyncio.wait_for(ws.recv(),timeout=2)
                except asyncio.TimeoutError:continue
                received=time.monotonic_ns()
                if not isinstance(raw,str):continue
                message=json.loads(raw)
                if message.get('type')!='readout':continue
                diag=message.get('foot_diagnostic')
                if diag is None:
                    missing+=1;continue
                telemetry=message.get('telemetry_timing',{})
                client=delivery_timing(diag,telemetry,received,before['best'])
                client['client_parse_wall_ms']=(time.monotonic_ns()-received)/1e6
                row=dict(elapsed_s=received/1e9-start,client_receive_monotonic_ns=received,
                         diagnostic=diag,telemetry_timing=telemetry,client_timing=client,
                         robot_pose=message.get('robot_pose'))
                rows.append(row);out.write(json.dumps(row,allow_nan=False)+'\n')
        print('STOP — capture complete. Checking clock offset again.',flush=True)
        after=await probe_clock(ws,count=3)
    calibration=dict(before=before,after=after)
    if before['best'] and after['best']:
        drift=after['best']['offset_ns']-before['best']['offset_ns']
        bounds=before['best']['error_bound_ns']+after['best']['error_bound_ns']
        calibration.update(offset_change_ms=drift/1e6,consistent_with_probe_bounds=abs(drift)<=bounds)
    (directory/'clock.json').write_text(json.dumps(calibration,indent=2)+'\n')
    summary=summarize(rows);summary['missing_diagnostic_messages']=missing
    summary['clock']=calibration
    (directory/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(f"{summary['unique_packets']} distinct packets; {summary['packets_with_feet']} with feet; {len(summary['no_foot_intervals'])} recorded gaps")
    for k,s in summary['stages_ms'].items():
        if s and k.endswith(('wall_ms','delivery_ms','wait_ms')):
            print(f"{k}: median {s['median']:.1f} ms, p95 {s['p95']:.1f} ms")
    print(f'Saved: {directory}')
    if not summary['packets_with_feet']:print('No foot regions captured. Inspect status/rejection reasons before a longer run.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='x3');p.add_argument('--port',type=int,default=8081)
    p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--label',required=True);p.add_argument('--output',default='evidence/foot-diagnostic')
    args=p.parse_args()
    if not 0<args.seconds<=300:p.error('--seconds must be in (0, 300]')
    asyncio.run(capture(args))
