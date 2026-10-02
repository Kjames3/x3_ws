"""Timing accounting and read-only clock exchange; no camera or robot required."""
import ast
import asyncio
import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('capture',ROOT/'scripts/foot_diagnostic_capture.py')
capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)


def test_clock_offset_bounds_and_delivery():
    # True offset +200 ns, symmetric 10 ns network paths and 2 ns handling.
    c=capture.clock_sample(1000,1210,1212,1022)
    assert c['offset_ns']==200 and c['error_bound_ns']==10
    # Asymmetric paths still bound the true offset instead of claiming exact sync.
    a=capture.clock_sample(1000,1220,1222,1032)
    assert a['offset_ns']-a['error_bound_ns']<=200<=a['offset_ns']+a['error_bound_ns']
    d=capture.delivery_timing({'timing':{'capture_monotonic_ns':1190}},
                             {'prepare_monotonic_ns':1210},1022,c)
    assert abs(d['prepare_to_client_estimate_ms']-.000012)<1e-12
    assert abs(d['capture_to_client_estimate_ms']-.000032)<1e-12
    assert capture.delivery_timing({}, {}, 1, None)=={}
    assert capture.clock_sample(10,30,20,40) is None


def test_summary_dedup_does_not_hide_stale_snapshots_or_restarts():
    def row(t,session,seq,feet,status):
        return dict(elapsed_s=t,diagnostic=dict(session_id=session,seq=seq,feet=feet,status=status,
            timing={'extraction_wall_ms':10.,'ready_to_snapshot_ms':t*1000},
            settings={'mono_fps':30,'depth_fps_observed':29.8}),
            telemetry_timing={'previous_frame':{'frame_seq':0,'encode_wall_ms':1,'broadcast_wall_ms':2}})
    rows=[row(0,'a',1,[{}],'ok'),row(.1,'a',1,[],'stale'),row(.4,'a',2,[{}],'ok'),row(.5,'b',1,[{}],'ok')]
    s=capture.summarize(rows)
    assert s['unique_packets']==3
    assert s['stages_ms']['extraction_wall_ms']['n']==3
    assert s['snapshots']['ready_to_snapshot_ms']['n']==4
    assert s['delivery_ms']['encode_wall_ms']['n']==2
    assert abs(s['no_foot_intervals'][0]['duration_s']-.3)<1e-9
    assert s['settings']==[{'mono_fps':30}]
    json.dumps(s,allow_nan=False)
    assert capture.summarize([])['unique_packets']==0


def test_probe_sends_only_timing_request():
    class Socket:
        sent=[]
        async def send(self,raw):self.sent.append(json.loads(raw))
        async def recv(self):
            return json.dumps(dict(type='diagnostic_clock_pong',nonce=self.sent[-1]['nonce'],
                server_receive_monotonic_ns=time.monotonic_ns()+1000000,
                server_send_monotonic_ns=time.monotonic_ns()+1000000))
    ws=Socket();result=asyncio.run(capture.probe_clock(ws,count=3))
    assert len(result['samples'])==3
    assert all(x['type']=='diagnostic_clock_ping' for x in ws.sent)
    assert result['best']['error_bound_ns']>=0


def test_driver_publishes_completed_timing_on_matching_packet_only():
    # Exercise the actual wrapper without importing hardware SDK / opening USB.
    tree=ast.parse((ROOT/'src/oakd_driver.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='OakDCamera')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_process_nn')
    ns={'time':time}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<driver timing>','exec'),ns)
    cam=SimpleNamespace(_latest_detections_t=0,_det_log=None,_latest_detection_meta=None,_lock=threading.Lock())
    def decode(packet,meta):
        assert 'host_decode_start_monotonic_ns' in meta
        assert 'host_decode_end_monotonic_ns' not in meta
        cam._latest_detection_meta=meta
    cam._decode_nn=decode
    original={'seq':5}
    ns['_process_nn'](cam,object(),original)
    meta=cam._latest_detection_meta
    assert original=={'seq':5}
    assert meta['host_decode_end_monotonic_ns']>=meta['host_decode_start_monotonic_ns']
    assert meta['host_decode_wall_ms']>=0 and meta['host_decode_cpu_ms']>=0
    cam._decode_nn=lambda packet,meta:None
    ns['_process_nn'](cam,object(),{'seq':6})
    assert cam._latest_detection_meta is meta  # failed decode cannot relabel old output


def test_server_clock_handler_has_no_motion_calls():
    tree=ast.parse((ROOT/'src/server_x3.py').read_text())
    branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
        and isinstance(n.test,ast.Compare) and any(isinstance(c,ast.Constant) and c.value=='diagnostic_clock_ping' for c in n.test.comparators))
    names={n.id for stmt in branch.body for n in ast.walk(stmt) if isinstance(n,ast.Name)}
    assert not names.intersection({'_enqueue_motion','ros_board','drive','ros_bridge','motors_enabled'})


def test_capture_over_real_websocket_with_clock_and_timing(tmp_path):
    import websockets
    received=[]
    async def run():
        async def handler(ws):
            async def produce():
                seq=0
                while True:
                    now=time.monotonic_ns();seq+=1
                    await ws.send(json.dumps(dict(type='readout',robot_pose={'x':0,'y':0,'theta':0},
                        foot_diagnostic=dict(session_id='test',seq=seq,status='ok',feet=[{'id':1}],
                            timing={'capture_monotonic_ns':now-100000000,'extraction_wall_ms':1,
                                    'snapshot_monotonic_ns':now-1000000},
                            settings={'mono_fps':30}),
                        telemetry_timing={'frame_seq':seq,'prepare_monotonic_ns':now})))
                    await asyncio.sleep(.04)
            task=asyncio.create_task(produce())
            try:
                async for raw in ws:
                    msg=json.loads(raw);received.append(msg['type'])
                    now=time.monotonic_ns()
                    await ws.send(json.dumps(dict(type='diagnostic_clock_pong',nonce=msg['nonce'],
                        server_receive_monotonic_ns=now,server_send_monotonic_ns=time.monotonic_ns())))
            finally:
                task.cancel()
                try:await task
                except asyncio.CancelledError:pass
        async with websockets.serve(handler,'127.0.0.1',0) as server:
            port=server.sockets[0].getsockname()[1]
            await capture.capture(SimpleNamespace(host='127.0.0.1',port=port,seconds=.2,
                                                  label='synthetic-timing',output=str(tmp_path)))
    asyncio.run(run())
    summary=json.loads(next(tmp_path.glob('*/summary.json')).read_text())
    assert summary['unique_packets']>=3
    assert summary['stages_ms']['extraction_wall_ms']['median']==1
    assert summary['delivery_ms']['capture_to_client_estimate_ms']['median']>=99
    assert summary['clock']['before']['best'] and summary['clock']['after']['best']
    assert summary['settings']==[{'mono_fps':30}]
    assert received and set(received)=={'diagnostic_clock_ping'}
