"""Test production control decisions without loading hardware server imports."""
import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace


def function(name, namespace):
    path = Path(__file__).resolve().parents[1] / 'src/server_x3.py'
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if getattr(n, 'name', None) == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


def test_continuous_waits_for_encoder_endpoint():
    moving = function('_tilt_target_moving', {'LIDAR_SWEEP_SETTLE_S': .2})
    assert moving(2500, 2100, False, 2., True, 12)
    assert not moving(2500, 2490, False, 2., True, 12)
    assert moving(2500, 2490, False, .1, True, 12)
    assert not moving(None, 2100, False, 2., True, 12)
    assert moving(2500, 2490, True, 2., False, 12)
    assert not moving(2500, 2100, False, 2., False, 12)


def test_failed_gate_stops_sweep_without_faking_settled():
    calls = []
    async def setter(value):
        calls.append(value)
        return (False, 'unavailable') if len(calls) == 1 else (True, 'restored')
    state = dict(_sweep_settled_bypass=False, lidar_3d_scan_enabled=True,
                 lidar_sweep_mode='continuous', _set_processor_require_settled=setter,
                 _sweep_gate_lock=asyncio.Lock(),
                 logger=SimpleNamespace(error=lambda *args: None))
    result = asyncio.run(function('_apply_sweep_gate', state)())
    assert result == (False, 'unavailable')
    assert state['lidar_3d_scan_enabled'] is False
    assert state['lidar_sweep_mode'] == 'continuous'
    assert state['_sweep_settled_bypass'] is False
    assert calls == [False, True]


def start_handler(namespace):
    path = Path(__file__).resolve().parents[1] / 'src/server_x3.py'
    tree = ast.parse(path.read_text())
    branch = next(n for n in ast.walk(tree) if isinstance(n, ast.If)
                  and ast.unparse(n.test) == "msg_type == 'toggle_3d_scan'")
    wrapper = ast.parse('async def start():\n    pass').body[0]
    wrapper.body = branch.body
    ast.fix_missing_locations(wrapper)
    exec(compile(ast.Module(body=[wrapper], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['start']


def test_start_waits_for_gate_and_rechecks_cancellation():
    import json
    import time
    async def scenario(cancel=False, failure=False):
        entered, finish = asyncio.Event(), asyncio.Event()
        replies = []
        async def send(payload):
            replies.append(json.loads(payload))
        async def gate():
            entered.set()
            await finish.wait()
            return not failure, 'injected failure'
        state = dict(data={'enabled': True, 'mode': 'continuous'},
                     lidar_3d_scan_enabled=False, lidar_sweep_mode='continuous',
                     _sweep_request_id=0, _sweep_owner=None, _sweep_started_at=0,
                     _tilt_nav_conflict=lambda: None, _apply_sweep_gate=gate,
                     websocket=SimpleNamespace(send=send), time=time, json=json,
                     logger=SimpleNamespace(info=lambda *a: None, warning=lambda *a: None))
        task = asyncio.create_task(start_handler(state)())
        await entered.wait()
        assert not state['lidar_3d_scan_enabled']
        assert replies == []
        if cancel:
            state['_sweep_request_id'] += 1
        finish.set()
        await task
        assert state['lidar_3d_scan_enabled'] is (not cancel and not failure)
        assert replies[-1]['refused'] is (cancel or failure)
    for cancel, failure in [(False, False), (True, False), (False, True)]:
        asyncio.run(scenario(cancel, failure))


def test_failed_gate_retry_keeps_continuous_request():
    calls = []
    async def setter(value):
        calls.append(value)
        return (False, 'timeout') if len(calls) == 1 else (True, 'ok')
    state = dict(_sweep_settled_bypass=False, lidar_3d_scan_enabled=False,
                 lidar_sweep_mode='continuous', _set_processor_require_settled=setter,
                 _sweep_gate_lock=asyncio.Lock(), logger=SimpleNamespace(error=lambda *a: None))
    gate = function('_apply_sweep_gate', state)
    async def scenario():
        assert not (await gate())[0]
        assert (await gate())[0]
    asyncio.run(scenario())
    assert calls == [False, True, False]
    assert state['lidar_sweep_mode'] == 'continuous'
