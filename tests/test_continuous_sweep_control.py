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
                 logger=SimpleNamespace(error=lambda *args: None))
    result = asyncio.run(function('_apply_sweep_gate', state)())
    assert result == (False, 'unavailable')
    assert state['lidar_3d_scan_enabled'] is False
    assert state['lidar_sweep_mode'] == 'step'
    assert state['_sweep_settled_bypass'] is False
    assert calls == [False, True]
