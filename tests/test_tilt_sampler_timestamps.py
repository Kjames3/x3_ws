"""Exercise the real sampler without importing the hardware server."""
import ast
import logging
from pathlib import Path
import sys
from threading import Lock
from types import ModuleType, SimpleNamespace

import pytest


@pytest.mark.parametrize('moving_delay', [0.0, 0.080])
def test_stamp_is_position_read_midpoint(monkeypatch, moving_delay):
    source = Path(__file__).resolve().parents[1] / 'src/server_x3.py'
    tree = ast.parse(source.read_text())
    function = next(n for n in tree.body
                    if isinstance(n, ast.FunctionDef) and n.name == '_tilt_sampler')
    seconds = [0.0]

    class RosTime:
        def __init__(self, nanoseconds):
            self.nanoseconds = nanoseconds

        def __add__(self, duration):
            return RosTime(self.nanoseconds + duration.nanoseconds)

        def to_msg(self):
            return self.nanoseconds

    class JointState:
        def __init__(self):
            self.header = SimpleNamespace()

    for name, attrs in [('sensor_msgs.msg', {'JointState': JointState}),
                        ('rclpy.duration', {'Duration': RosTime})]:
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)

    messages = []
    namespace = dict(
        TILT_SAMPLE_HZ=50, _shutting_down=False, _tilt_gate_moving=False,
        _servo_lock=Lock(), _tilt_lock=Lock(), _tilt_need_moving=True,
        _tilt_sample={'reads': 0, 'errors': 0},
        time=SimpleNamespace(monotonic=lambda: seconds[0], sleep=lambda _: None),
        math=__import__('math'), logger=logging.getLogger(__name__))

    def publish(message):
        messages.append(message)
        namespace['_shutting_down'] = True

    clock = SimpleNamespace(now=lambda: RosTime(
        100_000_000_000 + round(seconds[0] * 1e9)))
    namespace['ros_bridge'] = SimpleNamespace(
        _node=SimpleNamespace(get_clock=lambda: clock),
        joint_pub=SimpleNamespace(publish=publish),
        full_joint_pub=SimpleNamespace(publish=publish))

    class Servo:
        def read_pos(self, servo_id):
            seconds[0] += 0.008
            return 2032

        def read_moving(self, servo_id):
            seconds[0] += moving_delay
            return False

    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'),
         namespace)
    namespace['_tilt_sampler'](Servo(), 1, lambda _: 0.0)
    assert len(messages) == 2
    assert [m.header.stamp for m in messages] == [100_004_000_000] * 2
    assert namespace['_tilt_sample']['t'] == pytest.approx(0.004)
