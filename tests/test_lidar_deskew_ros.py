"""ROS message/TF integration; run with sourced Humble and /usr/bin/python3."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Point32, TransformStamped
from sensor_msgs.msg import PointCloud, ChannelFloat32
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_3d_processor_node import Lidar3dProcessorNode


@pytest.fixture
def processor():
    rclpy.init()
    node = Lidar3dProcessorNode()
    output = []
    node.pc_pub = SimpleNamespace(publish=output.append)
    start = node.get_clock().now().nanoseconds - 300_000_000
    def tf(parent, child, stamp, xyz, static=False):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = Time(nanoseconds=stamp).to_msg()
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
        msg.transform.rotation.w = 1.0
        setter = node.tf_buffer.set_transform_static if static else node.tf_buffer.set_transform
        setter(msg, 'test')
    tf('odom', 'lidar_mount_link', start - 50_000_000, (1., 2., .165))
    tf('odom', 'lidar_mount_link', start + 150_000_000, (1., 2., .165))
    tf('lidar_mount_link', 'lidar_tilt_link', start, (-.0125, 0., .153), True)
    tf('lidar_tilt_link', 'laser_link', start, (0., 0., .022), True)
    node.history.extend([(start * 1e-9 - .01, 0., True),
                         (start * 1e-9 + .06, 0., True),
                         (start * 1e-9 + .11, 0., True)])
    msg = PointCloud()
    msg.header.frame_id = 'laser_link'
    msg.header.stamp = Time(nanoseconds=start).to_msg()
    msg.points = [Point32(x=2., y=0., z=0.), Point32(x=1., y=1., z=0.)]
    msg.channels = [ChannelFloat32(name='acquisition_time', values=[0., .1]),
                    ChannelFloat32(name='scan_duration', values=[.1, .1])]
    yield node, msg, output
    node.destroy_node()
    rclpy.shutdown()


def test_message_tf_and_sensor_origin(processor):
    node, msg, output = processor
    node._project_timed(msg)
    assert len(output) == 1
    cloud = output[0]
    assert cloud.header == msg.header
    assert cloud.point_step == 12 and cloud.row_step == 24
    np.testing.assert_allclose(np.frombuffer(cloud.data, '<f4').reshape(-1, 3),
                               [[2, 0, 0], [1, 1, 0]], atol=1e-6)


@pytest.mark.parametrize('fault', ['timing', 'gap', 'moving', 'tf'])
def test_invalid_inputs_never_publish(processor, fault):
    node, msg, output = processor
    if fault == 'timing':
        msg.channels[0].name = 'stamps'
    elif fault == 'gap':
        node.history.clear()
    elif fault == 'moving':
        t, p, _ = node.history[1]
        node.history[1] = (t, p, False)
    else:
        msg.header.frame_id = 'missing_laser'
    node.timed_callback(msg)
    node.pending[0] = (node.pending[0][0] - 1., msg)
    node.process_pending()
    assert not output
    assert not node.pending
    assert node._deskew_drop == 1


def test_runtime_mode_changes_only_3d_gate(processor):
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import LaserScan
    node, msg, output = processor
    node.history = [(t, p, False) for t, p, _ in node.history]
    with pytest.raises(ValueError, match='moved'):
        node._project_timed(msg)
    assert node.set_parameters([Parameter('require_settled', value=False)])[0].successful
    node._project_timed(msg)
    assert len(output) == 1
    scans = []
    node.scan_pub = SimpleNamespace(publish=scans.append)
    node._tilt_state = lambda: (0.0, 'fresh')
    node._scan_captured_while_settled = lambda m: False
    node.scan_callback(LaserScan())
    assert not scans  # Crossing level must not leak a pitched revolution.
    node._scan_captured_while_settled = lambda m: True
    node.scan_callback(LaserScan())
    assert len(scans) == 1
    node.set_parameters([Parameter('require_settled', value=True)])
    with pytest.raises(ValueError, match='moved'):
        node._project_timed(msg)
