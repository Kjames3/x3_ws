"""ROS-backed lossless replay test; runs without devices or a ROS graph."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
rclpy = pytest.importorskip('rclpy')
rosbag2_py = pytest.importorskip('rosbag2_py')
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from oakd_ros_publisher import OakRosPublisher, DEPTH_FRAME
from c1_dataset import TOPICS, audit, load_pair
from tof_observation import ToFObservations


class Publisher:
    def __init__(self, topic, messages):
        self.topic, self.messages = topic, messages

    def publish(self, msg):
        self.messages.append((self.topic, msg))


class Node:
    def __init__(self):
        self.messages = []

    def create_publisher(self, cls, topic, qos):
        return Publisher(topic, self.messages)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000), clock_type='SYSTEM_TIME')


def packet(image, seq=1):
    import time
    return dict(image=image, meta=dict(seq=seq, device_ns=1_000_000_000,
                host_monotonic_estimate_ns=time.monotonic_ns(), session_id='synthetic'))


def pair():
    k = [100., 0., 1., 0., 100., 1., 0., 0., 1.]
    return dict(rgb=packet(np.arange(18, dtype='uint8').reshape(2, 3, 3)),
                depth=packet(np.array([[0, 65535, 123], [456, 789, 1000]], dtype='<u2')),
                pair_seq=1, epoch=0, skew_ns=0, counters={'paired': 1},
                calibration=dict(height=2, width=3, rgb_k=k, depth_k=k, rgb_d=[0.]*14))


def test_replay_preserves_original_pixels_and_is_deterministic(tmp_path):
    node = Node()
    pub = OakRosPublisher(node, SimpleNamespace(record_rgbd=True), publish_stereo=False, publish_detections=False)
    original = pair()
    pub._publish_rgbd(original)
    meta = json.loads(node.messages[-1][1].data)
    ns = meta['depth']['ros_stamp_ns']
    def stamp(t):
        return Time(sec=t // 1_000_000_000, nanosec=t % 1_000_000_000)
    for t in [ns - 50_000_000, ns + 50_000_000]:
        odom = Odometry(); odom.header.stamp = stamp(t)
        odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        node.messages.append(('/odom', odom))
        transform = TransformStamped(); transform.header = odom.header
        transform.child_frame_id = 'base_link'; transform.transform.rotation.w = 1.
        node.messages.append(('/tf', TFMessage(transforms=[transform])))
    transform = TransformStamped(); transform.header.frame_id = 'base_link'
    transform.child_frame_id = DEPTH_FRAME; transform.transform.rotation.w = 1.
    node.messages.append(('/tf_static', TFMessage(transforms=[transform])))
    node.messages.append(('/scan', LaserScan()))
    tracker = ToFObservations('synthetic'); tracker.connect()
    for sensor in ['upper', 'lower']:
        payload = dict(v=1, type='frame', sensor=sensor, seq=1, t_ms=100, read_us=14650,
                       distance_mm=[700]*64, target_status=[5]*64, nb_target_detected=[1]*64)
        row = tracker.envelope(payload, 1_000_000_000, 10_000_000_000)
        node.messages.append(('/tof/observations', String(data=json.dumps(row))))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for topic, cls in TOPICS.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=cls, serialization_format='cdr'))
    for i, (topic, msg) in enumerate(node.messages):
        writer.write(topic, serialize_message(msg), 10_000_000_000+i)
    del writer
    a = audit(tmp_path)
    assert a['complete_pairs'] == 1 and not a['failures']
    assert not a['c1_complete']  # synthetic data does not qualify hardware
    assert audit(tmp_path) == a
    arrays, row = load_pair(tmp_path, 0)
    for name in ['rgb', 'depth']:
        np.testing.assert_array_equal(arrays[name], original[name]['image'])
    assert row['tf_available'] and row['odom_bracketed_100ms']


def test_invalid_second_payload_publishes_neither_half():
    node = Node()
    pub = OakRosPublisher(node, SimpleNamespace(record_rgbd=True), publish_stereo=False, publish_detections=False)
    bad = pair(); bad['depth']['image'] = bad['depth']['image'].astype('float32')
    pub._publish_rgbd(bad)
    assert not node.messages


def test_nominal_registration_preserves_identity_and_marks_unsupported_pixels():
    from c1_dataset import rgb_on_depth_grid
    original = pair()
    arrays = {name: original[name]['image'].copy() for name in ('rgb', 'depth')}
    row = {'metadata': {'calibration': original['calibration']}}
    rgb, support = rgb_on_depth_grid(arrays, row)
    np.testing.assert_array_equal(rgb, arrays['rgb'])
    assert support.all()
    # Shift target grid by one pixel: left output column has no RGB support.
    row['metadata']['calibration']['depth_k'] = list(original['calibration']['depth_k'])
    row['metadata']['calibration']['depth_k'][2] += 1
    shifted, support = rgb_on_depth_grid(arrays, row)
    assert not support[:, 0].any() and support[:, 1:].all()
    np.testing.assert_array_equal(shifted[:, 1:], arrays['rgb'][:, :-1])
    np.testing.assert_array_equal(arrays['depth'], original['depth']['image'])
