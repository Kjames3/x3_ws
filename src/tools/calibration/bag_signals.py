#!/usr/bin/env python3
"""Shared rosbag2 reading for the calibration tools (time_offset_xcorr, lidar_camera_overlay).

Reads sqlite3 or mcap bags, compressed or not, through rosbag2_py, and decodes
only what the calibration tools need. Two message types are decoded straight
from the CDR bytes instead of through rclpy:

* livox_ros_driver2/CustomMsg: rclpy takes ~177 ms per 20k-point message, and
  this way the Livox workspace doesn't need to be sourced.
* the header stamp of any stamped message (sec int32 / nanosec uint32 at byte 4).

Every reader yields (topic, header_stamp_s or None, bag_receive_s, payload).
"""
from __future__ import annotations

import glob
import os
import struct

import numpy as np

LIVOX_POINT_DT = np.dtype({'names': ['t', 'x', 'y', 'z'],
                           'formats': ['<u4', '<f4', '<f4', '<f4'],
                           'offsets': [0, 4, 8, 12], 'itemsize': 20})

# Types whose CDR starts with std_msgs/Header (so the stamp sits at byte 4).
STAMPED_TYPES = {
    'sensor_msgs/msg/Imu', 'nav_msgs/msg/Odometry', 'sensor_msgs/msg/Image',
    'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/PointCloud2',
    'sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/CameraInfo',
    'livox_ros_driver2/msg/CustomMsg', 'geometry_msgs/msg/TwistStamped',
}


def _storage_id(path):
    if glob.glob(os.path.join(path, '*.mcap')) or glob.glob(os.path.join(path, '*.mcap.zstd')):
        return 'mcap'
    return 'sqlite3'


def _compressed(path):
    meta = os.path.join(path, 'metadata.yaml')
    if not os.path.exists(meta):
        return False
    with open(meta) as fh:
        for line in fh:
            line = line.strip()
            if line.startswith('compression_format:'):
                return line.split(':', 1)[1].strip().strip('"\'') != ''
    return False


def open_reader(path, topics=None):
    import rosbag2_py
    path = os.path.abspath(path)
    reader = rosbag2_py.SequentialCompressionReader() if _compressed(path) else rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=_storage_id(path)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader, types


def topic_types(path):
    return open_reader(path)[1]


def header_stamp(data):
    sec, nsec = struct.unpack_from('<iI', data, 4)
    return sec + nsec * 1e-9


def frame_id(data):
    n = struct.unpack_from('<I', data, 12)[0]
    return bytes(data[16:16 + n - 1]).decode()


def iter_messages(path, topics):
    """Yield (topic, type, header_stamp_s or None, receive_s, raw_bytes)."""
    reader, types = open_reader(path, topics)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        typ = types[topic]
        stamp = header_stamp(data) if typ in STAMPED_TYPES else None
        yield topic, typ, stamp, t_ns * 1e-9, data


def deserialize(data, typ):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    return deserialize_message(data, get_message(typ))


# ---------------------------------------------------------------- point clouds

def decode_livox_custom(data):
    """CustomMsg CDR bytes -> structured array (t = offset_time ns, x, y, z).

    Same trailer search as livox_scan_node.decode_points: the point array is the
    last field, so walk back to the length prefix that matches the byte count.
    """
    n = (len(data) + 1) // 20
    for n in range(n, max(n - 200, 0), -1):
        start = len(data) - (n * 20 - 1)
        if start >= 4 and struct.unpack_from('<I', data, start - 4)[0] == n:
            return np.frombuffer(bytes(data[start:]) + b'\0', dtype=LIVOX_POINT_DT, count=n)
    return None


def cloud_xyz(data, typ):
    """(N,3) float array of finite, non-zero points, plus per-point offset (s) or None."""
    if typ == 'livox_ros_driver2/msg/CustomMsg':
        pts = decode_livox_custom(data)
        if pts is None:
            return np.zeros((0, 3)), None
        xyz = np.stack([pts['x'], pts['y'], pts['z']], 1).astype(np.float64)
        dt = pts['t'].astype(np.float64) * 1e-9
    elif typ == 'sensor_msgs/msg/PointCloud2':
        from sensor_msgs_py import point_cloud2
        msg = deserialize(data, typ)
        arr = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
        dt = None
    else:
        raise ValueError(f'not a point cloud type: {typ}')
    keep = np.isfinite(xyz).all(1) & (np.abs(xyz).sum(1) > 1e-6)  # Livox: ~27% zero points
    return xyz[keep], (dt[keep] if dt is not None else None)


# ---------------------------------------------------------------- images

def decode_image(data, typ):
    """Image or CompressedImage -> BGR or mono uint8 numpy array."""
    import cv2
    msg = deserialize(data, typ)
    if typ == 'sensor_msgs/msg/CompressedImage':
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    if enc in ('mono16', '16uc1'):
        img = buf.view('<u2').reshape(msg.height, msg.step // 2)[:, :msg.width]
        return cv2.convertScaleAbs(img, alpha=255.0 / max(1, int(img.max())))
    if enc in ('bgr8', 'rgb8', '8uc3'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if enc == 'rgb8' else img
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    if enc in ('yuv422', 'uyvy', 'yuyv', 'yuv422_yuy2'):
        img = buf.reshape(msg.height, msg.width, 2)
        code = cv2.COLOR_YUV2BGR_UYVY if enc in ('yuv422', 'uyvy') else cv2.COLOR_YUV2BGR_YUY2
        return cv2.cvtColor(img, code)
    raise ValueError(f'unsupported image encoding {msg.encoding!r}')


# ---------------------------------------------------------------- geometry

def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_to_mat(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch),
                              np.sin(pitch), np.cos(yaw), np.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def mat_to_rpy(R):
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    return np.arctan2(R[2, 1], R[2, 2]), pitch, np.arctan2(R[1, 0], R[0, 0])


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def static_tf_tree(path):
    """{(parent, child): 4x4} from every /tf_static message (and /tf, first seen) in the bag."""
    tfs = {}
    names = [t for t in ('/tf_static', '/tf') if t in topic_types(path)]
    for topic, typ, _, _, data in iter_messages(path, names):
        for tr in deserialize(data, typ).transforms:
            key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
            if topic == '/tf' and key in tfs:
                continue
            q, p = tr.transform.rotation, tr.transform.translation
            tfs[key] = make_T(quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])
    return tfs


def lookup(tfs, target, source):
    """4x4 that maps points in `source` frame into `target` frame, by walking the tree."""
    edges = {}
    for (p, c), T in tfs.items():
        edges.setdefault(c, []).append((p, T))                 # child -> parent: T
        edges.setdefault(p, []).append((c, np.linalg.inv(T)))  # parent -> child
    stack, seen = [(source, np.eye(4))], {source}
    while stack:
        frame, T = stack.pop()
        if frame == target:
            return T
        for nxt, step in edges.get(frame, []):
            if nxt not in seen:
                seen.add(nxt)
                stack.append((nxt, step @ T))
    raise KeyError(f'no TF path from {source!r} to {target!r}; frames: {sorted(edges)}')
