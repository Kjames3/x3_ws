#!/usr/bin/env python3
"""Shared C1 rosbag dataset: capture via existing owners, audit deterministically.

python3 src/c1_dataset.py record --output /tmp/c1-dataset --seconds 30
python3 src/c1_dataset.py audit /tmp/c1-dataset
No serial/camera device access or robot commands. Requires sourced ROS2 Humble.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sqlite3
import time

TOPICS = {
    '/oak/rgbd/rgb/image_raw': 'sensor_msgs/msg/Image',
    '/oak/rgbd/depth/image_raw': 'sensor_msgs/msg/Image',
    '/oak/rgbd/rgb/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/oak/rgbd/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/oak/rgbd/metadata': 'std_msgs/msg/String',
    '/tof/observations': 'std_msgs/msg/String',
    '/odom': 'nav_msgs/msg/Odometry',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def record(output, seconds):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    from c1_tof_audit import freeze_manifest
    output.mkdir(parents=True, exist_ok=False)
    freeze_manifest(output)
    root = Path(__file__).resolve().parents[1]
    extra = ['src/oakd_driver.py', 'src/oakd_ros_publisher.py', 'src/rgbd_pairing.py',
             'src/c1_dataset.py', 'src/c1_camera_geometry.py', 'src/tof_clock.py', 'src/velocity_estimator.py',
             'src/scaler_params_v3.json', 'config/camera_ground_plane.json',
             'src/yahboomcar_nav/params/ekf_x3.yaml',
             'src/yahboomcar_nav/params/nav2_params_x3.yaml']
    manifest = json.loads((output / 'manifest.json').read_text())
    for name in extra:
        path = root / name
        if path.exists():
            data = path.read_bytes(); dest = output / 'snapshot' / name
            dest.parent.mkdir(parents=True, exist_ok=True); dest.write_bytes(data)
            manifest['files_sha256'][name] = hashlib.sha256(data).hexdigest()
    # Freeze detector/scaler inputs when present; byte hashes remain useful even
    # when large weights are intentionally not duplicated into every capture.
    manifest['models_sha256'] = {}
    for base in [root / 'blobs', root / 'src/blobs', root / 'models']:
        if base.exists():
            for path in base.rglob('*'):
                if path.is_file() and path.suffix in ('.blob', '.json', '.pth', '.pkl', '.pt', '.joblib'):
                    manifest['models_sha256'][str(path.relative_to(root))] = file_sha256(path)
    for name in ['src/velocity_mlp_v3.torchscript', 'src/scaler_params_v3.json']:
        path = root / name
        if path.exists():
            manifest['models_sha256'][name] = file_sha256(path)
    import importlib.metadata
    import platform
    manifest['software_versions'] = {'python': platform.python_version()}
    for package in ['depthai', 'numpy', 'opencv-python', 'torch', 'pyserial']:
        try: manifest['software_versions'][package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: pass
    manifest['topics'] = TOPICS
    manifest['kind'] = 'shared_ros_dataset'
    write_json(output / 'manifest.json', manifest)
    rclpy.init(); node = rclpy.create_node('c1_shared_dataset')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output / 'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    counts = Counter(); subs = []
    for topic, msg_type in TOPICS.items():
        reliable = topic.startswith('/oak/rgbd') or topic == '/tof/observations' or topic == '/tf_static'
        transient = topic == '/tf_static'
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
        offered = ('- history: 1\n  depth: 100\n  reliability: %d\n  durability: %d\n'
                   '  deadline: {sec: 9223372036, nsec: 854775807}\n'
                   '  lifespan: {sec: 9223372036, nsec: 854775807}\n'
                   '  liveliness: 1\n  liveliness_lease_duration: {sec: 9223372036, nsec: 854775807}\n'
                   '  avoid_ros_namespace_conventions: false\n') % (1 if reliable else 2, 1 if transient else 2)
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=msg_type,
                            serialization_format='cdr', offered_qos_profiles=offered))
        def receive(msg, topic=topic):
            writer.write(topic, serialize_message(msg), node.get_clock().now().nanoseconds)
            counts[topic] += 1
        subs.append(node.create_subscription(get_message(msg_type), topic, receive, qos))
    manifest['publishers_at_start'] = {topic: node.count_publishers(topic) for topic in TOPICS}
    print('Waiting up to 10s for first payload on every required topic...', flush=True)
    ready_deadline = time.monotonic() + 10
    while not all(counts[topic] for topic in TOPICS) and time.monotonic() < ready_deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    manifest['topics_ready_before_scoring'] = all(counts[topic] for topic in TOPICS)
    manifest['missing_at_scoring_start'] = [topic for topic in TOPICS if not counts[topic]]
    # Retain pose/TF history around the scored interval for delayed camera data.
    capture_ros_ns = node.get_clock().now().nanoseconds
    manifest['score_window_ros_ns'] = [capture_ros_ns + 2_000_000_000,
                                     capture_ros_ns + 2_000_000_000 + int(seconds * 1e9)]
    manifest['padding_seconds_each_side'] = 2
    print(f'CAPTURE START: {seconds:g}s scored + 4s padding -> {output}', flush=True)
    deadline = time.monotonic() + seconds + 4
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        manifest['received_counts'] = dict(counts)
        manifest['finished_unix_ns'] = time.time_ns()
        write_json(output / 'manifest.json', manifest)
        for sub in subs: node.destroy_subscription(sub)
        node.destroy_node(); rclpy.shutdown()
        del writer
    print('CAPTURE END', dict(counts), flush=True)


def audit(directory):
    """Index complete pairs by original stamps; retain all misses in the report."""
    import numpy as np
    from bisect import bisect_left
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.time import Time
    from rclpy.duration import Duration
    from tf2_ros import Buffer
    from c1_tof_audit import summarize
    bagfiles = sorted((directory / 'bag').glob('*.db3'))
    if not bagfiles:
        raise ValueError('no SQLite bag found')
    counts = Counter(); images = {}; infos = {}; metas = []; tof = []
    odom = []; transforms = []; invalid = Counter(); hashes = {}
    for db in bagfiles:
        hashes[db.name] = file_sha256(db)
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            topics = {id_: (name, get_message(type_)) for id_, name, type_ in conn.execute('select id,name,type from topics')}
            for id_, topic_id, timestamp, payload in conn.execute('select id,topic_id,timestamp,data from messages order by timestamp,id'):
                topic, cls = topics[topic_id]; counts[topic] += 1
                try:
                    msg = deserialize_message(payload, cls)
                    if topic.endswith('/image_raw'):
                        name = 'rgb' if '/rgb/' in topic else 'depth'
                        stamp = stamp_ns(msg.header.stamp); key = (name, stamp)
                        if key in images: invalid['duplicate_image_stamp'] += 1
                        expected = 'bgr8' if name == 'rgb' else '16UC1'
                        width_bytes = 3 if name == 'rgb' else 2
                        valid = (msg.encoding == expected and msg.step == msg.width * width_bytes
                                 and len(msg.data) == msg.height * msg.step and not msg.is_bigendian)
                        if not valid: invalid['bad_image_layout'] += 1
                        data = bytes(msg.data)
                        if name == 'depth' and valid:
                            d = np.frombuffer(data, dtype='<u2')
                            fraction = float(np.count_nonzero(d) / len(d)) if len(d) else 0
                        else: fraction = None
                        images[key] = dict(db=db.name, message_id=id_, stamp_ns=stamp,
                            height=msg.height, width=msg.width, encoding=msg.encoding,
                            frame_id=msg.header.frame_id, sha256=hashlib.sha256(data).hexdigest(),
                            valid_layout=valid, nonzero_fraction=fraction)
                    elif topic.endswith('/camera_info'):
                        name = 'rgb' if '/rgb/' in topic else 'depth'
                        infos[(name, stamp_ns(msg.header.stamp))] = dict(width=msg.width,height=msg.height,
                            k=list(msg.k),d=list(msg.d),frame_id=msg.header.frame_id)
                    elif topic == '/oak/rgbd/metadata': metas.append(json.loads(msg.data))
                    elif topic == '/tof/observations': tof.append(json.loads(msg.data))
                    elif topic == '/odom':
                        odom.append((stamp_ns(msg.header.stamp), msg.header.frame_id, msg.child_frame_id))
                    elif topic in ('/tf', '/tf_static'):
                        transforms.extend((t,topic == '/tf_static') for t in msg.transforms)
                except Exception as exc:
                    invalid[type(exc).__name__] += 1
    buffer = Buffer(cache_time=Duration(seconds=3600))
    for t, static in sorted(transforms, key=lambda x: stamp_ns(x[0].header.stamp)):
        try:
            if static: buffer.set_transform_static(t, 'recorded')
            else: buffer.set_transform(t, 'recorded')
        except Exception: invalid['invalid_tf'] += 1
    odom.sort(); odom_times = [o[0] for o in odom]
    metadata_total = len(metas)
    manifest_path = directory / 'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    score_window = manifest.get('score_window_ros_ns')
    if score_window is not None:
        # Boundary records remain in the bag; state explicitly which are unscored.
        metas = [m for m in metas if score_window[0] <= m.get('depth', {}).get('ros_stamp_ns', -1) <= score_window[1]]
    pairs=[]; skew=[]; seen=set(); age=[]; stamps_by_session={}; offsets=[]; offset_spans=[]
    for meta in metas:
        try:
            identity=(meta['rgb']['session_id'],meta['epoch'],meta['pair_seq'])
            if identity in seen: invalid['duplicate_pair'] += 1; continue
            seen.add(identity)
            delta=abs(meta['depth']['device_ns']-meta['rgb']['device_ns'])
            skew.append(delta / 1e6)
            if delta > 20_000_000: invalid['skew_over_20ms'] += 1; continue
            row=dict(identity=list(identity), skew_ms=delta/1e6, metadata=meta)
            for name in ('rgb','depth'):
                key=(name,meta[name]['ros_stamp_ns'])
                image=images[key];info=infos[key]
                if not image['valid_layout'] or (image['width'],image['height'])!=(info['width'],info['height']):
                    raise ValueError('image info mismatch')
                if info['frame_id'] != image['frame_id'] or image['frame_id'] != meta['frame_id']:
                    raise ValueError('frame mismatch')
                if not np.isfinite(info['k']).all() or info['k'][0] <= 0 or info['k'][4] <= 0:
                    raise ValueError('intrinsics')
                if not np.allclose(info['k'], meta['calibration'][name + '_k']):
                    raise ValueError('metadata intrinsics')
                row[name]=dict(image, camera_info=info)
            stamp=meta['depth']['ros_stamp_ns'];i=bisect_left(odom_times,stamp)
            bracket=(0<i<len(odom_times) and stamp-odom_times[i-1]<=100_000_000 and odom_times[i]-stamp<=100_000_000)
            row['odom_bracketed_100ms']=bracket
            row['odom_bracket_stamps_ns'] = [odom_times[i-1], odom_times[i]] if bracket else None
            if not bracket: invalid['unbracketed_pose'] += 1
            try:
                transform = buffer.lookup_transform('odom',meta['frame_id'],Time(nanoseconds=stamp))
                row['tf_available']=True
                tr = transform.transform.translation; qr = transform.transform.rotation
                row['odom_from_camera'] = dict(stamp_ns=stamp,
                    translation_xyz=[tr.x, tr.y, tr.z], quaternion_xyzw=[qr.x, qr.y, qr.z, qr.w])
            except Exception:
                row['tf_available']=False; invalid['missing_tf_at_capture']+=1
            estimated_age=(meta['publication_monotonic_ns']-meta['depth']['host_monotonic_estimate_ns'])/1e6
            age.append(estimated_age)
            offsets.append(meta['ros_minus_monotonic_ns'])
            offset_spans.append(meta['ros_clock_sample_span_ns'])
            stamps_by_session.setdefault(identity[:2],[]).append(meta['depth']['device_ns'])
            pairs.append(row)
        except (KeyError, ValueError, TypeError): invalid['incomplete_or_invalid_pair']+=1
    intervals=[(b-a)/1e6 for ts in stamps_by_session.values() for a,b in zip(ts,ts[1:])]
    expected={'upper','lower'}
    tof_report=summarize(tof)
    if {s['sensor'] for s in tof_report['streams'] if s['accepted']} != expected: invalid['missing_tof_sensor']+=1
    for topic in TOPICS:
        if not counts[topic]: invalid['missing_topic:'+topic]+=1
    offset_jump=max((abs(b-a)/1e6 for a,b in zip(offsets,offsets[1:])),default=0)
    if any(x > 500 for x in intervals):
        invalid['pair_gap_over_500ms'] = sum(x > 500 for x in intervals)
    if any(x > 500 for x in age):
        invalid['estimated_depth_age_over_500ms'] = sum(x > 500 for x in age)
    # A preemption during clock sampling widens the offset interval. Only the
    # separation beyond both brackets is evidence of an actual clock change.
    offset_jump_lower = max((max(0, abs(offsets[i]-offsets[i-1]) -
                                  (offset_spans[i]+offset_spans[i-1])/2)/1e6
                             for i in range(1, len(offsets))), default=0)
    if offset_jump_lower > 10:
        invalid['ros_clock_offset_jump_beyond_sampling_over_10ms'] += 1
    report=dict(schema='x3.c1.dataset.audit.v1',topic_counts=dict(counts),bag_sha256=hashes,
                pair_metadata_records=metadata_total, scored_pair_metadata_records=len(metas),
                boundary_pairs_unscored=metadata_total-len(metas), score_window_ros_ns=score_window,
                complete_pairs=len(pairs),failures=dict(invalid),
                rgb_depth_skew_ms_p95=float(np.percentile(skew,95)) if skew else None,
                rgb_depth_skew_ms_max=max(skew,default=None),
                max_pair_interval_ms=max(intervals,default=None),
                pair_intervals_over_500ms=sum(x>500 for x in intervals),
                pair_timestamp_reversals=sum(x<=0 for x in intervals),
                pairing_counters_last=metas[-1].get('counters', {}) if metas else {},
                publisher_queue_drops_last=metas[-1].get('publisher_queue_drops') if metas else None,
                ros_clock_offset_jump_ms_max=offset_jump,
                ros_clock_offset_jump_beyond_sampling_ms_max=offset_jump_lower,
                estimated_depth_age_ms_p95=float(np.percentile(age,95)) if age else None,
                tof=tof_report, c1_complete=False,
                unresolved=['MCU clock/acquisition bounds require qualification.',
                            'DepthAI SDK clock estimate is not a measured hard error bound.',
                            'RGB/depth spatial registration must be checked on the physical payload.'])
    write_json(directory/'pair-index.json',pairs)
    write_json(directory/'audit.json',report)
    print(json.dumps(report,indent=2))
    return report


def load_pair(directory, index):
    """Load original pixels for a pair-index row, checking the recorded hash.

    Returns arrays and the full indexed provenance. Does not resample depth,
    silently register pixels, interpolate pose, or refresh acquisition times.
    """
    import numpy as np
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    directory = Path(directory)
    row = json.loads((directory / 'pair-index.json').read_text())[index]
    arrays = {}
    for name in ('rgb', 'depth'):
        ref = row[name]
        db = directory / 'bag' / ref['db']
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            result = conn.execute('select data from messages where id=?',
                                  (ref['message_id'],)).fetchone()
        if result is None:
            raise ValueError('indexed message missing')
        msg = deserialize_message(result[0], Image)
        raw = bytes(msg.data)
        if hashlib.sha256(raw).hexdigest() != ref['sha256']:
            raise ValueError('image hash mismatch')
        if stamp_ns(msg.header.stamp) != ref['stamp_ns']:
            raise ValueError('image timestamp mismatch')
        shape = (ref['height'], ref['width'], 3) if name == 'rgb' else (ref['height'], ref['width'])
        arrays[name] = np.frombuffer(raw, dtype='uint8' if name == 'rgb' else '<u2').reshape(shape).copy()
    return arrays, row


def rgb_on_depth_grid(arrays, row):
    """Derived rectified RGB and support mask; original depth/pixels stay intact.

    Uses the saved CAM_A calibration. This is a nominal calibration transform,
    not a claim of measured registration accuracy or simultaneous exposure.
    """
    import cv2
    import numpy as np
    calibration = row['metadata']['calibration']
    k_rgb = np.asarray(calibration['rgb_k'], dtype=float).reshape(3, 3)
    k_depth = np.asarray(calibration['depth_k'], dtype=float).reshape(3, 3)
    distortion = np.asarray(calibration['rgb_d'], dtype=float)
    height, width = arrays['depth'].shape
    x, y = cv2.initUndistortRectifyMap(k_rgb, distortion, np.eye(3), k_depth,
                                      (width, height), cv2.CV_32FC1)
    rgb = cv2.remap(arrays['rgb'], x, y, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    support = cv2.remap(np.ones(arrays['rgb'].shape[:2], dtype='float32'),
                       x, y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                       borderValue=0) >= 1.0
    return rgb, support


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    rec=sub.add_parser('record');rec.add_argument('--output',type=Path,required=True);rec.add_argument('--seconds',type=float,default=30)
    replay=sub.add_parser('audit');replay.add_argument('directory',type=Path)
    args=parser.parse_args()
    if args.mode=='record':
        if args.seconds<=0: parser.error('seconds must be positive')
        record(args.output,args.seconds)
        report=audit(args.output)
    else: report=audit(args.directory)
    return 0 if report['complete_pairs'] and not report['failures'] else 2


if __name__=='__main__': raise SystemExit(main())
