"""Phase 1: does deskew compensate CHASSIS motion? Mount level, no sweep.

Tilt-motion deskew is already validated (parked continuous sweep, floor p95
7.87 -> 4.07 cm). The chassis half has never been tested: the deskew reads
odom->mount at scan start and end and interpolates, so its accuracy under
driving depends on odometry over a 137 ms window, and nothing has checked that.

This isolates that one variable. The mount stays LEVEL and no sweep runs, so
neither interlock is touched -- `_tilt_nav_conflict`'s teleop arm never fires
and `ROS2Bridge.move()`'s `if lidar_3d_scan_enabled: zero everything` guard is
never armed. /scan keeps flowing, so the CBF stays live and you drive with full
obstacle protection. It does not touch the live processor at all. Instead it launches a SECOND,
isolated instance publishing to /motion_test/*, so the production node keeps
serving /scan and the CBF throughout.

That indirection is not paranoia, it is the only thing that works:
`publish_cloud_when_level` is CACHED in the node's __init__, so `ros2 param
set` reports "Set parameter successful" and changes nothing. A 90 s capture on
2026-09-05 recorded zero clouds that way. Setting it at LAUNCH sidesteps the
cache. (The node has since been fixed to read it live, but launching our own
instance is still the safer pattern -- it cannot perturb the live stack.)

THE MEASUREMENT is an A/B on the same returns. For every scan we keep both:

  * the DESKEWED cloud from /pointcloud_raw, and
  * a RIGID cloud rebuilt from /lidar/points_timed with no compensation.

A level scan sees walls, which are straight lines in the horizontal plane.
Uncompensated chassis motion shears them, and the shear grows with speed. So
if the rigid residual grows with speed while the deskewed residual stays flat,
compensation works; if both grow together, it does not.

Yaw rate matters far more than translation: 0.5 rad/s over 137 ms is 3.9 deg,
which at 3 m is 0.20 m of smear, against 0.04 m for 0.3 m/s of translation.
Results are therefore binned by yaw rate as well as speed.

Usage (drive with the controller throughout; ~90 s is plenty):

    python3 motion_capture.py --probe        # preflight, changes nothing
    python3 motion_capture.py --seconds 90
"""

import argparse
import json
import os
import subprocess
import threading
import time
from collections import Counter, OrderedDict

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan, PointCloud, PointCloud2

OUT = os.path.dirname(os.path.abspath(__file__))
LEVEL_TOL_RAD = 0.05
EXE = ('/home/jetson/x3_ws/install/yahboomcar_bringup/lib/'
       'yahboomcar_bringup/lidar_3d_processor_node')
NS = '/motion_test'
# `ros2 run` execs the node as a CHILD, so terminate() kills the wrapper and
# ORPHANS the node -- 19 strays accumulated during the 2026-08-29 bench that
# way. Launch the executable directly, in its own session, and kill the group.
CLOUD_TOPIC = NS + '/pointcloud_raw'

rclpy.init()
node = Node('motion_deskew_capture')
counts = Counter()
timed = OrderedDict()
odom = []                 # (t_ros, x, y, yaw, vx, vy, wz)
tilt = []                 # (t_ros, rad, gate)
pairs = []                # matched (deskewed, rigid) per scan
_stop = False


def key(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    t = m.twist.twist
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    odom.append([key(m) * 1e-9, p.x, p.y, float(yaw),
                 t.linear.x, t.linear.y, t.angular.z])


def on_tilt(m):
    if 'lidar_tilt_joint' not in m.name:
        return
    i = m.name.index('lidar_tilt_joint')
    tilt.append([key(m) * 1e-9, m.position[i], float(m.velocity[i] > .5)])


def on_timed(m):
    counts['timed'] += 1
    timed[key(m)] = m
    while len(timed) > 120:
        timed.popitem(last=False)


def on_cloud(m):
    """Deskewed cloud; pair it with the raw timed scan of the same stamp."""
    counts['cloud'] += 1
    raw = timed.get(key(m))
    if raw is None:
        counts['unpaired'] += 1
        return
    xyz = np.frombuffer(m.data, dtype='<f4').reshape(-1, 3)
    rigid = np.array([(p.x, p.y, p.z) for p in raw.points], dtype=np.float32)
    # The processor drops returns outside [min_range, cloud_max_range); rebuild
    # the same mask so the two clouds describe the SAME returns.
    r = np.linalg.norm(rigid, axis=1)
    rigid = rigid[(r > 0) & (r < 6.0) & np.isfinite(rigid).all(axis=1)]
    if rigid.shape != xyz.shape:
        counts['shape_mismatch'] += 1
        return
    pairs.append({'t': key(m) * 1e-9, 'deskew': xyz.copy(), 'rigid': rigid})
    counts['paired'] += 1


def on_scan(m):
    counts['scan'] += 1


for typ, topic, cb, qos in [
        (PointCloud, '/lidar/points_timed', on_timed, qos_profile_sensor_data),
        (PointCloud2, CLOUD_TOPIC, on_cloud, 10),
        (JointState, '/lidar_tilt/joint_states', on_tilt, 10),
        (LaserScan, '/scan', on_scan, qos_profile_sensor_data),
        (Odometry, '/odom', on_odom, 10)]:
    node.create_subscription(typ, topic, cb, qos)


def spin():
    while not _stop:
        rclpy.spin_once(node, timeout_sec=0.05)


def start_processor():
    """Our own processor instance, isolated from the production one.

    Every OUTPUT is redirected: the cloud and scan topics are parameters, and
    /lidar_tilt/is_level is hardcoded in the node so it needs a remap. Miss any
    one of them and this instance publishes a competing /scan straight into
    slam_toolbox and the CBF.
    """
    cmd = [EXE, '--ros-args',
           '-r', '__node:=lidar_3d_processor_motion_test',
           '-r', '/lidar_tilt/is_level:=' + NS + '/is_level',
           '-p', 'publish_cloud_when_level:=true',
           '-p', 'scan_out_topic:=' + NS + '/scan',
           '-p', 'cloud_out_topic:=' + CLOUD_TOPIC,
           '-p', 'cloud_max_range_m:=6.0',
           '-p', 'require_settled:=true']
    log = open(os.path.join(OUT, 'processor.log'), 'w')
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)


def stop_processor(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), 15)
        proc.wait(timeout=8)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), 9)
        except Exception:
            pass


def velocity_at(t):
    """Chassis twist nearest this scan stamp."""
    if not odom:
        return None
    a = np.array(odom)
    i = int(np.argmin(np.abs(a[:, 0] - t)))
    if abs(a[i, 0] - t) > 0.25:
        return None
    return float(np.hypot(a[i, 4], a[i, 5])), float(a[i, 6])


def main(seconds, probe):
    th = threading.Thread(target=spin)
    th.start()
    proc = None
    try:
        time.sleep(4.0)
        print('preflight: %s' % dict(counts), flush=True)
        if not tilt:
            raise RuntimeError('no tilt source: is x3_server running?')
        pitch = tilt[-1][1]
        if abs(pitch) > LEVEL_TOL_RAD:
            raise RuntimeError('mount is at %.1f deg, not level -- home it '
                               'first (python3 src/dynamixel_tilt.py --home)'
                               % np.degrees(pitch))
        if any(t[2] > 0.5 for t in tilt[-20:]):
            raise RuntimeError('the mount is MOVING; a sweep must not be running')
        if not odom:
            raise RuntimeError('no /odom')
        print('mount level at %.2f deg, no sweep, /scan alive (%d msgs).'
              % (np.degrees(pitch), counts['scan']), flush=True)
        if probe:
            return

        if not os.path.exists(EXE):
            raise RuntimeError('processor executable not found at %s -- build '
                               'yahboomcar_bringup first' % EXE)
        proc = start_processor()
        time.sleep(6.0)
        if proc.poll() is not None:
            raise RuntimeError('the test processor exited immediately; see '
                               'processor.log')
        if counts['cloud'] == 0:
            raise RuntimeError('test processor started but published no cloud '
                               'in 6 s; see processor.log')
        print('\ntest processor up on %s (%d clouds already). DRIVE NOW for '
              '%.0f s -- vary speed, and include some on-the-spot rotation '
              '(yaw rate dominates smear).\n'
              % (CLOUD_TOPIC, counts['cloud'], seconds), flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            time.sleep(2.0)
            print('  %3.0f s: %d clouds paired' % (time.monotonic() - t0,
                                                   counts['paired']), flush=True)
    finally:
        stop_processor(proc)
        _stop_all()
        save()


def _stop_all():
    global _stop
    _stop = True
    time.sleep(0.3)
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


def save():
    if not pairs:
        print('no paired clouds captured; nothing written', flush=True)
        print('counts: %s' % dict(counts), flush=True)
        return
    dk, rg, ids, meta = [], [], [], []
    for i, p in enumerate(pairs):
        v = velocity_at(p['t'])
        if v is None:
            continue
        dk.append(p['deskew'])
        rg.append(p['rigid'])
        ids.append(np.full(len(p['deskew']), i, dtype=np.int32))
        meta.append([i, p['t'], len(p['deskew']), v[0], v[1]])
    path = os.path.join(OUT, 'motion_capture.npz')
    np.savez_compressed(path,
                        deskew=np.concatenate(dk), rigid=np.concatenate(rg),
                        cloud_id=np.concatenate(ids),
                        meta=np.array(meta, dtype=float),
                        odom=np.array(odom), tilt=np.array(tilt))
    m = np.array(meta, dtype=float)
    print('\nwrote %s: %d clouds' % (path, len(meta)), flush=True)
    print('speed  p50 %.2f max %.2f m/s | yaw rate p50 %.2f max %.2f rad/s'
          % (np.median(m[:, 3]), m[:, 3].max(),
             np.median(np.abs(m[:, 4])), np.abs(m[:, 4]).max()), flush=True)
    still = (m[:, 3] < 0.02) & (np.abs(m[:, 4]) < 0.05)
    print('parked baseline clouds: %d of %d' % (still.sum(), len(m)), flush=True)
    if still.sum() < 10:
        print('WARNING: few stationary clouds -- the A/B needs a parked '
              'baseline. Sit still for ~10 s next run.', flush=True)
    print('counts: %s' % dict(counts), flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=90.0)
    ap.add_argument('--probe', action='store_true')
    a = ap.parse_args()
    main(a.seconds, a.probe)
