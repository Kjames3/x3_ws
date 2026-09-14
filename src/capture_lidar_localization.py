"""New-data stop-and-go localization capture. User starts and drives this run.

First station is the starting pose: after preflight, keep hands off for its
10-second sweep. Later stations require --min-move-m travel (default 0.45 m) and 2.5 s stillness.
--probe reads input only. --label selects a new, never-overwritten output file.
All interlocks remain active. No chassis commands or motor enable are sent.
"""

import argparse
import asyncio
import json
import os
import threading
import time
from collections import Counter, OrderedDict

import numpy as np
import rclpy
import tf2_ros
import websockets
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2

from yahboomcar_bringup.lidar_deskew import rotate
from lidar_capture_quality import sweep_quality

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'artifacts', 'localization-validation')
MIN_MOVE_M = 0.45        # a station must be meaningfully away from the last
STILL_S = 2.5            # hold still this long to trigger (> the 1.5 s lockout)
STILL_EPS_M = 0.02       # odom jitter that still counts as stopped
SWEEP_S = 10.0           # ~2.5 ping-pong cycles at 45 deg/s
SWEEP_SPEED = 45.0

rclpy.init()
node = Node('registration_drive_capture')
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, node)

counts = Counter()
pending = []
joints = []
odom = []                # monotonic reception time, used only for stillness
odom_ros = []            # ROS acquisition timestamp + odom pose
cloud_odom = []          # cloud id + ROS stamp + exact-time TF base pose
record_after_ns = 0
capture_lock = threading.RLock()

def set_station(value):
    global station
    with capture_lock:
        station = value
cmd_max = 0.0
station = -1             # -1 = not recording
cloud_points, cloud_ids, cloud_meta = [], [], []
stations = []          # (index, x, y, yaw) captured at sweep start
_next_cloud_id = 0


def key(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec


def on_joint(m):
    if 'lidar_tilt_joint' not in m.name:
        return
    i = m.name.index('lidar_tilt_joint')
    joints.append([key(m) * 1e-9, m.position[i], float(m.velocity[i] > .5)])


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y ** 2 + q.z ** 2))
    odom.append([time.monotonic(), p.x, p.y, float(yaw)])
    odom_ros.append([key(m)*1e-9, p.x, p.y, float(yaw)])


_last_nonzero_cmd = float('-inf')


def on_cmd(m):
    global cmd_max, _last_nonzero_cmd
    mag = max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z))
    cmd_max = max(cmd_max, mag)
    # server_x3._enqueue_motion stamps _last_teleop_motion_t on ANY nonzero
    # command, including one too small to overcome the motor deadband. A
    # joystick resting slightly off-centre therefore holds the interlock shut
    # while odom reads perfectly still -- which is exactly how the first run
    # died at station 6. Gate on the same signal the server gates on.
    if mag > 0.0:
        _last_nonzero_cmd = time.monotonic()


def on_cloud(m):
    counts['cloud'] += 1
    pending.append(m)


def on_scan(m):
    counts['scan'] += 1


def process():
    """Transform each cloud to base_footprint and keep it WITH its identity."""
    with capture_lock:
        global _next_cloud_id
        while pending:
            m = pending[0]
            if station < 0 or key(m) < record_after_ns:
                pending.pop(0)
                continue
            try:
                t = buf.lookup_transform('base_footprint', m.header.frame_id,
                                         Time.from_msg(m.header.stamp))
                o = buf.lookup_transform('odom', 'base_footprint', Time.from_msg(m.header.stamp)).transform
            except tf2_ros.TransformException:
                if len(pending) > 10:
                    pending.pop(0)
                    counts['tf_failed'] += 1
                    continue
                return
            pending.pop(0)
            if station < 0:
                continue                      # not recording between stations
            xyz = np.frombuffer(m.data, dtype='<f4').reshape(-1, 3).copy()
            q, v = t.transform.rotation, t.transform.translation
            local = rotate(xyz, np.array([q.x, q.y, q.z, q.w])) + \
                np.array([v.x, v.y, v.z])
            cloud_points.append(local.astype(np.float32))
            cloud_ids.append(np.full(len(local), _next_cloud_id, dtype=np.int32))
            cloud_meta.append([_next_cloud_id, station, key(m) * 1e-9, len(local)])
            oq = o.rotation
            oyaw = np.arctan2(2*(oq.w*oq.z+oq.x*oq.y), 1-2*(oq.y**2+oq.z**2))
            cloud_odom.append([_next_cloud_id, key(m)*1e-9, o.translation.x, o.translation.y, float(oyaw)])
            _next_cloud_id += 1
            counts['recorded'] += 1


for typ, topic, cb, qos in [
        (PointCloud2, '/pointcloud_raw', on_cloud, 10),
        (JointState, '/lidar_tilt/joint_states', on_joint, 10),
        (LaserScan, '/scan', on_scan, qos_profile_sensor_data),
        (Twist, '/cmd_vel', on_cmd, 10),
        (Odometry, '/odom', on_odom, 10)]:
    node.create_subscription(typ, topic, cb, qos)
node.create_timer(0.03, process)

_stop = False


def spin():
    while not _stop:
        rclpy.spin_once(node, timeout_sec=0.05)


def pose_now():
    if not odom or time.monotonic()-odom[-1][0] > .5:
        raise RuntimeError('odometry stopped; aborting capture')
    return np.array(odom[-1][1:]) if odom else None


def still_since():
    """How long the robot has been within STILL_EPS_M, in seconds."""
    if len(odom) < 2:
        return 0.0
    here = np.array(odom[-1][1:3])
    for i in range(len(odom) - 1, -1, -1):
        if np.linalg.norm(np.array(odom[i][1:3]) - here) > STILL_EPS_M:
            return odom[-1][0] - odom[i][0]
    return odom[-1][0] - odom[0][0]


class SweepRefused(RuntimeError):
    """The server declined to start a sweep. Recoverable: wait and retake."""


sweep_events = []


async def drain(ws, seconds, until=None, monitor=False):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), min(.3, max(.001, end-time.monotonic()))))
            if m.get('type') in ('3d_scan_status', 'sweep_config'):
                sweep_events.append(dict(received_at=time.time(), station=station, message=m))
            if m.get('type') == '3d_scan_status' and m.get('refused'):
                raise SweepRefused(m.get('reason', 'unknown'))
            if monitor and m.get('type') == 'sweep_config' and (
                    m.get('mode') != 'continuous' or m.get('speed_deg_s') != SWEEP_SPEED or m.get('note')):
                raise RuntimeError('Sweep configuration changed during station: %s' % m)
            if monitor and m.get('type') == '3d_scan_status' and not m.get('enabled'):
                raise RuntimeError('Sweep stopped before capture finished: %s' % m)
            if until and until(m):
                return m
        except asyncio.TimeoutError:
            pass
    if until:
        raise RuntimeError('Timed out waiting for sweep acknowledgement')


async def capture(n_stations, probe):
    global station, cmd_max, record_after_ns
    async with websockets.connect('ws://localhost:8081', max_size=None) as ws:
        await drain(ws, 4)
        print('preflight: %s' % dict(counts), flush=True)
        if not odom or len(joints) < 10:
            raise RuntimeError('live inputs missing (odom/joints)')
        print('preflight OK (odom/joints present).', flush=True)
        if probe:
            return

        print('Station 1 is HERE. Keep hands off; do not drive until it is kept.', flush=True)
        last = pose_now()[:2]
        taken = 0
        refusals = 0
        while taken < n_stations:
            here = pose_now()[:2]
            moved = np.linalg.norm(here - last)
            quiet = time.monotonic() - _last_nonzero_cmd
            if (taken > 0 and moved < MIN_MOVE_M) or still_since() < STILL_S or quiet < STILL_S:
                if moved >= MIN_MOVE_M and still_since() >= STILL_S and quiet < STILL_S:
                    if int(time.monotonic()) % 5 == 0:
                        print('  waiting: stick still sending commands '
                              '(%.1f s quiet, need %.1f) -- check for drift'
                              % (quiet, STILL_S), flush=True)
                await asyncio.sleep(0.25)
                continue

            for i in range(len(cloud_meta)-1, -1, -1):
                if cloud_meta[i][1] == taken:
                    cloud_meta.pop(i); cloud_points.pop(i); cloud_ids.pop(i); cloud_odom.pop(i)
            # Reassert and acknowledge the requested style at EVERY station.
            await ws.send(json.dumps({'type': 'set_sweep_config', 'mode': 'continuous',
                                      'speed_deg_s': SWEEP_SPEED}))
            config = await drain(ws, 20, until=lambda m: m.get('type') == 'sweep_config')
            if config.get('mode') != 'continuous' or config.get('note'):
                raise RuntimeError('Continuous sweep configuration failed: %s' % config)
            record_after_ns = node.get_clock().now().nanoseconds
            set_station(taken)
            cmd_max = 0.0
            start_pose = pose_now()
            n0 = counts['recorded']
            print('\n[station %d/%d] parked %.2f m from the last one at '
                  '(%.2f, %.2f) -- sweeping %.0f s, HANDS OFF'
                  % (taken + 1, n_stations, moved, start_pose[0],
                     start_pose[1], SWEEP_S), flush=True)
            try:
                await ws.send(json.dumps({'type': 'toggle_3d_scan',
                                          'enabled': True, 'mode': 'continuous'}))
                await drain(ws, 20, until=lambda m: m.get('type') == '3d_scan_status' and m.get('enabled'), monitor=True)
                await drain(ws, SWEEP_S, monitor=True)
            except SweepRefused as e:
                set_station(-1)
                refusals += 1
                if refusals >= 3 or str(e).startswith('processor gate:'):
                    raise RuntimeError('Sweep could not start: %s; keep parked for diagnosis' % e) from e
                print('  REFUSED (%s) -- letting the interlock clear, retaking.'
                      % e, flush=True)
                await ws.send(json.dumps({'type': 'toggle_3d_scan',
                                          'enabled': False}))
                try:
                    await drain(ws, 3.0)
                except SweepRefused:
                    pass
                continue
            await ws.send(json.dumps({'type': 'toggle_3d_scan', 'enabled': False}))
            try:
                await drain(ws, 2.5)
            except SweepRefused:
                pass
            set_station(-1)

            end_pose = pose_now()
            drift = float(np.linalg.norm(end_pose[:2] - start_pose[:2]))
            got = counts['recorded'] - n0
            yaw_drift = abs(float(np.arctan2(np.sin(end_pose[2]-start_pose[2]), np.cos(end_pose[2]-start_pose[2]))))
            if cmd_max > 0.01 or drift > 0.03 or yaw_drift > np.deg2rad(2):
                print('  DISCARDED: robot moved during the sweep '
                      '(cmd %.3f, drift %.3f m). Retaking.'
                      % (cmd_max, drift), flush=True)
                for i in range(len(cloud_meta) - 1, -1, -1):
                    if cloud_meta[i][1] == taken:
                        cloud_meta.pop(i)
                        cloud_points.pop(i)
                        cloud_ids.pop(i)
                        cloud_odom.pop(i)
                continue
            with capture_lock:
                metadata = [m[:] for m in cloud_meta if m[1] == taken]
                joint_snapshot = list(joints)
            valid, quality = sweep_quality(metadata, joint_snapshot)
            print('  sweep quality: %s' % json.dumps(quality), flush=True)
            if not valid:
                # Stop the run instead of silently accepting partial geometry
                # or repeating a hardware fault indefinitely.
                raise RuntimeError('Station %d incomplete; saved earlier accepted stations only. '
                                   'Keep parked for diagnosis.' % (taken + 1))
            stations.append([taken, float(start_pose[0]), float(start_pose[1]),
                             float(start_pose[2])])
            print('  kept %d clouds, drift %.3f m. %s'
                  % (got, drift, 'Drive to the next spot.' if taken+1 < n_stations else 'Capture complete; stay parked.'), flush=True)
            last = here
            taken += 1
            refusals = 0
        print('\nall %d stations captured.' % n_stations, flush=True)


async def main(n_stations, probe):
    try:
        await capture(n_stations, probe)
    finally:
        if not probe:
            try:
                async with websockets.connect('ws://localhost:8081', max_size=None, close_timeout=1) as ws:
                    await ws.send(json.dumps({'type':'toggle_3d_scan','enabled':False}))
                    await ws.send(json.dumps({'type':'stop'}))
                    await ws.send(json.dumps({'type':'set_sweep_config','mode':'step'}))
                    await asyncio.sleep(1)
            except Exception as e:
                print('Cleanup connection failed; use GUI Stop: %s' % e, flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--stations', type=int, default=8)
    ap.add_argument('--probe', action='store_true')
    ap.add_argument('--label', required=True)
    ap.add_argument('--min-move-m', type=float, default=MIN_MOVE_M,
                    help='Minimum odometry displacement between stops; 0.25 m for doorway mapping')
    args = ap.parse_args()
    if not np.isfinite(args.min_move_m) or not .2 <= args.min_move_m <= 2.:
        ap.error('min-move-m must be finite and between 0.2 and 2.0')
    MIN_MOVE_M = args.min_move_m
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.label):
        ap.error('label must contain only letters, digits, underscores or hyphens')
    if not 1 <= args.stations <= 30: ap.error('stations must be 1..30')
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, args.label + '.npz')
    if os.path.exists(path): ap.error('output already exists; choose a new label')

    th = threading.Thread(target=spin)
    th.start()
    try:
        asyncio.run(main(args.stations, args.probe))
    finally:
        set_station(-1)
        _stop = True
        th.join()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        # Save only completed, accepted stations, including on Ctrl-C.
        accepted = {int(st[0]) for st in stations}
        for i in range(len(cloud_meta)-1, -1, -1):
            if int(cloud_meta[i][1]) not in accepted:
                cloud_meta.pop(i); cloud_points.pop(i); cloud_ids.pop(i); cloud_odom.pop(i)
        if cloud_points:
            np.savez_compressed(
                path,
                points=np.concatenate(cloud_points),
                cloud_id=np.concatenate(cloud_ids),
                cloud_meta=np.array(cloud_meta, dtype=float),
                joints=np.array(joints),
                odom=np.array(odom),
                odom_ros=np.array(odom_ros),
                cloud_odom=np.array(cloud_odom),
                stations=np.array(stations, dtype=float))
            print('wrote %s: %d points, %d clouds, %d stations'
                  % (path, sum(len(c) for c in cloud_points), len(cloud_meta),
                     len({int(m[1]) for m in cloud_meta})), flush=True)
        if not args.probe:
            with open(path.removesuffix('.npz') + '_sweep_events.json', 'w') as event_file:
                json.dump(sweep_events, event_file, indent=2)
        print('counts: %s' % dict(counts), flush=True)
