#!/usr/bin/env python3
"""Hardware tests for the tilting 2D->3D lidar chain.

Run ON the robot, with x3_server up:

    python3 src/lidar_3d_hw_test.py probe            # read-only, 14 s
    python3 src/lidar_3d_hw_test.py sweep [seconds]  # drives a real sweep
    python3 src/lidar_3d_hw_test.py interlock        # Nav2 interlock arms

`probe` is safe any time -- it only listens.  `sweep` and `interlock` both
command the mount and toggle GUI state, so keep Nav2 idle and the robot parked.

What each check is really testing:

  probe      that exactly one publisher owns lidar_tilt_joint.  When
             joint_state_publisher fights server_x3.py for /joint_states, the
             tilt reads 0 most of the time and TF flips between 0 and the true
             angle -- measured at 66% zeros / 32 flips per 14 s before the fix.

  sweep      that a tilted 2D scan really does produce 3D.  With every beam at
             range R, a mount tilted by beta puts points at elevation +/-beta,
             so the cloud's vertical extent in base_footprint is the proof.
             Also asserts /scan goes silent while tilted, because slam_toolbox
             and AMCL would otherwise be fed a tilted scan plane.

  interlock  that a sweep and Nav2 motion can never overlap in either
             direction.
"""

import asyncio
import json
import math
import sys
import threading
import time

WS = 'ws://127.0.0.1:8081'
LIDAR_Z = 0.345          # base_footprint -> laser_link, from the URDF chain
LEVEL_DEG = 2.9          # lidar_3d_processor_node's gate, in degrees


# ── shared ROS plumbing ─────────────────────────────────────────────────────
def _ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2, JointState
    from sensor_msgs_py import point_cloud2
    import tf2_ros
    return (rclpy, Node, qos_profile_sensor_data, LaserScan, PointCloud2,
            JointState, point_cloud2, tf2_ros)


def _quat_to_R(q):
    import numpy as np
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def _pitch_deg(q):
    sp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
    return math.degrees(math.asin(sp))


class _Collector:
    """Subscribes to everything both probe and sweep need."""

    def __init__(self, node_name):
        (self.rclpy, Node, sensor_qos, LaserScan, PointCloud2, JointState,
         self.pc2, tf2_ros) = _ros()
        import numpy as np
        self.np = np
        self.rclpy.init()
        self.node = Node(node_name)
        self.LaserScan = LaserScan

        self.raw_n = 0
        self.gated_n = 0
        self.gated_while_tilted = 0
        self.violations = []      # tilt (deg) at each /scan-while-tilted event
        self.stats = None
        self.js_merged = []
        self.js_tilt = []
        self.tf_pitch = []
        self.cloud_n = 0
        self.zmin, self.zmax = 1e9, -1e9
        self.elev = []
        self.cur_tilt = 0.0

        n = self.node
        n.create_subscription(LaserScan, '/scan_raw', self._on_raw, sensor_qos)
        n.create_subscription(LaserScan, '/scan', self._on_gated, sensor_qos)
        n.create_subscription(JointState, '/joint_states', self._on_js_merged, 10)
        n.create_subscription(JointState, '/lidar_tilt/joint_states',
                              self._on_js_tilt, 10)
        n.create_subscription(PointCloud2, '/pointcloud_raw', self._on_cloud, 10)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, n)
        n.create_timer(0.05, self._sample_tf)

        self._stop = threading.Event()
        self._th = threading.Thread(target=self._spin, daemon=True)
        self._th.start()

    def _spin(self):
        while not self._stop.is_set() and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def stop(self):
        self._stop.set()
        self._th.join(timeout=3)
        if self.rclpy.ok():
            self.rclpy.shutdown()

    # ── callbacks ───────────────────────────────────────────────────────
    def _on_raw(self, m):
        self.raw_n += 1
        if self.stats is None and self.raw_n >= 3:
            r = sorted(x for x in m.ranges if math.isfinite(x) and x > 0)
            if r:
                self.stats = dict(
                    frame=m.header.frame_id, beams=len(m.ranges), finite=len(r),
                    mn=r[0], p10=r[len(r)//10], med=r[len(r)//2],
                    p90=r[9*len(r)//10], mx=r[-1],
                    lt06=100.0*sum(1 for x in r if x < 0.60)/len(r))

    def _on_gated(self, m):
        self.gated_n += 1
        if abs(self.cur_tilt) > LEVEL_DEG:
            self.gated_while_tilted += 1
            self.violations.append(self.cur_tilt)

    @staticmethod
    def _tilt_of(m):
        if 'lidar_tilt_joint' in m.name:
            i = m.name.index('lidar_tilt_joint')
            if i < len(m.position):
                return math.degrees(m.position[i])
        return None

    def _on_js_merged(self, m):
        v = self._tilt_of(m)
        if v is not None:
            self.js_merged.append(v)

    def _on_js_tilt(self, m):
        v = self._tilt_of(m)
        if v is not None:
            self.cur_tilt = v
            self.js_tilt.append(v)

    def _sample_tf(self):
        try:
            t = self.buf.lookup_transform('base_footprint', 'laser_link',
                                          self.rclpy.time.Time())
        except Exception:
            return
        self.tf_pitch.append(_pitch_deg(t.transform.rotation))

    def _on_cloud(self, msg):
        self.cloud_n += 1
        try:
            t = self.buf.lookup_transform('base_footprint', msg.header.frame_id,
                                          self.rclpy.time.Time())
        except Exception:
            return
        np = self.np
        pts = np.array([[p[0], p[1], p[2]] for p in self.pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)])
        if pts.size == 0:
            return
        tr = t.transform.translation
        w = pts @ _quat_to_R(t.transform.rotation).T + np.array([tr.x, tr.y, tr.z])
        self.zmin = min(self.zmin, float(w[:, 2].min()))
        self.zmax = max(self.zmax, float(w[:, 2].max()))
        horiz = np.hypot(w[:, 0], w[:, 1])
        ok = horiz > 0.2
        if ok.any():
            e = np.degrees(np.arctan2(w[ok, 2] - LIDAR_Z, horiz[ok]))
            self.elev.append((float(e.min()), float(e.max())))


def _hdr(title):
    print('=' * 72)
    print('  ' + title)
    print('=' * 72)


# ── probe ───────────────────────────────────────────────────────────────────
def cmd_probe(duration=14.0):
    c = _Collector('lidar3d_probe')
    time.sleep(duration)
    c.stop()

    _hdr('READ-ONLY PROBE (%.0f s)' % duration)
    s = c.stats
    if s:
        print('/scan_raw frame=%s beams=%d finite=%d'
              % (s['frame'], s['beams'], s['finite']))
        print('  ranges  min=%.2f p10=%.2f med=%.2f p90=%.2f max=%.2f  (%%<0.6m=%.1f)'
              % (s['mn'], s['p10'], s['med'], s['p90'], s['mx'], s['lt06']))
    else:
        print('/scan_raw : NO DATA')
    print('/scan_raw msgs=%d   /scan msgs=%d' % (c.raw_n, c.gated_n))
    print('-' * 72)

    fails = []
    for label, arr in (('/joint_states', c.js_merged),
                       ('/lidar_tilt/joint_states', c.js_tilt)):
        if arr:
            zero = sum(1 for v in arr if abs(v) < 0.01)
            pct = 100.0 * zero / len(arr)
            print('%-26s n=%4d  min=%+.2f max=%+.2f deg  exactly-zero=%.0f%%'
                  % (label, len(arr), min(arr), max(arr), pct))
            if label == '/joint_states' and pct > 20 and abs(max(arr)) > 0.05:
                fails.append('joint_state_publisher is overwriting the tilt')
        else:
            print('%-26s (no lidar_tilt_joint seen)' % label)
            if label == '/lidar_tilt/joint_states':
                fails.append('nothing publishes /lidar_tilt/joint_states')

    print('-' * 72)
    if c.tf_pitch:
        p = c.tf_pitch
        flips = sum(1 for a, b in zip(p, p[1:])
                    if (abs(a) < 0.05 and abs(b) > 0.5)
                    or (abs(b) < 0.05 and abs(a) > 0.5))
        print('TF laser pitch  min=%+.2f max=%+.2f deg   0<->tilt flips: %d'
              % (min(p), max(p), flips))
        if flips:
            fails.append('TF flips between 0 and the real tilt (%d)' % flips)
    else:
        fails.append('no base_footprint<-laser_link TF')

    print('=' * 72)
    print('RESULT: %s' % ('PASS' if not fails else 'FAIL'))
    for f in fails:
        print('   XX ' + f)
    return 0 if not fails else 1


# ── sweep ───────────────────────────────────────────────────────────────────
async def _ws_sweep(seconds):
    import websockets
    async with websockets.connect(WS, max_size=None) as ws:
        await ws.send(json.dumps({"type": "toggle_3d_scan", "enabled": True}))
        print('[ws] 3D scan ON', flush=True)
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                await asyncio.wait_for(ws.recv(), timeout=0.5)
            except Exception:
                pass
        await ws.send(json.dumps({"type": "toggle_3d_scan", "enabled": False}))
        print('[ws] 3D scan OFF', flush=True)


def cmd_sweep(seconds=80.0):
    c = _Collector('lidar3d_sweep')
    time.sleep(2.0)
    asyncio.get_event_loop().run_until_complete(_ws_sweep(seconds))
    time.sleep(4.0)
    c.stop()

    _hdr('SWEEP ON HARDWARE (%.0f s enabled)' % seconds)
    span = (max(c.js_tilt) - min(c.js_tilt)) if c.js_tilt else 0.0
    if c.js_tilt:
        print('tilt joint     : n=%d  min=%+.1f  max=%+.1f deg   (span %.1f)'
              % (len(c.js_tilt), min(c.js_tilt), max(c.js_tilt), span))
    else:
        print('tilt joint     : NO DATA')
    if c.tf_pitch:
        print('TF laser pitch : min=%+.1f  max=%+.1f deg   (span %.1f)'
              % (min(c.tf_pitch), max(c.tf_pitch),
                 max(c.tf_pitch) - min(c.tf_pitch)))
    print('-' * 72)
    vert = (c.zmax - c.zmin) if c.zmax > -1e8 else 0.0
    print('/pointcloud_raw: %d clouds' % c.cloud_n)
    if vert:
        print('  world z      : %.3f .. %.3f m   (vertical extent %.3f m)'
              % (c.zmin, c.zmax, vert))
    if c.elev:
        lo = min(e[0] for e in c.elev)
        hi = max(e[1] for e in c.elev)
        print('  elevation    : %+.1f .. %+.1f deg  <- coverage cone' % (lo, hi))
    print('-' * 72)
    print('/scan_raw %d   /scan %d   published while tilted: %d'
          % (c.raw_n, c.gated_n, c.gated_while_tilted))

    # Distinguish a genuine leak from a threshold straddle.  The tilt is
    # published at 10 Hz and scans arrive at 8 Hz, so at the moment the mount
    # crosses +/-LEVEL_DEG the processor and this test can disagree by up to
    # one sample -- about 0.3 deg at the sweep's ~2 deg/s.  Anything well past
    # the gate is a real fault; a straddle is not.
    MARGIN = 1.0
    bad = [v for v in c.violations if abs(v) > LEVEL_DEG + MARGIN]
    if c.violations:
        print('  violation tilts: %s'
              % ', '.join('%+.2f' % v for v in c.violations[:10]))
        print('  beyond gate+%.1f deg (real leaks): %d' % (MARGIN, len(bad)))
    print('=' * 72)

    checks = [
        ('tilt swept > 60 deg', span > 60),
        ('cloud vertical extent > 1.0 m', vert > 1.0),
        ('/scan silent while clearly tilted', not bad),
        ('/scan alive at level', c.gated_n > 0),
    ]
    ok = all(v for _, v in checks)
    print('RESULT: %s' % ('PASS' if ok else 'FAIL'))
    for label, v in checks:
        print('   [%s] %s' % ('ok' if v else 'XX', label))
    return 0 if ok else 1


# ── interlock ───────────────────────────────────────────────────────────────
async def _drain(ws, want, window=3.0):
    """Collect wanted messages for a wall-clock window.

    Bounded by the clock, not by recv() timing out: the server broadcasts
    telemetry continuously, so a recv-timeout loop would never exit.
    """
    got = []
    loop = asyncio.get_event_loop()
    deadline = loop.time() + window
    while loop.time() < deadline:
        try:
            raw = await asyncio.wait_for(
                ws.recv(), timeout=max(0.05, deadline - loop.time()))
        except asyncio.TimeoutError:
            break
        if isinstance(raw, bytes):
            continue
        try:
            d = json.loads(raw)
        except Exception:
            continue
        if d.get('type') in want:
            got.append(d)
    return got


async def _interlock():
    import websockets
    out = []
    async with websockets.connect(WS, max_size=None) as ws:
        await ws.send(json.dumps({"type": "stop_auto_drive"}))
        await _drain(ws, set(), 1.0)

        # A: sweep refused while auto-drive is running
        await ws.send(json.dumps({"type": "start_auto_drive"}))
        await asyncio.sleep(1.0)
        await ws.send(json.dumps({"type": "toggle_3d_scan", "enabled": True}))
        m = await _drain(ws, {'3d_scan_status'}, 3.0)
        ref = [x for x in m if x.get('refused')]
        out.append(('sweep refused while auto-drive active', bool(ref),
                    ref[0].get('reason') if ref else 'no refusal seen'))
        await ws.send(json.dumps({"type": "stop_auto_drive"}))
        await asyncio.sleep(1.0)

        # B: sweep allowed when idle
        await ws.send(json.dumps({"type": "toggle_3d_scan", "enabled": True}))
        m = await _drain(ws, {'3d_scan_status'}, 3.0)
        allow = [x for x in m if x.get('refused') is False]
        out.append(('sweep allowed when idle', bool(allow),
                    'enabled=%s' % (allow[0].get('enabled') if allow else '?')))

        # C: nav goal refused while the sweep is running
        await asyncio.sleep(2.0)
        await ws.send(json.dumps({"type": "set_nav_goal",
                                  "x": 0.5, "y": 0.0, "theta": 0.0}))
        m = await _drain(ws, {'nav_goal_refused'}, 3.0)
        out.append(('nav goal refused during sweep', bool(m),
                    m[0].get('reason') if m else 'goal was NOT refused'))

        await ws.send(json.dumps({"type": "toggle_3d_scan", "enabled": False}))
        await asyncio.sleep(1.5)
    return out


def cmd_interlock():
    res = asyncio.get_event_loop().run_until_complete(_interlock())
    _hdr('NAV2 <-> TILT INTERLOCK')
    ok = all(p for _, p, _ in res)
    for label, passed, detail in res:
        print('  [%s] %-38s %s' % ('ok' if passed else 'XX', label, detail))
    print('=' * 72)
    print('RESULT: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'probe'
    arg = float(sys.argv[2]) if len(sys.argv) > 2 else None
    if cmd == 'probe':
        return cmd_probe(arg or 14.0)
    if cmd == 'sweep':
        return cmd_sweep(arg or 80.0)
    if cmd == 'interlock':
        return cmd_interlock()
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main())
