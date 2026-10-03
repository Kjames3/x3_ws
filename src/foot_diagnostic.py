"""Provisional depth-backed feet and a shadow-only moving-obstacle CBF.

No ROS imports, command publisher, drive handle, or production CBF mutation.
The suggested velocity assumes a stationary nominal command. It is evidence for
an experiment, NOT an executable safe command (no surrounding-obstacle checks,
motor deadband, acceleration limits, or verified foot segmentation yet).
"""
import bisect
import math
import os
import threading
import time
from collections import Counter, deque

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

MAX_AGE = 0.65


class StageTimer:
    """Wall time includes scheduling/GIL waits; CPU time is this worker thread only."""
    def __init__(self):
        self.start = self.last = time.monotonic_ns()
        self.cpu_start = self.cpu_last = time.thread_time_ns()
        self.stages = {}

    def mark(self, name):
        now, cpu = time.monotonic_ns(), time.thread_time_ns()
        self.stages[name + '_wall_ms'] = (now-self.last)/1e6
        self.stages[name + '_cpu_ms'] = (cpu-self.cpu_last)/1e6
        self.last, self.cpu_last = now, cpu

    def finish(self):
        return dict(self.stages, work_wall_ms=(self.last-self.start)/1e6,
                    work_cpu_ms=(self.cpu_last-self.cpu_start)/1e6,
                    work_start_monotonic_ns=self.start, result_ready_monotonic_ns=self.last)


def measure_feet(detections, depth, intrinsics, nn_size, mount):
    """Extract compact above-floor depth components near confident ankle pixels.

    Returns provisional occupied regions, not just the ankle or avatar points.
    Input depth is calibrated metres in the camera's aligned depth grid.
    mount = (forward offset, lens floor height, downward pitch radians).
    """
    rejected = Counter()
    if depth is None or depth.ndim != 2 or intrinsics is None:
        return [], {'missing_depth_or_calibration': 1}
    fx, fy, cx, cy, iw, ih = intrinsics
    h, w = depth.shape
    if min(fx, fy, iw, ih, *nn_size) <= 0:
        return [], {'invalid_calibration': 1}
    fx, fy, cx, cy = fx*w/iw, fy*h/ih, cx*w/iw, cy*h/ih
    mx, mz, pitch = mount
    cp, sp = math.cos(pitch), math.sin(pitch)
    out = []
    for person, det in enumerate(detections):
        if det.get('label') != 'person':
            continue
        k = det.get('keypoints') or []
        for side, knee_i, ankle_i in [('left', 13, 15), ('right', 14, 16)]:
            if len(k) <= ankle_i or not np.isfinite(k[ankle_i]).all() or k[ankle_i][2] < .65:
                rejected['ankle_not_visible'] += 1
                continue
            u, v, confidence = k[ankle_i]
            u, v = u*w/nn_size[0], v*h/nn_size[1]
            if not (0 <= u < w and 0 <= v < h):
                rejected['ankle_outside_image'] += 1
                continue
            radius = 18
            if len(k) > knee_i and np.isfinite(k[knee_i]).all() and k[knee_i][2] >= .5:
                ku, kv = k[knee_i][:2]
                radius = int(np.clip(.35*math.hypot(u-ku*w/nn_size[0], v-kv*h/nn_size[1]), 10, 36))
            x0, x1 = max(0, int(u)-radius), min(w, int(u)+radius+1)
            y0, y1 = max(0, int(v)-radius), min(h, int(v)+radius+1)
            if x0 == 0 or y0 == 0 or x1 == w or y1 == h:
                rejected['foot_crop_clipped'] += 1
                continue
            z = depth[y0:y1, x0:x1]
            yy, xx = np.mgrid[y0:y1, x0:x1]
            x = (xx-cx)*z/fx
            y = (yy-cy)*z/fy
            height = mz-y*cp-z*sp
            valid = np.isfinite(z) & (z >= .15) & (z <= 2.5) & (height >= .012) & (height <= .45)
            if valid.sum() < 16:
                rejected['no_above_floor_depth'] += 1
                continue
            # Prefer foreground support; do not use the torso's range. Reject
            # isolated near pixels via connected-component support below.
            near = float(np.percentile(z[valid], 20))
            mask = (valid & (np.abs(z-near) <= .10)).astype(np.uint8)
            n, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
            candidates = [j for j in range(1, n) if stats[j, cv2.CC_STAT_AREA] >= 16
                          and np.linalg.norm(centers[j]-[u-x0, v-y0]) <= radius*.85]
            if not candidates:
                rejected['no_compact_component'] += 1
                continue
            j = min(candidates, key=lambda q: np.linalg.norm(centers[q]-[u-x0, v-y0]))
            use = labels == j
            xy = np.column_stack((mx+z[use]*cp-y[use]*sp, -x[use]))
            center = np.median(xy, axis=0)
            spread = float(np.percentile(np.linalg.norm(xy-center, axis=1), 95))
            if spread > .22:
                rejected['component_too_wide'] += 1
                continue
            out.append(dict(side=side, person_detection=person, xy=center,
                            radius_m=max(.16, spread+.04), sigma_m=max(.025, spread*.5),
                            height_m=float(np.median(height[use])), confidence=float(confidence),
                            pixels=int(use.sum()), roi=[x0,y0,x1,y1],
                            _support_pixels=(yy[use]*w+xx[use])))
    # Only compare opposite ankles of the same detected person. ROI overlap
    # alone is not evidence of duplication: compare the actual selected pixels.
    ambiguous = set()
    for i, a in enumerate(out):
        for j in range(i):
            b = out[j]
            if a['person_detection'] != b['person_detection'] or a['side'] == b['side']:
                continue
            if np.linalg.norm(a['xy']-b['xy']) > .06:
                continue
            shared = np.intersect1d(a['_support_pixels'], b['_support_pixels'],
                                    assume_unique=True).size
            # Require substantial reuse relative to BOTH components. A small
            # crop inside a much larger region is not enough to decide identity.
            if shared >= .60 * max(a['pixels'], b['pixels']):
                ambiguous.update((i, j))
    for m in out:
        m.pop('_support_pixels')
    rejected['ambiguous_shared_depth'] += len(ambiguous)
    if not ambiguous:
        rejected.pop('ambiguous_shared_depth', None)
    # Neither ankle has independent evidence here; confidence cannot tell us
    # which physical foot owns this surface. Do not guess a side or feed two
    # copies to the shadow CBF. The normal tracker path drops these tracks.
    return [m for i, m in enumerate(out) if i not in ambiguous], dict(rejected)


def rotation(pose):
    c, s = math.cos(pose['theta']), math.sin(pose['theta'])
    return np.array([[c, -s], [s, c]])


class FootTracker:
    """Short-lived side-specific tracks in odom; never fill missing feet from pose."""
    def __init__(self):
        self.tracks = []
        self.next_id = 1

    def update(self, measurements, stamp, pose):
        rot = rotation(pose)
        origin = np.array([pose['x'], pose['y']])
        for m in measurements:
            m['world'] = origin+rot@m['xy']
        old = [t for t in self.tracks if 0 < stamp-t['stamp'] <= .35]
        costs = np.full((len(old), len(measurements)), 1e6)
        for i, t in enumerate(old):
            dt = stamp-t['stamp']
            for j, m in enumerate(measurements):
                if m['side'] == t['side']:
                    dist = np.linalg.norm(m['world']-(t['world']+t['velocity']*dt))
                    if dist <= .30:
                        costs[i,j] = dist
        matches = {}
        if costs.size:
            rows, cols = linear_sum_assignment(costs)
            matches = {j: old[i] for i,j in zip(rows,cols) if costs[i,j] < 1e5}
        tracks = []
        for j, m in enumerate(measurements):
            t = matches.get(j)
            velocity = np.zeros(2)
            raw_world = m['world'].copy()
            position = raw_world.copy()
            smoothing_lag = 0.0
            count = 1
            if t is not None:
                dt = stamp-t['stamp']
                raw = (raw_world-t['raw_world'])/dt
                if np.linalg.norm(raw) <= 3.0:
                    # Time-based coefficients preserve behavior across packet rates.
                    # Differentiate raw observations, not the lagging position.
                    velocity_alpha = -math.expm1(-dt/.18)
                    velocity = t['velocity']+velocity_alpha*(raw-t['velocity'])
                    # Reduce position lag once sustained motion is evident.
                    position_tau = .10/(1+np.linalg.norm(velocity)/.15)
                    position_alpha = -math.expm1(-dt/position_tau)
                    position = t['world']+position_alpha*(raw_world-t['world'])
                    smoothing_lag = float(np.linalg.norm(raw_world-position))
                    count = t['count']+1
                else:
                    t = None  # identity jump, do not fabricate a high-speed threat
            ident = t['id'] if t is not None else self.next_id
            if t is None:
                self.next_id += 1
            tracks.append(dict(m, world=position, raw_world=raw_world,
                               smoothing_lag_m=smoothing_lag, id=ident,
                               velocity=velocity, count=count, stamp=stamp))
        self.tracks = tracks

    def local(self, now, pose):
        rot = rotation(pose).T
        origin = np.array([pose['x'], pose['y']])
        out = []
        for t in self.tracks:
            age = now-t['stamp']
            if not 0 <= age <= MAX_AGE:
                continue
            vel = rot@t['velocity']
            xy = rot@(t['world']+t['velocity']*min(age,.3)-origin)
            sigma = t['sigma_m']+.20*age+t['smoothing_lag_m']
            out.append(dict(id=t['id'], side=t['side'], fwd=float(xy[0]), left=float(xy[1]),
                            vfwd=float(vel[0]), vleft=float(vel[1]), radius_m=t['radius_m'],
                            sigma_m=sigma, age_s=age, velocity_ready=t['count']>=3,
                            height_m=t['height_m'], confidence=t['confidence'], pixels=t['pixels'],
                            roi=t['roi'], smoothing_lag_m=t['smoothing_lag_m'], provisional=True, extent_kind='depth_region_with_0.16m_minimum_shoe_proxy'))
        return out


def minimum_norm_velocity(a, b, speed_limit):
    """Project zero onto 2-D half-planes a @ u + b >= 0, then check speed.

    The closest point of a nonempty closed polygon is zero, a perpendicular
    projection onto an edge, or a vertex. If that point exceeds the speed
    limit, intersecting the polygon with the speed disk cannot be feasible.
    Enumerating candidates avoids iterative optimization, including for
    contradictory constraints. O(n**3) checks; n is the number of ready feet.
    """
    a = np.asarray(a, dtype=float).reshape(-1, 2)
    b = np.asarray(b, dtype=float).reshape(-1)
    if (len(a) != len(b) or not np.isfinite(a).all()
            or not np.isfinite(b).all() or not math.isfinite(speed_limit)
            or speed_limit < 0):
        return None
    # Normalize to velocity units so tolerance does not depend on foot range.
    lengths = np.hypot(a[:, 0], a[:, 1])
    nonzero = lengths > 0
    if np.any(b[~nonzero] < 0):
        return None
    a = a[nonzero] / lengths[nonzero, None]
    b = b[nonzero] / lengths[nonzero]
    tolerance = 1e-10
    best = None
    best_norm = float('inf')

    def consider(u):
        nonlocal best, best_norm
        norm = math.hypot(float(u[0]), float(u[1]))
        if (np.isfinite(u).all() and norm <= speed_limit + tolerance
                and norm < best_norm and np.all(a @ u + b >= -tolerance)):
            best, best_norm = u, norm

    consider(np.zeros(2))
    if best is not None:
        return best
    for i in range(len(b)):
        consider(-b[i] * a[i])
        for j in range(i):
            det = a[i, 0]*a[j, 1] - a[i, 1]*a[j, 0]
            if det == 0:
                continue
            consider(np.array([(a[i, 1]*b[j]-b[i]*a[j, 1])/det,
                               (b[i]*a[j, 0]-a[i, 0]*b[j])/det]))
    return best


def shadow_cbf(feet):
    """Moving-obstacle CBF about nominal zero, radius incl. display uncertainty.

    h=|o|^2-R^2; -2 o.u + 2 o.v_obstacle + gamma*h >= 0.
    Foot velocities are inertial velocities expressed in current robot axes.
    Speed is bounded by a circle; an infeasible solve is reported as such.
    """
    active = [f for f in feet if f['velocity_ready']]
    if not active:
        return dict(status='warming_up' if feet else 'no_valid_feet', velocity=None)
    o = np.array([[f['fwd'],f['left']] for f in active])
    vel = np.array([[f['vfwd'],f['vleft']] for f in active])
    radii = np.array([.30+f['radius_m']+2*f['sigma_m'] for f in active])
    a = -2*o
    b = 2*np.sum(o*vel,axis=1)+np.sum(o*o,axis=1)-radii*radii
    solution = minimum_norm_velocity(a, b, .15)
    feasible = solution is not None
    return dict(status='suggestion' if feasible else 'infeasible',
                velocity={'fwd':float(solution[0]),'left':float(solution[1])} if feasible else None,
                min_margin_m=float(np.min(np.linalg.norm(o,axis=1)-radii)),
                nominal='stationary', speed_limit_mps=.15,
                scope='feet_only_no_actuator_or_surroundings_validation')


class FootDiagnostic:
    """Read-only consumer of the existing camera interface, with its own worker."""
    def __init__(self, oak, pose_fn, mount):
        self.oak, self.pose_fn, self.mount = oak, pose_fn, mount
        self.tracker = FootTracker()
        self.poses = deque()
        self.lock = threading.Lock()
        self.result = dict(mode='diagnostic_only', status='starting', feet=[], shadow=shadow_cbf([]))
        self.last_update = None
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run,name='foot-diagnostic',daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)

    def snapshot(self):
        with self.lock:
            r = dict(self.result)
            age = time.monotonic()-self.last_update if self.last_update else None
        r['update_age_s'] = age
        timing = dict(r.get('timing', {}))
        snapshot_ns = time.monotonic_ns()
        if timing:
            timing['snapshot_monotonic_ns'] = snapshot_ns
            timing['ready_to_snapshot_ms'] = (snapshot_ns-timing['result_ready_monotonic_ns'])/1e6
            timing['capture_to_snapshot_ms'] = (snapshot_ns-timing['capture_monotonic_ns'])/1e6
            r['timing'] = timing
        if age is None or age > MAX_AGE or r.get('capture_age_s', 0) + age > MAX_AGE:
            r.update(status='stale', feet=[], shadow=dict(status='stale',velocity=None))
        return r

    def _pose_at(self, stamp):
        if len(self.poses)<2 or not self.poses[0][0]<=stamp<=self.poses[-1][0]:
            return None
        i=bisect.bisect_left([x[0] for x in self.poses],stamp)
        if i==0:
            return dict(self.poses[0][1])
        t0,p0=self.poses[i-1];t1,p1=self.poses[i]
        a=(stamp-t0)/(t1-t0)
        angle=math.atan2(math.sin(p1['theta']-p0['theta']),math.cos(p1['theta']-p0['theta']))
        return dict(x=p0['x']+a*(p1['x']-p0['x']),y=p0['y']+a*(p1['y']-p0['y']),theta=p0['theta']+a*angle)

    def _run(self):
        last = None
        while self.running:
            time.sleep(.02)
            try:
                now=time.monotonic(); pose=self.pose_fn()
                if pose is not None and np.isfinite([pose['x'],pose['y'],pose['theta']]).all():
                    self.poses.append((now,dict(pose)))
                    while self.poses and now-self.poses[0][0]>2:
                        self.poses.popleft()
                else:
                    self.poses.clear();pose=None
                dets,meta=self.oak.get_detection_observation()
                if not meta:
                    continue
                if 'host_decode_start_monotonic_ns' in meta and 'host_decode_end_monotonic_ns' not in meta:
                    continue  # incomplete timing publication; retry this packet next poll
                key=(meta.get('session_id'),meta.get('seq'))
                if key==last:
                    continue
                last=key
                timer=StageTimer()
                cap=meta['host_monotonic_estimate_ns']/1e9
                sdk_cap=meta['sdk_host_ns']/1e9
                at=self._pose_at(cap)
                timer.mark('pose_lookup')
                near=self.oak.get_depth_near(sdk_cap,max_gap_s=.05)
                intr = self.oak.get_depth_intrinsics()
                timer.mark('depth_lookup')
                status='ok'
                if pose is None or at is None:
                    status='missing_capture_pose'
                elif not 0<=now-cap<=MAX_AGE:
                    status='stale_detection'
                elif near is None:
                    status='unsynchronized_depth'
                feet=[];rejected={}
                if status=='ok':
                    measurements,rejected=measure_feet(dets,near[1],intr,
                        (self.oak.nn_w,self.oak.nn_h),self.mount)
                    timer.mark('extraction')
                    self.tracker.update(measurements,cap,at)
                    feet=self.tracker.local(now,pose)
                else:
                    timer.mark('extraction')
                    self.tracker.tracks=[]
                timer.mark('tracking')
                shadow=shadow_cbf(feet)
                timer.mark('shadow_cbf')
                timing=timer.finish()
                timing['capture_monotonic_ns']=meta['host_monotonic_estimate_ns']
                receipt=meta.get('receipt_monotonic_ns')
                decoded=meta.get('host_decode_end_monotonic_ns')
                timing.update(
                    camera_delivery_ms=(receipt-meta['host_monotonic_estimate_ns'])/1e6 if receipt else None,
                    receipt_to_work_ms=(timer.start-receipt)/1e6 if receipt else None,
                    host_predecode_ms=(meta['host_decode_start_monotonic_ns']-receipt)/1e6
                        if receipt and meta.get('host_decode_start_monotonic_ns') else None,
                    host_decode_wall_ms=meta.get('host_decode_wall_ms'),
                    host_decode_cpu_ms=meta.get('host_decode_cpu_ms'),
                    diagnostic_wait_ms=(timer.start-decoded)/1e6 if decoded else None,
                    capture_to_ready_ms=(timer.last-meta['host_monotonic_estimate_ns'])/1e6,
                    stages_ran=status=='ok')
                timing.update(meta.get('camera_host_timing', {}))
                settings = {name:getattr(self.oak,name,None) for name in
                            ('nn_w','nn_h','nn_fps','mono_fps','record_rgbd','subpixel','nn_kpts','usb_speed')}
                settings['depth_fps_observed']=getattr(self.oak,'depth_fps',None)
                settings['model_blob']=str(getattr(self.oak,'spatial_blob','unknown'))
                settings['environment']={name:os.environ.get(name) for name in
                    ('X3_OAK_DEPTH_FPS','X3_OAK_BLOB_SHAVES','X3_OAK_NN_LATEST_OUTPUT','X3_OAK_NN_THREADS','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS')}
                result=dict(mode='diagnostic_only',status=status,feet=feet,shadow=shadow,
                    rejected=rejected,capture_age_s=now-cap,depth_gap_s=abs(near[0]-sdk_cap) if near else None,
                    session_id=meta.get('session_id'), seq=meta.get('seq'),
                    update_ms=timing['work_wall_ms'], timing=timing, settings=settings)
                with self.lock:
                    ready_ns, ready_cpu = time.monotonic_ns(), time.thread_time_ns()
                    timing['result_ready_monotonic_ns'] = ready_ns
                    timing['result_build_wall_ms'] = (ready_ns-timer.last)/1e6
                    timing['result_build_cpu_ms'] = (ready_cpu-timer.cpu_last)/1e6
                    timing['work_wall_ms'] = (ready_ns-timer.start)/1e6
                    timing['work_cpu_ms'] = (ready_cpu-timer.cpu_start)/1e6
                    timing['capture_to_ready_ms'] = (ready_ns-meta['host_monotonic_estimate_ns'])/1e6
                    result['update_ms'] = timing['work_wall_ms']
                    self.result=result;self.last_update=now
            except Exception as exc:
                self.tracker.tracks=[]
                with self.lock:
                    self.result=dict(mode='diagnostic_only',status='error',error=str(exc),feet=[],shadow=shadow_cbf([]))
                    self.last_update=time.monotonic()
