"""lidar_person_tracker.py -- people held by the Livox once the camera has confirmed them.

Why: in a busy lab (2026-10-22) the camera estimate had nobody 21% of the time; 139 of
354 dropouts began with the person closer than 1 m, where the camera cannot see them,
and 127 of the 142 long ones came back under a NEW id (684 ids in 3 h).  The Livox sees
the whole circle at 10 Hz and down to ~0.1 m, so it can hold the position and the
identity through those gaps.

Rules (scope approved 2026-10-30):

  * Birth only where a camera detection (C3, src/c3_live.py) lands on a person-sized
    lidar cluster (/livox/person_candidates).  Clusters alone never make a person.
  * After birth the lidar owns the track: id ('L<n>'), position and velocity.  A NEW
    camera id that lands on an existing track joins it (``relinked``) instead of
    starting over, which is the id-churn fix.
  * The track lives while the lidar keeps finding it (a gap > LIDAR_GAP_S ends it) and
    for at most HOLD_S after the camera last saw it.
  * Ambiguity is left alone: a track, cluster or camera seed with a near rival, or two
    tracks inside one cluster (two people merged), gets no update this frame.

Everything is in odom (metres, seconds of ROS time = wall time on the robot).
Pure Python, no ROS: importable by tests.  Driven from server_x3.py.
"""
import math

HOLD_S = 3.0             # longest lidar-only hold after the camera last saw the person
LIDAR_GAP_S = 0.4        # a track not found by the lidar for this long ends
CAMERA_MAX_AGE_S = 0.65  # a camera observation older than this seeds/confirms nothing
CAMERA_STALE_PACKET_S = 0.5
GATE_M = 0.55            # track <-> cluster and camera <-> track association radius
RIVAL_MARGIN_M = 0.15    # the best match must beat the next by this much
MERGE_PAD_M = 0.20       # two predicted tracks inside one cluster box (+pad) = merged
CONFIRM_HITS = 3         # lidar frames before a track is reported confirmed
VEL_TRUST_HITS = 5       # lidar frames before its velocity is offered as trusted
POS_GAIN = 0.6           # alpha-beta position gain on the cluster centroid
VEL_GAIN = 0.12          # alpha-beta velocity gain
CAMERA_VEL_GAIN = 0.3    # pull toward the camera's (Kalman) velocity when it sees them
MAX_SPEED = 2.0
CAMERA_FRESH_S = 0.4     # camera seen this recently = 'camera + lidar', else 'lidar held'
RESET_GAP_S = 1.0        # a frame gap this long (or a stamp going back) clears everything
MIN_CONFIDENCE = 0.3


def _finite(*v):
    return all(isinstance(a, (int, float)) and math.isfinite(a) for a in v)


def _unique_pairs(n_a, n_b, dist, gate=GATE_M, margin=RIVAL_MARGIN_M):
    """Pairs (i, j) whose nearest partner is each other, within gate, with no rival
    within margin on either side.  dist(i, j) -> metres."""
    d = [[dist(i, j) for j in range(n_b)] for i in range(n_a)]
    pairs = {}
    for i in range(n_a):
        opts = sorted((d[i][j], j) for j in range(n_b) if d[i][j] < gate)
        if not opts or (len(opts) > 1 and opts[1][0] - opts[0][0] < margin):
            continue
        j = opts[0][1]
        rivals = sorted((d[k][j], k) for k in range(n_a) if d[k][j] < gate)
        if rivals[0][1] != i or (len(rivals) > 1 and rivals[1][0] - rivals[0][0] < margin):
            continue
        pairs[i] = j
    return pairs


class _Track:
    __slots__ = ('id', 'x', 'y', 'vx', 'vy', 'at', 'hits', 'last_lidar', 'last_camera',
                 'camera_id', 'camera_ids', 'born', 'sigma', 'points', 'z_hi', 'held_s')

    def __init__(self, tid, cand, anchor, now):
        self.id = tid
        self.x, self.y = cand['x'], cand['y']
        self.vx, self.vy = anchor['vx'], anchor['vy']
        self.at = now
        self.hits = 1
        self.last_lidar = now
        self.last_camera = anchor['last_seen']
        self.camera_id = anchor['id']
        self.camera_ids = {anchor['id']}
        self.born = now
        self.sigma = 0.3
        self.points = int(cand['n'])
        self.z_hi = cand['z_hi']
        self.held_s = 0.0


class LidarPersonTracker:
    def __init__(self, hold_s=HOLD_S):
        self.hold_s = hold_s
        self.next_id = 1
        self.stats = {'frames': 0, 'births': 0, 'relinked': 0, 'hold_expired': 0,
                      'lost_lidar': 0, 'resets': 0, 'lidar_held_s': 0.0}
        self.clear()

    def clear(self):
        self.tracks = []
        self.last_stamp = None
        self.results = []

    # -- inputs ------------------------------------------------------------------
    @staticmethod
    def _anchors(camera, now, packet_age):
        """Fresh, measured camera tracks predicted to `now`."""
        out = []
        if packet_age >= CAMERA_STALE_PACKET_S:
            return out
        for c in camera or []:
            st, age = c.get('state_stamp_s'), c.get('obs_age_s')
            x, y, vx, vy = c.get('odom_x'), c.get('odom_y'), c.get('odom_vx'), c.get('odom_vy')
            if not _finite(st, age, x, y, vx, vy) or c.get('measured') is False:
                continue
            dt = now - st
            last_seen = st - age
            if dt < -0.15 or dt > CAMERA_MAX_AGE_S or now - last_seen > CAMERA_MAX_AGE_S:
                continue
            k = max(dt, 0.0)
            out.append({'id': str(c.get('id')), 'x': x + vx * k, 'y': y + vy * k,
                        'vx': vx, 'vy': vy, 'last_seen': last_seen})
        return out

    # -- main entry point --------------------------------------------------------
    def update(self, now, candidates, camera, packet_age=0.0):
        """now: frame stamp (s).  candidates: dicts with the person_candidates FIELDS
        (odom).  camera: c3_live.get_tracks()[0].  packet_age: C3 stats packet_age_s.
        Returns self.results (see _result)."""
        if self.last_stamp is not None:
            dt = now - self.last_stamp
            if dt == 0:
                return self.results
            if dt < 0 or dt > RESET_GAP_S:
                self.stats['resets'] += 1
                self.clear()
        frame_dt = 0.0 if self.last_stamp is None else now - self.last_stamp
        self.last_stamp = now
        self.stats['frames'] += 1
        cands = list(candidates)
        anchors = self._anchors(camera, now, packet_age)

        # 1. predict, then tracks <-> clusters
        pred = []
        for t in self.tracks:
            dt = min(0.4, max(0.0, now - t.at))
            pred.append((t.x + t.vx * dt, t.y + t.vy * dt, dt))
        matches = _unique_pairs(len(pred), len(cands),
                                lambda i, j: math.hypot(cands[j]['x'] - pred[i][0],
                                                        cands[j]['y'] - pred[i][1]))
        for i, j in list(matches.items()):
            c = cands[j]
            inside = sum(1 for px, py, _ in pred
                         if c['x_lo'] - MERGE_PAD_M <= px <= c['x_hi'] + MERGE_PAD_M
                         and c['y_lo'] - MERGE_PAD_M <= py <= c['y_hi'] + MERGE_PAD_M)
            if inside > 1:
                del matches[i]
        used = set(matches.values())
        for i, t in enumerate(self.tracks):
            px, py, dt = pred[i]
            j = matches.get(i)
            if j is None:
                t.x, t.y = px, py
                t.sigma = min(0.8, t.sigma + 0.12)
            else:
                c = cands[j]
                rx, ry = c['x'] - px, c['y'] - py
                t.x, t.y = px + POS_GAIN * rx, py + POS_GAIN * ry
                if dt > 0.05:
                    t.vx += VEL_GAIN * rx / dt
                    t.vy += VEL_GAIN * ry / dt
                t.hits += 1
                t.last_lidar = now
                t.sigma = max(0.12, t.sigma * 0.8)
                t.points, t.z_hi = int(c['n']), c['z_hi']
            t.at = now

        # 2. camera -> tracks: confirmation, and a new camera id joins the lidar id
        cam = _unique_pairs(len(anchors), len(self.tracks),
                            lambda i, j: math.hypot(anchors[i]['x'] - self.tracks[j].x,
                                                    anchors[i]['y'] - self.tracks[j].y))
        for i, j in cam.items():
            a, t = anchors[i], self.tracks[j]
            t.last_camera = max(t.last_camera, a['last_seen'])
            if a['id'] not in t.camera_ids:
                self.stats['relinked'] += 1
                t.camera_ids.add(a['id'])
            t.camera_id = a['id']
            t.vx += CAMERA_VEL_GAIN * (a['vx'] - t.vx)
            t.vy += CAMERA_VEL_GAIN * (a['vy'] - t.vy)
        for t in self.tracks:
            s = math.hypot(t.vx, t.vy)
            if s > MAX_SPEED:
                t.vx, t.vy = t.vx * MAX_SPEED / s, t.vy * MAX_SPEED / s

        # 3. births: an unclaimed camera seed on an unclaimed cluster
        free_a = [a for i, a in enumerate(anchors) if i not in cam]
        free_c = [c for j, c in enumerate(cands) if j not in used]
        born = _unique_pairs(len(free_a), len(free_c),
                             lambda i, j: math.hypot(free_a[i]['x'] - free_c[j]['x'],
                                                     free_a[i]['y'] - free_c[j]['y']))
        for i, j in born.items():
            self.tracks.append(_Track(f'L{self.next_id}', free_c[j], free_a[i], now))
            self.next_id += 1
            self.stats['births'] += 1

        # 4. endings
        alive = []
        for t in self.tracks:
            if now - t.last_lidar > LIDAR_GAP_S:
                self.stats['lost_lidar'] += 1
            elif now - t.last_camera > self.hold_s:
                self.stats['hold_expired'] += 1
            else:
                if t.hits >= CONFIRM_HITS and now - t.last_camera > CAMERA_FRESH_S:
                    t.held_s += frame_dt
                    self.stats['lidar_held_s'] += frame_dt
                alive.append(t)
        self.tracks = alive
        self.results = [self._result(t, now) for t in alive]
        return self.results

    def _result(self, t, now):
        camera_age = max(0.0, now - t.last_camera)
        lidar_age = max(0.0, now - t.last_lidar)
        if t.hits < CONFIRM_HITS:
            mode = 'tentative'
        elif lidar_age > 0.0:
            mode = 'coasting'
        elif camera_age <= CAMERA_FRESH_S:
            mode = 'camera + lidar'
        else:
            mode = 'lidar held'
        # Heuristic, not calibrated: identity confidence falls as the camera's last
        # confirmation ages toward the hold cap.
        conf = 1.0 if camera_age <= CAMERA_FRESH_S else max(
            MIN_CONFIDENCE, 1.0 - (1.0 - MIN_CONFIDENCE) * camera_age / self.hold_s)
        return {
            'id': t.id, 'camera_id': t.camera_id, 'camera_ids': len(t.camera_ids),
            'odom_x': round(t.x, 4), 'odom_y': round(t.y, 4),
            'odom_vx': round(t.vx, 4), 'odom_vy': round(t.vy, 4),
            'stamp': now, 'mode': mode, 'confirmed': t.hits >= CONFIRM_HITS,
            'vel_trusted': t.hits >= VEL_TRUST_HITS, 'hits': t.hits,
            'pos_sigma_m': round(t.sigma, 3), 'confidence': round(conf, 3),
            'camera_age_s': round(camera_age, 3), 'lidar_age_s': round(lidar_age, 3),
            'points': t.points, 'height_m': round(float(t.z_hi), 3),
            'held_s': round(t.held_s, 2),
        }


def to_local(results, pose):
    """Add robot-frame fields (fwd, left, vx, vy; velocity is the person's own, in base
    axes) given pose {'x','y','theta'} in odom.  Returns new dicts."""
    c, s = math.cos(pose['theta']), math.sin(pose['theta'])
    out = []
    for r in results:
        dx, dy = r['odom_x'] - pose['x'], r['odom_y'] - pose['y']
        vx, vy = r['odom_vx'], r['odom_vy']
        out.append(dict(r, fwd=round(c * dx + s * dy, 3), left=round(-s * dx + c * dy, 3),
                        vx=round(c * vx + s * vy, 3), vy=round(-s * vx + c * vy, 3),
                        speed=round(math.hypot(vx, vy), 3)))
    return out


def cbf_estimates(local_tracks, camera_estimates, now, max_age_s=0.3, dedupe_m=0.5):
    """Lidar-held people as velocity_estimator-style estimates for PersonMemory, only
    where the camera estimator has no person of its own (it stays the first source).
    local_tracks: to_local() output.  camera_estimates: velocity_estimator estimates."""
    cam = [(float(e.get('z', 0.0)), -float(e.get('x', 0.0))) for e in camera_estimates
           if e.get('category') == 'person' and e.get('age', 0) == 0]
    out = []
    for r in local_tracks:
        if not r['confirmed'] or r['lidar_age_s'] > 0.0 or now - r['stamp'] > max_age_s:
            continue
        if any(math.hypot(r['fwd'] - x, r['left'] - y) < dedupe_m for x, y in cam):
            continue
        trusted = r['vel_trusted']
        out.append({'id': r['id'], 'category': 'person', 'age': 0,
                    'z': r['fwd'], 'x': -r['left'], 'vx': r['vx'], 'vy': r['vy'],
                    'speed': r['speed'],
                    # PersonMemory/_cbf_movers trust gates: an untrusted lidar velocity
                    # enters as present-but-direction-unknown.
                    'person_frames': r['hits'] if trusted else 0,
                    'jump_age': 10 ** 6, 'source': 'lidar'})
    return out
