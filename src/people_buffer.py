"""people_buffer.py -- a social buffer around tracked people, for the costmaps and the gate.

Why: the CBF's margin (0.10 m round the robot's outline) is a safety distance, and Nav2's
0.35 m inflation is sized to the robot.  In the 2026-10-22 walk-up tests the robot went
round a walker correctly but passed at 0.21 m from its centre (about 10 cm from its side
edge) at 0.15-0.26 m/s: safe by those definitions, uncomfortable for a person.

Two pieces, both pure (no ROS), fed by PersonMemory.people:

  * people_cloud(): points marking each person as a filled disk, plus disks where a walker
    with a trusted velocity will be in 0.5 s and 1.0 s, published as /people/obstacles and
    marked (not cleared) by both costmaps.  The global planner and MPPI then route wide
    round people and away from where they are heading.  A person whose velocity is not
    trusted gets a larger disk instead of a predicted one: low confidence = more room.
  * speed_cap(): the robot slows as the nearest person gets close, whatever Nav2 asks for.

The numbers are first guesses for a comfortable pass (about 0.5 m of clearance) and a calm
approach; they are the thing to tune with the walk-up protocol.
"""
import math

import numpy as np

RADIUS_M = 0.30              # lethal disk marked for a person (plus the costmap inflation)
UNKNOWN_EXTRA_M = 0.10       # added when the velocity is not trusted
HORIZON_S = (0.5, 1.0)       # where a walker with a trusted velocity will be
HORIZON_SHRINK = (1.0, 0.8)  # their disk radius scale at each horizon
MIN_SPEED = 0.15             # below this a trusted person is treated as standing
RING_STEP_M = 0.05           # costmap resolution
Z_M = 0.30                   # inside every obstacle layer's height window
MAX_POINTS = 900
# Never mark the robot's own outline (nav2_params_x3.yaml footprint, + 5 cm).  A person
# cannot be there, so a disk reaching it is an estimate error -- and marking-only lethal
# cells on the robot make every MPPI trajectory collide and the backup recovery refuse:
# goal-on-r3 2026-10-30 aborted after a mis-posed lidar person landed inside the robot.
ROBOT_KEEPOUT = (-0.18, 0.21, -0.16, 0.16)   # x_min, x_max, y_min, y_max in base_link

CAP_NEAR_M = 0.5             # centre distance at or below which the cap is CAP_FLOOR
CAP_FAR_M = 1.3              # at or above which there is no cap
CAP_FLOOR = 0.10             # m/s: still enough to move, slow enough to stop in a few cm
FREE_SPEED = 0.26            # MPPI vx_max
RETREAT_MPS = 0.05           # moving away from the nearest person faster than this is never capped


def _disk(center, radius):
    """Points filling a disk: rings at radius, 2/3 and 1/3 of it, plus the centre."""
    cx, cy = center
    pts = [(cx, cy)]
    for frac in (1.0, 2.0 / 3.0, 1.0 / 3.0):
        r = radius * frac
        n = max(6, int(math.ceil(2.0 * math.pi * r / RING_STEP_M)))
        for k in range(n):
            a = 2.0 * math.pi * k / n
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def people_cloud(people, radius=RADIUS_M, unknown_extra=UNKNOWN_EXTRA_M,
                 horizon=HORIZON_S, shrink=HORIZON_SHRINK, min_speed=MIN_SPEED):
    """people: [{'pos': (x, y), 'vel': (vx, vy), 'trusted': bool}, ...] in the base frame.
    Returns an (N, 3) float32 array of points, N <= MAX_POINTS (nearest people first)."""
    ordered = sorted(people, key=lambda p: math.hypot(*p["pos"]))
    pts = []
    for p in ordered:
        trusted = bool(p.get("trusted"))
        r = radius + (0.0 if trusted else unknown_extra)
        pts += _disk(p["pos"], r)
        vx, vy = p.get("vel", (0.0, 0.0))
        if trusted and math.hypot(vx, vy) > min_speed:
            for t, k in zip(horizon, shrink):
                pts += _disk((p["pos"][0] + vx * t, p["pos"][1] + vy * t), r * k)
        if len(pts) >= MAX_POINTS:
            break
    if not pts:
        return np.zeros((0, 3), dtype=np.float32)
    arr = np.asarray(pts[:MAX_POINTS], dtype=np.float32)
    x0, x1, y0, y1 = ROBOT_KEEPOUT
    arr = arr[~((arr[:, 0] > x0) & (arr[:, 0] < x1) & (arr[:, 1] > y0) & (arr[:, 1] < y1))]
    if not len(arr):
        return np.zeros((0, 3), dtype=np.float32)
    return np.column_stack([arr, np.full(len(arr), Z_M, dtype=np.float32)])


def speed_cap(positions, near=CAP_NEAR_M, far=CAP_FAR_M, floor=CAP_FLOOR, free=FREE_SPEED):
    """Largest translational speed (m/s) allowed given the people at `positions` (base frame):
    `free` beyond `far`, falling linearly to `floor` at `near` and inside it."""
    if not positions:
        return free
    d = min(math.hypot(x, y) for x, y in positions)
    if d >= far:
        return free
    if d <= near:
        return floor
    return floor + (free - floor) * (d - near) / (far - near)


def cap_velocity(vx, vy, positions, **kw):
    """Scale (vx, vy) down to speed_cap() when the robot is not backing away from the
    nearest person.  A retreat is the CBF doing its job (it runs up to 0.45 m/s), so it is
    left alone; passing, approaching and standing-still-ish motion are capped."""
    s = math.hypot(vx, vy)
    if not positions or s < 1e-6:
        return vx, vy
    nearest = min(positions, key=lambda p: math.hypot(p[0], p[1]))
    d = math.hypot(nearest[0], nearest[1])
    if d < 1e-6:
        return vx, vy
    toward = (vx * nearest[0] + vy * nearest[1]) / d       # positive = toward the person
    if toward < -RETREAT_MPS:
        return vx, vy
    cap = speed_cap(positions, **kw)
    if s <= cap:
        return vx, vy
    k = cap / s
    return vx * k, vy * k
