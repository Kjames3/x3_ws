"""Control-barrier-function velocity filter for the holonomic (mecanum) base.

For every obstacle i at p_i moving at v_i, with the robot at p_r commanded u:

    h_i     = |p_r - p_i|^2 - d_safe^2
    dh_i/dt = 2 (p_r - p_i) . (u - v_i)
    require   dh_i/dt + gamma * h_i >= 0

Dividing by |2 (p_r - p_i)| gives each constraint in metres per second,

    n_i . u >= c_i,    n_i = unit vector from obstacle to robot,
                       c_i = n_i . v_i - gamma * h_i / (2 d_i)

so a margin n_i . u - c_i is a closing speed, comparable between a near
obstacle and a far one. The command is the solution of the quadratic program

    minimise  1/2 |u - u_nom|^2   subject to  n_i . u >= c_i  and the speed limit

which has two unknowns, a quadratic objective and linear constraints, so it is
solved exactly (_project) instead of with a general nonlinear solver: the only
way it can fail is by being infeasible. When it is infeasible (inside the safe
distance of obstacles on opposite sides, or an obstacle closing faster than the
base can retreat) stopping is not safe either, so the constraints are all
relaxed by the same smallest amount t* that makes them satisfiable -- the
command that minimises the worst violation -- and the program is solved again.

The speed limit is the mecanum wheel limit |vx| + |vy| + L |omega| <= v_wheel:
every wheel's rim speed is a signed sum of those three terms.
"""
import logging
import math
import time

import numpy as np
from scipy.optimize import linprog

logger = logging.getLogger(__name__)

_last_fail_log = 0.0   # monotonic timestamp, throttles the warnings below

_EPS = 1e-9
_SQRT2 = math.sqrt(2.0)
_SHUFFLE_AFTER = 6       # _project passes in sorted order before shuffling
_LP_BATCH = 48          # rows added per round of the fallback LP
# The four faces of |vx| + |vy| <= V, as unit normals pointing inward.
_DIAMOND = -np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]]) / _SQRT2


def _project(u_nom, n, c):
    """Closest point to u_nom with n @ u >= c (rows of n unit length), or None.

    Incremental half-plane method: keep the optimum for the constraints seen so
    far; when a later one is violated the new optimum lies on its boundary
    line, where the earlier constraints cut out an interval. An empty interval
    means the whole set is infeasible.
    """
    if len(c) == 0:
        return u_nom.copy()
    order = np.argsort(n @ u_nom - c)        # most violated first: few passes
    n, c = n[order], c[order]
    u = u_nom.copy()
    start = 0
    passes = 0
    while True:
        bad = np.flatnonzero(n[start:] @ u - c[start:] < -_EPS)
        if bad.size == 0:
            return u
        passes += 1
        if passes == _SHUFFLE_AFTER and len(c) - start > 2:
            # Sorted order settles a feasible scan in ~3 passes but crawls
            # through an infeasible one (each row a little tighter than the
            # last). The rows not yet used may be taken in any order, and a
            # random one needs only O(log N) passes on average.
            mix = start + np.random.default_rng(0).permutation(len(c) - start)
            n = np.concatenate((n[:start], n[mix]))
            c = np.concatenate((c[:start], c[mix]))
            continue
        i = start + int(bad[0])
        ni = n[i]
        p0 = c[i] * ni                        # the line's closest point to 0
        along = np.array([-ni[1], ni[0]])
        lo, hi = -math.inf, math.inf
        if i:
            den = n[:i] @ along
            num = c[:i] - n[:i] @ p0
            flat = np.abs(den) < 1e-12
            if np.any(num[flat] > _EPS):
                return None                   # parallel and out of reach
            up, down = den > 1e-12, den < -1e-12
            if up.any():
                lo = float(np.max(num[up] / den[up]))
            if down.any():
                hi = float(np.min(num[down] / den[down]))
            if lo > hi + _EPS:
                return None
        s = min(max(float(along @ (u_nom - p0)), lo), max(hi, lo))
        u = p0 + s * along
        start = i + 1


class HolonomicCBFFilter:
    def __init__(self, safe_distance=0.3, gamma=1.0, max_wheel_speed=0.5,
                 wheel_separation=0.165):
        """
        Args:
            safe_distance (float): minimum allowed distance to any obstacle
                point (m). 0.3 for the X3 chassis plus padding.
            gamma (float): 1/s. A static obstacle may be approached at up to
                roughly gamma * (distance - safe_distance), so larger is less
                conservative and brakes later.
            max_wheel_speed (float): wheel rim speed the base can deliver under
                load (m/s). The 205 rpm motors on 65 mm wheels do 0.70 m/s with
                no load.
            wheel_separation (float): half wheelbase + half track (m), as
                Mcnamu_driver_X3's wheel_separation_factor.
        """
        self.safe_distance = safe_distance
        self.gamma = gamma
        self.max_wheel_speed = max_wheel_speed
        self.wheel_separation = wheel_separation
        # How far the last solve had to relax the constraints, m/s. 0.0 when it
        # was feasible. For telemetry and tests.
        self.last_relaxation = 0.0

    def constraint_rows(self, robot_pos, obstacles, obstacle_velocities=None):
        """(n, c) with one row per obstacle: the command must keep n @ u >= c."""
        obs = np.asarray(obstacles, dtype=np.float64).reshape(-1, 2)
        rel = np.asarray(robot_pos, dtype=np.float64) - obs
        d = np.hypot(rel[:, 0], rel[:, 1])
        # Drop what has no usable direction: a NaN or infinite point (an
        # infinite one would put NaN into the LP, which raises), and a point on
        # top of the robot.
        keep = np.isfinite(d) & (d > 1e-6)
        rel, d = rel[keep], d[keep]
        n = rel / d[:, None]
        # -gamma*h/(2d) grows as 1/d inside the safe distance: a point 1 mm from
        # the centre would demand a 45 m/s retreat and, in the fallback, outvote
        # every other obstacle. Cap it at gamma*d_safe, what the plain-distance
        # barrier (d - d_safe) asks for at zero range. Reached only inside
        # 0.13 m at d_safe 0.30, closer than the scan's 0.12 m gate lets through.
        c = np.minimum(-self.gamma * (d * d - self.safe_distance ** 2) / (2.0 * d),
                       self.gamma * self.safe_distance)
        if obstacle_velocities is not None:
            vel = np.asarray(obstacle_velocities, dtype=np.float64).reshape(-1, 2)[keep]
            vel = np.where(np.isfinite(vel), vel, 0.0)   # unknown: treat as static
            c = c + np.einsum('ij,ij->i', n, vel)
        return n, c

    def speed_limit(self, omega=0.0):
        """Largest |vx| + |vy| the wheels can deliver while turning at omega."""
        return max(self.max_wheel_speed - self.wheel_separation * abs(omega), 0.0)

    def filter_velocity(self, v_nom_x, v_nom_y, robot_pos, obstacles,
                        obstacle_velocities=None, omega=0.0):
        """
        Filters the requested velocity so it keeps clear of the obstacles.

        Args:
            v_nom_x, v_nom_y (float): commanded velocity (m/s)
            robot_pos (tuple): robot (x, y), in the obstacles' frame
            obstacles: (x, y) per obstacle
            obstacle_velocities: (vx, vy) per obstacle in the same frame, or
                None when they are all static
            omega (float): commanded yaw rate (rad/s); it uses up wheel speed

        Returns:
            tuple: (safe_v_x, safe_v_y)
        """
        self.last_relaxation = 0.0
        u_nom = np.array([v_nom_x, v_nom_y], dtype=np.float64)
        if not np.all(np.isfinite(u_nom)):
            u_nom = np.zeros(2)
        c_spd = np.full(4, -self.speed_limit(omega) / _SQRT2)
        if np.asarray(obstacles).size == 0:
            # Nothing to avoid, but the wheel limit still holds. A command
            # inside it comes back untouched.
            if abs(u_nom[0]) + abs(u_nom[1]) <= self.speed_limit(omega):
                return float(u_nom[0]), float(u_nom[1])
            u = _project(u_nom, _DIAMOND, c_spd)
            return float(u[0]), float(u[1])

        n_obs, c_obs = self.constraint_rows(robot_pos, obstacles, obstacle_velocities)
        n_all = np.vstack((n_obs, _DIAMOND))

        u = _project(u_nom, n_all, np.concatenate((c_obs, c_spd)))
        if u is None:
            u = self._least_violation(u_nom, n_obs, c_obs, c_spd, n_all)
            self._log_infeasible(u, n_obs, c_obs, robot_pos, obstacles, obstacle_velocities)
        return float(u[0]), float(u[1])

    def _log_infeasible(self, u, n_obs, c_obs, robot_pos, obstacles, obstacle_velocities):
        """Say which obstacle the fallback could not satisfy, at most every 2 s."""
        global _last_fail_log
        now = time.monotonic()
        if now - _last_fail_log < 2.0 or not len(c_obs):
            return
        _last_fail_log = now
        obs = np.asarray(obstacles, dtype=np.float64).reshape(-1, 2)
        rel = obs - np.asarray(robot_pos, dtype=np.float64)
        d = np.hypot(rel[:, 0], rel[:, 1])
        keep = np.isfinite(d) & (d > 1e-6)
        vel = np.zeros_like(obs) if obstacle_velocities is None else \
            np.asarray(obstacle_velocities, dtype=np.float64).reshape(-1, 2)
        vel = np.where(np.isfinite(vel), vel, 0.0)
        i = int(np.argmin(n_obs @ u - c_obs))
        movers = int(np.count_nonzero(np.any(vel[keep] != 0.0, axis=1)))
        logger.warning(
            "CBF infeasible (%d constraints, %d moving): least-violating command "
            "(%.2f, %.2f), worst closing-speed violation %.2f m/s at obstacle "
            "(%.2f, %.2f) m moving (%.2f, %.2f) m/s; nearest obstacle %.2f m",
            len(c_obs), movers, u[0], u[1], self.last_relaxation,
            rel[keep][i, 0], rel[keep][i, 1], vel[keep][i, 0], vel[keep][i, 1],
            float(d[keep].min()))

    def _least_violation(self, u_nom, n_obs, c_obs, c_spd, n_all):
        """Infeasible: relax every obstacle constraint equally, as little as possible.

        maximise t  subject to  n_i . u - c_i >= t  and the speed limit, then
        the command nearest u_nom among those that achieve t.
        """
        global _last_fail_log
        k = len(c_obs)
        # The LP has three unknowns, so a handful of rows decide it. Solve it on
        # the rows that are worst at standstill, then add any row the answer
        # violates and repeat; when none is left the answer is exact for all of
        # them. (A subset can only overestimate t.) ~1 ms a round on the Orin,
        # against ~7 ms for every row of a full scan at once.
        live = np.zeros(k, dtype=bool)
        live[np.argsort(c_obs)[-_LP_BATCH:]] = True
        for _ in range(50):
            m = int(live.sum())
            a_ub = np.zeros((m + 4, 3))
            a_ub[:m, :2], a_ub[:m, 2] = -n_obs[live], 1.0
            a_ub[m:, :2] = -_DIAMOND
            res = linprog([0.0, 0.0, -1.0], A_ub=a_ub,
                          b_ub=-np.concatenate((c_obs[live], c_spd)),
                          bounds=[(None, None), (None, None), (None, 0.0)], method='highs')
            if res.status != 0:
                break
            short = (n_obs @ res.x[:2] - c_obs) - res.x[2]
            short[live] = 0.0
            if short.min() >= -1e-9:
                break
            worst = np.argsort(short)[:_LP_BATCH]
            live[worst[short[worst] < -1e-9]] = True
        now = time.monotonic()
        if res.status != 0:
            # Not expected: this LP always has a solution. Stop, and say so.
            if now - _last_fail_log > 2.0:
                _last_fail_log = now
                logger.warning("CBF fallback LP failed (%d constraints, status=%s: %s) "
                               "-- commanding stop", k, res.status, res.message)
            self.last_relaxation = float('nan')
            return np.zeros(2)
        t = float(res.x[2])
        self.last_relaxation = -t
        u = _project(u_nom, n_all, np.concatenate((c_obs + t - 1e-7, c_spd)))
        return u if u is not None else np.asarray(res.x[:2], dtype=np.float64)
