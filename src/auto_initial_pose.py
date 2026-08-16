"""
auto_initial_pose.py
Automatic AMCL initialisation for the Yahboom X3.

Two mechanisms, used together:

  1. Persisted last pose — /amcl_pose is throttle-saved to a JSON store keyed by
     map.  On the next Nav2 launch the stored pose for that map is re-proposed.
     AMCL's own `save_pose_rate` only writes back to an in-memory parameter and
     is lost when the process dies, so persistence has to live out here.

  2. Scan-match seed — a one-shot global localisation.  The live /map is turned
     into a likelihood field (distance transform -> gaussian), the newest /scan
     is projected into the robot base frame, and a coarse-to-fine brute-force
     search over (x, y, yaw) picks the best-scoring pose.  A confidence *margin*
     between the best mode and the best clearly-different mode gates acceptance,
     because geometrically repetitive spaces (corridors, near-identical rooms)
     will otherwise produce a high-scoring but wrong lock.

A stored pose is never trusted blindly: it is re-scored against the current scan
and only published if it still explains what the lidar sees.  If it does not, the
global search runs.  If the global search is ambiguous, nothing is published and
the operator sets the pose by hand exactly as before.

The matching core (`LikelihoodField`, `ScanMatcher`) has no ROS dependency and is
importable for offline testing; `AutoInitialPose` is the rclpy glue.

CLI (offline, no ROS needed):
    python3 src/auto_initial_pose.py selftest --map <map.yaml>
    python3 src/auto_initial_pose.py show-store
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# ── Where the persisted poses live ──────────────────────────────────────────
DEFAULT_POSE_STORE = os.path.expanduser("~/.x3/last_pose.json")

# ── Likelihood field ────────────────────────────────────────────────────────
# Deliberately sharper than amcl's own sigma_hit (0.20).  AMCL wants a forgiving
# field so a tracking filter does not lose lock; a one-shot global search wants a
# discriminating one.  Measured over 120 synthetic trials on RealLab1 and
# apartment3_test, 0.15 was the best compromise: perfect recovery at 15% unmapped
# clutter, and it degrades gracefully rather than confidently past that.
SIGMA_HIT = 0.15
MAX_LIKELIHOOD_DIST = 2.0   # m, matches amcl laser_likelihood_max_dist

# ── Search resolution ───────────────────────────────────────────────────────
COARSE_TRANS_STEP = 0.20            # m
COARSE_ROT_STEP   = math.radians(5.0)
FINE_TRANS_STEP   = 0.025           # m
FINE_ROT_STEP     = math.radians(1.0)
FINE_TRANS_WIN    = 0.30            # m, +/- around the coarse optimum
FINE_ROT_WIN      = math.radians(7.0)
# A pose from the store gets a wider window — the robot may have been nudged,
# or drifted, between the last save and this launch.
PRIOR_TRANS_WIN   = 0.50
PRIOR_ROT_WIN     = math.radians(15.0)

COARSE_BEAMS = 60
FINE_BEAMS   = 180

# ── Scan conditioning ───────────────────────────────────────────────────────
MAX_BEAM_RANGE = 6.0        # m; the X3 lidar gets unreliable past this
MIN_VALID_BEAMS = 20

# ── Candidate positions ─────────────────────────────────────────────────────
ROBOT_RADIUS = 0.18         # m; poses closer than this to a wall are impossible

# ── Acceptance gates ────────────────────────────────────────────────────────
# The margin is a *residual* ratio, (1 - runner_up) / (1 - winner), not a plain
# score ratio.  Good matches score ~0.95 and their rivals ~0.85, so the plain
# ratio is a hair over 1.0 for everything and cannot be thresholded.  Comparing
# what each pose fails to explain spreads the same data over a usable range.
#
# Measured over 120 synthetic trials (0-40% unmapped clutter, 20% beam dropout)
# across both real maps: every one of the 6 wrong matches scored <= 1.20, while
# 88% of correct matches scored >= 1.30.  Note the absolute score was useless as
# a discriminator (wrong median 0.79 vs correct 0.85) — the margin is the gate.
MIN_MARGIN        = 1.30
MIN_ACCEPT_SCORE  = 0.45    # sanity floor only; the margin does the real work
STORED_COMPETITIVE = 0.97   # stored pose wins ties within this fraction of best
DISTINCT_XY       = 0.60    # m   \ two modes closer than this are the same mode
DISTINCT_YAW      = math.radians(35.0)  # /
N_MODES           = 3       # top-K modes refined and compared

# ── Published covariance ────────────────────────────────────────────────────
# Tighter than a hand-clicked RViz estimate: a scan match that passed the margin
# gate is worth more than a mouse drag, and a tight cloud converges faster.
SEED_COV_XY_MATCH  = 0.06   # m^2  (~0.25 m 1-sigma)
SEED_COV_YAW_MATCH = 0.02   # rad^2 (~8 deg 1-sigma)
SEED_COV_XY_STORED = 0.12
SEED_COV_YAW_STORED = 0.04

# ── URDF fallback if TF is unavailable: base_link -> laser_link ─────────────
# yahboomcar_X3.urdf.xacro laser_joint xyz="-0.0115 ~0 0.191" rpy="0 0 pi"
FALLBACK_LASER_X = -0.0115
FALLBACK_LASER_Y = 0.0
FALLBACK_LASER_YAW = math.pi


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ═══════════════════════════════════════════════════════════════════════════
# Map -> likelihood field
# ═══════════════════════════════════════════════════════════════════════════

class LikelihoodField:
    """
    A map rasterised into a per-cell "how likely is a lidar return here" field.

    Cell (mx, my) covers world point (origin_x + (mx+0.5)*res,
    origin_y + (my+0.5)*res) — the OccupancyGrid convention, with my increasing
    upward.  A .pgm loaded from disk is flipped vertically to match, since image
    row 0 is the *top* of the map.
    """

    def __init__(
        self,
        occupied: np.ndarray,   # bool [H, W], True = wall
        known_free: np.ndarray,  # bool [H, W], True = observed free space
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        if occupied.shape != known_free.shape:
            raise ValueError("occupied/known_free shape mismatch")
        if not occupied.any():
            raise ValueError("map contains no occupied cells — cannot match against it")

        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)
        self.h, self.w = occupied.shape
        self.occupied = occupied
        self.known_free = known_free

        self.lf = self._build_field(occupied, self.res)
        self.candidates = self._build_candidates(known_free, occupied, self.res)

    # ── construction helpers ────────────────────────────────────────────────

    @staticmethod
    def _build_field(occupied: np.ndarray, res: float) -> np.ndarray:
        """exp(-d^2 / 2*sigma^2) where d = distance to the nearest wall."""
        import cv2
        # distanceTransform measures distance to the nearest ZERO pixel, so the
        # walls have to be the zeros.
        src = np.where(occupied, 0, 255).astype(np.uint8)
        dist_cells = cv2.distanceTransform(src, cv2.DIST_L2, 5)
        dist_m = np.minimum(dist_cells * res, MAX_LIKELIHOOD_DIST)
        return np.exp(-(dist_m ** 2) / (2.0 * SIGMA_HIT ** 2)).astype(np.float32)

    @staticmethod
    def _build_candidates(
        known_free: np.ndarray, occupied: np.ndarray, res: float
    ) -> np.ndarray:
        """
        Free cells the robot's centre could physically occupy, as world (x, y).

        Erodes the free space by the robot radius so the search never proposes a
        pose with the chassis inside a wall.
        """
        import cv2
        rad_cells = max(1, int(round(ROBOT_RADIUS / res)))
        k = 2 * rad_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        src = known_free.astype(np.uint8)
        eroded = cv2.erode(src, kernel).astype(bool)
        # Erosion can wipe out a tight map entirely; fall back to raw free space
        # rather than returning nothing to search over.
        if not eroded.any():
            logger.warning(
                "[AutoPose] free space vanished under a %.2f m erosion — "
                "searching raw free cells instead", ROBOT_RADIUS)
            eroded = known_free & ~occupied
        my, mx = np.nonzero(eroded)
        return np.stack([mx, my], axis=1).astype(np.int32)

    # ── factories ───────────────────────────────────────────────────────────

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "LikelihoodField":
        """Load a map_server .yaml/.pgm pair (used by the offline CLI/tests)."""
        import cv2
        import yaml as _yaml

        with open(yaml_path, "r") as fh:
            meta = _yaml.safe_load(fh)

        img_name = meta["image"]
        img_path = img_name if os.path.isabs(img_name) else os.path.join(
            os.path.dirname(os.path.abspath(yaml_path)), img_name)
        img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"could not read map image {img_path}")
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        negate = int(meta.get("negate", 0))
        occ_th = float(meta.get("occupied_thresh", 0.65))
        free_th = float(meta.get("free_thresh", 0.25))
        res = float(meta["resolution"])
        origin = meta.get("origin", [0.0, 0.0, 0.0])

        shade = (img / 255.0) if negate else ((255.0 - img) / 255.0)
        occupied = shade > occ_th
        # 205 ("unknown") shades to 0.196, which slips under a 0.25 free_thresh
        # and would be mistaken for free space.  Cap the free test at the classic
        # 0.196 so unknown stays unknown regardless of what the yaml claims.
        known_free = shade < min(free_th, 0.196)

        # Image row 0 is max-y; the grid convention is row 0 = min-y.
        return cls(np.flipud(occupied), np.flipud(known_free),
                   res, float(origin[0]), float(origin[1]))

    @classmethod
    def from_occupancy_grid(cls, msg) -> "LikelihoodField":
        """Build from a live nav_msgs/OccupancyGrid (the /map topic)."""
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        occupied = data >= 65
        known_free = (data >= 0) & (data <= 25)
        return cls(occupied, known_free, info.resolution,
                   info.origin.position.x, info.origin.position.y)

    # ── scoring ─────────────────────────────────────────────────────────────

    def score_positions(
        self, pts: np.ndarray, xy: np.ndarray, yaw: float
    ) -> np.ndarray:
        """
        Mean beam likelihood for many positions at one heading.

        pts : (B, 2) scan points in the robot base frame
        xy  : (N, 2) candidate world positions
        returns (N,) in [0, 1]
        """
        c, s = math.cos(yaw), math.sin(yaw)
        rx = pts[:, 0] * c - pts[:, 1] * s
        ry = pts[:, 0] * s + pts[:, 1] * c

        wx = xy[:, 0:1] + rx[None, :]
        wy = xy[:, 1:2] + ry[None, :]

        mx = ((wx - self.ox) / self.res).astype(np.int32)
        my = ((wy - self.oy) / self.res).astype(np.int32)
        inside = (mx >= 0) & (mx < self.w) & (my >= 0) & (my < self.h)
        np.clip(mx, 0, self.w - 1, out=mx)
        np.clip(my, 0, self.h - 1, out=my)

        vals = self.lf[my, mx]
        # A beam landing off the edge of the map explains nothing.
        vals *= inside
        return vals.mean(axis=1)

    def score_pose(self, pts: np.ndarray, x: float, y: float, yaw: float) -> float:
        return float(self.score_positions(
            pts, np.array([[x, y]], dtype=np.float64), yaw)[0])

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        return (int((x - self.ox) / self.res), int((y - self.oy) / self.res))

    def in_known_free(self, x: float, y: float) -> bool:
        mx, my = self.cell_of(x, y)
        if not (0 <= mx < self.w and 0 <= my < self.h):
            return False
        return bool(self.known_free[my, mx])


# ═══════════════════════════════════════════════════════════════════════════
# Scan matching
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class MatchResult:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    score: float = 0.0
    margin: float = 0.0
    accepted: bool = False
    reason: str = ""
    modes: List[Tuple[float, float, float, float]] = field(default_factory=list)
    elapsed: float = 0.0

    def __str__(self) -> str:
        return (f"({self.x:+.2f}, {self.y:+.2f}, {math.degrees(self.yaw):+.1f}deg) "
                f"score={self.score:.3f} margin={self.margin:.2f} "
                f"{'ACCEPT' if self.accepted else 'REJECT'} [{self.reason}] "
                f"{self.elapsed*1e3:.0f}ms")


def residual_margin(best: float, runner_up: Optional[float]) -> float:
    """
    How much better the winner explains the scan than its best rival.

    Ratio of unexplained residual rather than of score — see MIN_MARGIN.
    """
    if runner_up is None:
        return float("inf")
    return (1.0 - runner_up) / max(1.0 - best, 1e-6)


def subsample(pts: np.ndarray, n: int) -> np.ndarray:
    """Evenly thin a point set down to at most n points."""
    if len(pts) <= n:
        return pts
    idx = np.linspace(0, len(pts) - 1, n).astype(np.int32)
    return pts[idx]


class ScanMatcher:
    """Coarse-to-fine brute-force search of a scan against a likelihood field."""

    def __init__(self, lf: LikelihoodField) -> None:
        self.lf = lf

    # ── public ──────────────────────────────────────────────────────────────

    def locate(
        self, pts: np.ndarray, prior: Optional[Tuple[float, float, float]] = None
    ) -> MatchResult:
        """
        Global localisation: search the whole map.

        pts   : (B, 2) scan endpoints in the robot base frame.
        prior : optional (x, y, yaw) from the pose store.  The prior does not
                skip the search — it is refined, scored on the same footing as
                every other mode, and used only to *break a tie* the scan alone
                cannot.  That is the one thing it is genuinely good for: a wrong
                pose still scores ~0.79 against a correct 0.85, so trusting a
                stored pose on its absolute score would happily confirm a robot
                that had been picked up and moved.
        """
        t0 = time.time()
        res = MatchResult()

        if len(pts) < MIN_VALID_BEAMS:
            res.reason = f"only {len(pts)} valid beams"
            res.elapsed = time.time() - t0
            return res

        cand_cells = self.lf.candidates
        if len(cand_cells) == 0:
            res.reason = "no free cells to search"
            res.elapsed = time.time() - t0
            return res

        # Thin the candidate positions to the coarse translation step.
        step = max(1, int(round(COARSE_TRANS_STEP / self.lf.res)))
        keep = (cand_cells[:, 0] % step == 0) & (cand_cells[:, 1] % step == 0)
        cells = cand_cells[keep]
        if len(cells) == 0:
            cells = cand_cells
        xy = np.stack([
            self.lf.ox + (cells[:, 0] + 0.5) * self.lf.res,
            self.lf.oy + (cells[:, 1] + 0.5) * self.lf.res,
        ], axis=1)

        coarse_pts = subsample(pts, COARSE_BEAMS)
        yaws = np.arange(0.0, 2.0 * math.pi, COARSE_ROT_STEP)

        scores = np.empty((len(yaws), len(xy)), dtype=np.float32)
        for i, yaw in enumerate(yaws):
            scores[i] = self.lf.score_positions(coarse_pts, xy, float(yaw))

        # Pull out the top few *distinct* modes, then re-judge them all on the
        # full beam set so the margin compares like with like.
        modes = self._extract_modes(scores, xy, yaws, N_MODES)
        fine_pts = subsample(pts, FINE_BEAMS)
        refined = [self.refine(fine_pts, mx, my, myaw) for (mx, my, myaw, _) in modes]
        refined.sort(key=lambda r: r[3], reverse=True)

        # The prior gets a wider refinement window than a coarse-grid mode: the
        # robot may have been nudged since the pose was saved.
        prior_mode = None
        if prior is not None:
            prior_mode = self.refine(fine_pts, prior[0], prior[1], prior[2],
                                     trans_win=PRIOR_TRANS_WIN,
                                     rot_win=PRIOR_ROT_WIN)

        res.modes = refined
        best = refined[0]
        runner_up = self._best_distinct(refined, best)
        margin = residual_margin(best[3], None if runner_up is None else runner_up[3])

        res.x, res.y, res.yaw, res.score = best
        res.margin = margin

        if best[3] < MIN_ACCEPT_SCORE:
            res.reason = f"score {best[3]:.3f} < {MIN_ACCEPT_SCORE}"
        elif margin >= MIN_MARGIN:
            res.accepted = True
            res.reason = "scan match"
        elif prior_mode is not None:
            # Ambiguous on the scan alone.  If the stored pose is one of the
            # competing modes, it is the tie-breaker; if it is not, it has been
            # overruled by the scan and must not be published.
            same_mode = self._same_mode(prior_mode, best)
            competitive = prior_mode[3] >= STORED_COMPETITIVE * best[3]
            if prior_mode[3] >= MIN_ACCEPT_SCORE and (same_mode or competitive):
                res.x, res.y, res.yaw, res.score = prior_mode
                res.accepted = True
                res.reason = ("stored pose agrees with the best mode"
                              if same_mode else
                              f"stored pose broke a tie (margin {margin:.2f})")
            else:
                res.reason = (f"ambiguous (margin {margin:.2f} < {MIN_MARGIN}) and "
                              f"the stored pose scored {prior_mode[3]:.3f} vs "
                              f"{best[3]:.3f} — overruled by the scan")
        else:
            res.reason = (f"ambiguous: margin {margin:.2f} < {MIN_MARGIN} "
                          f"(map looks self-similar here)")

        res.elapsed = time.time() - t0
        return res

    def refine(
        self, pts: np.ndarray, x0: float, y0: float, yaw0: float,
        trans_win: float = FINE_TRANS_WIN, rot_win: float = FINE_ROT_WIN,
    ) -> Tuple[float, float, float, float]:
        """Local grid search around a seed pose.  Returns (x, y, yaw, score)."""
        offs = np.arange(-trans_win, trans_win + 1e-9, FINE_TRANS_STEP)
        gx, gy = np.meshgrid(offs, offs, indexing="ij")
        xy = np.stack([x0 + gx.ravel(), y0 + gy.ravel()], axis=1)

        yaws = np.arange(yaw0 - rot_win, yaw0 + rot_win + 1e-9, FINE_ROT_STEP)

        best = (x0, y0, yaw0, -1.0)
        for yaw in yaws:
            s = self.lf.score_positions(pts, xy, float(yaw))
            i = int(np.argmax(s))
            if s[i] > best[3]:
                best = (float(xy[i, 0]), float(xy[i, 1]),
                        _wrap_angle(float(yaw)), float(s[i]))
        return best

    # ── internals ───────────────────────────────────────────────────────────

    @staticmethod
    def _extract_modes(scores, xy, yaws, k):
        """Greedy non-maximum suppression over the coarse (yaw, position) grid."""
        work = scores.copy()
        out = []
        for _ in range(k):
            flat = int(np.argmax(work))
            if work.flat[flat] <= 0.0:
                break
            iy, ip = np.unravel_index(flat, work.shape)
            x, y = float(xy[ip, 0]), float(xy[ip, 1])
            yaw = _wrap_angle(float(yaws[iy]))
            out.append((x, y, yaw, float(work[iy, ip])))

            # Suppress everything belonging to this mode.
            near_xy = ((xy[:, 0] - x) ** 2 + (xy[:, 1] - y) ** 2) < DISTINCT_XY ** 2
            dyaw = np.abs(np.arctan2(np.sin(yaws - yaw), np.cos(yaws - yaw)))
            near_yaw = dyaw < DISTINCT_YAW
            work[np.ix_(near_yaw, near_xy)] = -1.0
        return out

    @staticmethod
    def _same_mode(a, b) -> bool:
        return (math.hypot(a[0] - b[0], a[1] - b[1]) < DISTINCT_XY
                and abs(_wrap_angle(a[2] - b[2])) < DISTINCT_YAW)

    @classmethod
    def _best_distinct(cls, refined, best):
        """Highest-scoring mode that is not just the winner again."""
        for cand in refined:
            if not cls._same_mode(cand, best):
                return cand
        return None


# ═══════════════════════════════════════════════════════════════════════════
# Scan -> base-frame points
# ═══════════════════════════════════════════════════════════════════════════

def scan_to_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float = FALLBACK_LASER_X,
    laser_y: float = FALLBACK_LASER_Y,
    laser_yaw: float = FALLBACK_LASER_YAW,
) -> np.ndarray:
    """Project a LaserScan into (B, 2) endpoints in the robot base frame."""
    r = np.asarray(ranges, dtype=np.float64)
    ang = angle_min + np.arange(len(r)) * angle_increment

    hi = min(range_max if range_max > 0 else MAX_BEAM_RANGE, MAX_BEAM_RANGE)
    lo = max(range_min, 0.05)
    valid = np.isfinite(r) & (r > lo) & (r < hi)
    r, ang = r[valid], ang[valid]

    lx = r * np.cos(ang)
    ly = r * np.sin(ang)
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    return np.stack([laser_x + lx * c - ly * s,
                     laser_y + lx * s + ly * c], axis=1)


# ═══════════════════════════════════════════════════════════════════════════
# Persisted pose store
# ═══════════════════════════════════════════════════════════════════════════

class PoseStore:
    """
    Last-known AMCL pose per map, on disk.

    Keyed by map name so a pose recorded in RealLab1 is never applied to
    apartment3_test.  Writes are atomic (tmp + rename) because the saver runs on
    a timer and a half-written file on power loss would be worse than none.
    """

    def __init__(self, path: str = DEFAULT_POSE_STORE) -> None:
        self.path = path
        self._lock = threading.Lock()
        self._data: Dict[str, dict] = self._read()

    def _read(self) -> Dict[str, dict]:
        try:
            with open(self.path, "r") as fh:
                data = json.load(fh)
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            return {}
        except Exception as exc:
            logger.warning("[AutoPose] pose store unreadable (%s) — starting empty", exc)
            return {}

    def get(self, map_key: str) -> Optional[dict]:
        with self._lock:
            entry = self._data.get(map_key)
            return dict(entry) if entry else None

    def put(self, map_key: str, x: float, y: float, yaw: float) -> None:
        entry = {"x": float(x), "y": float(y), "yaw": float(yaw),
                 "saved_at": time.time()}
        with self._lock:
            self._data[map_key] = entry
            payload = json.dumps(self._data, indent=2)
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            tmp = f"{self.path}.tmp"
            with open(tmp, "w") as fh:
                fh.write(payload)
            os.replace(tmp, self.path)
        except Exception as exc:
            logger.warning("[AutoPose] could not write pose store: %s", exc)

    def all(self) -> Dict[str, dict]:
        with self._lock:
            return dict(self._data)


def map_key_from_path(map_path: Optional[str]) -> Optional[str]:
    if not map_path:
        return None
    return os.path.splitext(os.path.basename(map_path))[0]


def map_key_from_grid(msg) -> str:
    """
    Fingerprint an OccupancyGrid so an externally-launched Nav2 (no map path in
    hand) still keys into the same store entry every time it loads that map.
    """
    import hashlib
    info = msg.info
    data = np.asarray(msg.data, dtype=np.int8)
    h = hashlib.md5(data[::97].tobytes()).hexdigest()[:10]
    return (f"grid_{info.width}x{info.height}_{info.resolution:.3f}_"
            f"{info.origin.position.x:+.2f}_{info.origin.position.y:+.2f}_{h}")


# ═══════════════════════════════════════════════════════════════════════════
# ROS glue
# ═══════════════════════════════════════════════════════════════════════════

class AutoInitialPose:
    """
    Watches for AMCL, then seeds it — from the store if the stored pose still
    matches the scan, otherwise from a global scan match.

    Also saves /amcl_pose back to the store so the *next* launch has something
    to restore.  Created once and shared; safe to construct even if Nav2 is
    never launched.
    """

    # AMCL is considered present once it subscribes to /initialpose.
    _WATCH_PERIOD = 2.0
    _SAVE_PERIOD = 5.0
    _SAVE_MIN_MOVE = 0.10           # m
    _SAVE_MIN_TURN = math.radians(5.0)

    def __init__(self, node, nav2_client, store_path: str = DEFAULT_POSE_STORE) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                               qos_profile_sensor_data)

        self._node = node
        self._nav2 = nav2_client
        self.store = PoseStore(store_path)

        self._lock = threading.Lock()
        self._scan = None               # raw LaserScan
        self._grid = None               # raw OccupancyGrid
        self._lf: Optional[LikelihoodField] = None
        self._lf_key: Optional[str] = None
        self._map_path: Optional[str] = None
        self._slam_mode = False
        self._enabled = True

        self._amcl_seen = False
        self._seeded_key: Optional[str] = None
        self._seeding = False
        self._last_result: Optional[MatchResult] = None
        self._last_saved: Optional[Tuple[float, float, float]] = None
        self._last_save_t = 0.0

        # /map is latched (transient_local) — a volatile subscription silently
        # receives nothing when the publisher sent the map before we started.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        node.create_subscription(LaserScan, "/scan", self._scan_cb,
                                 qos_profile_sensor_data)
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._amcl_pose_cb, 10)

        self._tf_buffer = None
        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        except Exception as exc:
            logger.warning("[AutoPose] tf2 unavailable (%s) — using URDF fallback "
                           "laser transform", exc)

        node.create_timer(self._WATCH_PERIOD, self._watch_cb)
        logger.info("[AutoPose] armed (store: %s)", self.store.path)

    # ── configuration from the launch path ──────────────────────────────────

    def arm(self, map_path: Optional[str] = None, slam: bool = False) -> None:
        """Called when Nav2 is launched from the server, so we know the map."""
        with self._lock:
            self._map_path = map_path
            self._slam_mode = bool(slam)
            self._seeded_key = None
            self._amcl_seen = False
            if slam:
                logger.info("[AutoPose] SLAM mode — auto-seeding disabled")
            else:
                logger.info("[AutoPose] will seed AMCL for map: %s",
                            map_path or "(from /map)")

    def set_enabled(self, on: bool) -> None:
        with self._lock:
            self._enabled = bool(on)

    def status(self) -> dict:
        with self._lock:
            r = self._last_result
            return {
                "enabled": self._enabled,
                "seeded": self._seeded_key is not None,
                "map_key": self._lf_key,
                "have_scan": self._scan is not None,
                "have_map": self._lf is not None,
                "last": None if r is None else {
                    "x": round(r.x, 3), "y": round(r.y, 3),
                    "yaw_deg": round(math.degrees(r.yaw), 1),
                    "score": round(r.score, 3),
                    "margin": (None if not math.isfinite(r.margin)
                               else round(r.margin, 2)),
                    "accepted": r.accepted, "reason": r.reason,
                },
            }

    # ── subscriptions ───────────────────────────────────────────────────────

    def _scan_cb(self, msg) -> None:
        with self._lock:
            self._scan = msg

    def _map_cb(self, msg) -> None:
        key = map_key_from_path(self._map_path) or map_key_from_grid(msg)
        with self._lock:
            if key == self._lf_key and self._lf is not None:
                return
        try:
            lf = LikelihoodField.from_occupancy_grid(msg)
        except Exception as exc:
            logger.warning("[AutoPose] could not build likelihood field: %s", exc)
            return
        with self._lock:
            self._grid = msg
            self._lf = lf
            self._lf_key = key
        logger.info("[AutoPose] map ready: %s (%dx%d @ %.3f m, %d candidate poses)",
                    key, lf.w, lf.h, lf.res, len(lf.candidates))

    def _amcl_pose_cb(self, msg) -> None:
        """Throttle-persist AMCL's pose so the next launch can restore it."""
        with self._lock:
            key, slam, enabled = self._lf_key, self._slam_mode, self._enabled
        if slam or not enabled or key is None:
            return

        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        x, y = p.position.x, p.position.y

        now = time.time()
        with self._lock:
            last = self._last_saved
            since = now - self._last_save_t
        moved = (last is None
                 or math.hypot(x - last[0], y - last[1]) > self._SAVE_MIN_MOVE
                 or abs(_wrap_angle(yaw - last[2])) > self._SAVE_MIN_TURN)
        if not moved or since < self._SAVE_PERIOD:
            return

        with self._lock:
            self._last_saved = (x, y, yaw)
            self._last_save_t = now
        self.store.put(key, x, y, yaw)

    # ── the watcher ─────────────────────────────────────────────────────────

    def _watch_cb(self) -> None:
        """Fire the seeder the moment AMCL shows up and we have map + scan."""
        with self._lock:
            if (not self._enabled or self._slam_mode or self._seeding
                    or self._seeded_key is not None):
                return
            have = self._lf is not None and self._scan is not None

        if not self._amcl_present():
            return
        if not have:
            return

        with self._lock:
            self._seeding = True
        threading.Thread(target=self._seed_worker, daemon=True).start()

    def _amcl_present(self) -> bool:
        try:
            n = self._nav2._initial_pose_pub.get_subscription_count()
        except Exception:
            return False
        if n > 0 and not self._amcl_seen:
            self._amcl_seen = True
            logger.info("[AutoPose] AMCL detected on /initialpose")
        return n > 0

    def _seed_worker(self) -> None:
        try:
            result = self.seed_now()
            if result is not None and result.accepted:
                with self._lock:
                    self._seeded_key = self._lf_key
        except Exception as exc:
            logger.exception("[AutoPose] seeding failed: %s", exc)
        finally:
            with self._lock:
                self._seeding = False

    # ── the actual work ─────────────────────────────────────────────────────

    def seed_now(self, ignore_stored: bool = False) -> Optional[MatchResult]:
        """
        Scan-match against the live map, using the stored pose as a tie-breaker,
        and publish to /initialpose if the result clears the confidence gate.

        Returns the MatchResult, or None if the map or scan is missing.
        """
        with self._lock:
            lf, scan, key = self._lf, self._scan, self._lf_key
        if lf is None or scan is None:
            logger.warning("[AutoPose] cannot seed — %s",
                           "no map yet" if lf is None else "no scan yet")
            return None

        prior = None
        if not ignore_stored and key:
            stored = self.store.get(key)
            if stored:
                prior = (stored["x"], stored["y"], stored["yaw"])
                age_h = (time.time() - stored.get("saved_at", 0)) / 3600.0
                logger.info("[AutoPose] stored pose for %s: (%+.2f, %+.2f, "
                            "%+.1fdeg), %.1f h old", key, prior[0], prior[1],
                            math.degrees(prior[2]), age_h)
            else:
                logger.info("[AutoPose] no stored pose for %s — global search only",
                            key)

        pts = self._scan_points(scan)
        res = ScanMatcher(lf).locate(pts, prior=prior)
        logger.info("[AutoPose] %s", res)
        for i, (mx, my, myaw, ms) in enumerate(res.modes):
            logger.debug("[AutoPose]   mode %d: (%+.2f, %+.2f, %+.1fdeg) %.3f",
                         i, mx, my, math.degrees(myaw), ms)

        if res.accepted:
            stored_win = res.reason.startswith("stored")
            self._publish(
                res,
                SEED_COV_XY_STORED if stored_win else SEED_COV_XY_MATCH,
                SEED_COV_YAW_STORED if stored_win else SEED_COV_YAW_MATCH)
        else:
            logger.warning("[AutoPose] not seeding: %s — set the pose manually",
                           res.reason)
        with self._lock:
            self._last_result = res
        return res

    def _publish(self, res: MatchResult, cov_xy: float, cov_yaw: float) -> None:
        self._nav2.set_initial_pose(res.x, res.y, res.yaw,
                                    cov_xy=cov_xy, cov_yaw=cov_yaw)
        logger.info("[AutoPose] seeded AMCL at (%+.2f, %+.2f, %+.1fdeg) "
                    "score=%.3f (%s)", res.x, res.y, math.degrees(res.yaw),
                    res.score, res.reason)

    def _scan_points(self, scan) -> np.ndarray:
        lx, ly, lyaw = self._laser_transform(scan.header.frame_id)
        return scan_to_points(
            np.asarray(scan.ranges, dtype=np.float64),
            scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            laser_x=lx, laser_y=ly, laser_yaw=lyaw)

    def _laser_transform(self, frame_id: str) -> Tuple[float, float, float]:
        """base_footprint <- laser frame, from TF, falling back to the URDF."""
        if self._tf_buffer is not None and frame_id:
            try:
                import rclpy.time
                tf = self._tf_buffer.lookup_transform(
                    "base_footprint", frame_id, rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return (t.x, t.y, yaw)
            except Exception as exc:
                logger.debug("[AutoPose] TF lookup failed (%s) — URDF fallback", exc)
        return (FALLBACK_LASER_X, FALLBACK_LASER_Y, FALLBACK_LASER_YAW)


# ═══════════════════════════════════════════════════════════════════════════
# Offline CLI / self-test
# ═══════════════════════════════════════════════════════════════════════════

def simulate_scan(
    lf: LikelihoodField, x: float, y: float, yaw: float,
    n_beams: int = 360, max_range: float = MAX_BEAM_RANGE,
    noise_sigma: float = 0.0, seed: int = 0,
) -> np.ndarray:
    """
    Raycast a synthetic 360-degree scan from a pose in the map.

    Only used by the self-test — it lets the matcher be checked against ground
    truth without a robot.
    """
    rng = np.random.default_rng(seed)
    angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
    step = lf.res * 0.5
    n_steps = int(max_range / step)

    t = (np.arange(1, n_steps + 1) * step)[None, :]
    px = x + np.cos(angles + yaw)[:, None] * t
    py = y + np.sin(angles + yaw)[:, None] * t

    mx = ((px - lf.ox) / lf.res).astype(np.int32)
    my = ((py - lf.oy) / lf.res).astype(np.int32)
    inside = (mx >= 0) & (mx < lf.w) & (my >= 0) & (my < lf.h)
    np.clip(mx, 0, lf.w - 1, out=mx)
    np.clip(my, 0, lf.h - 1, out=my)

    hit = lf.occupied[my, mx] & inside
    # first True along each ray, or no hit at all
    any_hit = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    ranges = np.where(any_hit, (first + 1) * step, np.inf)
    if noise_sigma > 0:
        ranges = np.where(any_hit, ranges + rng.normal(0, noise_sigma, len(ranges)),
                          ranges)
    return ranges


def _cmd_selftest(args) -> int:
    """
    Raycast scans from known poses and check the matcher recovers them.

    Runs three cohorts, because the two mechanisms fail in different directions:
      no-prior    – the pure global search
      good-prior  – a store entry near the truth: must not be made *worse*
      stale-prior – a store entry somewhere else entirely (robot was moved):
                    must be overruled, never published
    """
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    lf = LikelihoodField.from_yaml(args.map)
    print(f"map      : {args.map}")
    print(f"grid     : {lf.w}x{lf.h} @ {lf.res} m  origin=({lf.ox}, {lf.oy})")
    print(f"free     : {int(lf.known_free.sum())} cells, "
          f"{len(lf.candidates)} reachable centres")
    print(f"occupied : {int(lf.occupied.sum())} cells")
    print(f"settings : sigma={SIGMA_HIT} margin>={MIN_MARGIN} "
          f"score>={MIN_ACCEPT_SCORE}\n")

    matcher = ScanMatcher(lf)
    cells = lf.candidates
    rc = 0

    for cohort in ("no-prior", "good-prior", "stale-prior"):
        rng = np.random.default_rng(args.seed)
        picks = rng.choice(len(cells), size=min(args.trials, len(cells)),
                           replace=False)
        n_ok = n_rejected = n_wrong = 0
        errs = []
        times = []

        for ci in picks:
            gx = lf.ox + (cells[ci, 0] + 0.5) * lf.res
            gy = lf.oy + (cells[ci, 1] + 0.5) * lf.res
            gyaw = float(rng.uniform(-math.pi, math.pi))

            ranges = simulate_scan(lf, gx, gy, gyaw, noise_sigma=args.noise,
                                   seed=int(rng.integers(1 << 30)))
            if args.clutter > 0:
                hit = rng.random(len(ranges)) < args.clutter
                ranges = np.where(hit, rng.uniform(0.3, 2.0, len(ranges)), ranges)
            pts = scan_to_points(ranges, -math.pi, 2 * math.pi / len(ranges),
                                 0.05, MAX_BEAM_RANGE,
                                 laser_x=0.0, laser_y=0.0, laser_yaw=0.0)

            if cohort == "no-prior":
                prior = None
            elif cohort == "good-prior":
                prior = (gx + rng.normal(0, 0.10), gy + rng.normal(0, 0.10),
                         gyaw + rng.normal(0, math.radians(4)))
            else:
                oj = rng.choice(len(cells))
                prior = (lf.ox + (cells[oj, 0] + 0.5) * lf.res,
                         lf.oy + (cells[oj, 1] + 0.5) * lf.res,
                         float(rng.uniform(-math.pi, math.pi)))
                if math.hypot(prior[0] - gx, prior[1] - gy) < 1.0:
                    continue    # not actually stale

            res = matcher.locate(pts, prior=prior)
            times.append(res.elapsed)
            derr = math.hypot(res.x - gx, res.y - gy)
            aerr = abs(math.degrees(_wrap_angle(res.yaw - gyaw)))

            if not res.accepted:
                n_rejected += 1
            elif derr < 0.30 and aerr < 10.0:
                n_ok += 1
                errs.append((derr, aerr))
            else:
                n_wrong += 1
                print(f"  WRONG [{cohort}] truth=({gx:+.2f},{gy:+.2f},"
                      f"{math.degrees(gyaw):+6.1f}) got=({res.x:+.2f},{res.y:+.2f},"
                      f"{math.degrees(res.yaw):+6.1f}) err={derr*100:.0f}cm/"
                      f"{aerr:.0f}deg score={res.score:.3f} "
                      f"margin={res.margin:.2f} [{res.reason}]")

            n = n_ok + n_rejected + n_wrong
        print(f"{cohort:12s}: correct {n_ok:3d}  rejected {n_rejected:3d} (safe)  "
              f"WRONG {n_wrong:3d}   [{np.mean(times)*1e3:.0f} ms/match]")
        if errs:
            d = np.array([e[0] for e in errs]) * 100
            a = np.array([e[1] for e in errs])
            print(f"{'':12s}  accepted error: med {np.median(d):.1f} cm / "
                  f"{np.median(a):.1f} deg, max {d.max():.1f} cm / {a.max():.1f} deg")
        if n_wrong:
            rc = 1

    print("\nPASS — no bad pose was ever published" if rc == 0
          else "\nFAIL — a bad pose was published")
    return rc


def _cmd_show_store(args) -> int:
    store = PoseStore(args.store)
    data = store.all()
    if not data:
        print(f"(empty) {store.path}")
        return 0
    print(store.path)
    for key, e in sorted(data.items()):
        age = (time.time() - e.get("saved_at", 0)) / 3600.0
        print(f"  {key:36s} ({e['x']:+.2f}, {e['y']:+.2f}, "
              f"{math.degrees(e['yaw']):+.1f}deg)  {age:.1f} h ago")
    return 0


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[2])
    sub = ap.add_subparsers(dest="cmd", required=True)

    st = sub.add_parser("selftest", help="raycast scans and check recovery")
    st.add_argument("--map", required=True, help="path to a map .yaml")
    st.add_argument("--trials", type=int, default=10)
    st.add_argument("--noise", type=float, default=0.03, help="range noise sigma (m)")
    st.add_argument("--clutter", type=float, default=0.15,
                    help="fraction of beams hitting unmapped objects")
    st.add_argument("--seed", type=int, default=1)
    st.set_defaults(func=_cmd_selftest)

    sh = sub.add_parser("show-store", help="print the persisted poses")
    sh.add_argument("--store", default=DEFAULT_POSE_STORE)
    sh.set_defaults(func=_cmd_show_store)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
