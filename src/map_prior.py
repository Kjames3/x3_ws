"""
map_prior.py
Prior-map change detection and site recognition for the Yahboom X3.

Two jobs, both built on the likelihood-field matcher in auto_initial_pose.py:

  1. MapPrior.label() — given lidar points already in the map frame, label each
     one STATIC (on or near a mapped wall), NEW (in mapped free space, so
     something that was not there when the map was made) or UNKNOWN (unknown
     cell or off the map).  Only NEW points may become dynamic candidates; a
     point on a mapped wall cannot be a mover.  See
     notes/map_prior_dynamic_detection_plan.md.

  2. identify_site() — the robot only ever lives in a few places (the lab, the
     apartment).  Run the global scan match against every site's map and pick
     the one whose best pose explains the scan clearly better than any other
     site does.  Refuses rather than guesses.

No ROS dependency.  CLI (offline):
    python3 src/map_prior.py selftest            # synthetic scans, all sites
    python3 src/map_prior.py label --map M.yaml --x X --y Y --yaw DEG
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from auto_initial_pose import (  # noqa: E402
    LikelihoodField, MatchResult, ScanMatcher, MAX_BEAM_RANGE, _wrap_angle,
    scan_to_points, simulate_scan,
)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SITES = os.path.join(REPO, "config", "site_maps.json")
MAPS_DIR = os.path.join(REPO, "src", "yahboomcar_nav", "maps")

STATIC, NEW, UNKNOWN, LEARNED = 0, 1, 2, 3
LABEL_NAMES = ("static", "new", "unknown", "learned")

# Plan's starting tolerance; Phase 0 sweeps 0.05-0.20.
D_TOL = 0.10

# A site wins only if its residual is this much smaller than the next site's.
# Same residual-ratio form as the auto-pose gate (MIN_MARGIN 1.30) and for the
# same reason: good matches all score 0.85-0.95, so a score ratio can't be
# thresholded.
SITE_MARGIN = 1.30


# ═══════════════════════════════════════════════════════════════════════════
# Change labelling
# ═══════════════════════════════════════════════════════════════════════════

class MapPrior:
    """A saved map, precomputed for point labelling.  2D now; keep the
    label(points_map_frame) -> labels interface if this goes 3D."""

    def __init__(self, lf: LikelihoodField, d_tol: float = D_TOL,
                 name: str = "") -> None:
        import cv2
        self.lf = lf
        self.name = name
        self.d_tol = float(d_tol)
        src = np.where(lf.occupied, 0, 255).astype(np.uint8)
        self.wall_dist = cv2.distanceTransform(src, cv2.DIST_L2, 5) * lf.res

    @classmethod
    def from_yaml(cls, yaml_path: str, d_tol: float = D_TOL) -> "MapPrior":
        name = os.path.splitext(os.path.basename(yaml_path))[0]
        return cls(LikelihoodField.from_yaml(yaml_path), d_tol, name)

    def label(self, pts: np.ndarray, d_tol: Optional[float] = None) -> np.ndarray:
        """pts (N, 2+) in the map frame -> uint8 labels (N,)."""
        lf = self.lf
        tol = self.d_tol if d_tol is None else d_tol
        mx = np.floor((pts[:, 0] - lf.ox) / lf.res).astype(np.int64)
        my = np.floor((pts[:, 1] - lf.oy) / lf.res).astype(np.int64)
        inside = (mx >= 0) & (mx < lf.w) & (my >= 0) & (my < lf.h)
        mx = np.clip(mx, 0, lf.w - 1)
        my = np.clip(my, 0, lf.h - 1)

        out = np.full(len(pts), UNKNOWN, dtype=np.uint8)
        near_wall = inside & (self.wall_dist[my, mx] <= tol)
        out[near_wall] = STATIC
        # Only cells the map positively saw as free count as change.  Unknown
        # cells next to a wall are already STATIC via the distance test.
        free = inside & ~near_wall & lf.known_free[my, mx]
        out[free] = NEW
        return out

    @staticmethod
    def counts(labels: np.ndarray) -> Dict[str, int]:
        return {n: int((labels == i).sum()) for i, n in enumerate(LABEL_NAMES)}


class ChangeDetector:
    """
    MapPrior plus a learned layer of "new but not moving" cells.

    A saved map goes stale (chairs move), and on the C3 lab captures ~22% of an
    empty room's returns landed in mapped free space while the walls lined up to
    ~2 cm.  So a cell that keeps getting NEW hits is absorbed: each scan, an EMA
    per cell rises toward 1 where NEW points land and decays elsewhere.  Above
    PROMOTE it is LEARNED (static, not on the map).  A walker crosses a cell in
    well under a second and never gets there; anything parked for ~tau seconds
    does, a standing person included -- the camera person path covers people.
    """

    def __init__(self, prior: MapPrior, tau_s: float = 3.0,
                 promote: float = 0.5, scan_hz: float = 7.2) -> None:
        import cv2
        self.prior = prior
        self.decay = math.exp(-1.0 / (tau_s * scan_hz))
        self.promote = promote
        self.ema = np.zeros_like(prior.wall_dist, dtype=np.float32)
        # one-cell dilation absorbs pose jitter (~2 cm) and beam scatter
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self._learned = np.zeros(self.ema.shape, dtype=bool)

    def update(self, pts_map: np.ndarray, d_tol: Optional[float] = None) -> np.ndarray:
        import cv2
        lf = self.prior.lf
        lab = self.prior.label(pts_map, d_tol)
        mx = np.clip(np.floor((pts_map[:, 0] - lf.ox) / lf.res).astype(np.int64), 0, lf.w - 1)
        my = np.clip(np.floor((pts_map[:, 1] - lf.oy) / lf.res).astype(np.int64), 0, lf.h - 1)
        new = lab == NEW
        # Judge against the layer learned BEFORE this scan, so a fresh object
        # is NEW on its first frames.
        lab[new & self._learned[my, mx]] = LEARNED
        hit = np.zeros(self.ema.shape, dtype=np.uint8)
        hit[my[new], mx[new]] = 1
        hit = cv2.dilate(hit, self._kernel)
        self.ema *= self.decay
        self.ema += (1.0 - self.decay) * hit
        self._learned = cv2.dilate((self.ema > self.promote).astype(np.uint8),
                                   self._kernel).astype(bool)
        return lab


def base_to_map(pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + pts[:, 0] * c - pts[:, 1] * s,
                     y + pts[:, 0] * s + pts[:, 1] * c], axis=1)


# ═══════════════════════════════════════════════════════════════════════════
# Site recognition
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class SiteResult:
    site: Optional[str] = None
    map_path: Optional[str] = None
    pose: Optional[MatchResult] = None
    margin: float = 0.0
    accepted: bool = False
    reason: str = ""
    per_site: Dict[str, float] = field(default_factory=dict)
    elapsed: float = 0.0

    def __str__(self) -> str:
        scores = ", ".join(f"{k}={v:.3f}" for k, v in self.per_site.items())
        return (f"{'ACCEPT' if self.accepted else 'REJECT'} site={self.site} "
                f"margin={self.margin:.2f} [{self.reason}] ({scores}) "
                f"{self.elapsed*1e3:.0f}ms")


def load_sites(path: str = DEFAULT_SITES) -> Dict[str, str]:
    """{site name: absolute map yaml}.  Relative paths are under MAPS_DIR."""
    with open(path) as fh:
        raw = json.load(fh)["sites"]
    return {k: v if os.path.isabs(v) else os.path.join(MAPS_DIR, v)
            for k, v in raw.items()}


class SiteIdentifier:
    """Holds one matcher per site map; build once, call identify() per scan."""

    def __init__(self, sites: Dict[str, str]) -> None:
        self.sites = sites
        self.matchers = {k: ScanMatcher(LikelihoodField.from_yaml(p))
                         for k, p in sites.items()}

    def identify(self, pts: np.ndarray) -> SiteResult:
        """pts: (B, 2) scan endpoints in the robot base frame."""
        t0 = time.time()
        res = SiteResult()
        matches = {k: m.locate(pts) for k, m in self.matchers.items()}
        res.per_site = {k: r.score for k, r in matches.items()}
        ranked = sorted(matches.items(), key=lambda kv: kv[1].score, reverse=True)
        best_name, best = ranked[0]
        runner = ranked[1][1].score if len(ranked) > 1 else None
        res.margin = (float("inf") if runner is None else
                      (1.0 - runner) / max(1.0 - best.score, 1e-6))
        res.site, res.map_path, res.pose = best_name, self.sites[best_name], best

        if res.margin < SITE_MARGIN:
            res.reason = (f"sites too alike: margin {res.margin:.2f} < "
                          f"{SITE_MARGIN}")
        elif not best.accepted:
            # Right building, but the pose inside it is ambiguous.  The site is
            # still known; AMCL just needs a manual pose.
            res.accepted = True
            res.reason = f"site clear, pose ambiguous ({best.reason})"
        else:
            res.accepted = True
            res.reason = "site and pose clear"
        res.elapsed = time.time() - t0
        return res


# ═══════════════════════════════════════════════════════════════════════════
# Offline CLI
# ═══════════════════════════════════════════════════════════════════════════

def _cmd_selftest(args) -> int:
    """Raycast scans in every site, check site recognition and labelling."""
    sites = load_sites(args.sites)
    ident = SiteIdentifier(sites)
    rng = np.random.default_rng(args.seed)
    rc = 0
    for name, path in sites.items():
        lf = ident.matchers[name].lf
        prior = MapPrior(lf, name=name)
        cells = lf.candidates
        n_ok = n_rej = n_wrong = 0
        new_frac_clean, new_frac_person = [], []
        for ci in rng.choice(len(cells), size=args.trials, replace=False):
            gx = lf.ox + (cells[ci, 0] + 0.5) * lf.res
            gy = lf.oy + (cells[ci, 1] + 0.5) * lf.res
            gyaw = float(rng.uniform(-math.pi, math.pi))
            ranges = simulate_scan(lf, gx, gy, gyaw, n_beams=720,
                                   noise_sigma=0.02,
                                   seed=int(rng.integers(1 << 30)))
            if args.clutter > 0:
                hit = rng.random(len(ranges)) < args.clutter
                ranges = np.where(hit, rng.uniform(0.3, 2.0, len(ranges)), ranges)
            pts = scan_to_points(ranges, -math.pi, 2 * math.pi / len(ranges),
                                 0.05, MAX_BEAM_RANGE, 0.0, 0.0, 0.0)
            r = ident.identify(pts)
            if not r.accepted:
                n_rej += 1
            elif r.site == name:
                n_ok += 1
            else:
                n_wrong += 1
                print(f"  WRONG truth={name} {r}")

            # Labelling at the true pose: clean scan, then a 0.4 m "person"
            # occluding a few beams 1.0-1.5 m out.
            clean = scan_to_points(simulate_scan(lf, gx, gy, gyaw, n_beams=720,
                                                 noise_sigma=0.02, seed=1),
                                   -math.pi, 2 * math.pi / 720, 0.05,
                                   MAX_BEAM_RANGE, 0.0, 0.0, 0.0)
            lab = prior.label(base_to_map(clean, gx, gy, gyaw))
            new_frac_clean.append((lab == NEW).mean())
            ang = math.atan2(clean[0, 1], clean[0, 0])
            d = rng.uniform(1.0, 1.5)
            blob = np.stack([d * np.cos(np.linspace(-.2, .2, 15) + ang),
                             d * np.sin(np.linspace(-.2, .2, 15) + ang)], axis=1)
            mp = base_to_map(blob, gx, gy, gyaw)
            # only count blobs that actually sit in mapped free space
            if lf.in_known_free(*mp.mean(axis=0)):
                new_frac_person.append((prior.label(mp) == NEW).mean())
        print(f"{name:10s} site: correct {n_ok:3d}  rejected {n_rej:3d}  "
              f"WRONG {n_wrong:3d}")
        print(f"{'':10s} label: clean-scan NEW frac median "
              f"{np.median(new_frac_clean)*100:.2f}% (p95 "
              f"{np.percentile(new_frac_clean, 95)*100:.2f}%); "
              f"person blob NEW frac {np.mean(new_frac_person)*100:.0f}% "
              f"(n={len(new_frac_person)})")
        rc |= n_wrong > 0
    print("\nPASS" if rc == 0 else "\nFAIL — a wrong site was accepted")
    return rc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    st = sub.add_parser("selftest")
    st.add_argument("--sites", default=DEFAULT_SITES)
    st.add_argument("--trials", type=int, default=30)
    st.add_argument("--clutter", type=float, default=0.15)
    st.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    return _cmd_selftest(args)


if __name__ == "__main__":
    sys.exit(main())
