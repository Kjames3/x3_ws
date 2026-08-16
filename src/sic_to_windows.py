"""
sic_to_windows.py — Convert SIC scene clips into velocity-MLP training windows.

Produces (X, y) pairs in exactly the convention `velocity_estimator.py` uses at
inference time, so a model fine-tuned on this output can be dropped straight into
the robot without a frame-convention mismatch.

The critical detail: at inference the estimator does NOT feed raw per-frame ego
coordinates. It keeps a global (map-frame) history per track and re-projects the
whole window into the robot's pose at the CURRENT frame (velocity_estimator.py
~line 525). So the window is ego-motion compensated, and the velocity it implies
is the pedestrian's world velocity expressed in the robot's current frame — with
no ego-translation term. This converter reproduces that exactly.

Usage:
    # single scene, stats only
    python3 src/sic_to_windows.py /path/to/Courtyard_8-005 --stats

    # convert several scenes into one training set
    python3 src/sic_to_windows.py scenes/*/ --out sic_windows.npz --max-range 1.8

    # verify our feature builder is byte-identical to the deployed one
    python3 src/sic_to_windows.py /path/to/scene --verify

A scene directory is one unpacked SIC clip: label_3d/, ego_trajectory/ (and
optionally tf/, calib/). Only label_3d/ and ego_trajectory/ are read — you can
unpack just those two and skip the ~4 GB of cam_img/ and velo/ per clip:

    unzip -q Corridor_1.zip 'label_3d/*' 'ego_trajectory/*' -d Corridor_1
"""

import argparse
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

# ── Must match velocity_estimator.py ──────────────────────────────────────────
WINDOW_SIZE = 10          # T — frames of history per sample
FEATURE_DIM = 4 * WINDOW_SIZE
DT          = 0.1         # seconds/frame. SIC ego odometry steps ~0.093 m/frame
                          # at walking pace, consistent with the 10 Hz INFER_HZ.
STOP_EPS    = 0.01        # metres — matches the is_stopped gate (Idea 108)
CLIP_D      = 0.25        # metres — matches the dx/dy clamp (Idea 63)

# Deployment gates worth mirroring when filtering training data.
DEPLOY_RANGE_M = 1.8      # velocity_estimator.py forces v=0 beyond this depth
MAX_RANGE_M    = 5.0      # the estimator's detection cutoff

PEDESTRIAN_CLASSES = ("Pedestrian", "Pedestrain_sitting")  # dataset's own typo


# ── Scene loading ─────────────────────────────────────────────────────────────
def load_pose(path):
    """ego_trajectory/<i>.txt -> 4x4 T_world_ego (row-major, comma separated)."""
    vals = [float(v) for v in path.read_text().replace("\n", "").split(",") if v.strip()]
    if len(vals) != 16:
        raise ValueError(f"{path}: expected 16 floats, got {len(vals)}")
    return np.array(vals, dtype=np.float64).reshape(4, 4)


def load_labels(path, classes):
    """label_3d/<i>.txt -> list of (track_id, class, xyz).

    KITTI-style row: class  class:id  h w l  x y z  yaw

    TRAP: despite the KITTI-style layout, x/y/z here are already in the WORLD
    (map) frame, NOT the ego frame. Verified by --check-frames: under the
    identity transform the Pedestrain_sitting class measures 0.004 m/s (static,
    as it must be) and walking measures 0.91 m/s; lifting through the ego pose
    instead injects the robot's own motion and doubles walking to 1.81 m/s.
    """
    out = []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) < 9:
            continue
        if classes and p[0] not in classes:
            continue
        out.append((p[1], p[0], np.array([float(p[5]), float(p[6]), float(p[7])])))
    return out


def load_scene(scene_dir, classes=PEDESTRIAN_CLASSES):
    """Read one unpacked SIC clip. Returns (name, poses, labels) keyed by frame."""
    scene_dir = Path(scene_dir)
    lab_dir, ego_dir = scene_dir / "label_3d", scene_dir / "ego_trajectory"
    if not lab_dir.is_dir():
        raise FileNotFoundError(f"{scene_dir}: no label_3d/ — clip is unlabelled")
    if not ego_dir.is_dir():
        raise FileNotFoundError(f"{scene_dir}: no ego_trajectory/")

    frames = sorted(int(p.stem) for p in lab_dir.glob("*.txt"))
    poses, labels = {}, {}
    for f in frames:
        ego = ego_dir / f"{f}.txt"
        if not ego.is_file():
            continue
        poses[f] = load_pose(ego)
        labels[f] = load_labels(lab_dir / f"{f}.txt", classes)
    return scene_dir.name, poses, labels


def build_world_tracks(poses, labels):
    """tid -> {frame: (world_xyz, class)}.

    Labels are already world-frame (see load_labels), so this only regroups by
    track. Poses are still required per frame, but for projecting INTO the robot
    frame at window end — not for lifting out of it.
    """
    tracks = defaultdict(dict)
    for f, dets in labels.items():
        if f not in poses:
            continue
        for tid, cls, p_world in dets:
            tracks[tid][f] = (p_world, cls)
    return tracks


# ── Target velocity ───────────────────────────────────────────────────────────
def smooth_velocity(fr, xyz, dt=DT, window=9, polyorder=2):
    """World-frame velocity per frame, Savitzky-Golay differentiated.

    Finite-differencing annotated 3D boxes at 10 Hz mostly measures label noise,
    so the target is smoothed before differentiation. Falls back to a central
    difference on runs too short for the SG window.
    """
    n = len(fr)
    if n < 3:
        return np.zeros((n, 3))
    win = min(window, n if n % 2 else n - 1)
    if win >= polyorder + 2:
        try:
            from scipy.signal import savgol_filter
            return savgol_filter(xyz, win, polyorder, deriv=1, delta=dt, axis=0)
        except ImportError:
            pass
    return np.gradient(xyz, dt, axis=0)


# ── Feature builder (mirrors velocity_estimator._build_window_features) ───────
def build_window_features(history_local):
    """Exact re-implementation of the deployed builder. Verified by --verify."""
    hist = list(history_local)
    while len(hist) < WINDOW_SIZE:
        hist.insert(0, hist[0] if hist else (0.0, 0.0, 1.0))
    hist = hist[-WINDOW_SIZE:]

    rx0 = hist[0][2]
    ry0 = -hist[0][0]

    is_stopped = False
    if len(hist) >= 3:
        last_3 = hist[-3:]
        disps = []
        for j in range(1, len(last_3)):
            dx_j = last_3[j][2] - last_3[j - 1][2]
            dy_j = -last_3[j][0] - (-last_3[j - 1][0])
            disps.append(math.hypot(dx_j, dy_j))
        if all(d < STOP_EPS for d in disps):
            is_stopped = True

    features = []
    for i, (cx, cy, cz) in enumerate(hist):
        rx, ry = cz, -cx
        rx_norm, ry_norm = rx - rx0, ry - ry0
        if i == 0:
            dx, dy = 0.0, 0.0
        else:
            dx = rx - hist[i - 1][2]
            dy = ry - (-hist[i - 1][0])
            dx = float(np.clip(dx, -CLIP_D, CLIP_D))
            dy = float(np.clip(dy, -CLIP_D, CLIP_D))
        features.extend([rx_norm, ry_norm, dx, dy])

    return np.array(features, dtype=np.float32).reshape(1, -1), is_stopped


# ── Window extraction ─────────────────────────────────────────────────────────
def contiguous_runs(frames):
    """Split a sorted frame list into maximal runs of consecutive integers."""
    runs, cur = [], [frames[0]] if frames else []
    for f in frames[1:]:
        if f == cur[-1] + 1:
            cur.append(f)
        else:
            runs.append(cur)
            cur = [f]
    if cur:
        runs.append(cur)
    return runs


def scene_windows(name, poses, labels, max_range=MAX_RANGE_M, keep_stopped=True):
    """Extract training windows from one scene.

    Returns (X, y, meta, counters). Each window is projected into the robot pose
    at its LAST frame, matching how the estimator reconstructs history at runtime.
    """
    tracks = build_world_tracks(poses, labels)
    X, y, meta = [], [], []
    counters = defaultdict(int)

    for tid, per_frame in tracks.items():
        frames = sorted(per_frame)
        for run in contiguous_runs(frames):
            if len(run) < WINDOW_SIZE:
                counters["short_run_frames"] += len(run)
                continue

            xyz = np.array([per_frame[f][0] for f in run])
            vel = smooth_velocity(run, xyz)

            for end in range(WINDOW_SIZE - 1, len(run)):
                counters["candidate"] += 1
                sl = slice(end - WINDOW_SIZE + 1, end + 1)

                # Project the whole window into the robot frame at the end frame.
                T = poses[run[end]]
                R_T, t = T[:3, :3].T, T[:3, 3]
                p_rob = (xyz[sl] - t) @ R_T.T          # (T,3) x fwd, y left

                rng = float(np.hypot(p_rob[-1, 0], p_rob[-1, 1]))
                if rng > max_range:
                    counters["out_of_range"] += 1
                    continue

                hist_local = [(-float(p[1]), 0.0, float(p[0])) for p in p_rob]
                feats, is_stopped = build_window_features(hist_local)
                if is_stopped:
                    counters["stop_gated"] += 1
                    if not keep_stopped:
                        continue

                v_rob = R_T @ vel[end]
                X.append(feats[0])
                y.append([v_rob[0], v_rob[1]])
                meta.append((name, tid, per_frame[run[end]][1], run[end], rng))
                counters["kept"] += 1

    Xa = np.array(X, dtype=np.float32) if X else np.zeros((0, FEATURE_DIM), np.float32)
    ya = np.array(y, dtype=np.float32) if y else np.zeros((0, 2), np.float32)
    return Xa, ya, meta, counters


# ── Self-checks ───────────────────────────────────────────────────────────────
def verify_features(n=200, seed=0):
    """Assert our builder matches the deployed one bit for bit."""
    sys.path.insert(0, str(Path(__file__).parent))
    from velocity_estimator import _SRC_DIR  # noqa: F401  (import cost is torch)
    import velocity_estimator as ve

    if ve.WINDOW_SIZE != WINDOW_SIZE:
        raise AssertionError(f"WINDOW_SIZE drift: {ve.WINDOW_SIZE} vs {WINDOW_SIZE}")

    est = ve.VelocityEstimator.__new__(ve.VelocityEstimator)
    rng = np.random.default_rng(seed)
    for _ in range(n):
        T = int(rng.integers(1, WINDOW_SIZE + 4))
        scale = float(rng.choice([0.001, 0.05, 0.4]))   # stopped / walking / jumpy
        hist = [(float(v[0]), 0.0, float(v[1]))
                for v in np.cumsum(rng.normal(0, scale, (T, 2)), axis=0) + [0.0, 2.0]]
        ours, ours_stop = build_window_features(hist)
        theirs, their_stop = est._build_window_features(hist)
        if not np.allclose(ours, theirs, atol=0, rtol=0):
            raise AssertionError(f"feature mismatch, max |d| = {np.abs(ours-theirs).max()}")
        if ours_stop != their_stop:
            raise AssertionError("is_stopped mismatch")
    return n


def check_frames(poses, labels):
    """Decide the label frame empirically: sitting people must be static.

    The Pedestrain_sitting class is ground truth for zero velocity. Whichever
    transform leaves it at ~0 m/s is the correct one; any transform that leaks
    ego motion shows sitting speed ~= the robot's own speed. This is the check
    that caught the world-vs-ego trap, so it is worth re-running on any new
    scene before trusting its output.
    """
    variants = {
        "identity (world, assumed)": lambda T, p: p,
        "lift  R@p + t":             lambda T, p: T[:3, :3] @ p + T[:3, 3],
        "project  R.T@(p - t)":      lambda T, p: T[:3, :3].T @ (p - T[:3, 3]),
    }
    out = {}
    for name, fn in variants.items():
        tracks = defaultdict(dict)
        for f, dets in labels.items():
            if f not in poses:
                continue
            for tid, cls, p in dets:
                tracks[tid][f] = (fn(poses[f], p), cls)
        per_cls = defaultdict(list)
        for per_frame in tracks.values():
            for run in contiguous_runs(sorted(per_frame)):
                if len(run) < 5:
                    continue
                xy = np.array([per_frame[f][0][:2] for f in run])
                v = np.linalg.norm(np.diff(xy, axis=0), axis=1) / DT
                per_cls[per_frame[run[0]][1]].append(np.median(v))
        out[name] = {c: float(np.median(v)) for c, v in per_cls.items()}

    ego = np.array([poses[f][:3, 3][:2] for f in sorted(poses)])
    out["_ego_speed"] = float(np.median(np.linalg.norm(np.diff(ego, axis=0), axis=1) / DT))
    return out


# ── Stats ─────────────────────────────────────────────────────────────────────
def report(name, y, meta, counters, max_range):
    sp = np.hypot(y[:, 0], y[:, 1]) if len(y) else np.zeros(0)
    print(f"\n── {name} ──")
    print(f"  candidate windows       : {counters['candidate']}")
    print(f"  dropped, out of {max_range:.1f} m   : {counters['out_of_range']}")
    print(f"  kept                    : {counters['kept']}"
          f"  ({100*counters['kept']/max(1,counters['candidate']):.1f}%)")
    print(f"  ...of which stop-gated  : {counters['stop_gated']}")
    if len(sp):
        q = lambda p: float(np.quantile(sp, p))
        print(f"  |v| p10/p50/p90/max     : {q(.1):.2f} / {q(.5):.2f} / "
              f"{q(.9):.2f} / {sp.max():.2f} m/s")
        print(f"  frac <0.4 m/s           : {(sp < 0.4).mean():.1%}")
        print(f"  frac >1.83 m/s          : {(sp > 1.83).mean():.1%}")
        sitting = sum(1 for m in meta if m[2] == "Pedestrain_sitting")
        print(f"  sitting-class windows   : {sitting}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", nargs="*", help="unpacked SIC scene directories")
    ap.add_argument("--out", help="write windows to this .npz")
    ap.add_argument("--max-range", type=float, default=MAX_RANGE_M,
                    help=f"drop windows ending beyond this range (default {MAX_RANGE_M}; "
                         f"the deployed estimator zeroes velocity past {DEPLOY_RANGE_M} m)")
    ap.add_argument("--drop-stopped", action="store_true",
                    help="discard windows the is_stopped gate would zero out")
    ap.add_argument("--classes", default=",".join(PEDESTRIAN_CLASSES))
    ap.add_argument("--stats", action="store_true", help="print per-scene stats")
    ap.add_argument("--check-frames", action="store_true",
                    help="empirically confirm the label frame on this scene")
    ap.add_argument("--verify", action="store_true",
                    help="assert feature parity with velocity_estimator, then exit")
    args = ap.parse_args(argv)

    if args.verify:
        print(f"feature parity vs velocity_estimator: OK ({verify_features()} cases)")
        return 0
    if not args.scenes:
        ap.error("no scenes given")

    classes = tuple(c for c in args.classes.split(",") if c)
    Xs, ys, metas, totals = [], [], [], defaultdict(int)

    for scene in args.scenes:
        try:
            name, poses, labels = load_scene(scene, classes)
        except FileNotFoundError as e:
            print(f"skip: {e}", file=sys.stderr)
            continue

        if args.check_frames:
            res = check_frames(poses, labels)
            print(f"\n── {name} frame check (median |v| by class, m/s) ──")
            print(f"  robot's own speed: {res.pop('_ego_speed'):.3f} m/s"
                  f"   (a leaking transform shows sitting ~= this)")
            for k, v in res.items():
                cols = "  ".join(f"{c}={s:.3f}" for c, s in sorted(v.items()))
                print(f"  {k:26s} {cols}")

        X, y, meta, counters = scene_windows(
            name, poses, labels, args.max_range, keep_stopped=not args.drop_stopped)
        Xs.append(X); ys.append(y); metas += meta
        for k, v in counters.items():
            totals[k] += v
        if args.stats:
            report(name, y, meta, counters, args.max_range)

    if not Xs:
        print("no scenes converted", file=sys.stderr)
        return 1

    X = np.concatenate(Xs); y = np.concatenate(ys)
    if args.stats and len(Xs) > 1:
        report("TOTAL", y, metas, totals, args.max_range)

    if args.out:
        np.savez_compressed(
            args.out, X=X, y=y,
            scene=np.array([m[0] for m in metas]),
            track=np.array([m[1] for m in metas]),
            cls=np.array([m[2] for m in metas]),
            frame=np.array([m[3] for m in metas], dtype=np.int32),
            rng=np.array([m[4] for m in metas], dtype=np.float32),
            dt=DT, max_range=args.max_range)
        print(f"\nwrote {args.out}: X{X.shape} y{y.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
