#!/usr/bin/env python3
"""Grade a live trajectory-shadow log (server_x3 --traj-shadow DIR).

Every logged prediction is scored against where that track id actually was
h seconds later, interpolated from the same log's observed odom positions.
A prediction is skipped when the track has a gap longer than GAP_S in
between or ends before t + h (no ground truth).

Reported per horizon, for the GRU and cv_est (constant velocity, the CBF's
assumption today): mean / median final displacement error, the GRU's change
vs cv, and, with an NLL model, 2-sigma coverage (ideal 86.5 % for an
isotropic 2-D Gaussian). "moving" keeps windows whose speed at t0 is above
MOVING_MPS: standing-person jitter otherwise dominates the averages (it did
offline, 2026-11-08).

    python3 src/tools/perception/traj_shadow_score.py LOG.ndjson [LOG2 ...] [--json out.json]
"""
import argparse
import json
import math
import sys

import numpy as np

GAP_S = 0.5
MOVING_MPS = 0.3
COVER_2SIG = 1 - math.exp(-2.0)     # P(|e| < 2 sigma), isotropic 2-D Gaussian = 0.8647


def load(paths):
    meta, ticks = None, []
    for p in paths:
        with open(p) as f:
            for line in f:
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if row.get("type") == "meta":
                    meta = meta or row
                elif row.get("type") == "tick":
                    ticks.append(row)
    ticks.sort(key=lambda r: r["t"])
    return meta, ticks


def observed_tracks(ticks):
    obs = {}
    for r in ticks:
        for tr in r["tracks"]:
            obs.setdefault(tr["id"], []).append((r["t"], tr["x"], tr["y"]))
    return {k: np.array(v, float) for k, v in obs.items()}


def truth_at(track, t):
    """Interpolated position at t, or None across a gap / past the end."""
    ts = track[:, 0]
    if t > ts[-1] or t < ts[0]:
        return None
    j = int(np.searchsorted(ts, t))
    if j == 0:
        return track[0, 1:3]
    if ts[j] - ts[j - 1] > GAP_S:
        return None
    a = (t - ts[j - 1]) / max(ts[j] - ts[j - 1], 1e-9)
    return track[j - 1, 1:3] + a * (track[j, 1:3] - track[j - 1, 1:3])


def contiguous(track, t0, t1):
    ts = track[:, 0]
    seg = ts[(ts >= t0) & (ts <= t1)]
    return len(seg) > 0 and (np.diff(np.concatenate([[t0], seg, [t1]])) <= GAP_S).all()


def score(meta, ticks):
    horizons = meta["horizons_s"] if meta else [0.5, 1.0, 2.0, 3.0]
    obs = observed_tracks(ticks)
    acc = {sub: {h: {"gru": [], "cv": [], "cover": []} for h in horizons} for sub in ("all", "moving")}
    n_pred = n_model = 0
    for r in ticks:
        for tr in r["tracks"]:
            n_pred += 1
            if not tr["used_model"]:
                continue                     # cv fallback: identical to cv by construction
            n_model += 1
            moving = math.hypot(tr["vx"], tr["vy"]) > MOVING_MPS
            track = obs[tr["id"]]
            for i, h in enumerate(horizons):
                t1 = r["t"] + h
                if not contiguous(track, r["t"], t1):
                    continue
                gt = truth_at(track, t1)
                if gt is None:
                    continue
                eg = float(np.hypot(*(np.asarray(tr["gru"][i]) - gt)))
                ec = float(np.hypot(*(np.asarray(tr["cv"][i]) - gt)))
                for sub in ("all", "moving") if moving else ("all",):
                    acc[sub][h]["gru"].append(eg)
                    acc[sub][h]["cv"].append(ec)
                    if tr.get("sigma"):
                        acc[sub][h]["cover"].append(eg < 2 * tr["sigma"][i])
    out = {"predictions": n_pred, "used_model": n_model, "horizons_s": horizons, "subsets": {}}
    for sub, by_h in acc.items():
        rows = []
        for h in horizons:
            g, c = np.array(by_h[h]["gru"]), np.array(by_h[h]["cv"])
            if len(g) == 0:
                rows.append({"h": h, "n": 0})
                continue
            cov = by_h[h]["cover"]
            rows.append({"h": h, "n": int(len(g)),
                         "gru_mean": float(g.mean()), "cv_mean": float(c.mean()),
                         "gru_median": float(np.median(g)), "cv_median": float(np.median(c)),
                         "change_pct": float(100 * (g.mean() - c.mean()) / max(c.mean(), 1e-9)),
                         "cover_2sig": float(np.mean(cov)) if cov else None})
        out["subsets"][sub] = rows
    return out


def report(out, file=sys.stdout):
    print(f"predictions {out['predictions']}, from the GRU {out['used_model']} "
          f"(the rest had < 1 s of history and fell back to cv)", file=file)
    for sub, rows in out["subsets"].items():
        print(f"\n{sub} (FDE m, GRU vs cv; 2-sigma coverage ideal {100 * COVER_2SIG:.1f} %)", file=file)
        print(f"{'h s':>5} {'n':>6} {'gru mean':>9} {'cv mean':>8} {'change':>7} {'gru med':>8} {'cv med':>7} {'2sig':>6}",
              file=file)
        for r in rows:
            if not r["n"]:
                print(f"{r['h']:5.1f} {0:6d}  (no ground truth)", file=file)
                continue
            cov = "" if r["cover_2sig"] is None else f"{100 * r['cover_2sig']:5.1f}%"
            print(f"{r['h']:5.1f} {r['n']:6d} {r['gru_mean']:9.3f} {r['cv_mean']:8.3f} {r['change_pct']:+6.1f}% "
                  f"{r['gru_median']:8.3f} {r['cv_median']:7.3f} {cov:>6}", file=file)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json")
    a = ap.parse_args()
    meta, ticks = load(a.logs)
    if not ticks:
        sys.exit("no tick rows in the log(s)")
    out = score(meta, ticks)
    report(out)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
