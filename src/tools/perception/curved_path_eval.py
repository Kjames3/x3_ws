#!/usr/bin/env python3
"""Score the GUI's predicted person paths (src/person_paths.py) on recorded tracks.

Replays lidar_persons.ndjson recordings (Livox tracker 'lidar', C3 camera
tracks 'c3', or both 'fused', loaded the trajectory_baselines way) through
person_paths.PersonPaths at the trackers' own stamps, one instance per mode,
and scores every prediction against where that track actually went.

Reported per mode and horizon (1 s, 2 s): ADE / FDE on MOVING windows (ground
truth mean speed > 0.3 m/s) and on TURNING windows (MOVING and the ground truth
heading turns > 30 deg over the horizon; see the notes for the exact
definition), plus n windows / tracks / runs, per-run spread of the 2 s FDE
change vs cv, and the frame-to-frame wag of the 2 s endpoint.

Only predictions where every mode had a full 1 s history are scored, so all
modes see the same windows. No ROS needed.

  python3 src/tools/perception/curved_path_eval.py ~/x3_ws/artifacts/person-lidar-2026-1*/* \\
      --source lidar --split 2026-11-10 --json out.json
  python3 ... --grid      # tune the CTRV turn gate on the TUNE runs only
"""
import argparse
import itertools
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2]))          # src/
sys.path.insert(0, str(HERE.parent))

import person_paths as pp  # noqa: E402
import trajectory_baselines as tb  # noqa: E402

MODEL = HERE.parents[3] / "config" / "traj_gru_deploy.npz"           # OAK v3 tracks (2026-11-08)
MODEL_PP = HERE.parents[3] / "config" / "traj_gru_person_paths.npz"  # lidar + C3 TUNE runs (2026-11-27)
# eval mode name -> (PersonPaths mode, model)
MODES = {"cv": ("cv", None), "ctrv": ("ctrv", None), "gru": ("gru", MODEL), "gru_pp": ("gru", MODEL_PP)}
STEP_S = 0.1
HORIZONS = (1.0, 2.0)
MOVING_MPS = 0.3
TURN_DEG = 30.0
HEAD_S = 0.3
GAP_S = 0.35


def run_date(p):
    m = re.search(r"(\d{4}-\d{2}-\d{2})", str(p))
    return m.group(1) if m else "0000-00-00"


def track_at(tr, t):
    return np.array([np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])]).T


def contiguous(ts, t0, t1):
    seg = ts[(ts >= t0) & (ts <= t1)]
    return t1 <= ts[-1] and (np.diff(np.concatenate([[t0], seg, [t1]])) <= GAP_S).all()


def heading(a, b):
    d = b - a
    return math.atan2(d[1], d[0]), math.hypot(d[0], d[1])


XT_M = 0.4            # robust turning: the future also ends this far off the t0 heading line


def label(tr, t0, gt, grid, h):
    """(moving, turning, turning_xt) for the prediction at t0 over horizon h."""
    k = int(round(h / STEP_S))
    p0 = gt[0]
    moving = np.hypot(*(gt[k] - p0)) / h > MOVING_MPS
    if not moving or t0 - HEAD_S < tr[0, 0]:
        return moving, False, False
    q = track_at(tr, t0 - HEAD_S)
    a0, s0 = heading(q, p0)
    j = int(round((h - HEAD_S) / STEP_S))
    a1, s1 = heading(gt[j], gt[k])
    ok = s0 / HEAD_S > MOVING_MPS and s1 / HEAD_S > MOVING_MPS
    turning = bool(ok and abs(pp.wrap(a1 - a0)) > math.radians(TURN_DEG))
    e = gt[k] - p0
    xt = abs(math.cos(a0) * e[1] - math.sin(a0) * e[0])
    return moving, turning, turning and xt > XT_M


def collect(paths, source):
    runs = []
    for p in paths:
        try:
            tracks = tb.load_tracks(p, source)
        except Exception as e:  # noqa: BLE001
            print("skip %s: %s" % (p, e), file=sys.stderr)
            continue
        runs.append((Path(p).name, run_date(p), tb.dedupe(tracks)))
    return runs


def replay(runs, modes, ctrv=None, smooth=None, poly=None):
    """Rows of per-prediction records, with every mode's path on the same windows."""
    grid = np.arange(0.0, max(HORIZONS) + 1e-9, STEP_S)
    rows = []
    for ri, (name, date, tracks) in enumerate(runs):
        for tid, tr in tracks.items():
            preds = {m: pp.PersonPaths(MODES[m][0], MODES[m][1], horizon_s=max(HORIZONS),
                                       n_points=len(grid), ctrv=ctrv, smooth_s=smooth, poly_deg=poly)
                     for m in modes if m != "ctrv_base"}
            ref = next(iter(preds.values()))
            for t, x, y, vx, vy in tr:
                for p in preds.values():
                    p.observe("k", t, x, y, vx, vy)
                hist = ref.history("k")
                if hist is None or not contiguous(tr[:, 0], t, t + max(HORIZONS)):
                    continue
                out = {m: p.predict_many(["k"])["k"] for m, p in preds.items()}
                if not all(o["used_hist"] for o in out.values()):
                    continue
                gt = track_at(tr, t + grid)
                gt[0] = (x, y)
                rec = {"run": ri, "track": (ri, tid), "t": t, "gt": gt, "hist": hist,
                       "v0": (vx, vy), "paths": {m: o["path"] for m, o in out.items()},
                       "alts": {m: o["alts"] for m, o in out.items()}}
                if "ctrv_base" in modes:
                    f = tb.predict("ctrv", hist, len(grid) - 1, STEP_S)
                    rec["paths"]["ctrv_base"] = np.vstack([[x, y], f])
                    rec["alts"]["ctrv_base"] = []
                rec["lab"] = {h: label(tr, t, gt, grid, h) for h in HORIZONS}
                rows.append(rec)
    return rows


def errors(rows, mode, h):
    k = int(round(h / STEP_S))
    e = np.array([np.linalg.norm(r["paths"][mode][1:k + 1] - r["gt"][1:k + 1], axis=1) for r in rows])
    return e.mean(1), e[:, -1]


def wag(rows, mode, h=2.0):
    """Median / p90 change of the predicted h-second displacement between consecutive predictions."""
    k = int(round(h / STEP_S))
    d, prev = [], None
    for r in rows:
        disp = r["paths"][mode][k] - r["paths"][mode][0]
        if prev is not None and prev[0] == r["track"] and 0 < r["t"] - prev[1] <= 0.15 \
                and math.hypot(*r["v0"]) > MOVING_MPS:
            d.append(float(np.hypot(*(disp - prev[2]))))
        prev = (r["track"], r["t"], disp)
    d = np.array(d) if d else np.zeros(1)
    return float(np.median(d)), float(np.percentile(d, 90))


def subset(rows, h, kind, runs=None, exclude=None):
    out = []
    for r in rows:
        mv, tn, tx = r["lab"][h]
        if (kind == "moving" and mv) or (kind == "turning" and tn) or (kind == "turning_xt" and tx):
            name = runs[r["run"]][0] if runs else ""
            if exclude and re.search(exclude, name):
                continue
            out.append(r)
    return out


def summarize(rows, runs, modes, exclude_circle=False):
    res = {}
    for h in HORIZONS:
        for kind in ("moving", "turning", "turning_xt"):
            sub = subset(rows, h, kind, runs, "circle" if exclude_circle else None)
            key = "%s@%.0fs" % (kind, h)
            if not sub:
                res[key] = {"n": 0}
                continue
            d = {"n": len(sub), "tracks": len({r["track"] for r in sub}), "runs": len({r["run"] for r in sub})}
            cv_fde = errors(sub, "cv", h)[1]
            for m in modes:
                ade, fde = errors(sub, m, h)
                d[m] = {"ade": float(ade.mean()), "fde": float(fde.mean()), "fde_med": float(np.median(fde)),
                        "fde_vs_cv_pct": float(100 * (fde.mean() - cv_fde.mean()) / max(cv_fde.mean(), 1e-9))}
                # per-run spread of the change vs cv (runs with >= 10 windows)
                per = []
                for ri in sorted({r["run"] for r in sub}):
                    rs = [r for r in sub if r["run"] == ri]
                    if len(rs) >= 10:
                        a, c = errors(rs, m, h)[1].mean(), errors(rs, "cv", h)[1].mean()
                        per.append(100 * (a - c) / max(c, 1e-9))
                if per:
                    d[m]["per_run_pct"] = [round(float(np.percentile(per, q)), 1) for q in (25, 50, 75)]
                    d[m]["per_run_n"] = len(per)
                if h == 2.0 and kind != "moving":
                    alts = [r for r in sub if r["alts"][m]]
                    if alts:
                        k = int(round(h / STEP_S))
                        best = [min([np.hypot(*(r["paths"][m][k] - r["gt"][k]))]
                                    + [np.hypot(*(a[k] - r["gt"][k])) for a in r["alts"][m]]) for r in alts]
                        d[m]["fan_shown_pct"] = 100.0 * len(alts) / len(sub)
                        d[m]["fan_best_fde_where_shown"] = float(np.mean(best))
                        d[m]["main_fde_where_shown"] = float(errors(alts, m, h)[1].mean())
            res[key] = d
    res["wag_2s"] = {m: wag(rows, m) for m in modes}
    return res


def print_summary(title, res, modes, file=sys.stdout):
    print("\n== %s" % title, file=file)
    for key, d in res.items():
        if key == "wag_2s":
            print("  wag of the 2 s displacement, median / p90 (m): " +
                  "  ".join("%s %.3f/%.3f" % (m, *d[m]) for m in modes), file=file)
            continue
        if not d.get("n"):
            print("  %-11s n=0" % key, file=file)
            continue
        print("  %-11s n=%d tracks=%d runs=%d" % (key, d["n"], d["tracks"], d["runs"]), file=file)
        for m in modes:
            r = d[m]
            extra = ""
            if "per_run_pct" in r:
                extra += "  per-run %+.1f/%+.1f/%+.1f%% (n=%d)" % (*r["per_run_pct"], r["per_run_n"])
            if "fan_shown_pct" in r:
                extra += "  fan shown %.0f%%: best-of-fan FDE %.2f vs main %.2f" % (
                    r["fan_shown_pct"], r["fan_best_fde_where_shown"], r["main_fde_where_shown"])
            print("    %-10s ADE %.3f  FDE %.3f (med %.3f)  %+6.1f%% vs cv%s" % (
                m, r["ade"], r["fde"], r["fde_med"], r["fde_vs_cv_pct"], extra), file=file)


def grid_search(rows):
    """CTRV gate/damping grid on cached histories (the runtime functions, no replay)."""
    grid = np.arange(0.0, max(HORIZONS) + 1e-9, STEP_S)
    mov = subset(rows, 2.0, "moving")
    trn = subset(rows, 2.0, "turning")
    cv_m, cv_t = errors(mov, "cv", 2.0)[1].mean(), errors(trn, "cv", 2.0)[1].mean()
    space = dict(fit_s=[0.5, 0.8, 1.0], gate_t=[1.0, 2.0, 3.0], min_w=[0.15, 0.3],
                 tau_s=[0.7, 1.0, 1.5, 3.0], shrink=[0.5, 0.8, 1.0], max_w=[1.2])
    out = []
    for vals in itertools.product(*space.values()):
        p = dict(pp.CTRV, **dict(zip(space, vals)))

        def fde(sub):
            e = []
            for r in sub:
                w, se = pp.turn_rate(r["hist"], STEP_S, p)
                path = pp.ctrv_path(r["gt"][0], np.array(r["v0"]), pp.gated_rate(w, se, p), grid, p["tau_s"])
                e.append(np.hypot(*(path[-1] - r["gt"][-1])))
            return float(np.mean(e))
        fm, ft = fde(mov), fde(trn)
        out.append((dict(zip(space, vals)), 100 * (fm - cv_m) / cv_m, 100 * (ft - cv_t) / cv_t))
    out.sort(key=lambda o: o[2])
    return out, len(mov), len(trn)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--source", choices=("lidar", "c3", "fused"), default="lidar")
    ap.add_argument("--split", default="2026-11-10", help="runs dated before this are TUNE, the rest REPORT")
    ap.add_argument("--modes", default="cv,ctrv_base,ctrv,gru,gru_pp",
                    help="gru = config/traj_gru_deploy.npz (OAK), gru_pp = config/traj_gru_person_paths.npz")
    ap.add_argument("--ctrv", default=None, help="JSON overrides for person_paths.CTRV")
    ap.add_argument("--grid", action="store_true", help="grid-search the CTRV gate on the TUNE runs")
    ap.add_argument("--smooth", type=float, default=None,
                    help="path-shape smoothing time constant for every mode (default: person_paths.SMOOTH_S)")
    ap.add_argument("--poly", type=int, default=None, help="GRU path polynomial degree (default person_paths.POLY_DEG)")
    ap.add_argument("--only", choices=("TUNE", "REPORT", "ALL"), help="score one split only")
    ap.add_argument("--json")
    a = ap.parse_args()
    modes = a.modes.split(",")
    ctrv = json.loads(a.ctrv) if a.ctrv else None
    paths = sorted({p for p in a.runs if (Path(p) / "lidar_persons.ndjson").exists()})
    runs = collect(paths, a.source)
    tune = [r for r in runs if r[1] < a.split]
    report = [r for r in runs if r[1] >= a.split]
    print("source %s: %d TUNE runs (< %s), %d REPORT runs" % (a.source, len(tune), a.split, len(report)))
    if a.grid:
        rows = replay(tune, ["cv"], ctrv)
        out, nm, nt = grid_search(rows)
        print("grid on TUNE: %d moving, %d turning windows @2 s. FDE change vs cv (moving, turning):" % (nm, nt))
        for cfg, dm, dt in out[:25]:
            print("  moving %+5.1f%%  turning %+6.1f%%  %s" % (dm, dt, cfg))
        ok = [o for o in out if o[1] <= 3.0]
        if ok:
            print("best with moving <= +3%%: moving %+.1f%% turning %+.1f%% %s" % (ok[0][1], ok[0][2], ok[0][0]))
        return
    allres = {}
    for tag, rs in (("TUNE", tune), ("REPORT", report), ("ALL", tune + report)):
        if not rs or (a.only and tag != a.only):
            continue
        rows = replay(rs, modes, ctrv, a.smooth, a.poly)
        names = rs
        res = summarize(rows, names, modes)
        print_summary("%s (%d runs, %d scored predictions)" % (tag, len(rs), len(rows)), res, modes)
        res_nc = summarize(rows, names, modes, exclude_circle=True)
        print_summary("%s without circle runs" % tag, res_nc, modes)
        allres[tag] = {"all": res, "no_circle": res_nc, "runs": [r[0] for r in rs]}
    if a.json:
        Path(a.json).write_text(json.dumps(dict(source=a.source, split=a.split, ctrv=dict(pp.CTRV, **(ctrv or {})),
                                                results=allres), indent=1))


if __name__ == "__main__":
    main()
