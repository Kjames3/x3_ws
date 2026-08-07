#!/usr/bin/env python3
"""
perf_report.py — summarise the perf snapshots written by perf_monitor.py.

The server appends one JSON snapshot every ~10 s to src/logs/perf/perf_*.jsonl.
This prints the latest scorecard and the trend across a run, so you can answer
"is the robot performing better or worse than yesterday / than before the model
swap" without opening a notebook.

Usage:
    python3 src/perf_report.py                 # newest log file
    python3 src/perf_report.py --file <path>   # a specific run
    python3 src/perf_report.py --all           # every run, one line each
    python3 src/perf_report.py --watch         # live tail of the newest run
"""

import argparse
import json
import time
from pathlib import Path

LOG_DIR = Path(__file__).parent.resolve() / "logs" / "perf"


def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def newest():
    files = sorted(LOG_DIR.glob("perf_*.jsonl"))
    if not files:
        raise SystemExit(f"no perf logs in {LOG_DIR} — run the server first "
                         f"(logging is on unless X3_PERF_LOG=0)")
    return files[-1]


def _f(v, fmt="{:.3f}", dash="  --  "):
    return dash if v is None else fmt.format(v)


def print_scorecard(snap):
    lat = snap.get("latency", {})
    rates = snap.get("rates_hz", {})
    det = snap.get("detection", {})
    vel = snap.get("velocity_mlp", {})

    print(f"\n=== snapshot @ {snap.get('wall', '?')}  (uptime {snap.get('uptime_s', 0)}s) ===")

    print("\n-- latency (ms) ------------------------------------------------")
    print(f"{'stage':<26}{'n':>7}{'p50':>9}{'p95':>9}{'p99':>9}{'max':>9}")
    for name, s in lat.items():
        if not s.get("n"):
            continue
        print(f"{name:<26}{s['n']:>7}{s['p50']:>9.2f}{s['p95']:>9.2f}"
              f"{s['p99']:>9.2f}{s['max']:>9.2f}")

    if rates:
        print("\n-- rates (Hz) --------------------------------------------------")
        for name, r in rates.items():
            print(f"  {name:<24}{r:>8.2f}")

    print("\n-- detection quality (label-free proxies) ----------------------")
    print(f"  frames in window     {det.get('n_frames', 0)}")
    print(f"  detections/frame     {_f(det.get('det_per_frame'), '{:.2f}')}")
    print(f"  empty frames         {_f(det.get('empty_frame_pct'), '{:.1f}')} %")
    print(f"  confidence mean/p10  {_f(det.get('conf_mean'))} / {_f(det.get('conf_p10'))}")
    print(f"  boxes with depth     {_f(det.get('depth_valid_pct'), '{:.1f}')} %")
    print(f"  flicker              {_f(det.get('flicker_per_100f'), '{:.2f}')} /100 frames"
          f"  ({_f(det.get('flicker_per_min'), '{:.2f}')} /min) "
          f"{det.get('flicker_labels') or ''}")
    lab = det.get("labeled")
    if lab:
        print(f"  labelled eval        P={_f(lab.get('precision'))} R={_f(lab.get('recall'))} "
              f"mAP50={_f(lab.get('map50'))}  ({lab.get('source')} @ {lab.get('at')})")

    print("\n-- velocity MLP accuracy (self-supervised) ---------------------")
    print(f"  scored predictions   {vel.get('n', 0)}  "
          f"(pending {vel.get('pending', 0)}, unlabelable {vel.get('unlabelable', 0)})")
    print(f"  MAE vx / vy          {_f(vel.get('mae_vx'))} / {_f(vel.get('mae_vy'))} m/s")
    print(f"  RMSE (vector)        {_f(vel.get('rmse'))} m/s")
    print(f"  bias vx / vy         {_f(vel.get('bias_vx'))} / {_f(vel.get('bias_vy'))} m/s")
    print(f"  speed MAE / bias     {_f(vel.get('speed_mae'))} / {_f(vel.get('speed_bias'))} m/s")
    print(f"  heading MAE          {_f(vel.get('heading_mae_deg'), '{:.1f}')} deg")
    print(f"  true speed mean      {_f(vel.get('gt_speed_mean'))} m/s "
          f"({_f(vel.get('moving_pct'), '{:.1f}')} % of samples moving)")
    print(f"  R^2                  {_f(vel.get('r2'))}")
    print()


def print_trend(rows):
    print("\n-- trend -------------------------------------------------------")
    print(f"{'time':<21}{'det_hz':>8}{'nn_p95':>9}{'conf':>7}"
          f"{'flk/100f':>10}{'vel_hz':>8}{'cyc_p95':>9}{'rmse':>8}{'R2':>7}")
    for r in rows:
        lat = r.get("latency", {})
        det = r.get("detection", {})
        vel = r.get("velocity_mlp", {})
        rates = r.get("rates_hz", {})
        nn = lat.get("oak.nn_decode_ms", {})
        cyc = lat.get("vel.cycle_ms", {})
        z = lambda v: 0.0 if v is None else v      # snapshots carry nulls
        print(f"{(r.get('wall') or '?')[:19]:<21}"
              f"{z(rates.get('oak.det')):>8.1f}"
              f"{z(nn.get('p95')):>9.1f}"
              f"{z(det.get('conf_mean')):>7.2f}"
              f"{z(det.get('flicker_per_100f')):>10.2f}"
              f"{z(rates.get('vel.cycle')):>8.1f}"
              f"{z(cyc.get('p95')):>9.1f}"
              f"{z(vel.get('rmse')):>8.3f}"
              f"{z(vel.get('r2')):>7.2f}")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", help="specific perf_*.jsonl to read")
    ap.add_argument("--all", action="store_true", help="list every run")
    ap.add_argument("--watch", action="store_true", help="live tail")
    ap.add_argument("--trend", action="store_true", help="per-snapshot trend table")
    args = ap.parse_args()

    if args.all:
        for p in sorted(LOG_DIR.glob("perf_*.jsonl")):
            rows = load(p)
            if not rows:
                continue
            last = rows[-1]
            vel = last.get("velocity_mlp", {})
            det = last.get("detection", {})
            print(f"{p.name:<32} {len(rows):>4} snaps  "
                  f"conf={det.get('conf_mean', 0):.2f}  "
                  f"rmse={_f(vel.get('rmse'))}  n={vel.get('n', 0)}")
        return

    path = Path(args.file) if args.file else newest()
    if args.watch:
        seen = 0
        print(f"watching {path} (ctrl-c to stop)")
        try:
            while True:
                rows = load(path)
                if len(rows) > seen:
                    print_scorecard(rows[-1])
                    seen = len(rows)
                time.sleep(2.0)
        except KeyboardInterrupt:
            return

    rows = load(path)
    if not rows:
        raise SystemExit(f"{path} has no snapshots yet")
    print(f"file: {path}  ({len(rows)} snapshots)")
    print_scorecard(rows[-1])
    if args.trend or len(rows) > 1:
        print_trend(rows)


if __name__ == "__main__":
    main()
