#!/usr/bin/env python3
"""Score ab_comparison_test.py CSV logs against a tape-measured ground truth.

The A/B logger records `max_obs_speed` at 10 Hz but has no ground truth, so a
run on its own only says "the estimator reported something". Give it the true
walking speed (measured with a tape and a stopwatch) and it becomes a real
accuracy number.

Usage
  # phantom-velocity check: robot parked, nobody moving
  python3 score_ab_logs.py logs/ab_predictive_*.csv --static

  # accuracy check: person walked a measured line at a known speed
  python3 score_ab_logs.py logs/ab_predictive_*.csv --true-speed 1.35

  # compare two models (same protocol, one run each)
  python3 score_ab_logs.py logs/ab_v3.csv logs/ab_c25.csv --true-speed 2.1
"""
import argparse
import csv
import statistics as st
from pathlib import Path


def load(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def summarize(path, true_speed, static, gate):
    rows = load(path)
    if not rows:
        return f"{Path(path).name}: empty"
    spd = [float(r["max_obs_speed"]) for r in rows]
    dist = [float(r["min_obstacle_dist"]) for r in rows]
    nobs = [int(r["n_obstacles"]) for r in rows]
    # Only rows where a pedestrian was actually inside the useful depth window.
    seen = [(s, d) for s, d, n in zip(spd, dist, nobs) if n and d <= gate]

    out = [f"\n{Path(path).name}   {len(rows)} rows @10Hz = {len(rows)/10:.1f}s"]
    out.append(f"  frames with a tracked obstacle <= {gate:.1f} m : {len(seen)}"
               f" ({100*len(seen)/len(rows):.0f}%)")

    if static:
        # Nothing is moving, so every non-zero reading is a phantom.
        out.append(f"  PHANTOM: mean {st.mean(spd):.3f}  p95 {sorted(spd)[int(.95*len(spd))]:.3f}"
                   f"  max {max(spd):.3f} m/s   (all of this should be ~0)")
        out.append(f"  frames reporting > 0.30 m/s : "
                   f"{sum(s > 0.30 for s in spd)} / {len(spd)}")
        return "\n".join(out)

    if not seen:
        out.append("  no in-range detections -- walk closer than the gate or check the estimator")
        return "\n".join(out)

    s = sorted(x[0] for x in seen)
    peak = s[int(0.95 * (len(s) - 1))]      # p95, not max: max is a single-frame spike
    med = st.median(s)
    out.append(f"  reported speed  median {med:.2f}  p95 {peak:.2f}  max {max(s):.2f} m/s")
    out.append(f"  closest approach {min(x[1] for x in seen):.2f} m")
    if true_speed:
        out.append(f"  TRUE {true_speed:.2f} m/s -> p95 error "
                   f"{peak - true_speed:+.2f} m/s ({100*(peak-true_speed)/true_speed:+.0f}%)")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="ab_*.csv logs to score")
    ap.add_argument("--true-speed", type=float, default=None,
                    help="tape-measured walking speed for the run, m/s "
                         "(distance between the marks / stopwatch time)")
    ap.add_argument("--static", action="store_true",
                    help="nothing was moving: score every reading as a phantom")
    ap.add_argument("--gate", type=float, default=1.8,
                    help="only count detections inside this range, m (default 1.8, "
                         "the deploy gate)")
    a = ap.parse_args()
    for p in a.csv:
        print(summarize(p, a.true_speed, a.static, a.gate))
    print()


if __name__ == "__main__":
    main()
