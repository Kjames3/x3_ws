# Seg vs pose as the C3 person front end

**Question:** which OAK model should feed the person tracker, `yolo26n-pose`
(box + torso depth) or `yolo26n-seg` (silhouette depth)? Pose measured
41–63 % side-on detection; seg's side-on recall is unknown and may behave like
the plain detector's 2–5 %. Seg should give cleaner depth. Pick the one the
tracker does better with; the choice is one `--oak-model` flag.

**Method:** record once, score both offline on identical frames (TRIZ plan:
matched inputs). The 2026-09-29 pose A/B showed offline Ultralytics matches
the OAK blob. Both use conf 0.5, the live `nn_conf`.

## Setup (lab, ~5 min)

- Robot parked, clear floor, lab lighting as usual. Leave the towel on the
  chair: it is the known phantom-person test.
- Tape marks on the floor **measured from the OAK lens** (not the bumper),
  straight ahead: 1.2 m, 2.4 m, 3.5 m. Tape two side marks for the crossings.
- `ssh x3`, check `x3_server` is running with `--c1-recording` (it is by
  default via `90-c1-recording.conf`):
  `systemctl show x3_server -p Environment --value | grep -o c1-recording`

## Runs (`bash scripts/segpose_capture.sh <name> [seconds]`)

Stand with **toes on the mark** for stand runs. Start the command, then walk
into place; the first ~2 s are fine to be moving.

| # | Name | s | What to do |
|---|---|---|---|
| 1 | `empty-r1` | 30 | Nobody in view (stay behind the robot) |
| 2–4 | `stand-1.2m-r1`, `stand-2.4m-r1`, `stand-3.5m-r1` | 20 | Face the robot, arms relaxed |
| 5–7 | `stand-side-1.2m-r1`, `stand-side-2.4m-r1`, `stand-side-3.5m-r1` | 20 | Side-on (shoulder to the robot) |
| 8–10 | `cross-1.2m-r1`, `cross-2.4m-r1`, `cross-3.5m-r1` | 25 | Walk left→right across at that line, turn out of view, walk back; repeat |
| 11 | `approach-r1` | 25 | From 4 m walk straight to 0.8 m, stop 3 s, walk back |
| 12 | `startstop-r1` | 30 | Cross at 2.4 m; stop mid-view 3 s, continue; repeat |

Then repeat 2–12 as `-r2` (skip `empty`). Total ≈ 23 runs, ~25 min of
recording, ~10 GB (the robot has >300 GB free).

Each run prints its frame count; redo any flagged `under 8 fps`.

## Scoring (robot, unattended, ~1–2 h)

```bash
cd ~/x3_ws && source /opt/ros/humble/setup.bash
nohup python3 src/c3_seg_vs_pose.py artifacts/c3-captures-segpose/* \
    --output artifacts/c3-eval-segpose > /tmp/segpose-score.log 2>&1 &
```

Writes `artifacts/c3-eval-segpose/RESULTS.md` (side-by-side) and
`seg_vs_pose.json`. Needs `models/yolo11x.pt` on the robot (the independent
reference).

## Decision rule

Pick **seg** only if all hold; otherwise stay on **pose**:
1. Side-on recall (`stand-side-*`, and `cross/person_*`) within 10 points of pose.
2. Taped-stand jitter no worse, and bias spread across 1.2/2.4/3.5 m smaller
   (both carry a ~0.1–0.15 m torso-behind-toes offset; compare spreads).
3. Crossing speed error (`walk_median_abs_err`) and ratio no worse.
4. No new phantom: `empty/nonperson` and stand-run `ids` not higher.

Caveats: the reference is yolo11x on the same depth sensor, so position vs
reference is consistency, not truth; the taped stands are the truth check.
Close-range (< 1.8 m) upper-body visibility is limited by the low portrait
camera for both models (see the pose A/B, 2026-09-29).
