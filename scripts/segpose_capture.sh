#!/usr/bin/env bash
# Record one run for the seg-vs-pose C3 comparison. Run ON THE ROBOT.
#   bash scripts/segpose_capture.sh stand-2.4m-r1        # 20 s default
#   bash scripts/segpose_capture.sh cross-2.4m-r1 25
# Run names drive the scoring (see notes/seg_vs_pose_test_plan.md):
#   empty-rN, stand-<d>m-rN, stand-side-<d>m-rN, cross-<d>m-rN, approach-rN, startstop-rN
# Records into artifacts/c3-captures-segpose/<name>. Needs x3_server running
# with --c1-recording (the RGB-D topics). The OAK model does not matter: both
# front ends are run offline on the recorded RGB.
set -euo pipefail
name="${1:?usage: segpose_capture.sh <run-name> [seconds]}"
secs="${2:-20}"
cd "$HOME/x3_ws"
out="artifacts/c3-captures-segpose/$name"
if [ -e "$out" ]; then echo "$out exists; pick the next -rN" >&2; exit 1; fi
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
set +u; source /opt/ros/humble/setup.bash; set -u   # ROS setup reads unset vars
export ROS_DOMAIN_ID=42
# The recorder exits nonzero when its audit finds ANY failure, e.g. one
# incomplete RGB/depth pair; the scorer just skips those. Judge the audit.
python3 src/c1_dataset.py record --output "$out" --seconds "$secs" > "/tmp/segpose-$name.log" 2>&1 || true
if [ ! -f "$out/audit.json" ]; then
    echo "record FAILED (no audit), see /tmp/segpose-$name.log" >&2; exit 1
fi
python3 - "$out" "$name" "$secs" <<'PY'
import json, sys
out, name, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
a = json.load(open(out + '/audit.json'))
good = a.get('complete_pairs', 0)
bad = sum(a.get('failures', {}).values())
print(f"{name}: {good} complete RGB/depth pairs, {bad} bad, "
      f"max gap {a.get('max_pair_interval_ms', 0):.0f} ms -> {out}")
if good < secs * 8 or bad > 0.05 * max(good, 1) or a.get('pair_intervals_over_500ms', 0):
    print("WARNING: too few pairs or gaps over 500 ms -- redo this run", file=sys.stderr)
PY
