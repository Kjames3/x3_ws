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
python3 src/c1_dataset.py record --output "$out" --seconds "$secs" > "/tmp/segpose-$name.log" 2>&1 \
    || { echo "record FAILED, see /tmp/segpose-$name.log" >&2; exit 1; }
n=$(python3 -c "import json;print(json.load(open('$out/audit.json'))['topic_counts'].get('/oak/rgbd/rgb/image_raw',0))")
echo "$name: $n RGB frames in ${secs}s -> $out"
[ "$n" -ge $((secs * 8)) ] || echo "WARNING: under 8 fps -- redo this run" >&2
