#!/bin/bash
# recover_map_from_posegraph.sh — rebuild a map raster from a slam_toolbox pose graph.
#
#   bash scripts/recover_map_from_posegraph.sh <src-name> <dst-name>
#   e.g. bash scripts/recover_map_from_posegraph.sh apartment2 apartment2_recovered
#
# WHY THIS EXISTS
# ---------------
# A mapping run was saved while a second node was also publishing /map, so the
# .pgm captured the wrong source (the tell: 0% unknown cells, and an occupied-cell
# count byte-identical to an older map). The .pgm was garbage.
#
# The .posegraph/.data pair, however, is written by slam_toolbox straight to disk
# and never travels over a topic — so it survives the collision intact. It is the
# real mapping run and can be re-rastered offline, with the robot parked. Two
# laps around an apartment were recovered this way after the fact.
#
# So: if a save looks wrong, DO NOT redrive the map until you have tried this.
#
# TWO NON-OBVIOUS THINGS THIS RELIES ON
# -------------------------------------
# 1. The graph is loaded via the map_file_name PARAMETER at startup, not via the
#    /slam_toolbox/deserialize_pose_graph service. That service is visible in the
#    ROS graph with the correct type but never DDS-matches on this robot —
#    service_is_ready() stays false and requests time out.
# 2. The raster is written by nav2's map_saver_cli with
#    map_subscribe_transient_local:=true. slam_toolbox's own save_map service
#    uses a volatile subscription with a ~2 s window and fails with
#    "Failed to spin map subscription" (result=255).
#
# scan_topic is pointed at a topic that does not exist, so the recovered graph
# cannot be extended by live scans while it is being re-rastered.
set -e

SRC="${1:?usage: recover_map_from_posegraph.sh <src-name> <dst-name>}"
DST="${2:?usage: recover_map_from_posegraph.sh <src-name> <dst-name>}"

WS="${X3_WS:-$HOME/x3_ws}"
MAPS="$WS/src/yahboomcar_nav/maps"
PARAMS="$WS/install/yahboomcar_nav/share/yahboomcar_nav/params/slam_toolbox_params.yaml"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

for ext in .posegraph .data; do
  [ -f "$MAPS/$SRC$ext" ] || { echo "missing $MAPS/$SRC$ext"; exit 1; }
done
if pgrep -f "[m]ap_server" >/dev/null || pgrep -f "[a]sync_slam_toolbox" >/dev/null; then
  echo "REFUSING: map_server or slam_toolbox already running — they would collide on /map."
  exit 1
fi

echo "=== loading $SRC into an isolated slam_toolbox ==="
nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$PARAMS" \
  -p scan_topic:=/__no_scan__ \
  -p map_file_name:="$MAPS/$SRC" \
  -p map_start_at_dock:=true \
  > /tmp/recover_slam.log 2>&1 &
SLAM_PID=$!
# `ros2 run` execs a child; killing only $SLAM_PID leaves async_slam_toolbox_node
# alive and still publishing /map, which is exactly the collision this whole
# script exists to undo. Kill the node by name as well.
cleanup() {
  kill -9 "$SLAM_PID" 2>/dev/null || true
  for p in $(pgrep -f "[a]sync_slam_toolbox_node"); do kill -9 "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

sleep 45
grep -q "Finished serializing Dataset" /tmp/recover_slam.log || {
  echo "pose graph did not load:"; tail -20 /tmp/recover_slam.log; exit 1; }
echo "pose graph loaded."

echo "=== rastering -> $DST ==="
cd "$MAPS"
ros2 run nav2_map_server map_saver_cli -f "$DST" --ros-args \
  -p map_subscribe_transient_local:=true -p save_map_timeout:=20.0 2>&1 | tail -6

echo
echo "=== verify (a real SLAM map has a large UNKNOWN region) ==="
python3 - "$MAPS/$DST.pgm" <<'PY'
import sys
try:
    import cv2
except ImportError:
    print("  (cv2 unavailable — skipping)"); raise SystemExit(0)
img = cv2.imread(sys.argv[1], cv2.IMREAD_UNCHANGED)
if img is None:
    print("  FAIL: cannot read raster"); raise SystemExit(1)
t = img.size
occ, unk = int((img == 0).sum()), int((img == 205).sum())
print(f"  {img.shape[1]}x{img.shape[0]}  occupied {100*occ/t:.2f}%  unknown {100*unk/t:.2f}%")
if unk == 0:
    print("  FAIL: zero unknown cells — you captured the wrong /map publisher again.")
    raise SystemExit(1)
print("  OK: looks like a genuine SLAM map.")
PY
