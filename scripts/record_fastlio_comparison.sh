#!/usr/bin/env bash
# Run after fastlio_shadow.sh, in a second Jetson terminal.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
SECONDS_TO_RECORD=${1:-120}
OUT="$ROOT/artifacts/fastlio-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
ros2 bag record -o "$OUT/bag" /livox/lidar /livox/imu /fastlio/odom /odom /odom_raw /imu/data /tf /tf_static /cmd_vel /cmd_vel_safe >"$OUT/bag.log" 2>&1 &
BAG_PID=$!
cleanup() { kill -INT "$BAG_PID" 2>/dev/null || true; wait "$BAG_PID" || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 "$ROOT/src/tools/lidar/fastlio_compare.py" --seconds "$SECONDS_TO_RECORD" --out "$OUT/metrics"
echo "Saved comparison: $OUT"
