#!/usr/bin/env bash
# laptop_viz.sh — Open RViz2 on the laptop to visualise the X3 robot.
#
# Usage:
#   bash scripts/laptop_viz.sh [DOMAIN_ID]      # default domain 42
#
# Prerequisites:
#   - ROS2 Humble installed at /opt/ros/humble/
#   - This workspace built (colcon build) so package share paths are available
#   - Jetson running: bash scripts/jetson_bringup.sh [DOMAIN_ID]
#   - Both machines on the same LAN with the same DOMAIN_ID
#
# RViz will show: robot model, YDLidar scan, RGB camera image,
#                 depth image (2D), depth cloud (3D), and SLAM map.
# Switch configs with the rvizconfig argument:
#   ros2 launch yahboomcar_nav x3_remote_viz.launch.py \
#       rvizconfig:=$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/rviz/nav.rviz

set -e

DOMAIN_ID=${1:-42}
export ROS_DOMAIN_ID="$DOMAIN_ID"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

echo "[laptop_viz] ROS_DOMAIN_ID=$DOMAIN_ID — opening RViz2 for X3 robot..."
echo "[laptop_viz] Workspace: $WS_ROOT"
echo "[laptop_viz] Tip: ensure jetson_bringup.sh is running on the robot with the same domain ID."
echo ""

exec ros2 launch yahboomcar_nav x3_remote_viz.launch.py "$@"
