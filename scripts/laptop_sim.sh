#!/usr/bin/env bash
# laptop_sim.sh
# Start Gazebo simulation on the laptop for cross-machine development.
#
# The Jetson runs server_x3.py --ros2 --domain-id <N>.
# This script exports the SAME domain ID so ROS2 DDS discovery connects
# the Gazebo topics on the laptop to the ROS2Bridge on the Jetson.
#
# Requirements (laptop):
#   - ROS2 Humble installed
#   - x3_ws built:  bash scripts/build_ros2.sh
#   - Ignition Fortress installed:
#       sudo apt install ros-humble-ros-gz
#
# Usage:
#   bash scripts/laptop_sim.sh [DOMAIN_ID]
#   bash scripts/laptop_sim.sh        # uses DOMAIN_ID=42
#   bash scripts/laptop_sim.sh 0      # ROS2 default domain

set -e

DOMAIN_ID="${1:-42}"
export ROS_DOMAIN_ID="$DOMAIN_ID"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "============================================================"
echo " Yahboom X3 — Gazebo Simulation (laptop)"
echo " Workspace     : $WS_DIR"
echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "============================================================"
echo ""
echo " Start the Jetson server in a separate terminal:"
echo "   python3 src/server_x3.py --ros2 --domain-id $DOMAIN_ID"
echo ""
echo " Then open the browser and connect to the Jetson IP."
echo "============================================================"
echo ""

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

ros2 launch yahboomcar_nav x3_gazebo.launch.py
