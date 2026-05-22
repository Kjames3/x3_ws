#!/usr/bin/env bash
# jetson_bringup.sh
# Start the hardware bringup stack on the Jetson for physical robot operation.
#
# Run this in Terminal 1 on the Jetson before starting server_x3.py.
# The ROS_DOMAIN_ID here MUST match the value passed to the server and to
# any laptop tools (e.g. laptop_sim.sh or RViz).
#
# Usage:
#   bash scripts/jetson_bringup.sh [DOMAIN_ID]
#   bash scripts/jetson_bringup.sh        # uses DOMAIN_ID=42
#   bash scripts/jetson_bringup.sh 0      # ROS2 default domain

set -e

DOMAIN_ID="${1:-42}"
export ROS_DOMAIN_ID="$DOMAIN_ID"

# Resolve Jetson's active IP to avoid loopback (127.0.0.1) locator advertising in FastDDS
JETSON_IP=""
for ip in $(hostname -I); do
    if [[ ! "$ip" =~ ^172\. ]]; then
        JETSON_IP="$ip"
        break
    fi
done
if [ -n "$JETSON_IP" ]; then
    export ROS_DISCOVERY_SERVER="${JETSON_IP}:11811"
else
    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "============================================================"
echo " Yahboom X3 — Hardware Bringup"
echo " Workspace  : $WS_DIR"
echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "============================================================"
echo ""
echo " Starts: Mcnamu_driver_X3, base_node_X3, IMU filter, EKF,"
echo "         YDLidar driver"
echo " Does NOT start SLAM Toolbox — use the GUI 'Start SLAM' button"
echo "============================================================"
echo ""

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

ros2 launch yahboomcar_nav x3_bringup.launch.py
