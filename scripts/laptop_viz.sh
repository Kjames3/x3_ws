#!/usr/bin/env bash
# laptop_viz.sh — Open RViz2 on the laptop to visualise the X3 robot.
#
# Usage:
#   bash scripts/laptop_viz.sh <JETSON_IP> [DOMAIN_ID]
#
# Example:
#   bash scripts/laptop_viz.sh 10.13.244.35
#   bash scripts/laptop_viz.sh 10.13.244.35 42
#
# Find the Jetson IP with:  hostname -I   (run on Jetson)
#
# Prerequisites:
#   - ROS2 Humble installed at /opt/ros/humble/
#   - This workspace built (colcon build) so package share paths are available
#   - Jetson running all three services: fastdds_discovery, x3_server, orbbec_depth
#
# RViz shows: robot model, YDLidar scan, depth image (2D), depth cloud (3D), SLAM map.

set -e

# Auto-detect argument order — accept IP then domain or domain then IP
_is_ip() { echo "$1" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$'; }

JETSON_IP=""
DOMAIN_ID=42
for arg in "$@"; do
    if _is_ip "$arg"; then
        JETSON_IP="$arg"
    elif [[ "$arg" =~ ^[0-9]+$ ]]; then
        DOMAIN_ID="$arg"
    fi
done

if [ -z "$JETSON_IP" ]; then
    echo "Usage: bash scripts/laptop_viz.sh <JETSON_IP> [DOMAIN_ID]"
    echo "  JETSON_IP  IP of the Jetson — run 'hostname -I' on Jetson to find it"
    echo "  DOMAIN_ID  ROS domain (default: 42)"
    echo ""
    echo "Example: bash scripts/laptop_viz.sh 10.13.244.35"
    echo "Example: bash scripts/laptop_viz.sh 10.13.244.35 42"
    echo "Example: bash scripts/laptop_viz.sh 42 10.13.244.35   # order doesn't matter"
    exit 1
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
# Point this machine at the Jetson's FastDDS discovery server.
# This bypasses multicast and works on school/enterprise WiFi with client isolation.
export ROS_DISCOVERY_SERVER="${JETSON_IP}:11811"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

echo "[laptop_viz] ROS_DOMAIN_ID=$DOMAIN_ID"
echo "[laptop_viz] ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo "[laptop_viz] Workspace: $WS_ROOT"
echo ""

# ── Verify discovery server is reachable ─────────────────────────────────────
echo "[laptop_viz] Waiting for Jetson topics (up to 15s)..."
FOUND=0
for i in $(seq 1 15); do
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        FOUND=1
        break
    fi
    sleep 1
done

if [ "$FOUND" -eq 0 ]; then
    echo ""
    echo "[laptop_viz] WARNING: No topics visible. Check:"
    echo "  On Jetson: sudo systemctl status fastdds_discovery x3_server orbbec_depth"
    echo "  On Jetson: journalctl -u fastdds_discovery -n 20"
    echo "  Port reachable? nc -uvz $JETSON_IP 11811"
    echo "  Firewall:   sudo ufw allow 11811/udp   (run on Jetson)"
    echo ""
    echo "[laptop_viz] Launching RViz anyway..."
else
    echo "[laptop_viz] Topics found:"
    ros2 topic list 2>/dev/null | grep -E "/scan|/odom|/camera|/map|/robot_description|/tf" | sed 's/^/  /'
fi
echo ""

exec ros2 launch yahboomcar_nav x3_remote_viz.launch.py
