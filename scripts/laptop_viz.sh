#!/usr/bin/env bash
# laptop_viz.sh — Open RViz2 on the laptop to visualise the X3 robot.
#
# Usage:
#   bash scripts/laptop_viz.sh [DOMAIN_ID [JETSON_IP]]
#
# Examples:
#   bash scripts/laptop_viz.sh              # domain 42, multicast discovery
#   bash scripts/laptop_viz.sh 42           # same, explicit domain
#   bash scripts/laptop_viz.sh 42 10.0.0.5  # unicast to Jetson (use if topics missing)
#
# Prerequisites:
#   - ROS2 Humble installed at /opt/ros/humble/
#   - This workspace built (colcon build) so package share paths are available
#   - Jetson running x3_server.service and orbbec_depth.service
#   - Both machines on the same LAN with the same DOMAIN_ID
#
# RViz shows: robot model, YDLidar scan, depth image (2D), depth cloud (3D), SLAM map.

set -e

DOMAIN_ID=${1:-42}
JETSON_IP=${2:-""}

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0        # must be 0 for cross-machine DDS

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

# ── Optional unicast peer discovery ──────────────────────────────────────────
# WiFi routers often block UDP multicast, preventing ROS2 DDS from finding the
# Jetson automatically. Providing the Jetson IP writes a FastDDS XML profile
# that adds the Jetson as an explicit unicast peer, bypassing multicast.
if [ -n "$JETSON_IP" ]; then
    FASTDDS_XML="/tmp/fastdds_x3_unicast.xml"
    cat > "$FASTDDS_XML" << XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default_participant" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>${JETSON_IP}</address>
              <port>7412</port>
            </udpv4>
          </locator>
          <locator>
            <udpv4>
              <address>${JETSON_IP}</address>
              <port>7413</port>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
XML
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML"
    echo "[laptop_viz] Unicast peer: $JETSON_IP (FastDDS XML: $FASTDDS_XML)"
fi

echo "[laptop_viz] ROS_DOMAIN_ID=$DOMAIN_ID  ROS_LOCALHOST_ONLY=0"
echo "[laptop_viz] Workspace: $WS_ROOT"
echo ""

# ── Verify topics are reachable before launching RViz ────────────────────────
echo "[laptop_viz] Waiting for Jetson topics (up to 10s)..."
FOUND=0
for i in $(seq 1 10); do
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        FOUND=1
        break
    fi
    sleep 1
done

if [ "$FOUND" -eq 0 ]; then
    echo ""
    echo "[laptop_viz] WARNING: No topics visible after 10s. Common fixes:"
    echo "  1. Re-run with Jetson IP:  bash scripts/laptop_viz.sh $DOMAIN_ID <jetson-ip>"
    echo "     Find Jetson IP with:    hostname -I   (run on Jetson)"
    echo "  2. Confirm both machines:  echo \$ROS_DOMAIN_ID  (must both be $DOMAIN_ID)"
    echo "  3. On the Jetson check:    sudo systemctl status x3_server orbbec_depth"
    echo "  4. Firewall: sudo ufw allow in proto udp to any port 7400:7500"
    echo ""
    echo "[laptop_viz] Launching RViz anyway — displays will populate when topics appear."
else
    echo "[laptop_viz] Topics found:"
    ros2 topic list 2>/dev/null | grep -E "/scan|/odom|/camera|/map|/robot_description" | sed 's/^/  /'
fi
echo ""

exec ros2 launch yahboomcar_nav x3_remote_viz.launch.py
