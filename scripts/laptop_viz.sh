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
EXTRA_ARGS=()
VIEW=""
for arg in "$@"; do
    if _is_ip "$arg"; then
        JETSON_IP="$arg"
    elif [[ "$arg" =~ ^[0-9]+$ ]]; then
        DOMAIN_ID="$arg"
    elif [[ "$arg" =~ ^(3d|full|map|nav)$ ]]; then
        VIEW="$arg"
    elif [[ "$arg" != *":="* ]] && getent hosts "$arg" >/dev/null 2>&1; then
        # Tailscale gives the robot the stable name `x3`, so accept a hostname
        # and resolve it -- the initial-peers list below needs a literal IP.
        JETSON_IP="$(getent hosts "$arg" | awk '{print $1; exit}')"
        echo "[laptop_viz] resolved $arg -> $JETSON_IP"
    else
        EXTRA_ARGS+=("$arg")
    fi
done

# Short view names beat making the user paste an absolute path into
# rvizconfig:=.  Resolved against the *installed* share dir below, once the
# workspace has been sourced.
case "$VIEW" in
    3d)   RVIZ_FILE="x3_3d_map.rviz" ;;
    full) RVIZ_FILE="x3_full_viz.rviz" ;;
    map)  RVIZ_FILE="map.rviz" ;;
    nav)  RVIZ_FILE="nav.rviz" ;;
    *)    RVIZ_FILE="" ;;
esac

if [ -z "$JETSON_IP" ]; then
    echo "Usage: bash scripts/laptop_viz.sh <JETSON_IP> [DOMAIN_ID] [extra_launch_arguments]"
    echo "  JETSON_IP  IP of the Jetson — run 'hostname -I' on Jetson to find it"
    echo "  DOMAIN_ID  ROS domain (default: 42)"
    echo ""
    echo "Example: bash scripts/laptop_viz.sh 10.13.244.35"
    echo "Example: bash scripts/laptop_viz.sh 10.13.244.35 42"
    echo "Example: bash scripts/laptop_viz.sh 42 10.13.244.35   # order doesn't matter"
    echo "Example: bash scripts/laptop_viz.sh 10.13.244.35 rvizconfig:=nav.rviz"
    echo ""
    echo "  VIEW       one of: 3d | full | map | nav   (default: full)"
    echo "    3d   -> OctoMap 3D map + live tilted cloud + SLAM map"
    echo "    full -> robot + lidar + camera + depth cloud + map"
    echo "    map  -> /scan + /map only (lightweight)"
    echo "    nav  -> Nav2 costmaps, path, goal tool"
    echo ""
    echo "Example: bash scripts/laptop_viz.sh x3 3d     # watch the 3D map build"
    exit 1
fi

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
# Default ROS2 Simple discovery — NOT the FastDDS discovery server.
# The discovery server's EDP forwarding between clients is broken in this setup
# (topics become visible but publishers never learn the subscriber exists, so
# data never flows).  Instead we use plain Simple discovery driven by explicit
# UNICAST initial peers below.  Multicast is blocked across the school subnets,
# but unicast UDP between the laptop and Jetson works — verified.
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# ── Generate a FastDDS profile with unicast initial peers ────────────────────
# Multicast can't reach the Jetson across subnets, so we tell our participant to
# announce directly (unicast) to each Jetson ROS node's metatraffic port.
# We also include a TCP transport for large messages like maps (to bypass UDP
# fragmentation drops on enterprise WiFi), matching fastdds_tcp_client.xml.
PROFILE="/tmp/fastdds_unicast_${JETSON_IP}_d${DOMAIN_ID}.xml"
{
    echo '<?xml version="1.0" encoding="utf-8" ?>'
    echo '<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
    echo '  <profiles>'
    echo '    <transport_descriptors>'
    echo '      <transport_descriptor>'
    echo '        <transport_id>udp_transport</transport_id>'
    echo '        <type>UDPv4</type>'
    echo '        <receiveBufferSize>10485760</receiveBufferSize>'
    echo '        <sendBufferSize>10485760</sendBufferSize>'
    echo '        <maxMessageSize>65500</maxMessageSize>'
    echo '      </transport_descriptor>'
    echo '      <transport_descriptor>'
    echo '        <transport_id>tcp_transport</transport_id>'
    echo '        <type>TCPv4</type>'
    echo '        <calculate_crc>false</calculate_crc>'
    echo '        <listening_ports><port>11813</port></listening_ports>'
    echo '        <receiveBufferSize>10485760</receiveBufferSize>'
    echo '        <sendBufferSize>10485760</sendBufferSize>'
    echo '        <maxMessageSize>65500</maxMessageSize>'
    echo '      </transport_descriptor>'
    echo '    </transport_descriptors>'
    echo '    <participant profile_name="unicast_peer" is_default_profile="true">'
    echo '      <rtps>'
    echo '        <userTransports>'
    echo '          <transport_id>udp_transport</transport_id>'
    echo '          <transport_id>tcp_transport</transport_id>'
    echo '        </userTransports>'
    echo '        <useBuiltinTransports>false</useBuiltinTransports>'
    echo '        <builtin><initialPeersList>'
    for pid in $(seq 0 31); do
        port=$(( 7400 + 250 * DOMAIN_ID + 10 + 2 * pid ))
        echo "          <locator><udpv4><address>${JETSON_IP}</address><port>${port}</port></udpv4></locator>"
    done
    echo '        </initialPeersList></builtin>'
    echo '      </rtps>'
    echo '    </participant>'
    echo '  </profiles>'
    echo '</dds>'
} > "$PROFILE"
# Set BOTH env var names: ROS2 Humble ships Fast-DDS 2.6, which honors the legacy
# FASTRTPS_DEFAULT_PROFILES_FILE.  The newer FASTDDS_* name is silently IGNORED on
# this version — if only that is set, the profile never loads, the unicast initial
# peers never take effect, discovery falls back to (blocked) multicast, and no
# topics appear.  Setting both is harmless and forward-compatible.
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
export FASTDDS_DEFAULT_PROFILES_FILE="$PROFILE"

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

echo "[laptop_viz] ROS_DOMAIN_ID=$DOMAIN_ID"
echo "[laptop_viz] Discovery: Simple + unicast initial peers -> $JETSON_IP (ids 0-31)"
echo "[laptop_viz] Workspace: $WS_ROOT"
if [ -n "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "[laptop_viz] FastDDS profile: $FASTDDS_DEFAULT_PROFILES_FILE"
fi
if [ ${#EXTRA_ARGS[@]} -ne 0 ]; then
    echo "[laptop_viz] Extra launch args: ${EXTRA_ARGS[*]}"
fi
echo ""

# Restart the ROS2 daemon so it picks up the unicast-peer profile and domain id
# from the env exported above.  A stale daemon started without these won't see
# any Jetson topics.
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null

# ── Verify the Jetson is reachable before doing anything ─────────────────────
# Discovery now rides on unicast UDP, which can't be probed as easily as a TCP
# port, so we just confirm there is an IP path (ICMP).  If the robot is down or
# the IP is wrong, fail fast with an actionable message instead of an empty RViz.
# echo "[laptop_viz] Checking the Jetson is reachable at ${JETSON_IP} ..."
# if ! ping -c 1 -W 3 "$JETSON_IP" >/dev/null 2>&1; then
#     echo ""
#     echo "[laptop_viz] WARNING: cannot ping Jetson at ${JETSON_IP} (ICMP might be blocked)."
#     echo "  Continuing anyway..."
# fi
echo "[laptop_viz] Assuming Jetson is reachable (skipping ping test)."
echo "[laptop_viz] Jetson reachable."

# ── Verify topics actually arrive ────────────────────────────────────────────
# With Simple discovery the discovery path and the data path are the same UDP
# transport, so unlike the old discovery-server setup, visible topics here mean
# data will actually flow.  Poll until /scan appears or 30s elapses.
echo "[laptop_viz] Waiting for Jetson topics (up to 30s)..."
FOUND=0
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -q "/scan"; then
        FOUND=1
        break
    fi
    sleep 1
done

if [ "$FOUND" -eq 0 ]; then
    echo ""
    echo "[laptop_viz] WARNING: No topics visible. Check:"
    echo "  On Jetson: sudo systemctl status x3_server orbbec_depth"
    echo "  Jetson must use Simple discovery (NO ROS_DISCOVERY_SERVER) for this to work."
    echo "  Reachable?  ping $JETSON_IP"
    echo "  UDP open?   sudo ufw status   (run on Jetson; allow if active)"
    echo ""
    echo "[laptop_viz] Launching RViz anyway..."
else
    echo "[laptop_viz] Topics found:"
    ros2 topic list 2>/dev/null | grep -E "/scan|/odom|/camera|/map|/robot_description|/tf" | sed 's/^/  /'
fi
echo ""

echo "========================================================================="
echo " TIME SYNCHRONIZATION TIP:"
echo " ROS2 TF transforms require precise clock synchronization between machines."
echo " If the robot model is missing/red or does not move in accordance to"
echo " reality in RViz, please synchronize your laptop's clock with the Jetson:"
echo "   sudo date -s \"\$(ssh jetson@$JETSON_IP date -R)\""
echo "========================================================================="
echo ""

# Resolve the short view name now that install/setup.bash has been sourced.
# Skip if the caller already passed an explicit rvizconfig:=.
if [ -n "$RVIZ_FILE" ] && [[ ! "${EXTRA_ARGS[*]}" =~ rvizconfig: ]]; then
    RVIZ_SHARE="$(ros2 pkg prefix yahboomcar_nav 2>/dev/null)/share/yahboomcar_nav/rviz"
    if [ -f "$RVIZ_SHARE/$RVIZ_FILE" ]; then
        EXTRA_ARGS+=("rvizconfig:=$RVIZ_SHARE/$RVIZ_FILE")
        echo "[laptop_viz] view '$VIEW' -> $RVIZ_SHARE/$RVIZ_FILE"
    else
        echo "[laptop_viz] WARNING: $RVIZ_FILE is not in the install tree."
        echo "  Rebuild it:  colcon build --packages-select yahboomcar_nav"
    fi
fi

# 3D mapping accumulates the octree in `odom` and needs NO SLAM, so the octomap
# topics carry frame_id 'odom'.  Every view except `3d` uses a `map` fixed frame,
# and transforming odom -> map needs a map->odom that only exists while AMCL or
# slam_toolbox is localized.  Pick the wrong view and RViz cannot transform a
# single octomap message: they pile up in the tf2 MessageFilter and come out as
#     "Message Filter dropping message: frame 'odom' ... queue is full"
# with the map lagging seconds behind.  Catch that here rather than let it look
# like a network fault.
if [ "$VIEW" != "3d" ] && ros2 topic list 2>/dev/null | grep -qx "/octomap_binary"; then
    echo ""
    echo "[laptop_viz] ================================ WARNING ================================"
    echo "[laptop_viz] 3D mapping is RUNNING on the robot, but view '${VIEW:-full}' uses a"
    echo "[laptop_viz] 'map' fixed frame while the octree is published in 'odom'."
    echo "[laptop_viz] Unless SLAM/AMCL is localized there is no map->odom, so RViz will drop"
    echo "[laptop_viz] every octomap message with \"queue is full\" and the map will lag badly."
    echo "[laptop_viz]"
    echo "[laptop_viz]   Use:  bash scripts/laptop_viz.sh $1 3d"
    echo "[laptop_viz] =========================================================================="
    echo ""
fi

if [ "$VIEW" = "3d" ]; then
    echo ""
    echo "[laptop_viz] 3D map view. On the Jetson you also need:"
    echo "    ros2 launch yahboomcar_nav x3_octomap.launch.py"
    echo "  and the octomap topics require ros-humble-octomap-server installed there."
    for t in /pointcloud_raw /octomap_binary; do
        if ros2 topic list 2>/dev/null | grep -qx "$t"; then
            echo "    [ok]      $t"
        else
            echo "    [MISSING] $t"
        fi
    done
    echo ""
fi

exec ros2 launch yahboomcar_nav x3_remote_viz.launch.py "${EXTRA_ARGS[@]}"
