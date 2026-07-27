#!/usr/bin/env bash
# =============================================================================
# map_classroom.sh — Interactive SLAM & Costmap Mapping Utility
# =============================================================================
# Run this script ON YOUR LAPTOP to map the classroom using SLAM + Nav2,
# with real-time global/local costmap visualization in RViz2.
#
# Usage (on laptop):
#   bash scripts/map_classroom.sh
# =============================================================================

set -eo pipefail

# ── Color Definitions ────────────────────────────────────────────────────────
GREEN="\033[92m"
YELLOW="\033[93m"
RED="\033[91m"
BLUE="\033[94m"
CYAN="\033[96m"
BOLD="\033[1m"
RESET="\033[0m"

# ── Script Location ──────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
CACHE_FILE="$HOME/.x3_jetson_ip"

echo -e "${CYAN}=====================================================================${RESET}"
echo -e "${BOLD}${CYAN}          YAHBOOM X3 MECANUM CAR — CLASSROOM MAPPING UTILITY          ${RESET}"
echo -e "${CYAN}=====================================================================${RESET}"

# ── 1. Target IP Resolution & Caching ────────────────────────────────────────
DEFAULT_IP=""
if [ -f "$CACHE_FILE" ]; then
    DEFAULT_IP=$(cat "$CACHE_FILE" | xargs)
fi

echo -e "${BLUE}[DDS Setup] Resolving Jetson target IP...${RESET}"
if [ -n "$DEFAULT_IP" ]; then
    read -p "Enter Jetson IP Address [default: $DEFAULT_IP]: " JETSON_IP
    JETSON_IP="${JETSON_IP:-$DEFAULT_IP}"
else
    while [ -z "${JETSON_IP:-}" ]; do
        read -p "Enter Jetson IP Address: " JETSON_IP
    done
fi

# Cache the IP address for future runs
echo "$JETSON_IP" > "$CACHE_FILE"
echo -e "${GREEN}✓ Jetson target IP set to: $JETSON_IP${RESET}\n"

# ── 2. Network Diagnostics ────────────────────────────────────────────────────
echo -e "${BLUE}[DDS Setup] Testing network connectivity to $JETSON_IP...${RESET}"
if ! ping -c 2 -W 2 "$JETSON_IP" >/dev/null 2>&1; then
    echo -e "${RED}✗ Error: Jetson IP $JETSON_IP is unreachable via ping.${RESET}"
    echo -e "  - Verify both devices are on the same local network."
    echo -e "  - Double-check the IP using 'hostname -I' on the Jetson."
    exit 1
fi
echo -e "${GREEN}✓ Network ping to Jetson is successful!${RESET}"

# Check WebSocket server port 8081
echo -e "${BLUE}[DDS Setup] Checking if WebSocket service is active on Jetson (port 8081)...${RESET}"
if ! timeout 2 bash -c "</dev/tcp/$JETSON_IP/8081" >/dev/null 2>&1; then
    echo -e "${RED}✗ Error: WebSocket server (port 8081) is not running on the Jetson.${RESET}"
    echo -e "  Make sure that the control server is running on the Jetson:"
    echo -e "  Run: ${YELLOW}sudo systemctl start x3_server${RESET} on the Jetson."
    exit 1
fi
echo -e "${GREEN}✓ WebSocket service is active!${RESET}\n"

# ── 3. Local ROS2 Environment Configuration ──────────────────────────────────
DOMAIN_ID=42
export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER="TCPv4:[${JETSON_IP}]:11811"
export ROS_SUPER_CLIENT=TRUE
export FASTDDS_DEFAULT_PROFILES_FILE="$WS_ROOT/config/fastdds_tcp_client.xml"

# Sourcing ROS2 Humble & the Workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}✗ Error: ROS2 Humble setup not found at /opt/ros/humble/setup.bash${RESET}"
    exit 1
fi

if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
else
    echo -e "${YELLOW}[WARN] Workspace install directory not found. Building first...${RESET}"
    colcon build --packages-select yahboomcar_nav yahboomcar_description yahboomcar_msgs
    source "$WS_ROOT/install/setup.bash"
fi

# Stop and restart the local daemon to inherit discovery parameters immediately
echo -e "${BLUE}[DDS Setup] Restarting local ROS2 daemon to apply DDS TCP parameters...${RESET}"
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1
sleep 1
echo -e "${GREEN}✓ Local ROS2 daemon restarted successfully.${RESET}\n"

# ── 4. Remote Mapping Orchestration (SLAM + Nav2) ─────────────────────────────
echo -e "${BLUE}[DDS Setup] Orchestrating live Mapping Stack on Jetson...${RESET}"
if ! python3 "$SCRIPT_DIR/control_mapping.py" "$JETSON_IP" start; then
    echo -e "${RED}✗ Error: Failed to trigger SLAM mapping on the Jetson.${RESET}"
    exit 1
fi
echo ""

# ── 5. Pre-flight Topic Health Check ──────────────────────────────────────────
echo -e "${BLUE}[DDS Setup] Waiting for Jetson mapping topics to propagate (up to 30s)...${RESET}"
FOUND_TOPICS=0
for _ in $(seq 1 30); do
    TOPIC_LIST=$(ros2 topic list 2>/dev/null || echo "")
    if echo "$TOPIC_LIST" | grep -q "/scan" && echo "$TOPIC_LIST" | grep -q "/map"; then
        FOUND_TOPICS=1
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

if [ "$FOUND_TOPICS" -eq 0 ]; then
    echo -e "${RED}✗ Timeout warning: ROS2 topics /scan and /map are not yet visible on laptop.${RESET}"
    echo -e "  Check that fastdds_discovery is active on the Jetson:"
    echo -e "    ${YELLOW}sudo systemctl status fastdds_discovery${RESET}"
    echo -e "  Launching RViz anyway..."
else
    echo -e "${GREEN}✓ Success! Jetson topics detected on laptop:${RESET}"
    echo -e "  - /scan"
    echo -e "  - /map"
    if echo "$TOPIC_LIST" | grep -q "/odom"; then
        echo -e "  - /odom (EKF odometry — SLAM tracking active)"
    else
        echo -e "${YELLOW}  ⚠ Warning: /odom not visible — SLAM will use scan-matching only (slower).${RESET}"
        echo -e "  If the map builds slowly or looks fragmented, check the Jetson:"
        echo -e "    ${YELLOW}sudo systemctl restart x3_server${RESET}  then re-run this script."
    fi
fi
echo ""

# ── 6. RViz2 Costmap & Planning Interface ────────────────────────────────────
echo -e "${GREEN}=========================================================================${RESET}"
echo -e "${BOLD}${GREEN} LAUNCHING RVIZ INTERFACE ON YOUR LAPTOP...${RESET}"
echo -e " Use the '2D Goal Pose' button at the top of RViz to navigate the robot."
echo -e " Watch the costmaps (colored gradients) to see traversable space!"
echo -e "${GREEN}=========================================================================${RESET}"
echo ""

# Locate the Nav2-specific RViz configuration
NAV_RVIZ_CFG="$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/rviz/nav.rviz"

# Launch RViz2 in the background so our terminal menu remains active
ros2 launch yahboomcar_nav x3_remote_viz.launch.py rvizconfig:="$NAV_RVIZ_CFG" >/dev/null 2>&1 &
RVIZ_PID=$!

# Ensure RViz is killed on script interruption
cleanup_local() {
    echo -e "\n${YELLOW}[Shutdown] Cleaning up local processes...${RESET}"
    if kill -0 "$RVIZ_PID" 2>/dev/null; then
        kill "$RVIZ_PID"
    fi
}
trap cleanup_local INT TERM EXIT

# ── 7. Interactive Control Loop ───────────────────────────────────────────────
sleep 2
while kill -0 "$RVIZ_PID" 2>/dev/null; do
    echo -e "${CYAN}------------------------------------------------------------${RESET}"
    echo -e "${BOLD}${CYAN}               MAPPING INTERACTIVE CONSOLE${RESET}"
    echo -e "${CYAN}------------------------------------------------------------${RESET}"
    echo -e "  ${BOLD}[1] Save Map${RESET}            - Save the generated SLAM map on Jetson"
    echo -e "  ${BOLD}[2] Clear Costmaps${RESET}      - Reset obstacles/noise in costmaps"
    echo -e "  ${BOLD}[3] Synchronize Time${RESET}    - Align laptop clock with Jetson (fixes TF)"
    echo -e "  ${BOLD}[4] Restart ROS2 Daemon${RESET} - Reconnect DDS if topics freeze"
    echo -e "  ${BOLD}[5] Stop Mapping & Exit${RESET} - Cleanly terminate SLAV/Nav2 stack"
    echo -e "  ${BOLD}[6] Keep Mapping & Exit${RESET} - Leave nodes running, close console only"
    echo -e "${CYAN}------------------------------------------------------------${RESET}"
    
    # Prompt with timeout to continuously check if RViz is still running
    opt=""
    if read -t 10 -p "Select option [1-6]: " opt; then
        case "$opt" in
            1)
                echo ""
                read -p "Enter map filename to save [default: classroom_map]: " MAP_NAME
                MAP_NAME="${MAP_NAME:-classroom_map}"
                python3 "$SCRIPT_DIR/control_mapping.py" "$JETSON_IP" save "$MAP_NAME"
                echo ""
                ;;
            2)
                echo -e "\n${BLUE}[Console] Resetting global & local costmaps...${RESET}"
                ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {} >/dev/null 2>&1 || true
                ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {} >/dev/null 2>&1 || true
                echo -e "${GREEN}✓ Costmaps cleared successfully.${RESET}\n"
                ;;
            3)
                echo -e "\n${BLUE}[Console] Syncing laptop clock with Jetson... (Requires local sudo password)${RESET}"
                if sudo date -s "$(ssh -o ConnectTimeout=3 "jetson@$JETSON_IP" date -R)"; then
                    echo -e "${GREEN}✓ Time synchronized successfully!${RESET}\n"
                else
                    echo -e "${RED}✗ Failed to sync time. Verify ssh access to jetson@$JETSON_IP.${RESET}\n"
                fi
                ;;
            4)
                echo -e "\n${BLUE}[Console] Restarting local ROS2 daemon...${RESET}"
                ros2 daemon stop >/dev/null 2>&1 || true
                ros2 daemon start >/dev/null 2>&1
                echo -e "${GREEN}✓ ROS2 daemon restarted.${RESET}\n"
                ;;
            5)
                echo -e "\n${YELLOW}[Console] Stopping SLAM + Nav2 stack on Jetson...${RESET}"
                python3 "$SCRIPT_DIR/control_mapping.py" "$JETSON_IP" stop || true
                echo -e "${GREEN}✓ Done. Closing RViz...${RESET}"
                break
                ;;
            6)
                echo -e "\n${GREEN}[Console] Leaving mapping stack running on Jetson.${RESET}"
                echo -e "Closing interactive console. Have a nice flight!"
                # Remove trap so we don't kill RViz on exit
                trap - INT TERM EXIT
                exit 0
                ;;
            *)
                echo -e "${RED}Invalid option.${RESET}\n"
                ;;
        esac
    fi
done

echo -e "\n${GREEN}✓ Classroom Mapping Utility session ended.${RESET}"
