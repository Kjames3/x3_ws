#!/usr/bin/env bash
# =============================================================================
# record_bag.sh — Domain Adaptation Data Collection
# =============================================================================
# Run this script ON THE JETSON to record a rosbag for Week 4 domain
# adaptation. The robot is teleoperated while people walk around it.
#
# Usage (on Jetson):
#   chmod +x record_bag.sh
#   ./record_bag.sh [OUTPUT_DIR]
#
# Default output dir: ~/bags/domain_adapt/
#
# Topics recorded:
#   /camera/depth/image_raw   — Orbbec Astra Pro depth stream (16UC1)
#   /scan                     — YDLidar X3 laser scan
#   /odom                     — EKF-fused odometry (robot position/velocity)
#   /tf                       — Dynamic coordinate frame transforms
#   /tf_static                — Static coordinate frame transforms
#
# After recording, transfer the bag to your laptop:
#   scp -r <JETSON_IP>:~/bags/domain_adapt/ ~/EE_244_Final_Project/bags/
# =============================================================================

set -euo pipefail

# ── Configuration ──────────────────────────────────────────────────────────────
OUTPUT_DIR="${1:-$HOME/bags/domain_adapt}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="domain_adapt_${TIMESTAMP}"
BAG_PATH="${OUTPUT_DIR}/${BAG_NAME}"

TOPICS=(
    "/camera/depth/image_raw"
    "/scan"
    "/odom"
    "/tf"
    "/tf_static"
)

# ── Checks ─────────────────────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo "[ERROR] ros2 not found. Did you source the ROS2 environment?"
    echo "  Try: source /opt/ros/humble/setup.bash && source ~/x3_ws/install/setup.bash"
    exit 1
fi

# Check ROS_DOMAIN_ID
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    echo "[WARN] ROS_DOMAIN_ID not set — defaulting to 42 (Jetson default)"
    export ROS_DOMAIN_ID=42
fi

# Check ROS_DISCOVERY_SERVER
if [[ -z "${ROS_DISCOVERY_SERVER:-}" ]]; then
    echo "[INFO] ROS_DISCOVERY_SERVER not set — defaulting to 127.0.0.1:11811"
    export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
fi

# Force Super Client mode so that CLI/introspection tools can query the DDS topology
export ROS_SUPER_CLIENT=TRUE

# ── Setup ──────────────────────────────────────────────────────────────────────
mkdir -p "${OUTPUT_DIR}"

echo "======================================================================="
echo "  EE244 Domain Adaptation — Rosbag Recording"
echo "======================================================================="
echo "  Bag path:       ${BAG_PATH}"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID}"
echo "  ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER}"
echo "  Topics:"
for t in "${TOPICS[@]}"; do echo "    $t"; done
echo ""

# Check if pre-flight check should be bypassed
if [[ "${BYPASS_TOPIC_CHECK:-false}" == "true" ]]; then
    echo "  [WARN] Pre-flight checks bypassed by user."
else
    echo "  [$(date +%H:%M:%S)] Running pre-flight topic and publisher checks..."
    MISSING_TOPICS=()
    for t in "${TOPICS[@]}"; do
        if ! ros2 topic info "$t" &>/dev/null; then
            MISSING_TOPICS+=("$t (not found on network)")
        else
            # Extract publisher count
            PUB_COUNT=$(ros2 topic info "$t" 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
            if [[ -z "$PUB_COUNT" || "$PUB_COUNT" -eq 0 ]]; then
                MISSING_TOPICS+=("$t (no active publishers)")
            fi
        fi
    done

    if [[ ${#MISSING_TOPICS[@]} -gt 0 ]]; then
        echo ""
        echo "======================================================================="
        echo "  [ERROR] Pre-flight Check FAILED: The following topics are not active"
        echo "          or have no publishers on the ROS2 network:"
        for mt in "${MISSING_TOPICS[@]}"; do
            echo "    - $mt"
        done
        echo ""
        echo "  If recording on the physical robot (Jetson), make sure:"
        echo "    1. The fastdds_discovery service is active:"
        echo "       sudo systemctl status fastdds_discovery"
        echo "    2. The hardware drivers are running (x3_server / jetson_bringup):"
        echo "       sudo systemctl status x3_server"
        echo "    3. The Orbbec depth camera service is running:"
        echo "       sudo systemctl status orbbec_depth"
        echo "    4. Your ROS_DOMAIN_ID matches ($ROS_DOMAIN_ID) and ROS_DISCOVERY_SERVER"
        echo "       is pointing to 127.0.0.1:11811"
        echo ""
        echo "  Aborting recording to prevent saving an empty bag."
        echo "  (You can bypass this check by setting BYPASS_TOPIC_CHECK=true)"
        echo "======================================================================="
        exit 1
    else
        echo "  ✓ All required topics have active publishers!"
        echo ""
    fi
fi

echo "  INSTRUCTIONS:"
echo "    1. Make sure the robot bringup is running (x3_bringup.launch.py)"
echo "    2. Make sure the Orbbec camera is streaming on /camera/depth/image_raw"
echo "    3. Use the joystick/GUI to teleoperate the robot through the classroom"
echo "    4. Have 2–3 people walk naturally around the robot at 0.5–2m distance"
echo "    5. Target: ~30 minutes of diverse walking scenarios"
echo "    6. Press Ctrl+C to stop recording when done"
echo ""
echo "  Starting in 5 seconds... (Ctrl+C to abort)"
sleep 5

echo ""
echo "  [$(date +%H:%M:%S)] Recording started — bag: ${BAG_NAME}"
echo "  -----------------------------------------------------------------------"

# ── Record ─────────────────────────────────────────────────────────────────────
# Compression keeps bag size manageable (depth images are large)
ros2 bag record \
    --output "${BAG_PATH}" \
    --compression-mode file \
    --compression-format zstd \
    "${TOPICS[@]}"

# ── Summary ────────────────────────────────────────────────────────────────────
echo ""
echo "  [$(date +%H:%M:%S)] Recording stopped."
echo ""

if [[ -d "${BAG_PATH}" ]]; then
    SIZE=$(du -sh "${BAG_PATH}" | cut -f1)
    echo "  Bag saved:   ${BAG_PATH}"
    echo "  Bag size:    ${SIZE}"
    echo ""
    echo "  Next steps:"
    echo "    scp -r ${BAG_PATH} <YOUR_LAPTOP_USER>@<LAPTOP_IP>:~/EE_244_Final_Project/bags/"
    echo "    Then on laptop: python3 preprocessing/05_process_rosbag.py --bag bags/${BAG_NAME}"
else
    echo "  [WARN] Bag directory not found — recording may have failed."
fi
echo "======================================================================="
