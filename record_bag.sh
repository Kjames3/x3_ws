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

# ── Setup ──────────────────────────────────────────────────────────────────────
mkdir -p "${OUTPUT_DIR}"

echo "======================================================================="
echo "  EE244 Domain Adaptation — Rosbag Recording"
echo "======================================================================="
echo "  Bag path:       ${BAG_PATH}"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID}"
echo "  Topics:"
for t in "${TOPICS[@]}"; do echo "    $t"; done
echo ""
echo "  INSTRUCTIONS:"
echo "    1. Make sure the robot bringup is running (x3_bringup.launch.py)"
echo "    2. Make sure the Orbbec camera is streaming on /camera/depth/image_raw"
echo "    3. Use the joystick/GUI to teleoperate the robot through the classroom"
echo "    4. Have 2–3 people walk naturally around the robot at 0.5–2m distance"
echo "    5. Target: ~30 minutes of diverse walking scenarios"
echo "    6. Press Ctrl+C to stop recording when done"
echo ""
echo "  Verify topics are live before starting:"
echo "    ros2 topic hz /scan"
echo "    ros2 topic hz /camera/depth/image_raw"
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
