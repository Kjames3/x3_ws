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
# Topics recorded (always):
#   /oak/depth/image_raw      — OAK-D Lite on-device stereo depth (16UC1, mm)
#   /oak/depth/camera_info    — OAK-D CAM_A intrinsics (needed to reproject depth)
#   /oak/imu                  — OAK-D 6-axis IMU (accel + gyro)
#   /oak/detections           — OAK-D on-device YOLO spatial detections (3D)
#   /scan                     — YDLidar X3 laser scan
#   /odom                     — EKF-fused odometry (robot position/velocity)
#   /tf                       — Dynamic coordinate frame transforms
#   /tf_static                — Static coordinate frame transforms
#
# Optional topics (opt in — they roughly double or triple the bag size):
#   RECORD_OAK_STEREO=true    /oak/left/image_raw, /oak/right/image_raw (mono8)
#   RECORD_ASTRA_DEPTH=true   /camera/depth/image_raw (Orbbec Astra Pro, 16UC1)
#
# Other knobs:
#   RECORD_DURATION=60        stop automatically after N seconds (trial runs)
#   BYPASS_TOPIC_CHECK=true   skip the pre-flight publisher check
#
# IMPORTANT — the /oak/* topics only exist if x3_server was started with
# --oak-ros-publish. DepthAI opens the OAK-D exclusively, so the server process
# that owns the device is the only thing that can publish its streams; a second
# process cannot open the camera. To enable:
#
#   sudo mkdir -p /etc/systemd/system/x3_server.service.d
#   sudo tee /etc/systemd/system/x3_server.service.d/oak-record.conf >/dev/null <<'EOF'
#   [Service]
#   ExecStart=
#   ExecStart=/usr/bin/python3 /home/jetson/x3_ws/src/server_x3.py --domain-id 42 --oak-ros-publish
#   EOF
#   sudo systemctl daemon-reload && sudo systemctl restart x3_server
#
# ...then remove that drop-in and restart once the recording session is done, so
# normal operation isn't paying for the extra publishing.
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

# Opt-in extras (see header). Default off to keep ~30 min sessions a sane size.
RECORD_OAK_STEREO="${RECORD_OAK_STEREO:-false}"
RECORD_ASTRA_DEPTH="${RECORD_ASTRA_DEPTH:-false}"

TOPICS=(
    "/oak/depth/image_raw"
    "/oak/depth/camera_info"
    "/oak/imu"
    "/scan"
    "/odom"
    "/tf"
    "/tf_static"
)

# Recorded when present but never fatal: /oak/detections needs the on-device YOLO
# blob (skipped if it was never extracted), and the optional streams below are
# only published on request. ros2 bag record accepts topics that appear later.
OPTIONAL_TOPICS=(
    "/oak/detections"
)

if [[ "${RECORD_OAK_STEREO}" == "true" ]]; then
    OPTIONAL_TOPICS+=("/oak/left/image_raw" "/oak/right/image_raw")
fi

if [[ "${RECORD_ASTRA_DEPTH}" == "true" ]]; then
    OPTIONAL_TOPICS+=("/camera/depth/image_raw")
fi

# ── Checks ─────────────────────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
    echo "[ERROR] ros2 not found. Did you source the ROS2 environment?"
    echo "  Try: source /opt/ros/humble/setup.bash && source ~/x3_ws/install/setup.bash"
    exit 1
fi

# ── DDS configuration ──────────────────────────────────────────────────────────
# The recorder MUST join the DDS network exactly the way the robot's own nodes
# did, or it discovers nothing and silently records an empty bag. The single
# source of truth is the environment of the LIVE server_x3.py process — the
# service definition can lie (its ExecStart currently unsets the discovery-server
# vars that the [Service] Environment= lines set).
#
# As of 2026-07 the robot runs PLAIN DDS MULTICAST on ROS_DOMAIN_ID=42 with no
# discovery server. Do not re-add a ROS_DISCOVERY_SERVER guess here: pointing the
# recorder at a discovery server that nothing else uses is exactly how you get a
# bag with zero messages in it.
SERVER_PID=$(pgrep -f "server_x3.py" | head -n 1 || echo "")
SERVER_ENV=""
if [[ -n "$SERVER_PID" ]]; then
    SERVER_ENV=$(tr '\0' '\n' < "/proc/${SERVER_PID}/environ" 2>/dev/null || echo "")
fi

# ROS_DOMAIN_ID — inherit from the live server when we can read it.
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    DETECTED_DOMAIN=$(grep -m1 "^ROS_DOMAIN_ID=" <<<"$SERVER_ENV" | cut -d= -f2 || echo "")
    if [[ -n "$DETECTED_DOMAIN" ]]; then
        echo "  [$(date +%H:%M:%S)] ROS_DOMAIN_ID=$DETECTED_DOMAIN (inherited from running server)"
        export ROS_DOMAIN_ID="$DETECTED_DOMAIN"
    else
        echo "  [WARN] ROS_DOMAIN_ID not set and no running server — defaulting to 42"
        export ROS_DOMAIN_ID=42
    fi
fi

# ROS_DISCOVERY_SERVER — adopt ONLY if the live server actually uses one.
if [[ -z "${ROS_DISCOVERY_SERVER:-}" ]]; then
    DETECTED_DS=$(grep -m1 "^ROS_DISCOVERY_SERVER=" <<<"$SERVER_ENV" | cut -d= -f2- || echo "")
    if [[ -n "$DETECTED_DS" ]]; then
        echo "  [$(date +%H:%M:%S)] Discovery server in use by the robot: $DETECTED_DS"
        export ROS_DISCOVERY_SERVER="$DETECTED_DS"
        # Super Client mode lets CLI/introspection tools query the DDS topology.
        export ROS_SUPER_CLIENT=TRUE
        DETECTED_PROFILE=$(grep -m1 "^FASTDDS_DEFAULT_PROFILES_FILE=" <<<"$SERVER_ENV" | cut -d= -f2- || echo "")
        if [[ -n "$DETECTED_PROFILE" ]]; then
            export FASTDDS_DEFAULT_PROFILES_FILE="$DETECTED_PROFILE"
        fi
    elif [[ -n "$SERVER_PID" ]]; then
        echo "  [$(date +%H:%M:%S)] Robot is on plain DDS multicast (no discovery server) — matching it"
        unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE 2>/dev/null || true
    else
        echo "  [WARN] No running server_x3.py found — assuming plain DDS multicast"
    fi
fi

# Restart the ROS2 daemon so it inherits the correct DDS network settings immediately
echo "  [$(date +%H:%M:%S)] Restarting ROS2 daemon with updated discovery configuration..."
ros2 daemon stop &>/dev/null || true
ros2 daemon start &>/dev/null
sleep 2

# ── Setup ──────────────────────────────────────────────────────────────────────
mkdir -p "${OUTPUT_DIR}"

echo "======================================================================="
echo "  EE244 Domain Adaptation — Rosbag Recording"
echo "======================================================================="
echo "  Bag path:       ${BAG_PATH}"
echo "  ROS_DOMAIN_ID:  ${ROS_DOMAIN_ID}"
echo "  Discovery:      ${ROS_DISCOVERY_SERVER:-plain DDS multicast (no discovery server)}"
echo "  Topics (required):"
for t in "${TOPICS[@]}"; do echo "    $t"; done
echo "  Topics (optional — recorded if published):"
for t in "${OPTIONAL_TOPICS[@]}"; do echo "    $t"; done
echo "  OAK stereo L/R:  ${RECORD_OAK_STEREO}   (RECORD_OAK_STEREO=true to enable)"
echo "  Astra depth:     ${RECORD_ASTRA_DEPTH}   (RECORD_ASTRA_DEPTH=true to enable)"
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

    # Optional topics only warn — a missing one costs you a stream, not the session.
    for t in "${OPTIONAL_TOPICS[@]}"; do
        PUB_COUNT=$(ros2 topic info "$t" 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
        if [[ -z "$PUB_COUNT" || "$PUB_COUNT" -eq 0 ]]; then
            echo "  [WARN] optional topic $t has no publisher — it will be absent from the bag"
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
        echo "    3. For any /oak/* topic: x3_server must be running WITH --oak-ros-publish."
        echo "       The OAK-D is opened exclusively by that process, so nothing else can"
        echo "       publish its streams. Check the flag is in effect:"
        echo "         systemctl show x3_server -p ExecStart | grep -o -- --oak-ros-publish"
        echo "       and add it via a drop-in (see the header of this script), then:"
        echo "         sudo systemctl daemon-reload && sudo systemctl restart x3_server"
        echo "    4. For /camera/depth/image_raw: the Orbbec depth camera service is running:"
        echo "       sudo systemctl status orbbec_depth"
        echo "    5. Your ROS_DOMAIN_ID matches ($ROS_DOMAIN_ID) and ROS_DISCOVERY_SERVER"
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
echo "    2. Make sure x3_server runs with --oak-ros-publish so /oak/* is on the network"
echo "    3. Use the joystick/GUI to teleoperate the robot through the classroom"
echo "    4. Have 2–3 people walk naturally around the robot at 0.5–2m distance"
echo "    5. Target: ~30 minutes of diverse walking scenarios"
echo "    6. Keep an eye on free disk space — OAK depth alone is roughly 1–2 GB per"
echo "       10 min after zstd; enabling stereo/Astra depth multiplies that"
echo "    7. Press Ctrl+C to stop recording when done"
echo ""
echo "  Countdown to recording (Ctrl+C to abort):"
for i in 5 4 3 2 1; do
    echo "    ${i}..."
    sleep 1
done

echo ""
echo "  [$(date +%H:%M:%S)] Recording started — bag: ${BAG_NAME}"
echo "  -----------------------------------------------------------------------"

# ── Record ─────────────────────────────────────────────────────────────────────
# Compression keeps bag size manageable (depth images are large)
#
# RECORD_DURATION=<seconds> stops automatically instead of waiting for Ctrl+C —
# used for short trial runs to size the data rate before a real session.
# ros2 bag record has no --duration in Humble, so send it SIGINT (the same signal
# Ctrl+C sends) to get a cleanly finalised bag.
if [[ -n "${RECORD_DURATION:-}" ]]; then
    echo "  [$(date +%H:%M:%S)] Timed run — stopping automatically after ${RECORD_DURATION}s"
    ros2 bag record \
        --output "${BAG_PATH}" \
        --compression-mode file \
        --compression-format zstd \
        "${TOPICS[@]}" "${OPTIONAL_TOPICS[@]}" &
    REC_PID=$!
    # Stop on our timer, but don't hang forever if the user Ctrl+Cs first.
    ( sleep "${RECORD_DURATION}"; kill -INT "${REC_PID}" 2>/dev/null || true ) &
    TIMER_PID=$!
    wait "${REC_PID}" || true
    kill "${TIMER_PID}" 2>/dev/null || true
else
    ros2 bag record \
        --output "${BAG_PATH}" \
        --compression-mode file \
        --compression-format zstd \
        "${TOPICS[@]}" "${OPTIONAL_TOPICS[@]}"
fi

# ── Summary ────────────────────────────────────────────────────────────────────
echo ""
echo "  [$(date +%H:%M:%S)] Recording stopped."
echo ""

if [[ -d "${BAG_PATH}" ]]; then
    SIZE=$(du -sh "${BAG_PATH}" | cut -f1)
    echo "  Bag saved:   ${BAG_PATH}"
    echo "  Bag size:    ${SIZE}"
    echo ""
    echo "  --- Recorded Bag Details ---"
    ros2 bag info "${BAG_PATH}" 2>/dev/null || echo "    [WARN] Could not parse bag info."
    echo "  ----------------------------"
    echo ""
    echo "  Next steps:"
    echo "    scp -r ${BAG_PATH} <YOUR_LAPTOP_USER>@<LAPTOP_IP>:~/EE_244_Final_Project/bags/"
    echo "    Then on laptop: python3 preprocessing/05_process_rosbag.py --bag bags/${BAG_NAME}"
else
    echo "  [WARN] Bag directory not found — recording may have failed."
fi
echo "======================================================================="
