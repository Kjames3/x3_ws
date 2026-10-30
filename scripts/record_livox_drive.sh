#!/usr/bin/env bash
# Record a Livox Mid-360S driving bag ON THE JETSON (x3_server already running).
#
#   scripts/record_livox_drive.sh NAME [SECONDS]
#
# Starts livox_ros_driver2 (CustomMsg, per-point timestamps -- what FAST-LIO
# and src/tools/lidar/fastlio_compare.py read) unless /livox/lidar is already
# published, records to ~/bags/livox/NAME, and stops the driver it started.
# Ctrl-C ends an open-ended recording cleanly.
set -eo pipefail
NAME=${1:?usage: record_livox_drive.sh NAME [SECONDS]}
SECS=${2:-}
OUT=~/bags/livox/$NAME
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
[ -e "$OUT" ] && { echo "$OUT exists"; exit 1; }
mkdir -p ~/bags/livox

DRV=
if ! timeout 4 ros2 topic hz /livox/lidar 2>/dev/null | grep -q average; then
    setsid ros2 launch livox_ros_driver2 msg_MID360s_launch.py >/tmp/livox_driver.log 2>&1 &
    DRV=$!
    sleep 6
fi
trap '[ -n "$DRV" ] && kill -TERM -- -$DRV 2>/dev/null; wait' EXIT

TOPICS=(/livox/lidar /livox/imu /odom /odom_raw /imu/data /imu/data_raw /tf /tf_static
        /tof/upper/obstacles /tof/lower/obstacles /tof/upper/points /tof/lower/points /cmd_vel_safe)
echo "recording ${SECS:-until Ctrl-C}s -> $OUT"
if [ -n "$SECS" ]; then
    timeout -s INT "$SECS" ros2 bag record -o "$OUT" "${TOPICS[@]}" || true
else
    ros2 bag record -o "$OUT" "${TOPICS[@]}" || true
fi
ros2 bag info "$OUT" | grep -E "Duration|Size|livox|odom |imu/data_raw|tf_static"
