#!/usr/bin/env bash
# Offline FAST-LIO on a Livox bag, ON THE JETSON, in an isolated ROS domain
# (77, localhost only) so the replayed /livox/* never mixes with the live robot.
#
#   scripts/fastlio_replay.sh BAG OUT_DIR [RATE=0.5] [CONFIG.yaml]
#
# CONFIG defaults to the stock mid360.yaml (Livox internal IMU);
# config/fastlio_mid360_icm.yaml uses the robot's ICM-42688-P instead.
#
# Writes OUT_DIR/fastlio_out (FAST-LIO /Odometry + /path); compare with
# src/tools/lidar/fastlio_bag_compare.py.  Uses ~/ws_fastlio (Ericsii FAST_LIO
# ROS 2 port, config mid360.yaml).
set -e
BAG=${1:?bag}; OUT=${2:?out}; RATE=${3:-0.5}; CFG=${4:-}
# Resolve before the cd below: relative paths silently broke the run.
BAG=$(readlink -f "$BAG"); mkdir -p "$OUT"; OUT=$(readlink -f "$OUT")
[ -n "$CFG" ] && CFG=$(readlink -f "$CFG")
unset FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/ws_fastlio/install/setup.bash
mkdir -p "$OUT"
cd "$OUT"
if [ -n "$CFG" ]; then
    LARGS="config_path:=$(dirname "$CFG") config_file:=$(basename "$CFG")"
    IMU=$(grep -oP 'imu_topic:\s*"\K[^"]+' "$CFG")
else
    LARGS="config_file:=mid360.yaml"; IMU=/livox/imu
fi
setsid ros2 launch fast_lio mapping.launch.py $LARGS rviz:=false > fastlio.log 2>&1 &
FL=$!
sleep 5
setsid ros2 bag record -o "$OUT/fastlio_out" /Odometry /path > rec.log 2>&1 &
REC=$!
sleep 3
ros2 bag play "$BAG" --rate "$RATE" --topics /livox/lidar "$IMU" > play.log 2>&1
sleep 5
kill -INT -- -$REC; sleep 3
kill -INT -- -$FL; sleep 5
kill -TERM -- -$FL 2>/dev/null || true
echo DONE
