#!/usr/bin/env bash
# laptop_sim.sh
# Two laptop-side modes:
#
#   view    Render the URDF in RViz with a joint slider, no robot and no Gazebo.
#           Use this to eyeball a model change (e.g. the tilting lidar mount)
#           before pushing it to the Jetson.
#
#   gazebo  (default) Start the Gazebo simulation for cross-machine development.
#           The Jetson runs server_x3.py --ros2 --domain-id <N>; this script
#           exports the SAME domain ID so ROS2 DDS discovery connects the Gazebo
#           topics on the laptop to the ROS2Bridge on the Jetson.
#
# Requirements (laptop):
#   - ROS2 Humble installed
#   - x3_ws built:  bash scripts/build_ros2.sh
#   - Ignition Fortress (gazebo mode only):
#       sudo apt install ros-humble-ros-gz
#
# Usage:
#   bash scripts/laptop_sim.sh                # Gazebo, DOMAIN_ID=42
#   bash scripts/laptop_sim.sh 0              # Gazebo, ROS2 default domain
#   bash scripts/laptop_sim.sh view           # RViz URDF preview, isolated
#   bash scripts/laptop_sim.sh view 42        # RViz preview on domain 42
#
# Env overrides (view mode):
#   X3_NO_BUILD=1        skip the re-install step (meshes/URDF already staged)
#   X3_VIEW_CONNECT=1    do NOT force localhost-only; join the real domain
#   X3_MODEL=<path>      preview a different .urdf / .urdf.xacro

set -e

MODE="gazebo"
case "${1:-}" in
    view|viz|urdf|rviz|--view) MODE="view"; shift ;;
esac

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ─────────────────────────────────────────────────────────────────────
# view — RViz URDF preview
# ─────────────────────────────────────────────────────────────────────
if [ "$MODE" = "view" ]; then
    # Default to a domain of its own. robot_state_publisher here would
    # otherwise publish /tf and /robot_description for the same frames the
    # powered-on robot is publishing, and the two would fight.
    DOMAIN_ID="${1:-99}"
    export ROS_DOMAIN_ID="$DOMAIN_ID"
    if [ -z "${X3_VIEW_CONNECT:-}" ]; then
        export ROS_LOCALHOST_ONLY=1
    fi

    source /opt/ros/humble/setup.bash

    # The install space is a real copy of urdf/ and meshes/, not a live view of
    # src/, and RViz resolves package://yahboomcar_description/meshes/... through
    # it. Editing the URDF in src/ alone shows you the OLD model with MISSING
    # meshes, so re-install the description package first.
    if [ -z "${X3_NO_BUILD:-}" ]; then
        echo "── staging yahboomcar_description into the install space ──"
        # Plain copy install on purpose: this workspace's install/ was built in
        # copy mode, and re-running with --symlink-install over it fails.
        ( cd "$WS_DIR" && colcon build \
            --packages-select yahboomcar_description \
            --event-handlers console_cohesion- ) \
            || { echo "!! build failed. Try:  rm -rf '$WS_DIR/install/yahboomcar_description'"
                 echo "   then re-run, or skip staging with X3_NO_BUILD=1"; exit 1; }
        echo ""
    fi

    source "$WS_DIR/install/setup.bash"

    MODEL="${X3_MODEL:-$(ros2 pkg prefix yahboomcar_description)/share/yahboomcar_description/urdf/yahboomcar_X3.urdf}"

    echo "============================================================"
    echo " Yahboom X3 — URDF preview (laptop, no robot)"
    echo " Workspace     : $WS_DIR"
    echo " Model         : $MODEL"
    echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
    echo " Localhost only: ${ROS_LOCALHOST_ONLY:-0}   (X3_VIEW_CONNECT=1 to disable)"
    echo "============================================================"

    # Fail here with a readable message rather than deep inside RViz.
    xacro "$MODEL" > /tmp/x3_preview.urdf \
        || { echo "!! xacro failed to parse $MODEL"; exit 1; }
    check_urdf /tmp/x3_preview.urdf > /dev/null \
        || { echo "!! check_urdf rejected $MODEL"; exit 1; }

    echo ""
    echo " Movable joints (sliders in the joint_state_publisher window):"
    python3 - "/tmp/x3_preview.urdf" <<'PY'
import sys, xml.etree.ElementTree as ET
r = ET.parse(sys.argv[1]).getroot()
for j in r.findall('joint'):
    if j.get('type') == 'fixed':
        continue
    lim = j.find('limit')
    rng = ''
    if lim is not None and lim.get('lower') is not None:
        import math
        lo, hi = float(lim.get('lower')), float(lim.get('upper'))
        rng = f"  [{math.degrees(lo):7.1f} .. {math.degrees(hi):7.1f} deg]"
    print(f"   {j.get('name'):24s} {j.get('type'):11s}{rng}")
PY
    echo ""
    echo " Drag lidar_tilt_joint to pitch the lidar carousel forward/back."
    echo " Close RViz to exit."
    echo "============================================================"
    echo ""

    exec ros2 launch yahboomcar_description display_X3.launch.py \
        gui:=true model:="$MODEL"
fi

# ─────────────────────────────────────────────────────────────────────
# gazebo — cross-machine simulation
# ─────────────────────────────────────────────────────────────────────
DOMAIN_ID="${1:-42}"
export ROS_DOMAIN_ID="$DOMAIN_ID"

echo "============================================================"
echo " Yahboom X3 — Gazebo Simulation (laptop)"
echo " Workspace     : $WS_DIR"
echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "============================================================"
echo ""
echo " Start the Jetson server in a separate terminal:"
echo "   python3 src/server_x3.py --ros2 --domain-id $DOMAIN_ID"
echo ""
echo " Then open the browser and connect to the Jetson IP."
echo ""
echo " (To preview a URDF change instead: bash scripts/laptop_sim.sh view)"
echo "============================================================"
echo ""

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

ros2 launch yahboomcar_nav x3_gazebo.launch.py
