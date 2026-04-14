#!/usr/bin/env bash
# build_ros2.sh
# Run this once on a new machine (laptop or Jetson) to install dependencies
# and build the x3_ws ROS2 packages.
#
# Usage:
#   cd ~/x3_ws
#   bash scripts/build_ros2.sh

set -e  # exit on first error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo " x3_ws ROS2 build setup"
echo " Workspace: $WS_DIR"
echo "============================================================"

# ── 1. ROS2 Python build tools ────────────────────────────────────────────────
# Colcon may pick up the system Python or conda's Python depending on
# the environment. Install build deps in whichever python3 is active.
echo ""
echo "[1/4] Installing ROS2 Python build dependencies..."
python3 -m pip install --quiet catkin_pkg "empy==3.3.4" lark
echo "      catkin_pkg, empy==3.3.4, lark installed."

# ── 2. YDLidar C++ SDK ────────────────────────────────────────────────────────
# Required by ydlidar_ros2_driver before colcon can build it.
echo ""
echo "[2/4] Building & installing YDLidar-SDK..."
SDK_DIR="$HOME/YDLidar-SDK"

if [ ! -d "$SDK_DIR" ]; then
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
else
    echo "      YDLidar-SDK already cloned at $SDK_DIR — pulling latest..."
    git -C "$SDK_DIR" pull
fi

mkdir -p "$SDK_DIR/build"
cmake -S "$SDK_DIR" -B "$SDK_DIR/build" -DCMAKE_BUILD_TYPE=Release
make -C "$SDK_DIR/build" -j"$(nproc)"
sudo make -C "$SDK_DIR/build" install
sudo ldconfig
echo "      YDLidar-SDK installed."

# ── 3. Source ROS2 ────────────────────────────────────────────────────────────
echo ""
echo "[3/4] Sourcing ROS2 Humble..."
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

# ── 4. Colcon build ───────────────────────────────────────────────────────────
echo ""
echo "[4/4] Building ROS2 packages..."
cd "$WS_DIR"
colcon build --symlink-install \
    --packages-select \
    yahboomcar_msgs \
    yahboomcar_description \
    yahboomcar_base_node \
    yahboomcar_bringup \
    yahboomcar_nav \
    ydlidar_ros2_driver

echo ""
echo "============================================================"
echo " Build complete!"
echo " Run the following to activate the workspace:"
echo "   source $WS_DIR/install/setup.bash"
echo "============================================================"
