#!/usr/bin/env bash
# Separate workspace; existing Livox SDK2 + driver with MID360S support required.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
WS=${FASTLIO_WS:-$HOME/ws_fastlio}
REV=2fffc570a25d0df172720bac034fbdb6a13d2162
mkdir -p "$WS/src"
if [ ! -d "$WS/src/FAST_LIO" ]; then
    git clone --branch ros2 https://github.com/Ericsii/FAST_LIO.git "$WS/src/FAST_LIO"
    git -C "$WS/src/FAST_LIO" checkout "$REV"
    git -C "$WS/src/FAST_LIO" submodule update --init --recursive
fi
if [ "$(git -C "$WS/src/FAST_LIO" rev-parse HEAD)" != "$REV" ]; then
    echo "Expected FAST-LIO $REV; existing checkout differs. Inspect before building." >&2
    exit 1
fi
cd "$WS"
MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
