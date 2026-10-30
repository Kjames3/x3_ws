#!/usr/bin/env bash
# Run on the Jetson. Ctrl-C stops both diagnostic processes.
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
source "$HOME/ws_fastlio/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
if pgrep -f '^/.*/(livox_ros_driver2_node|fastlio_mapping)( |$)' >/dev/null; then
    echo 'A Livox driver or FAST-LIO is already running; stop it before starting this launch.' >&2
    exit 1
fi
exec ros2 launch "$ROOT/scripts/fastlio_shadow.launch.py"
