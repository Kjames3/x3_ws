#!/usr/bin/env bash
# Record one walk-up / goal run for offline controller comparison. Run ON THE ROBOT.
#   bash scripts/record_walk.sh walkup-r1 60          # parked robot, person walks at it
#   bash scripts/record_walk.sh goal-r1 90            # Nav2 goal while someone walks past
# Then, on any machine with ROS + install/setup.bash sourced:
#   python3 src/tools/nav/orca_cbf_compare.py artifacts/walk-<date>/walkup-r1
#   python3 src/tools/nav/orca_cbf_compare.py artifacts/walk-<date>/goal-r1 --nominal track
#
# Since c38485d the driver listens on /cmd_vel_safe, so a bag holds BOTH the request
# (/cmd_vel_nominal: whatever entered the CBF, GUI joystick included; /cmd_vel is
# only Nav2 and scripts) and what the CBF let through (/cmd_vel_safe). The
# comparison needs both; this refuses to record if the gate is not up.
set -euo pipefail
name="${1:?usage: record_walk.sh <run-name> [seconds]}"; secs="${2:-60}"
cd "$HOME/x3_ws"; out="artifacts/walk-$(date +%F)/$name"
if [ -e "$out" ]; then echo "$out exists; pick the next name" >&2; exit 1; fi
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash 2>/dev/null; set -u
export ROS_DOMAIN_ID=42

topics=$(ros2 topic list 2>/dev/null)
if ! grep -qx /cmd_vel_safe <<<"$topics"; then
    echo "/cmd_vel_safe is not advertised: is x3_server up with the gate on (X3_CMD_VEL_GATE != 0)?" >&2
    exit 1
fi
for t in /cmd_vel_nominal /pedestrian_states /scan /odom; do
    grep -qx "$t" <<<"$topics" || echo "warning: $t is not advertised; the replay needs it" >&2
done

mkdir -p "$out"
since=$(date '+%Y-%m-%d %H:%M:%S')
echo "recording $secs s into $out ..."
# stdin from /dev/null: under `timeout` the recorder is a background job, and
# reading the terminal for its keyboard controls stops it (SIGTTIN) forever.
timeout -s INT "$secs" ros2 bag record -o "$out/bag" \
    /cmd_vel /cmd_vel_nominal /cmd_vel_safe /odom /odom_raw /vel_raw /imu/data /scan /tf /tf_static \
    /pedestrian_states /pedestrian_poses /people/obstacles \
    /tof/upper/obstacles /tof/lower/obstacles /plan \
    < /dev/null > "$out/record.log" 2>&1 || true
sleep 1
# the server's view of the run: estimator, person-memory, CBF infeasible lines
journalctl -u x3_server --since "$since" --no-pager -o short-precise > "$out/server.log" 2>/dev/null || true
python3 - "$out/bag/metadata.yaml" <<'PY'
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
d = m['duration']['nanoseconds'] / 1e9
print('recorded %.0f s' % d)
counts = {t['topic_metadata']['name']: t['message_count'] for t in m['topics_with_message_count']}
for name, n in sorted(counts.items()):
    print('  %-22s %6d  %.1f Hz' % (name, n, n / d))
for need in ('/cmd_vel_safe', '/cmd_vel_nominal', '/scan', '/odom', '/pedestrian_states'):
    if not counts.get(need):
        print('WARNING: no %s messages; the comparison cannot use this run' % need)
PY
echo "done: $out"
