#!/usr/bin/env bash
# =============================================================================
# fetch_bag.sh — run a recording on the robot, then pull the bag to this laptop
# =============================================================================
# Run this ON THE LAPTOP. It is the "pull" counterpart to record_bag.sh's
# built-in push: your laptop initiates every connection, so it needs no sshd,
# no open port, and no inbound access of any kind. Use this when you'd rather
# drive from the laptop; use record_bag.sh's own auto-scp when you're already
# logged into the robot.
#
# Usage:
#   ./fetch_bag.sh                      # record until Ctrl+C, then pull
#   ./fetch_bag.sh 120                  # record 120s, then pull
#
# Environment:
#   ROBOT=jetson@10.13.196.218          robot ssh target (default as shown)
#   DEST_DIR=~/EE_244_Final_Project/bags     where bags land locally
#   REMOTE_SCRIPT=~/x3_ws/record_bag.sh      path to the recorder on the robot
#   KEEP_REMOTE=true                    keep the robot-side copy (default true)
#   ROS_SETUP="..."                     shell snippet sourcing ROS on the robot
#   Any record_bag.sh knob is forwarded: RECORD_OAK_STEREO, RECORD_ASTRA_DEPTH,
#   BYPASS_TOPIC_CHECK, OUTPUT_DIR.
#
# Note the recorder is invoked with AUTO_SCP=false so its push path stays out of
# the way — otherwise a bag would be transferred twice.
# =============================================================================

set -euo pipefail

ROBOT="${ROBOT:-jetson@10.13.196.218}"
DEST_DIR="${DEST_DIR:-$HOME/EE_244_Final_Project/bags}"
REMOTE_SCRIPT="${REMOTE_SCRIPT:-\$HOME/x3_ws/record_bag.sh}"
KEEP_REMOTE="${KEEP_REMOTE:-true}"
# A non-interactive `ssh host cmd` gets no ROS on PATH: Ubuntu's ~/.bashrc
# returns early for non-interactive shells, and the robot's .bashrc doesn't
# source ROS anyway. Source it explicitly or record_bag.sh aborts with
# "ros2 not found".
ROS_SETUP="${ROS_SETUP:-source /opt/ros/humble/setup.bash; source \$HOME/x3_ws/install/setup.bash}"
RECORD_DURATION="${1:-${RECORD_DURATION:-}}"

SSH_OPTS=(-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new)

echo "======================================================================="
echo "  EE244 Bag Fetch — record on robot, pull to laptop"
echo "======================================================================="
echo "  Robot:       ${ROBOT}"
echo "  Local dest:  ${DEST_DIR}"
echo "  Duration:    ${RECORD_DURATION:-until Ctrl+C}"
echo ""

if ! ssh "${SSH_OPTS[@]}" -o BatchMode=yes "${ROBOT}" true 2>/dev/null; then
    echo "  [ERROR] Cannot reach ${ROBOT} over SSH."
    echo "          Check the robot is powered and on the network, and that its"
    echo "          IP hasn't changed:  ROBOT=jetson@<NEW_IP> $0"
    exit 1
fi

mkdir -p "${DEST_DIR}"

# Forward only the knobs that are actually set, so the recorder's own defaults
# still apply to everything else.
REMOTE_ENV="AUTO_SCP=false"
for v in RECORD_DURATION RECORD_OAK_STEREO RECORD_ASTRA_DEPTH BYPASS_TOPIC_CHECK OUTPUT_DIR; do
    if [[ -n "${!v:-}" ]]; then
        REMOTE_ENV+=" ${v}=$(printf '%q' "${!v}")"
    fi
done

# -tt forces a tty even when this wrapper's own stdin isn't one, so Ctrl+C
# reaches ros2 bag record and the bag is finalised cleanly. With plain -t and no
# local tty, ssh silently skips allocation and the recorder gets orphaned.
echo "  [$(date +%H:%M:%S)] Starting recorder on the robot (Ctrl+C to stop)..."
echo "  -----------------------------------------------------------------------"
ssh "${SSH_OPTS[@]}" -tt "${ROBOT}" \
    "${ROS_SETUP}; ${REMOTE_ENV} bash ${REMOTE_SCRIPT}" || true
echo "  -----------------------------------------------------------------------"

# ── Locate and pull ────────────────────────────────────────────────────────────
REMOTE_BAG=$(ssh "${SSH_OPTS[@]}" -o BatchMode=yes "${ROBOT}" 'cat ~/.x3_last_bag 2>/dev/null' || true)
REMOTE_BAG="${REMOTE_BAG//$'\r'/}"   # strip CR left by the tty session

if [[ -z "${REMOTE_BAG}" ]]; then
    echo "  [WARN] No bag marker found on the robot — recording may have failed"
    echo "         or aborted before saving. Nothing pulled."
    exit 1
fi

BAG_NAME=$(basename "${REMOTE_BAG}")
echo ""
echo "  [$(date +%H:%M:%S)] Pulling ${BAG_NAME} → ${DEST_DIR}/"

# rsync resumes a partial transfer if wifi drops; fall back to scp if absent.
if command -v rsync &>/dev/null; then
    rsync -az --partial --info=progress2 -e "ssh ${SSH_OPTS[*]}" \
        "${ROBOT}:${REMOTE_BAG}" "${DEST_DIR}/"
else
    scp "${SSH_OPTS[@]}" -r "${ROBOT}:${REMOTE_BAG}" "${DEST_DIR}/"
fi

LOCAL_BAG="${DEST_DIR}/${BAG_NAME}"
echo "  ✓ Pulled to ${LOCAL_BAG}  ($(du -sh "${LOCAL_BAG}" | cut -f1))"

if [[ "${KEEP_REMOTE}" != "true" ]]; then
    echo "  [$(date +%H:%M:%S)] Removing robot-side copy (KEEP_REMOTE=${KEEP_REMOTE})"
    ssh "${SSH_OPTS[@]}" -o BatchMode=yes "${ROBOT}" "rm -rf $(printf '%q' "${REMOTE_BAG}")"
fi

echo ""
echo "  Next: python3 preprocessing/05_process_rosbag.py --bag bags/${BAG_NAME}"
echo "======================================================================="
