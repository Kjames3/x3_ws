#!/usr/bin/env bash
# Switch the robot's OAK model for pose testing, with detection logging.
#   bash scripts/pose_test.sh yolo26n-pose   # or yolo11n-pose / yolo26n (logged baseline)
#   bash scripts/pose_test.sh off            # remove the override, back to normal
# Run ON THE ROBOT. Adds a systemd drop-in that appends to the current
# SERVER_ARGS, so the other overrides (webrtc, c1 recording) stay in force.
# Logs land in ~/x3_ws/artifacts/pose-apartment/<model>-<time>.ndjson.
set -euo pipefail
DROPIN=/etc/systemd/system/x3_server.service.d/95-pose-test.conf
LOGDIR="$HOME/x3_ws/artifacts/pose-apartment"
# systemctl prints "SERVER_ARGS=a b c" quoted when it has spaces.
server_args() { grep -oP '"?SERVER_ARGS=\K[^"]*' | head -1 | sed 's/ [A-Z_]*=.*//'; }
model="${1:?usage: pose_test.sh <yolo26n|yolo11n-pose|yolo26n-pose|off>}"

if [ "$model" = off ]; then
    sudo rm -f "$DROPIN"
else
    case "$model" in yolo26n|yolo11n-pose|yolo26n-pose) ;; *) echo "unknown model $model" >&2; exit 1;; esac
    sudo rm -f "$DROPIN"
    sudo systemctl daemon-reload
    base=$(systemctl show x3_server -p Environment --value | server_args)
    printf '[Service]\nEnvironment="SERVER_ARGS=%s --oak-model %s --oak-det-log %s"\n' \
        "$base" "$model" "$LOGDIR" | sudo tee "$DROPIN" >/dev/null
fi
sudo systemctl daemon-reload
sudo systemctl restart x3_server
echo "SERVER_ARGS now: $(systemctl show x3_server -p Environment --value | server_args)"
echo "Waiting for the OAK model to load..."
timeout 60 journalctl -u x3_server -f -n 0 | grep -m1 -E "locked decode mode|NN config|init failed" || true
