#!/usr/bin/env bash
# A/B switch for the OAK's on-device depth rate. Run ON THE ROBOT.
#   bash scripts/oak_depth_fps.sh 15     # depth at 15 fps
#   bash scripts/oak_depth_fps.sh off    # back to the default (30)
# Writes a systemd drop-in setting X3_OAK_DEPTH_FPS and restarts x3_server.
# Why: depth at 30 fps shares the camera chip with the NN, which then gives
# ~5.7 packets/s against 12 requested. Nothing downstream uses more than 15.
set -euo pipefail
DROPIN=/etc/systemd/system/x3_server.service.d/97-oak-depth-fps.conf
fps="${1:?usage: oak_depth_fps.sh <fps 5..30|off>}"
if [ "$fps" = off ]; then
    sudo rm -f "$DROPIN"
else
    case "$fps" in ''|*[!0-9.]*) echo "fps must be a number or 'off'" >&2; exit 1;; esac
    printf '[Service]\nEnvironment=X3_OAK_DEPTH_FPS=%s\n' "$fps" | sudo tee "$DROPIN" >/dev/null
fi
sudo systemctl daemon-reload
sudo systemctl restart x3_server
echo "Waiting for the camera..."
timeout 60 journalctl -u x3_server -f -n 0 | grep -m1 -E "OakDCamera: depth [0-9.]+ fps|init failed" || true
journalctl -u x3_server --since "-90s" --no-pager | grep -E "X3_OAK_DEPTH_FPS" | tail -1 || true
