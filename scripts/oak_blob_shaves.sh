#!/usr/bin/env bash
# A/B switch for which compiled blob the OAK loads. Run ON THE ROBOT.
#   bash scripts/oak_blob_shaves.sh 4     # use <model>_4shave.blob
#   bash scripts/oak_blob_shaves.sh off   # back to the default 8-shave blob
# Writes a systemd drop-in setting X3_OAK_BLOB_SHAVES and restarts x3_server.
# Why: the pipeline runs 2 inference threads and the device asks for blobs
# compiled for 4 SHAVEs; the 8-shave pose blob gives ~5.7 packets/s.
set -euo pipefail
DROPIN=/etc/systemd/system/x3_server.service.d/98-oak-blob-shaves.conf
n="${1:?usage: oak_blob_shaves.sh <shaves|off>}"
if [ "$n" = off ]; then
    sudo rm -f "$DROPIN"
else
    case "$n" in ''|*[!0-9]*) echo "shaves must be a number or 'off'" >&2; exit 1;; esac
    printf '[Service]\nEnvironment=X3_OAK_BLOB_SHAVES=%s\n' "$n" | sudo tee "$DROPIN" >/dev/null
fi
sudo systemctl daemon-reload
sudo systemctl restart x3_server
echo "Waiting for the camera..."
timeout 60 journalctl -u x3_server -f -n 0 | grep -m1 -E "NN config|init failed" || true
journalctl -u x3_server --since "-90s" --no-pager | grep -E "X3_OAK_BLOB_SHAVES|compiled for [0-9]+ shaves" | tail -2 || true
