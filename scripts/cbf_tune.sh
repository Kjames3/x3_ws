#!/usr/bin/env bash
# Per-site CBF tuning without a code change. Run ON THE ROBOT.
#   bash scripts/cbf_tune.sh gamma=2 deadband=0.22          # set some
#   bash scripts/cbf_tune.sh gamma=2 safe=0.25 deadband=0.22
#   bash scripts/cbf_tune.sh off                            # back to the defaults
# Writes a systemd drop-in and restarts x3_server. Defaults: gamma 1.0,
# safe 0.30 m, deadband 0.14 m/s (see CBF_GAMMA etc. in src/server_x3.py).
#   gamma     1/s; allowed approach speed is about gamma * clearance
#   safe      m; minimum distance from the robot's centre to an obstacle point
#   deadband  m/s; speed the base adds to any nonzero command on this floor
set -euo pipefail
DROPIN=/etc/systemd/system/x3_server.service.d/98-cbf-tune.conf
[ $# -ge 1 ] || { sed -n 2,11p "$0"; exit 1; }
if [ "$1" = off ]; then
    sudo rm -f "$DROPIN"
else
    body='[Service]'
    for kv in "$@"; do
        key="${kv%%=*}"; val="${kv#*=}"
        case "$val" in ''|*[!0-9.]*) echo "bad value in '$kv'" >&2; exit 1;; esac
        case "$key" in
            gamma)    var=X3_CBF_GAMMA;;
            safe)     var=X3_CBF_SAFE_DISTANCE_M;;
            deadband) var=X3_CMD_DEADBAND_MPS;;
            *) echo "unknown setting '$key' (gamma, safe, deadband)" >&2; exit 1;;
        esac
        body+=$'\n'"Environment=$var=$val"
    done
    printf '%s\n' "$body" | sudo tee "$DROPIN" >/dev/null
fi
sudo systemctl daemon-reload
sudo systemctl restart x3_server
echo "Waiting for the server..."
timeout 90 journalctl -u x3_server -f -n 0 | grep -m1 -E "CBF: gamma" || echo "no 'CBF: gamma' line within 90 s; check journalctl -u x3_server"
