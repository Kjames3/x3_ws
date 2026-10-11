#!/usr/bin/env bash
# Per-site CBF tuning without a code change. Run ON THE ROBOT.
#   bash scripts/cbf_tune.sh gamma=2 deadband=0.22          # set some
#   bash scripts/cbf_tune.sh gamma=2 safe=0.25 deadband=0.22
#   bash scripts/cbf_tune.sh gamma=2 deadband=0 min=0.03    # closed-loop base
#   bash scripts/cbf_tune.sh off                            # back to the defaults
# Writes a systemd drop-in and restarts x3_server. Unset values fall back to
# the defaults in src/server_x3.py (CBF_GAMMA etc.): gamma 1.0, footprint
# 0.16,0.13,0.11,0.11 + margin 0.10, deadband 0.14 m/s, min 0.002 m/s.
#   gamma     1/s; allowed approach speed is about gamma * clearance
#   footprint FRONT,REAR,LEFT,RIGHT: each side's distance from the centre in m
#             (default 0.16,0.13,0.11,0.11, measured), or HALF_LENGTH,HALF_WIDTH,
#             or 'circle' for the old safe-distance circle round the centre
#   margin    m; clearance kept round the footprint (default 0.10)
#   lidar_yaw deg; yaw error of the lidar mount (src/lidar_yaw_calib.py), added
#             to lidar bearings for the CBF
#   safe      m; circle mode only: minimum distance from the centre
#   deadband  m/s; speed the base adds to any nonzero command on this floor
#             (0 with the driver's closed-loop speed_control)
#   min       m/s; slowest command worth sending (0.03 with speed_control)
set -euo pipefail
DROPIN=/etc/systemd/system/x3_server.service.d/98-cbf-tune.conf
[ $# -ge 1 ] || { sed -n 2,21p "$0"; exit 1; }
if [ "$1" = off ]; then
    sudo rm -f "$DROPIN"
else
    body='[Service]'
    for kv in "$@"; do
        key="${kv%%=*}"; val="${kv#*=}"
        if [ "$key" = footprint ]; then
            case "$val" in circle|[0-9.]*,[0-9.]*) ;; *) echo "footprint is FRONT,REAR,LEFT,RIGHT, or HALF_LENGTH,HALF_WIDTH, or circle" >&2; exit 1;; esac
        else
            num="$val"; [ "$key" = lidar_yaw ] && num="${val#-}"     # the only one that may be negative
            case "$num" in ''|*[!0-9.]*) echo "bad value in '$kv'" >&2; exit 1;; esac
        fi
        case "$key" in
            footprint) var=X3_CBF_FOOTPRINT;;
            margin)   var=X3_CBF_MARGIN_M;;
            lidar_yaw) var=X3_LIDAR_YAW_OFFSET_DEG;;
            gamma)    var=X3_CBF_GAMMA;;
            safe)     var=X3_CBF_SAFE_DISTANCE_M;;
            deadband) var=X3_CMD_DEADBAND_MPS;;
            min)      var=X3_CMD_MIN_MPS;;
            *) echo "unknown setting '$key' (gamma, safe, footprint, margin, lidar_yaw, deadband, min)" >&2; exit 1;;
        esac
        body+=$'\n'"Environment=$var=$val"
    done
    printf '%s\n' "$body" | sudo tee "$DROPIN" >/dev/null
fi
sudo systemctl daemon-reload
sudo systemctl restart x3_server
echo "Waiting for the server..."
timeout 90 journalctl -u x3_server -f -n 0 | grep -m1 -E "CBF: gamma" || echo "no 'CBF: gamma' line within 90 s; check journalctl -u x3_server"
