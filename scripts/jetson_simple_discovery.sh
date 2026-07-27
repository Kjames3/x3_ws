#!/usr/bin/env bash
# jetson_simple_discovery.sh — switch the Jetson's ROS2 services off the FastDDS
# discovery server and onto plain Simple discovery, so a laptop running RViz can
# actually RECEIVE topic data cross-machine.
#
# Why: the FastDDS discovery server's EDP forwarding between clients is broken in
# this setup — the laptop sees all topics but publishers never learn the laptop's
# subscriptions exist, so /tf, /scan, /map data never flows.  Multicast is blocked
# across the school subnets, but plain unicast UDP between the machines works, so
# Simple discovery driven by the laptop's unicast initial-peers profile delivers
# data reliably.  (See scripts/laptop_viz.sh for the laptop side.)
#
# This does two things, both reversible:
#   1. Patches server_x3.py to honour X3_SIMPLE_DISCOVERY=1 (it otherwise pins the
#      discovery server in code for itself and every node it launches).
#   2. Installs systemd drop-in overrides that set X3_SIMPLE_DISCOVERY=1 and unset
#      the discovery-server / TCP-profile env (original unit files left intact).
#
# Run ON THE JETSON, with sudo:
#   sudo bash jetson_simple_discovery.sh            # apply
#   sudo bash jetson_simple_discovery.sh --revert   # undo
#
# After applying, x3_server relaunches the whole ROS bringup, so allow ~40-90s.

set -e

SERVER_PY=/home/jetson/x3_ws/src/server_x3.py
DROPIN_X3=/etc/systemd/system/x3_server.service.d/10-simple-discovery.conf
DROPIN_ORB=/etc/systemd/system/orbbec_depth.service.d/10-simple-discovery.conf

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: run with sudo (needs to write /etc/systemd/system and restart services)." >&2
    exit 1
fi

if [ "${1:-}" = "--revert" ]; then
    rm -f "$DROPIN_X3" "$DROPIN_ORB"
    if [ -f "${SERVER_PY}.predisc.bak" ]; then
        cp "${SERVER_PY}.predisc.bak" "$SERVER_PY"
        echo "[jetson] Restored server_x3.py from backup."
    fi
    systemctl daemon-reload
    systemctl restart x3_server orbbec_depth
    echo "[jetson] Reverted: discovery-server config restored."
    exit 0
fi

# ── 1. Patch server_x3.py to support X3_SIMPLE_DISCOVERY ──────────────────────
# Idempotent: only edits if the opt-out is not already present.  Truncating the
# existing file in-place preserves its owner (jetson).
[ -f "${SERVER_PY}.predisc.bak" ] || cp "$SERVER_PY" "${SERVER_PY}.predisc.bak"
python3 - "$SERVER_PY" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
if 'X3_SIMPLE_DISCOVERY' not in s:
    old_main = ("if ROS2_MODE:\n"
                "    os.environ['ROS_DISCOVERY_SERVER'] = _resolve_discovery_server()")
    new_main = (
        "SIMPLE_DISCOVERY = os.environ.get('X3_SIMPLE_DISCOVERY', '').lower() in ('1', 'true', 'yes', 'on')\n\n"
        "if ROS2_MODE:\n"
        "    if SIMPLE_DISCOVERY:\n"
        "        os.environ.pop('ROS_DISCOVERY_SERVER', None)\n"
        "    else:\n"
        "        os.environ['ROS_DISCOVERY_SERVER'] = _resolve_discovery_server()")
    assert old_main in s, "main discovery block not found — server_x3.py differs from expected"
    s = s.replace(old_main, new_main)

    old_child = ("    child_env = os.environ.copy()\n"
                 "    child_env['ROS_DISCOVERY_SERVER'] = os.environ.get('ROS_DISCOVERY_SERVER', '127.0.0.1:11811')")
    new_child = ("    child_env = os.environ.copy()\n"
                 "    if SIMPLE_DISCOVERY:\n"
                 "        child_env.pop('ROS_DISCOVERY_SERVER', None)\n"
                 "    else:\n"
                 "        child_env['ROS_DISCOVERY_SERVER'] = os.environ.get('ROS_DISCOVERY_SERVER', '127.0.0.1:11811')")
    n = s.count(old_child)
    s = s.replace(old_child, new_child)
    open(p, 'w').write(s)
    print(f"[patch] server_x3.py patched (main + {n} child-launch sites)")
else:
    print("[patch] server_x3.py already supports X3_SIMPLE_DISCOVERY — skipping")
PY
python3 -m py_compile "$SERVER_PY" && echo "[jetson] server_x3.py compiles OK"

# ── 2. Install systemd drop-ins ──────────────────────────────────────────────
mkdir -p "$(dirname "$DROPIN_X3")" "$(dirname "$DROPIN_ORB")"

cat > "$DROPIN_X3" <<'EOF'
[Service]
# Use plain Simple discovery (no FastDDS discovery server / TCP profile).
# X3_SIMPLE_DISCOVERY=1 makes server_x3.py keep ROS_DISCOVERY_SERVER unset for
# itself and every node it launches; the unset below clears the inherited env.
Environment=X3_SIMPLE_DISCOVERY=1
ExecStart=
ExecStart=/bin/bash -c 'unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE && source /opt/ros/humble/setup.bash && source /home/jetson/x3_ws/install/local_setup.bash 2>/dev/null || true && exec python3 /home/jetson/x3_ws/src/server_x3.py $SERVER_ARGS'
EOF

cat > "$DROPIN_ORB" <<'EOF'
[Service]
ExecStart=
ExecStart=/bin/bash -c 'unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE && source /opt/ros/humble/setup.bash && source /home/jetson/x3_ws/install/local_setup.bash 2>/dev/null || true && exec python3 /home/jetson/EE_244_Final_Project/robot/orbbec_depth_node.py'
EOF

# ── 3. Reload + restart ──────────────────────────────────────────────────────
systemctl daemon-reload
echo "[jetson] Restarting x3_server (relaunches ROS bringup; ~40-90s) ..."
systemctl restart x3_server
echo "[jetson] Restarting orbbec_depth ..."
systemctl restart orbbec_depth

sleep 3
echo "[jetson] Service state:"
systemctl is-active x3_server orbbec_depth || true
echo "[jetson] Done. All nodes now use Simple discovery."
