#!/usr/bin/env bash
# open_ros2_firewall.sh
# Opens only the ports required for ROS2 cross-machine communication.
# Run with sudo on BOTH the Jetson and the laptop.
#
# Usage:
#   sudo bash scripts/open_ros2_firewall.sh [DOMAIN_ID]
#
# Defaults to domain 42 (matches jetson_bringup.sh / laptop_viz.sh).
# Pass "no-gui" as second arg to skip web GUI ports (laptop side).
#
# Example:
#   sudo bash scripts/open_ros2_firewall.sh          # domain 42, all ports
#   sudo bash scripts/open_ros2_firewall.sh 42 no-gui # laptop-only (no GUI ports)

set -e

DOMAIN_ID="${1:-42}"
GUI_PORTS="${2:-yes}"

if [ "$EUID" -ne 0 ]; then
    echo "Run with sudo: sudo bash scripts/open_ros2_firewall.sh"
    exit 1
fi

echo "[firewall] Configuring UFW for ROS2 domain $DOMAIN_ID"
echo ""

# ── FastDDS discovery server ──────────────────────────────────────────────────
# The Jetson's fastdds_discovery service listens on 11811 TCP and UDP.
# The laptop connects via TCPv4 but we open both transports for safety.
ufw allow 11811/tcp comment "FastDDS discovery server (TCP)"
ufw allow 11811/udp comment "FastDDS discovery server (UDP)"
echo "  [OK] 11811/tcp + 11811/udp  (FastDDS discovery)"

# ── TCP RTPS data transport (school WiFi — UDP between subnets is blocked) ────
# fastdds_tcp_server.xml configures the Jetson to listen on 11812/tcp for all
# RTPS data traffic.  The laptop (fastdds_tcp_client.xml) connects to this port
# and also listens on 11813/tcp for return subscriptions (e.g. /goal_pose).
# Open these on BOTH machines so data can flow in both directions.
ufw allow 11812/tcp comment "FastDDS RTPS data — Jetson TCP listener"
ufw allow 11813/tcp comment "FastDDS RTPS data — laptop TCP listener"
echo "  [OK] 11812/tcp              (Jetson RTPS TCP data port)"
echo "  [OK] 11813/tcp              (laptop RTPS TCP data port)"

# ── DDS RTPS data traffic (UDP — kept for same-subnet / wired setups) ─────────
# Port formula (RTPS spec): PB=7400, DG=250
#   metatraffic base = 7400 + domain * 250
# For domain 42: base = 17900. Opening 17880–18200 covers ~150 participants.
BASE=$(( 7400 + DOMAIN_ID * 250 ))
START=$(( BASE - 20 ))
END=$(( BASE + 300 ))
ufw allow "${START}:${END}/udp" comment "DDS RTPS data UDP (domain $DOMAIN_ID)"
echo "  [OK] ${START}:${END}/udp         (DDS RTPS data UDP, domain $DOMAIN_ID)"

# ── Web GUI (Jetson / server side only) ───────────────────────────────────────
if [ "$GUI_PORTS" != "no-gui" ]; then
    ufw allow 8080/tcp comment "X3 web GUI (HTTP)"
    ufw allow 8081/tcp comment "X3 web GUI (WebSocket)"
    echo "  [OK] 8080/tcp + 8081/tcp    (web GUI)"
fi

echo ""
echo "[firewall] UFW rules added. Current status:"
ufw status numbered | grep -E "11811|11812|11813|${START}|8080|8081|Status" | sed 's/^/  /'
echo ""
echo "Done. Run this script on the OTHER machine too (Jetson or laptop)."
echo "No reboot needed — rules take effect immediately."
