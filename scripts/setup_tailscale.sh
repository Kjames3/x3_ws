#!/usr/bin/env bash
# setup_tailscale.sh — Install and join Tailscale, so the robot keeps one
#                      address for the whole deployment no matter which campus
#                      subnet it lands on.
#
# Run it on BOTH machines (it detects which one it is from the hostname):
#   bash scripts/setup_tailscale.sh            # auto-detect role
#   bash scripts/setup_tailscale.sh robot      # force robot role
#   bash scripts/setup_tailscale.sh laptop     # force laptop role
#
# Needs sudo, and `tailscale up` prints a URL you must open once to authorise
# the machine. After that it re-authenticates itself across reboots forever.
#
# Why the robot is joined with --accept-dns=false:
#   MagicDNS otherwise rewrites /etc/resolv.conf to point at 100.100.100.100.
#   The robot does not need to *resolve* tailnet names — only the laptop does —
#   and leaving its resolver untouched keeps ROS2, apt and the camera stack on
#   exactly the DNS path they use today. The laptop DOES accept DNS, which is
#   what makes plain `ssh x3` work there.
#
# Known interaction, not fixed here: FastDDS enumerates every interface, so it
# will discover tailscale0 once this is up. If cross-machine ROS2 discovery gets
# slower or flakier afterwards, whitelist the real interface in the FastDDS
# profile rather than removing Tailscale.
# =============================================================================

set -euo pipefail

ROLE="${1:-auto}"
TS_HOSTNAME_ROBOT="${TS_HOSTNAME_ROBOT:-x3}"
TS_HOSTNAME_LAPTOP="${TS_HOSTNAME_LAPTOP:-legion}"

if [ "$ROLE" = "auto" ]; then
    # The robot is the only one of the two running as user `jetson`.
    if [ "$(id -un)" = "jetson" ] || [ -d /etc/nv_tegra_release ] || [ -f /etc/nv_tegra_release ]; then
        ROLE=robot
    else
        ROLE=laptop
    fi
    echo "  Detected role: $ROLE  (override: $0 robot|laptop)"
fi

case "$ROLE" in
    robot)  TS_HOSTNAME="$TS_HOSTNAME_ROBOT"; UP_FLAGS=(--accept-dns=false) ;;
    laptop) TS_HOSTNAME="$TS_HOSTNAME_LAPTOP"; UP_FLAGS=() ;;
    *)      echo "unknown role '$ROLE' (expected robot or laptop)" >&2; exit 2 ;;
esac

echo "======================================================================="
echo "  Tailscale setup — role=$ROLE, tailnet hostname=$TS_HOSTNAME"
echo "======================================================================="

# --- install ---------------------------------------------------------------
if command -v tailscale >/dev/null 2>&1; then
    echo "  tailscale already installed: $(tailscale version | head -n1)"
else
    echo "  Installing tailscale (needs sudo)..."
    # The official installer picks the right repo for Ubuntu 22.04 on both
    # x86_64 (laptop) and arm64 (Jetson), so one path covers both machines.
    curl -fsSL https://tailscale.com/install.sh | sh
fi

sudo systemctl enable --now tailscaled

# --- join ------------------------------------------------------------------
# --ssh is deliberately NOT enabled: key-based ssh already works, and Tailscale
# SSH would put a second authentication path in front of the robot.
if tailscale status >/dev/null 2>&1; then
    echo "  Already joined to a tailnet:"
    tailscale status | head -n5
    echo
    echo "  To rename this machine to '$TS_HOSTNAME', run:"
    echo "      sudo tailscale up --hostname=$TS_HOSTNAME ${UP_FLAGS[*]:-}"
else
    echo "  Joining the tailnet. Open the URL it prints to authorise."
    sudo tailscale up --hostname="$TS_HOSTNAME" "${UP_FLAGS[@]}"
fi

echo
echo "  ---------------------------------------------------------------------"
echo "  This machine:  $(tailscale ip -4 2>/dev/null | head -n1)"
if [ "$ROLE" = "laptop" ]; then
    echo
    echo "  Next, on the ROBOT:   bash scripts/setup_tailscale.sh robot"
    echo "  Then from here:       ssh x3"
    echo "                        scripts/x3-ip status"
    echo
    echo "  Check you got a direct connection rather than a relay:"
    echo "                        tailscale status | grep $TS_HOSTNAME_ROBOT"
    echo "  'direct' is good. 'relay' still works but is slower for big"
    echo "  transfers like pulling bags."
else
    echo
    echo "  Enable MagicDNS once, in the admin console, so the laptop can use"
    echo "  the bare name 'x3':   https://login.tailscale.com/admin/dns"
fi
echo "  ---------------------------------------------------------------------"
