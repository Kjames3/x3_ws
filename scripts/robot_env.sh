#!/usr/bin/env bash
# robot_env.sh — Resolve the X3 robot's current address, without asking anyone.
#
# Usage (source it; do not execute):
#   source scripts/robot_env.sh          # sets ROBOT_IP / ROBOT_USER / ROBOT_SSH
#   source scripts/robot_env.sh || exit 1
#
# Exports:
#   ROBOT_IP        the resolved address (an IP, or a Tailscale name)
#   ROBOT_USER      ssh user            (default: jetson)
#   ROBOT_SSH       "user@addr", ready to hand to ssh/scp/rsync
#   ROBOT_VIA       which path resolved it (override|tailscale|cache|lan|mdns)
#   ROBOT_SSH_OPTS  array of ssh options with a sane connect timeout
#
# Environment knobs:
#   ROBOT_IP=<addr>     explicit override; trusted without probing
#   ROBOT=<user@addr>   same, in the form the bag scripts already use
#   ROBOT_USER=<user>   ssh user (default jetson)
#   X3_TS_HOST=<name>   Tailscale/MagicDNS name of the robot (default x3)
#   X3_LAN_HOST=<name>  router-DNS name when on the GL.iNet LAN (default x3.lan)
#   X3_MDNS_HOST=<name> mDNS name (default jetson-desktop.local)
#   X3_CACHE=<path>     cache file (default ~/.x3_robot)
#   X3_PROBE_TIMEOUT=<s> per-candidate TCP probe timeout (default 2)
#
# Resolution order — the first candidate that answers a TCP probe on port 22
# wins, and the answer is written back to the cache:
#
#   1. explicit override          trusted, never probed
#   2. Tailscale name             stable across campus subnets; the primary path
#   3. cache file                 the fast path once anything has resolved once
#   4. router DNS (x3.lan)        only resolves while on the GL.iNet bench LAN
#   5. mDNS                       best-effort; rarely crosses campus subnets
#   6. fail                       tells you to run `x3-ip set <IP>`
#
# A TCP probe is used rather than an ssh handshake because it is ~10x cheaper,
# which matters: ~/.ssh/config calls this via ProxyCommand on every connection.
# =============================================================================

ROBOT_USER="${ROBOT_USER:-jetson}"
X3_TS_HOST="${X3_TS_HOST:-x3}"
X3_LAN_HOST="${X3_LAN_HOST:-x3.lan}"
X3_MDNS_HOST="${X3_MDNS_HOST:-jetson-desktop.local}"
X3_CACHE="${X3_CACHE:-$HOME/.x3_robot}"
X3_PROBE_TIMEOUT="${X3_PROBE_TIMEOUT:-2}"

# Diagnostics go to stderr so `ROBOT_IP=$(x3-ip)` stays clean.
_x3_log() { [ -n "${X3_VERBOSE:-}" ] && echo "  [x3-ip] $*" >&2; return 0; }

# Is this host reachable on ssh? The single test every candidate must pass.
_x3_probe() {
    [ -n "$1" ] || return 1
    nc -z -w"$X3_PROBE_TIMEOUT" "$1" 22 >/dev/null 2>&1
}

_x3_cache_read() {
    [ -r "$X3_CACHE" ] || return 1
    # shellcheck disable=SC2002
    sed -n 's/^ip=//p' "$X3_CACHE" | head -n1
}

_x3_cache_write() {
    # $1 = address, $2 = via. Never fail the caller over an unwritable cache.
    {
        echo "ip=$1"
        echo "via=$2"
        echo "updated=$(date -Iseconds)"
    } > "$X3_CACHE" 2>/dev/null || _x3_log "warning: could not write $X3_CACHE"
}

# --- 1. explicit override --------------------------------------------------
# Accepts either ROBOT_IP=<addr> or the bag scripts' ROBOT=<user@addr>.
_x3_resolve() {
    local cand

    if [ -n "${ROBOT_IP:-}" ]; then
        ROBOT_VIA=override
        _x3_log "override: $ROBOT_IP"
        return 0
    fi
    if [ -n "${ROBOT:-}" ]; then
        ROBOT_IP="${ROBOT##*@}"
        [ "$ROBOT" = "$ROBOT_IP" ] || ROBOT_USER="${ROBOT%@*}"
        ROBOT_VIA=override
        _x3_log "override via ROBOT: $ROBOT_IP"
        return 0
    fi

    # --- 2. Tailscale ------------------------------------------------------
    # Preferred: it is the only candidate that survives the robot changing
    # buildings. Ask tailscaled for the address rather than trusting MagicDNS,
    # so this works even with --accept-dns=false on either end.
    if command -v tailscale >/dev/null 2>&1; then
        cand=$(tailscale ip -4 "$X3_TS_HOST" 2>/dev/null | head -n1)
        if [ -n "$cand" ] && _x3_probe "$cand"; then
            ROBOT_IP="$cand"; ROBOT_VIA=tailscale
            _x3_cache_write "$cand" tailscale
            _x3_log "tailscale: $cand"
            return 0
        fi
        _x3_log "tailscale: no answer for $X3_TS_HOST"
    fi

    # --- 3. cache ----------------------------------------------------------
    cand=$(_x3_cache_read)
    if [ -n "$cand" ] && _x3_probe "$cand"; then
        ROBOT_IP="$cand"; ROBOT_VIA=cache
        _x3_log "cache: $cand"
        return 0
    fi
    [ -n "$cand" ] && _x3_log "cache: $cand is stale"

    # --- 4. router DNS -----------------------------------------------------
    # dnsmasq on the GL.iNet serves .lan names, so this resolves instantly on
    # the bench LAN and fails instantly anywhere else. Cheap either way.
    cand=$(getent hosts "$X3_LAN_HOST" 2>/dev/null | awk '{print $1}' | head -n1)
    if [ -n "$cand" ] && _x3_probe "$cand"; then
        ROBOT_IP="$cand"; ROBOT_VIA=lan
        _x3_cache_write "$cand" lan
        _x3_log "router dns: $cand"
        return 0
    fi

    # --- 5. mDNS -----------------------------------------------------------
    if command -v avahi-resolve >/dev/null 2>&1; then
        cand=$(avahi-resolve -4 -n "$X3_MDNS_HOST" 2>/dev/null | awk '{print $2}' | head -n1)
        if [ -n "$cand" ] && _x3_probe "$cand"; then
            ROBOT_IP="$cand"; ROBOT_VIA=mdns
            _x3_cache_write "$cand" mdns
            _x3_log "mdns: $cand"
            return 0
        fi
    fi

    # --- 6. give up --------------------------------------------------------
    return 1
}

if _x3_resolve; then
    ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"
    # Consumed by whoever sources this file (see scripts/rjet). It stays an array
    # so the options survive word-splitting, which is also why it cannot be
    # exported like the scalars below — shellcheck cannot see that use.
    # shellcheck disable=SC2034
    ROBOT_SSH_OPTS=(-o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new)
    export ROBOT_IP ROBOT_USER ROBOT_SSH ROBOT_VIA
    return 0 2>/dev/null || exit 0
else
    cat >&2 <<EOF
  [x3-ip] Could not find the robot.
          Tried: tailscale ($X3_TS_HOST), cache ($X3_CACHE), $X3_LAN_HOST, $X3_MDNS_HOST
          Fix it with:   scripts/x3-ip set <IP>
          Or override:   ROBOT_IP=<IP> <your command>
EOF
    return 1 2>/dev/null || exit 1
fi
