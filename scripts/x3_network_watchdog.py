#!/usr/bin/env python3
"""Bounded Tailscale startup readiness and conservative local recovery."""
import argparse
import json
from pathlib import Path
import subprocess
import time

STATE = Path('/run/x3-network-watchdog/state.json')


def run(*args):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=15)
        return p.returncode == 0, p.stdout + p.stderr
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)


def ready():
    ok, routes = run('ip', '-4', 'route', 'show', 'default')
    return ok and bool(routes.strip()) and run('getent', 'ahostsv4', 'controlplane.tailscale.com')[0]


def healthy():
    ok, output = run('tailscale', 'status', '--json')
    try:
        status = json.loads(output)
        return ok and status.get('BackendState') == 'Running' and status.get('Self', {}).get('Online') is True
    except (ValueError, AttributeError):
        return False


def should_restart(failures, now, last):
    return failures >= 3 and now >= 180 and (last is None or now - last >= 900)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait-ready', action='store_true')
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    if args.wait_ready:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if ready():
                print('Network route and DNS ready', flush=True)
                return
            time.sleep(3)
        print('Readiness deadline reached; letting Tailscale try its own recovery', flush=True)
        return
    if args.check_only:
        print(json.dumps({'network_ready': bool(ready()), 'tailscale_healthy': healthy()}))
        return
    STATE.parent.mkdir(parents=True, exist_ok=True)
    try:
        state = json.loads(STATE.read_text())
    except (OSError, ValueError):
        state = {'failures': 0, 'last_restart': None}
    now = time.monotonic()
    if healthy():
        state['failures'] = 0
    else:
        state['failures'] += 1
        print(f'Tailscale unhealthy: consecutive checks={state["failures"]}', flush=True)
        if should_restart(state['failures'], now, state['last_restart']) and ready():
            # HTTPS verifies DNS, routing, TLS and internet access independently of the tunnel.
            internet, _ = run('curl', '--noproxy', '*', '-4', '-fsS', '--connect-timeout', '5', '--max-time', '10', 'https://controlplane.tailscale.com/key?v=142')
            if internet:
                for command in [('ip', 'route'), ('resolvectl', 'status'), ('tailscale', 'status'), ('journalctl', '-u', 'tailscaled', '-n', '40', '--no-pager')]:
                    print(run(*command)[1], flush=True)
                state['last_restart'] = now
                state['failures'] = 0
                STATE.write_text(json.dumps(state))
                print('Internet works; requesting Tailscale-only restart (15 minute cooldown)', flush=True)
                print(run('systemctl', '--no-block', 'restart', 'tailscaled.service')[1], flush=True)
    STATE.write_text(json.dumps(state))


if __name__ == '__main__':
    main()
