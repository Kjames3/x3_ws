# Implementation Plan — Automation Items 1–3

Companion to [SESSION_AUTOMATION_ANALYSIS.md](SESSION_AUTOMATION_ANALYSIS.md).
Covers build-order items 1–3. Items 4 (`verify_bag.sh`) and 5 (sudoers
NOPASSWD) are deferred by decision; items 6–8 remain unscheduled.

## Context

Eight days of transcripts showed that most session time goes to re-establishing
robot connectivity rather than doing robot work: Bash is 68% of all tool calls,
the robot appeared at nine IPs, and `fetch_bag.sh:30` still defaults to an IP
that went stale weeks ago. These three items remove that overhead.

### Constraint discovered while planning

The robot does **not** live on a stable subnet. Its nine historical addresses
span four different /22 blocks (`10.13.149.x`, `10.13.196.x`, `10.13.245.x`,
`10.13.247.x`), and this laptop's own address has since moved to
`10.13.45.94/22`. Campus DHCP moves both machines between blocks.

Consequences for the design:

- **A subnet sweep is not viable.** It would scan the wrong /22 most of the
  time, and `nmap`/`arp-scan` are not installed. Do not build one.
- **mDNS is best-effort only.** Campus networks rarely forward mDNS across
  subnets. `jetson-desktop.local` does not resolve right now. `avahi-resolve`
  is installed and worth one cheap attempt, but must never be the only path.
- Available discovery primitives on this laptop: `avahi-resolve`, `nc`, `ping`,
  `ssh`, `rsync`.

**Chosen strategy (user decision): cache now, Tailscale later.** Build the cache
layer so it works today, and have the resolver check a Tailscale name *first* so
Tailscale can be adopted later without touching any other file.

---

## Item 1 — Robot address resolution

### 1a. `scripts/robot_env.sh` (new, sourceable)

Defines `ROBOT_IP`, `ROBOT_USER` (default `jetson`), `ROBOT_SSH`
(`user@ip`), and `ROBOT_SSH_OPTS`. Sourcing it must be side-effect-free apart
from refreshing the cache.

Resolution order — first path that answers a **2-second TCP probe to port 22**
wins (`nc -z -w2 "$ip" 22`; cheaper than an ssh handshake):

| # | Source | Notes |
|---|---|---|
| 1 | `$ROBOT_IP` / `$ROBOT` already set | explicit override; trust it, skip probing |
| 2 | Tailscale name `${X3_TS_HOST:-jetson-x3}` | only if the `tailscale` binary exists — the forward hook |
| 3 | Cache file `~/.x3_robot` | the common case |
| 4 | mDNS `avahi-resolve -n jetson-desktop.local` | best-effort, often fails off-subnet |
| 5 | fail | exit non-zero with `x3-ip set <IP>` in the message |

On success via paths 2–4, rewrite the cache. Cache format is
`key=value` lines (`ip=`, `updated=`, `via=`) so it stays greppable and easy to
hand-edit.

### 1b. `scripts/x3-ip` (new, small CLI)

- `x3-ip` / `x3-ip get` — print the resolved IP, exit 1 if unresolvable
- `x3-ip set <IP>` — probe, then write the cache (this is what you run when you
  would previously have pasted the IP into a prompt)
- `x3-ip status` — cached IP, its age, reachability, which path resolved it
- `x3-ip clear` — drop the cache

### 1c. `~/.ssh/config` — `Host x3`

Outside the repo, so it is a documented manual step, not something CI covers.

```
Host x3
    User jetson
    StrictHostKeyChecking accept-new
    ProxyCommand sh -c 'exec nc $(%d/x3_ws/scripts/x3-ip) 22'
```

The `ProxyCommand` is what makes this work across IP changes: the alias resolves
through `x3-ip` at connect time, so `ssh x3` keeps working when the lease moves.
It also **fixes the recurring `Host key verification failed` errors** — the host
key is now stored against the name `x3` rather than against an IP that changes
weekly.

Requires `x3-ip` to be fast; a cache hit is one `nc` probe, so it is.

### 1d. Fix `fetch_bag.sh:30`

```bash
ROBOT="${ROBOT:-jetson@10.13.196.218}"     # stale since ~Jul 31
```
becomes a `robot_env.sh` source with `ROBOT="${ROBOT:-x3}"`, keeping the
existing `ROBOT=` override contract intact. Its `ROS_SETUP` default should be
replaced by the shared preamble from Item 2 so there is one definition.

**Note:** `fetch_bag.sh` is currently **untracked** and `record_bag.sh` has
uncommitted local edits. Commit both before editing, or the work lands on a
moving base.

**Out of scope:** `record_bag.sh`'s `DEST_HOST` detection (lines 313–337) solves
the *reverse* direction — robot finding the laptop — and already auto-detects via
`SSH_CONNECTION`. Leave it alone.

---

## Item 2 — `scripts/rjet` remote-ROS wrapper

One place that owns the preamble whose omission silently breaks `ros2` over ssh:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/x3_ws/install/setup.bash" 2>/dev/null
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
```

The `unset` is the load-bearing line — leave it out and `ros2 topic list`
returns empty with no error.

Interface:

```
rjet 'ros2 topic list'                      # preamble + run on robot
rjet --raw 'ls ~/bags'                      # no ROS, just ssh
rjet --sudo 'systemctl restart x3_server'   # sudo path
rjet -- ros2 topic hz /scan                 # argv form, no quoting
```

Host comes from `robot_env.sh`. Remote exit code propagates to the caller.

`--sudo` pipes a password via `printf '%s\n' "$pw" | sudo -S -p ''`, reading
from `$X3_SUDO_PASS` or `~/.x3_sudo` (mode 600). **The password is never
committed** — the file lives outside the repo and the script errors with
instructions if it is absent. Item 5 (sudoers NOPASSWD drop-in) retires this
path entirely; `--sudo` should degrade to plain `sudo -n` once that lands.

---

## Item 3 — `/robot-status` preflight skill

### 3a. `scripts/robot_status.sh`

**One** ssh round trip that answers everything a session currently opens by
asking piecemeal:

- resolved IP and which path found it; hostname; uptime
- `x3_server`: active/enabled, MainPID, full cmdline (reveals whether
  `--oak-ros-publish` is on), and any drop-ins under
  `/etc/systemd/system/x3_server.service.d/`
- ROS: topic count, plus presence of `/scan`, `/odom`, `/oak/*`
- hardware: `/dev/ttyCH341USB0`, `/dev/ttyUSB0`, OAK-D on USB
- disk free on `/`, SoC temperature
- last bag from `~/.x3_last_bag` with its size

Emits `key: value` lines — readable by you, trivially parseable by me. Exits
non-zero when the robot is unreachable so callers can branch.

### 3b. `.claude/skills/robot-status/SKILL.md`

Project-level (not `~/.claude/skills/`, where `dream` lives) so it is versioned
alongside the scripts it calls and travels with the repo. Frontmatter follows
the `dream` pattern: `name` + a `description` that fires on "is the robot up",
"robot status", and at the start of any robot-touching task.

Body: run `scripts/robot_status.sh`, interpret the output, and on failure walk
the `x3-ip` recovery path rather than re-deriving ssh probes by hand.

---

## Repo housekeeping folded in

- **CI does not lint extensionless scripts.** `.github/workflows/checks.yml`
  collects `git ls-files '*.sh'`, so `scripts/rjet` and `scripts/x3-ip` would
  slip past shellcheck. Extend the collector to also pick up files whose first
  line matches a bash shebang. All new scripts must pass
  `shellcheck --severity=warning` and `bash -n`; `.shellcheckrc` already
  disables SC1090/SC1091, which the ROS `source` lines would otherwise trip.
- **Gitignore `.claude/settings.local.json`.** It is local machine state (the 70
  one-off permission literals from Finding 5) and should not be committed
  alongside the skill directory.

---

## Verification

Each item is testable without the robot except where noted.

**Item 1 — no robot needed**
```bash
x3-ip clear && x3-ip status          # reports no cache, exits non-zero
x3-ip set 10.13.45.94                # probe this laptop's own :22... expect
                                     # failure (no sshd) — proves probing is real
ROBOT_IP=1.2.3.4 bash -c 'source scripts/robot_env.sh; echo $ROBOT_SSH'
                                     # override path, no probe
```

**Item 1 — with robot on**
```bash
x3-ip set <current-ip> && ssh x3 hostname     # expect: jetson-desktop
x3-ip status                                  # via=cache, age fresh
```
Then reboot the robot onto a new lease and confirm `ssh x3` still connects with
**no** host-key warning — that is the real regression test for 1c.

**Item 2 — with robot on**
```bash
rjet 'ros2 topic list' | wc -l       # non-zero; empty means the unset broke
rjet --raw 'echo $ROS_DISTRO'        # empty, proving --raw skips the preamble
rjet 'exit 7'; echo $?               # 7 — exit code propagates
```

**Item 3**
```bash
bash scripts/robot_status.sh         # one round trip, all fields
```
Then in a fresh Claude session ask "is the robot up?" and confirm the skill
fires instead of hand-rolled ssh probes.

**Regression — the thing that was actually broken**
```bash
./fetch_bag.sh 30                    # must resolve without ROBOT= being set
```

**CI**
```bash
shellcheck --severity=warning scripts/robot_env.sh scripts/rjet scripts/x3-ip scripts/robot_status.sh
bash -n scripts/rjet scripts/x3-ip
```

---

## Deferred

| Item | Why deferred |
|---|---|
| 4 — `verify_bag.sh` | user decision; note 42 corpus bags are already empty, so this stays worth doing |
| 5 — sudoers NOPASSWD | user decision; until then `rjet --sudo` carries the password path |
| 6 — permission allowlist cleanup | unscheduled |
| 7 — `jetson-mcp` IP default | unscheduled; largely obsoleted if it adopts `~/.x3_robot` |
| 8 — promote recurring heredocs | unscheduled |
