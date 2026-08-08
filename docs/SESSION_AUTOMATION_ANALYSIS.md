# Session Pattern Analysis — Jul 31 → Aug 7, 2026

What 8 days of Claude Code sessions in `x3_ws` actually spent their time on, and
which of those patterns are worth turning into skills, scripts, or config.

## Method

Twelve session transcripts (~4.3 MB of JSONL) across `x3_ws`, the
`obsidian-vault` worktree, and the `rosmaster-perf` worktree, plus four subagent
transcripts. Counted tool calls, extracted all 1,777 Bash invocations, and
tallied error strings in tool results.

**Tool mix:** Bash 338, Edit 56, Write 31, Read 20, RemoteTrigger 8,
`jetson-mcp` 5, Agent 4.

That ratio is the headline. Bash is 68% of all tool calls, and the bulk of it is
not doing work — it is re-establishing context that was already true last
session.

---

## Finding 1 — Robot IP churn is the single biggest tax

**Nine distinct robot/laptop IPs appear across 8 days** (DHCP lease churn):

| IP | mentions |
|---|---|
| 10.13.247.131 | 108 |
| 10.13.196.218 | 58 |
| 10.13.149.173 (laptop) | 36 |
| 10.13.245.167 | 29 |
| 10.13.245.176 | 26 |
| 10.13.245.150 | 26 |
| 10.13.197.88 | 23 |
| 10.13.199.34 | 6 |
| 10.13.149.42 | 3 |

Every session opens the same way: the user types the current IP into the prompt,
then Claude spends several turns on `ping` → `ssh` probe → `hostname` →
`systemctl is-active x3_server` before any real work starts.

Worse, the IP is **hardcoded and already stale in committed code**:

- `fetch_bag.sh:30` — `ROBOT="${ROBOT:-jetson@10.13.196.218}"`. The robot has
  been at `.247.131` and `.245.167` since. Running `fetch_bag.sh` with no
  override today fails.
- There is **no `~/.ssh/config` entry** for the jetson at all. Every one of the
  66 ssh invocations spelled out `jetson@<literal-ip>`.

**Automation:** a `scripts/robot_env.sh` that resolves the robot once and caches
it to `~/.x3_robot`, in priority order: `$ROBOT_IP` → cached IP (if it still
answers) → mDNS (`jetson-desktop.local`) → ARP sweep of the /24 matching the
jetson's MAC. Source it from `fetch_bag.sh` and every helper, replacing the
hardcoded default. Add a `Host x3` block to `~/.ssh/config` so `ssh x3` works and
the IP lives in exactly one place.

---

## Finding 2 — The remote-ROS preamble is re-derived every single time

Non-interactive ssh gets no ROS environment (already recorded in memory, still
re-derived in practice). Eighteen commands carry some variant of this preamble,
and the full correct form is long:

```bash
source /opt/ros/humble/setup.bash && source install/local_setup.bash 2>/dev/null
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=42
```

The `unset` of the FastDDS discovery-server vars is the subtle part — omit it and
`ros2 topic list` silently returns nothing. That trap was hit and re-diagnosed
more than once.

**Automation:** `scripts/rjet` — one wrapper that takes a command string, applies
the preamble, and runs it on the resolved host. `rjet 'ros2 topic list'` replaces
a 200-character line that is easy to get subtly wrong.

---

## Finding 3 — sudo password handling gets rediscovered

The robot's sudo password is `1` and is not passwordless. Sessions converge on
`printf '%s\n' 1 | sudo -S -p "" <cmd>` — but only after first probing
`sudo -n true` and getting `SUDO_NEEDS_PASS`. Four `systemctl restart`s and four
`daemon-reload`s in the window each paid this tax.

**Automation, better than encoding the password:** install a sudoers drop-in on
the robot granting NOPASSWD for exactly the commands that need it —
`systemctl {start,stop,restart} x3_server`, `systemctl daemon-reload`, and
`tee /etc/systemd/system/x3_server.service.d/*`. That removes the password from
the loop entirely rather than automating typing it. Fold `sudo -S` fallback into
`rjet` for anything not covered.

---

## Finding 4 — `jetson-mcp` exists, has 21 tools, and is ~unused

`X3_MCP_tools` provides `check_connection`, `check_ros_status`, `check_service_status`,
`manage_service`, `check_rosbag_info`, `diagnose_errors`, `colcon_build`, and 14
more — precisely the operations that were instead hand-rolled as 66 raw ssh
commands. It was called **5 times in 8 days, all `sync_code`**.

Two reasons it loses to raw ssh:

1. Its own README says *"for each of these tools, you will need to provide the IP
   address of the Jetson robot, as it may change between sessions"* — so it
   inherits Finding 1's problem instead of solving it.
2. It had startup trouble (the user asked "why the jetson-mcp won't start" on
   Jul 31, around adding a token).

**Automation:** make the IP optional in the MCP server, defaulting to the same
`~/.x3_robot` cache from Finding 1, and fix the startup path. A working MCP with
zero required arguments would absorb most of the 66 ssh calls, and MCP calls do
not trigger Bash permission prompts.

---

## Finding 5 — The permission allowlist is a graveyard of one-offs

`.claude/settings.local.json` holds **~70 entries**, nearly all hyper-specific
literals that will never match again:

```
Bash(nc -z -w3 10.13.199.34 11811)
Bash(scp /home/kamren/x3_ws/src/drivers_x3.py jetson@10.13.199.34:...)
Bash(awk '{print $2, $3}')
Bash(do)
Bash(wait)
Bash(echo "exit: $?")
Bash(cp /tmp/claude-1000/.../bag_viewer.py /tmp/.../bv.py)
```

`Bash(do)` and `Bash(wait)` are fragments of for-loops that got split by the
permission parser. Each of these represents a real interruption where the user
had to click approve.

**Automation:** run `/fewer-permission-prompts`, then collapse the list to a
small set of broad read-only rules (`Bash(ssh x3 *)`, `Bash(git --no-pager *)`,
`Bash(ros2 topic:*)`, `Bash(jq:*)`) and delete the literals.

---

## Finding 6 — Bag verification is hand-written every time, and it has failed before

The record → verify → fetch cycle ran repeatedly. Verification was ad-hoc each
time: a fresh `ros2 bag info`, a fresh sqlite query, a fresh Python heredoc to
count messages per topic. Memory already records the consequence — **42 bags in
the corpus turned out to be empty**, plus truncated `.zstd` files needing the
sqlite zero-pad repair trick.

`bag_viewer.py` exists and is good for browsing, but there is no *gate* that says
"this recording is valid" at the moment of recording, when re-recording is still
cheap.

**Automation:** `scripts/verify_bag.sh <bag>` — asserts non-zero message count on
every topic in the expected set (with `/oak/*` conditional on
`RECORD_OAK_STEREO`), flags any topic present but empty, detects truncated
`.zstd`, and prints duration and size. Call it from the tail of `record_bag.sh`
so a bad bag is caught on the spot instead of a week later.

---

## Finding 7 — 29 throwaway heredoc analysis scripts

Twenty-nine `<<'EOF'` / `<<'PY'` heredocs were written and discarded. Some are
genuinely one-off. Several are not — `struct.unpack('h', ...)` appears 8 times,
all decoding the Rosmaster serial protocol during the performance work.

**Automation:** promote the recurring ones into `scripts/`. A
`scripts/rosmaster_probe.py` (frame decode + timing histogram) would have been
written once instead of three times.

---

## Finding 8 — No standard session preflight

Sessions open cold and burn 5–15 tool calls answering: is the robot up, what IP,
is `x3_server` running, what topics exist, how much disk is left, what was the
last bag recorded. The answers are cheap and always the same shape.

**Automation:** a `/robot-status` skill that resolves the host and returns all of
it in one pass. Optionally schedule it so the answer is already in context when
the session starts.

---

## Recommended build order

Ordered by (time saved) ÷ (effort). The first three are half a day total and
retire the majority of the observed overhead.

| # | Item | Kind | Addresses |
|---|---|---|---|
| 1 | `scripts/robot_env.sh` + `~/.ssh/config` `Host x3` + fix `fetch_bag.sh:30` | script/config | 1 |
| 2 | `scripts/rjet` remote-ROS wrapper | script | 2, 3 |
| 3 | `/robot-status` preflight skill | skill | 8, 1 |
| 4 | `scripts/verify_bag.sh`, called from `record_bag.sh` | script | 6 |
| 5 | Sudoers NOPASSWD drop-in on the robot | robot config | 3 |
| 6 | `/fewer-permission-prompts` + allowlist cleanup | config | 5 |
| 7 | Default the IP in `jetson-mcp`; fix its startup | MCP fix | 4 |
| 8 | Promote recurring heredocs into `scripts/` | scripts | 7 |

### Two things NOT worth automating

- **A deploy pipeline.** `rsync`/`scp`/`colcon build` appear only 2, 2, and 0
  times respectively. The deploy loop is not where time goes — connection setup
  is. Building CI/CD here would optimize a cold path.
- **More GitHub Actions.** Shellcheck and the big-file guard were added Aug 1 and
  are working. Nothing in the transcripts suggests a gap.

---

*Generated Aug 7, 2026 from session transcripts in
`~/.claude/projects/-home-kamren-x3-ws*/`.*
