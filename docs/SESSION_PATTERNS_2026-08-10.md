# Session patterns → skills, automations, and token cost (Aug 7–10, 2026)

Analysis of 28 transcripts (22 sessions + 6 subagents) covering 2026-08-07 → 2026-08-10:
106 user prompts, 4,811 API calls, 2,388 tool calls, 677M input-side tokens.

This extends [SESSION_AUTOMATION_ANALYSIS.md](SESSION_AUTOMATION_ANALYSIS.md) and
[AUTOMATION_PLAN_1-3.md](AUTOMATION_PLAN_1-3.md) (branch `worktree-session-pattern-analysis`,
written 2026-08-08). The headline finding is that the top recommendation from that
analysis **was built and then stranded**.

---

## Part 1 — Patterns worth turning into skills or automations

### 1. Merge `worktree-robot-ip-automation` into main — the fix already exists

Commit `ea5abbd` ("feat(ops): resolve the robot's address instead of asking for it")
adds `scripts/x3-ip`, `scripts/robot_env.sh`, `scripts/setup_tailscale.sh`, and a
CLAUDE.md section headed **"Never ask for the robot's IP address, and never hardcode
one."** It has never been merged. `main` is at `2b26d26` and contains none of it.

The laptop-side install is live and working *right now*:

```
$ x3-ip status
  cache:     100.64.52.55  (via tailscale, updated 2026-08-10T18:11:13-07:00)
  reachable: yes (port 22 open)
$ tailscale status
  100.64.52.55  x3  active; direct 10.13.245.167:41641
```

But every new session reads `main`'s CLAUDE.md, which says nothing about it. Result
over the four days:

| | count |
|---|---|
| Bash calls hardcoding `10.13.245.167` | **602 of 1,707 (35%)** |
| Bash calls using the `x3` alias | **3** |
| Times the user pasted the IP into a prompt | **8** |
| `Host key verification failed` in tool output | 9 |

Those 8 pastes are the exact failure the branch was written to eliminate. The 9
host-key failures are what the `ProxyCommand` alias fixes as a side effect (the key
gets stored against the name `x3`, not a rotating IP).

**Action:** merge `ea5abbd` into `main`. It is a single commit, additive only
(374 insertions, 0 deletions, 4 files), and `git merge-tree` reports **zero
conflicts**. This is the highest-value change in this document and it costs one merge.

### 2. `rjet` — the remote-ROS command wrapper (planned, never built)

Item 2 of `AUTOMATION_PLAN_1-3.md`. Every remote ROS command re-derives the same
preamble by hand:

| fragment re-typed | bash calls |
|---|---|
| `source /opt/ros/humble/setup.bash` (+ `install/setup.bash`) | 175 |
| `export ROS_DOMAIN_ID=42` | 154 |
| `unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE` | 44 |

643 ssh invocations averaging **312 characters each** — 201k characters (~50k tokens)
of command text that is mostly boilerplate, with ~400 preamble fragments among them.
The `unset` is the trap: omit it and `ros2 topic list` silently returns empty rather
than erroring. `ros2: command not found` appeared 5 times — the trap firing when the
preamble was forgotten (non-interactive ssh has no ROS).

`rjet 'ros2 topic hz /scan'` replaces all of it and makes the trap unforgettable.

### 3. `/robot-status` preflight skill (planned, never built)

Item 3 of the same plan. Sessions repeatedly open by re-establishing the same facts:
is the robot reachable, is `x3_server` up, is Nav2/slam_toolbox running, and — the
one that actually bites — **is the ROS data plane delivering at all**. Memory records
that stall as hitting roughly 1 startup in 3, with a fixed remedy (restart
`x3_server`, then `foxglove_bridge`), yet it is rediscovered conversationally each time.

Transcript evidence of the preflight dance: "nav2 should be up I clicked the launch
nav2 button about 30 seconds ago", "I restarted the x3_server service", "They should
both be connected can you confirm this". One scripted check collapses 6–10 turns.

Fold the Foxglove runbook into it — 8 websocket/Foxglove errors appeared in the
window, and memory already holds the three facts that resolve them (bridge 3.2.5
accepts only subprotocol `foxglove.sdk.v1`; check the `ssh -L` tunnel first; restart
the bridge *after* `x3_server`).

### 4. `/x3-deploy` — sync, restart, verify

A closed loop that recurs in almost every hardware session and is hand-assembled each
time: 140 `scp` calls, 9 `jetson-mcp sync_code` calls, 75 bash calls referencing
`x3_server`, 17 `colcon build`. The user has to ask "Just to confirm the code is
pushed to the robot and I can proceed with short trial?" and "Im assuming that you
synced the updated code to the physical robot".

A skill that scp's the changed files, restarts the service, and verifies it came back
healthy would also retire the `sudo -S` password hack — `[sudo] password` prompts hit
tool output **8 times**, and the user has volunteered the password (`1`) in **three
separate sessions** because it isn't recorded anywhere the agent reads.

### 5. GL.iNet router watchdog — automate on the router, not in a session

"The router failed to reconnect" was a full session on Aug 9, was patched, and
**recurred on Aug 10**. This is not a skill candidate; it is a device-side automation
plus a scheduled check. Memory already has the key constraint (`ifdown/ifup wwan` can
never fix `NO_DEVICE`; it needs `wifi up` / repeater restart).

### 6. The meta-pattern: finished work is stranded on unmerged branches

`main` has **no `docs/` directory at all**. Both prior analysis documents
(`SESSION_AUTOMATION_ANALYSIS.md`, `AUTOMATION_PLAN_1-3.md`) live only on
`worktree-session-pattern-analysis`, and the ops tooling lives only on
`worktree-robot-ip-automation`. `git worktree list` shows **15 worktrees**, most on
branches never merged, several holding hardware-verified fixes:
`a5d8b54` (battery calibration), `67c7403` (Nav2 localization), `ca99bcd` (TTC gate).

This is why finding #1 exists: the fix was written, was correct, and had no effect,
because nothing reads an unmerged branch. Sessions each start a fresh worktree and end
by pushing a branch, and no step merges. Worth a periodic sweep — or a habit of
opening a PR at the end of each session rather than only pushing.

### Not worth building

**More `jetson-mcp` tooling.** It exposes 21 tools covering exactly the operations
being hand-rolled as raw ssh, and was called ~20 times in four days (mostly
`sync_code`). It still requires the IP to be passed in, so it inherits problem #1
instead of solving it. Fix the resolver, not the MCP server.

---

## Part 2 — Token usage and how to get more work per session

### The measurement

| metric | 4-day total |
|---|---|
| input-side tokens | 676,950,436 |
| output tokens | 3,980,418 |
| API calls | 4,811 |
| **average context per API call** | **140,709** |
| cache hit rate | **97.4%** |
| cache creation (billed 1.25×) | 2.6% |

**Caching is already near-optimal — there is nothing to win there.** Cost is
essentially `API calls × context size`, because every turn resends the whole context.
Total tool output for the four days was only ~700k tokens; it was re-sent enough times
to produce 677M. That ratio is where the levers are.

### Lever A — session length dominates everything else

| session | input tokens | API calls | avg context |
|---|---|---|---|
| velocity-estimator-fixes | **297.0M** | 1,166 | **254,679** |
| lidar-3d-mapping-plan | 111.2M | 658 | 169,066 |
| offline-fixtures | 34.4M | 294 | 116,971 |
| nav2-mapping-fixes | 32.6M | 323 | 101,042 |

One session consumed **44% of the four-day total**. It ran at an average context of
255k — nearly 2× the fleet average — and hit `/compact`. Three `/compact` events
occurred in the window.

Because cost is quadratic in a long session (each new turn pays for all prior turns),
**ending a session when the task changes is worth more than every other optimization
in this document combined.** The velocity-estimator session covered MLP review, TTC
gating, map recovery, Nav2 tuning, and hardware trials in one thread. Split at the
task boundary and the same work costs a fraction.

### Lever B — turn count

1,707 bash calls against 106 user prompts is **16 bash calls per prompt**, and 1,662
of the 1,707 were unique one-shots. Each ssh round-trip is a full turn that resends
the entire context. Items 1–4 above attack this directly; so does batching independent
probes into a single call rather than issuing them serially.

### Lever C — read narrowly

`Read` averaged **5,830 chars/call** versus Bash's 1,160 — 5× the cost per call.
`src/velocity_estimator.py` (35k chars ≈ 9k tokens) was read **17 times**;
`ab_comparison_test.py` (59.5k chars) and `Rosmaster_Lib.py` (60.1k chars) were each
read whole more than once. Grep for the symbol first, then `Read` with `offset`/`limit`.

### Lever D — images are expensive

A single PNG read (`map_compare.png`) cost 79.7k chars ≈ 20k tokens; a second
(`apartment_view.png`) 21.1k. Two screenshots ≈ 25k tokens of permanent context.
Worth it for a real visual question, wasteful as a routine check — prefer a numeric
summary of the map.

### Lever E — fan out more

Only **6 `Agent` calls** in four days, despite a standing preference for parallel
subagents on broad investigations. The `rosmaster-perf` session is the model: four
subagents did the work, and the parent session finished at 22M tokens with only 214
messages, because subagent context never lands in the parent. Compare the
velocity-estimator session's 297M for a comparable amount of investigation done inline.

### Lever F — permission allowlist hygiene

`.claude/settings.local.json` holds 70 allow entries, **42 of them non-wildcard
literals** that can never match again — including `Bash(do)` and `Bash(wait)`, where
the parser split a for-loop. Each miss is an interruption rather than a token cost.
The `/fewer-permission-prompts` skill regenerates this properly.

---

## Recommended order

1. ~~**Merge `ea5abbd`**~~ — **done** on this branch (`worktree-session-patterns-aug10`).
   Still needs to reach `main`; sweep the other unmerged branches at the same
   time (finding #6).
2. ~~Build `rjet`~~ — **done**, verified against the live robot (71 topics returned,
   `--raw` confirmed ROS-free, exit codes propagate, `--sudo` runs as root).
3. Build `/robot-status` (item 3), with the Foxglove runbook folded in.
4. Build `/x3-deploy`, and record the sudo password somewhere the agent reads.
5. Run `/fewer-permission-prompts` to rebuild the allowlist.
6. Adopt as working habits: split sessions at task boundaries; fan out investigation
   to subagents; grep before reading whole files.

### Known follow-up

`fetch_bag.sh:30` still defaults to `jetson@10.13.196.218` — the stale IP the Aug 8
analysis flagged. It is untracked in the main checkout, so it could not be fixed from
an isolated worktree. It should become `source scripts/robot_env.sh`.
