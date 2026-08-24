---
id: A-47
title: "Exported Track Position Is the Stale Body-Frame Centroid From the Last Matched Frame While the Re-Referenced One Sits Unused"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-47"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-47 — Exported Track Position Is the Stale Body-Frame Centroid From the Last Matched Frame While the Re-Referenced One Sits Unused

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Hoist one existing three-line block above the gates and change one tuple unpack; the corrected position is already computed and discarded)
- **Problem:** `ObstacleTracker` writes `track['centroid']` — the **body-frame** $(x, y, z)$ triple — only on a frame where the track matched a detection (`src/velocity_estimator.py:89`, and at creation, line 112). There is no `else` branch: an unmatched track keeps the tuple it was given in whatever body frame the robot occupied at its last match, while `age` climbs toward `max_age = 10` (line 49). Every consumer of a track's position reads that stale tuple — the $1.8\text{ m}$ proximity gate at line 485, all five estimate constructions (lines 486, 499, 513, 537, 576), and the debug overlay at line 624.

  Meanwhile the loop **already computes the correct value and throws it away.** Lines 526–533 rotate the whole `history_global` deque out of the map frame and into the robot's *current* frame using this cycle's `rx_rob, ry_rob, cos_r, sin_r`:
  ```python
  dxy      = hist_g_arr[:, :2] - np.array([rx_rob, ry_rob])
  local_xy = dxy @ R.T
  hist_local = [(-float(local_xy[i, 1]), 0.0, float(local_xy[i, 0])) for i in ...]
  ```
  `hist_local[-1]` is exactly the track's position in the frame the robot is in *right now*, correctly translated and rotated regardless of how long ago the last match was. Line 576 then discards it and reads `tracks[tid]['centroid']` instead.

  The error is dominated by ego-**rotation**, not translation, and it does not need a long outage to matter. For a track at range $R$ whose last match was $\Delta\theta$ of yaw ago, the exported lateral coordinate is wrong by $R\sin\Delta\theta$ and the exported range by $R(1 - \cos\Delta\theta)$. At the $\omega \approx 1.5\text{ rad/s}$ Nav2's `rotate_to_goal` and the ROTATE states in `src/ab_comparison_test.py` command, **one** missed frame is $\Delta\theta = 0.15\text{ rad}$, so a pedestrian at $2.0\text{ m}$ is reported $0.30\text{ m}$ off laterally. Three missed frames — well inside the `visible_count` budget, since a track that reached $10$ survives seven misses and still clears the `< 3` gate at line 481 — give $0.45\text{ rad}$, i.e. $0.87\text{ m}$ lateral and $0.20\text{ m}$ of range error. Pure translation adds up to $0.15\text{ m}$ per $0.3\text{ s}$ at the X3's $0.5\text{ m/s}$ ceiling.

  Three consequences, in descending order of severity:
  1. **The path-corridor test is destroyed.** `_get_speed_scaling` in `src/ab_comparison_test.py:829` admits an obstacle to proximity braking only if `abs(ry_t) <= LATERAL_THRESHOLD`, and `LATERAL_THRESHOLD = 0.35` (line 795). A single missed frame during a turn produces $0.30\text{ m}$ of lateral error — **86% of the entire corridor half-width.** A pedestrian standing dead in the path at $r_y = 0.10\text{ m}$ is reported at $0.40\text{ m}$ and drops out of the brake entirely; a bystander at $0.50\text{ m}$ is pulled to $0.20\text{ m}$ and triggers a false stop. The gate flips on ego-rotation alone.
  2. **The proximity gate at line 485 mis-fires on stale depth.** A pedestrian truly at $1.75\text{ m}$ whose stale $z$ reads $1.90\text{ m}$ is forced to `vx = vy = 0.0` (lines 486–496) — and that policy zero is then written into `_prev_estimates` at line 602, so releasing it costs a further $0.5\text{ s}$ acceleration-limiter ramp by the exact mechanism Idea A-32 describes. The two defects chain.
  3. **TTC is quadratic in the error.** $\text{TTC} = -d^2/(\mathbf{r}\cdot\mathbf{v})$ with $d = \text{hypot}(r_{x,t}, r_{y,t})$ (`ab_comparison_test.py:838–841`), so a $0.15\text{ m}$ range error at $d = 1.5\text{ m}$ is a $21\%$ TTC error, and `ROS2Bridge.publish_pedestrians` (`src/server_x3.py:602–603, 613–614, 635–636`) plants the RViz arrow and the `/pedestrian_states` entry at the stale point in `base_link`.

  Note the input side is already correct — the MLP's window is re-referenced every cycle at lines 529–533, which is precisely why this survived: only the *exported* position was left behind. It is the positional twin of Idea A-46, which found the same staleness on the exported *velocity*.
- **Proposed Solution:** Export the value the loop already has, and hoist the reconstruction above the gates that need it.
  1. **Move lines 526–533 above the proximity gate at line 485.** They are three NumPy operations on a $(\le 10, 2)$ array — sub-microsecond — and the gate must test the fresh range, not the stale one, or fix (2) above does not land. The short-history guard at line 498 keeps its own `len < 2` check, which is what makes the hoist safe.
  2. Replace `cx, cy, cz = tracks[tid]['centroid']` at lines 486, 499, 513, 537 and 576 with the current-frame position:
     ```python
     cz_cur = float(local_xy[-1, 0])          # forward range, r_x
     cx_cur = -float(local_xy[-1, 1])         # camera-x, i.e. -r_y
     ```
     Caveat: line 533 hardcodes the vertical component to `0.0`, so the exported `y` becomes zero. That is already harmless — `y` is dead downstream (Idea A-38 shows `y_m` is never read after line 288, and `ab_comparison_test.py:806` uses it only as the dead fallback Idea 337 removes) — but carry the vertical through `history_global` as a fourth column if Idea A-38's height test is landing in the same change, since it wants the same quantity.
  3. **Say when a position is a prediction rather than a measurement.** Export `'age': track['age']` alongside the estimate. A zero-age position is measured; a non-zero-age one is a constant-position extrapolation that is only valid while the *pedestrian* has not moved, and after $\approx 3$ frames should be dropped by the consumer rather than reported as fact. This is the same export discipline as Idea J-08 and Idea A-36's `conf` field and belongs in the same payload change.
  4. Independent of, and required by, Idea A-35: A-35 keeps the robot pose from jumping to the origin, but even with a perfect pose the exported centroid is still frozen at its last match. Both are needed before `est['x']`/`est['z']` mean "where this person is now".
- **Expected Benefit:** Removes up to $0.30\text{ m}$ of lateral and $0.02\text{ m}$ of range error per missed frame at $\omega = 1.5\text{ rad/s}$ ($0.87\text{ m}$ / $0.20\text{ m}$ after three), on a corridor test whose entire half-width is $0.35\text{ m}$ — the difference between braking for the pedestrian in the path and braking for the bystander beside it, precisely during the turns when the avoidance decision is being made. Also stops the $1.8\text{ m}$ proximity gate from flipping on stale depth (which chains into A-32's $0.5\text{ s}$ ramp), removes a $21\%$ TTC error at typical approach range, and puts the RViz arrow and `/pedestrian_states` entry where the person actually is. Costs one hoisted block and one changed tuple unpack, with no new computation — the corrected value is being computed today and discarded.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-47`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
