---
id: A-35
title: "Stale-Odometry Fallback Substitutes the Map Origin and Annihilates Every Track"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-35"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-35 — Stale-Odometry Fallback Substitutes the Map Origin and Annihilates Every Track

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Eight-line change, converts a full 1.0 s blind window per odometry hiccup into graceful degradation)
- **Problem:** `get_robot_pose_and_twist` (`src/server_x3.py:1136–1145`) returns `None` on two separate paths: Idea 225's staleness guard at lines 1139–1140 (`time.monotonic() - ros_bridge._odom_stamp > 0.5`), and line 1145 when no bridge exists at all. `_inference_loop` handles `None` by silently substituting the **identity transform** — `rx_rob, ry_rob, rtheta_rob = 0.0, 0.0, 0.0` at `src/velocity_estimator.py:455–456` — which is exactly what Idea 225 specified. The exception handler at lines 449–450 lands on the same path. But the identity is not a neutral fallback: it is a *different coordinate frame*, and the projection at lines 463–469 (`global_xy = local_coords @ R.T + T`) then teleports every centroid.

  With the robot at $(3.0, 1.5)\text{ m}, \theta = 0.6\text{ rad}$ and a pedestrian $2.0\text{ m}$ dead ahead, the global centroid moves from
  $$(3.0 + 2\cos 0.6,\; 1.5 + 2\sin 0.6) = (4.651,\, 2.629) \quad\longrightarrow\quad (2.0,\, 0.0)$$
  a jump of $\|\Delta\| = \sqrt{2.651^2 + 2.629^2} = 3.73\text{ m}$ in one frame. The tracker's association radius is `max_dist = 0.8` (line 45), so **not one track matches**. Every track goes unmatched, `visible_count` decays (line 100), and the new-track loop hits `if len(self.tracks) >= MAX_OBSTACLES: break` at lines 103–105 — with five live tracks squatting until `age > max_age = 10` frames, **no new track can be created for a full $1.0\text{ s}$** and the estimator publishes nothing at all for a pedestrian it is detecting perfectly well every frame. Below five tracks the damage is smaller but not small: fresh tracks start at `visible_count = 1`, so the `< 3` gate at line 481 suppresses inference for another $0.3\text{ s}$, and any track that *does* survive now holds a `history_global` window mixing two frames — a $3.73\text{ m}$ step into `dx`, clamped to $0.25\text{ m}$ (line 413), i.e. a saturated $2.5\text{ m/s}$ phantom that stays inside the 10-frame window for a full second. When odometry recovers the transform snaps back and the entire sequence repeats in reverse. A single dropped-odom window therefore costs up to $2\text{ s}$ of blindness plus a saturated phantom velocity, for a robot whose whole safety case rests on this estimator.
- **Proposed Solution:** Never change frames mid-session; hold the last good pose and fail loudly rather than silently.
  1. Cache it: `self._last_pose = (rx_rob, ry_rob, rtheta_rob)` and `self._last_pose_t` on every successful query; on `None`, reuse the cached triple. At the X3's $0.3\text{ m/s}$ nominal drive speed a $0.5\text{ s}$ odom gap displaces the robot at most $0.15\text{ m}$ — comfortably inside `max_dist = 0.8` — so every track survives the outage intact. Optionally dead-reckon the held pose with the `twist` the callback already returns: $x \mathrel{+}= (v_x\cos\theta - v_y\sin\theta)\Delta t$.
  2. Bound the hold. If no pose has arrived for `POSE_HOLD_S = 1.0`, stop advancing the tracker entirely — skip `self._tracker.update()`, republish the previous estimates tagged `'stale': True` — instead of fusing fresh detections into a frame known to be wrong. Explicitly blind beats confidently mislocalised.
  3. Handle the one case where the frame legitimately changes (first cycle, or a SLAM relocalisation jump) with an explicit `ObstacleTracker.reset()` rather than a silent reprojection. Detect it as $\|\mathbf{p}_t - \mathbf{p}_{t-1}\| > 0.8\text{ m}$ over $\Delta t \le 0.2\text{ s}$ — kinematically impossible at the X3's $0.5\text{ m/s}$ ceiling — so histories are never mixed across frames.
  4. Independently, replace the `break` at lines 103–105 with eviction of the oldest (or farthest) track, so *any* event that invalidates all five at once cannot lock the estimator out for a second. This also caps the damage from Idea A-27's clutter case.
- **Expected Benefit:** Removes a $1.0\text{–}2.0\text{ s}$ total-blindness window and a saturated $2.5\text{ m/s}$ phantom velocity from every odometry gap, `base_node_X3` restart, or EKF hiccup — events that occur precisely during bringup and hard braking, when pedestrian tracking matters most. Costs one cached tuple and about eight lines, and it makes Idea 225's staleness guard actually protective instead of self-defeating.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-35`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
