---
id: A-46
title: "Acceleration Limiter Differences Two Velocities Expressed in Different Rotating Body Frames"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-46"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-46 — Acceleration Limiter Differences Two Velocities Expressed in Different Rotating Body Frames

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (One cached float and five lines, reclaims up to 70% of the per-frame acceleration budget that ego-yaw currently consumes)
- **Problem:** Idea 152's kinematic limiter at `src/velocity_estimator.py:589–601` clamps the frame-to-frame change of the exported velocity to `max_delta = 3.0 / INFER_HZ = 0.3 m/s`, comparing `est['vx']` against `self._prev_estimates[tid]['vx']` (stored at line 602). Both quantities are **body-frame** velocities, and they are expressed in *different* body frames. The MLP is fed a history rotated into the robot's instantaneous frame — `R = [[cos_r, sin_r], [-sin_r, cos_r]]` at lines 530–531, built from `rtheta_rob` re-read from `robot_pose_fn()` every cycle at line 454 — so its output is expressed at that cycle's yaw. `_prev_estimates` holds the previous cycle's output at the previous cycle's yaw. Nothing rotates one into the other.

  For a pedestrian whose **world** velocity is constant at speed $v$, the apparent body-frame change across two cycles separated by a yaw change $\Delta\theta = \omega\,\Delta t$ is
  $$\|\Delta \mathbf{v}_{\text{body}}\| = 2 v \sin(\Delta\theta / 2) \;\approx\; v\,\omega\,\Delta t$$
  — pure ego-rotation, with zero physical acceleration. On this robot:
  - Nav2's `rotate_to_goal` and spin recovery, and the ROTATE states in `src/ab_comparison_test.py`, drive $\omega \approx 1.0\text{–}1.5\text{ rad/s}$. At $\omega = 1.5$, $\Delta\theta = 0.15\text{ rad}$, and a pedestrian at $1.4\text{ m/s}$ produces $0.21\text{ m/s}$ of apparent change — **70% of the entire $0.3\text{ m/s}$ per-frame budget consumed before a single unit of real acceleration can be represented.**
  - A runner at $2.5\text{ m/s}$ (the output clip at line 569) produces $0.375\text{ m/s} > \texttt{max\_delta}$: the limiter fires on **ego-rotation alone**, permanently lagging the true velocity for as long as the turn lasts.

  Two properties make this pernicious. First, the error scales with the *target's* speed and is identically zero for a stationary target, so it is invisible in exactly the stationary-pedestrian tests the stop gate (Idea A-29) is tuned against, and maximal for the fast approaching pedestrian the estimator exists to catch. Second, it fires precisely *during turns* — when the robot is re-orienting toward or away from the person — which is when the avoidance decision is being made. Because the clamp is applied per-axis (lines 597–600, the box-vs-circle defect Idea J-11 identifies), a partial clip also **rotates** the exported vector rather than merely shrinking it, so the bypass-direction logic and Idea J-13's CBF drift term receive a velocity pointing the wrong way. Note the *input* side is already correct — the whole history is re-rotated into the current frame each cycle at lines 529–533 — which is why this survives: only the output-side memory was left in the stale frame.
- **Proposed Solution:** Difference in a common frame. The loop already holds everything required.
  1. Cache the yaw used for the previous publish: `self._prev_theta = rtheta_rob` beside line 602, initialised to `None` in `__init__` (skip the limiter entirely on the first cycle after a `None`, per Idea A-35's cached-pose handling).
  2. Rotate the previous body-frame velocity into the current body frame before clamping:
     ```python
     dth = math.atan2(math.sin(rtheta_rob - self._prev_theta),
                      math.cos(rtheta_rob - self._prev_theta))
     c, s = math.cos(dth), math.sin(dth)
     pvx, pvy = prev.get('vx', 0.0), prev.get('vy', 0.0)
     pvx, pvy = c * pvx + s * pvy, -s * pvx + c * pvy
     ```
     then clamp `est` against `(pvx, pvy)`. The `atan2` wrap is required, not cosmetic: a $\pm\pi$ yaw crossing would otherwise yield a $2\pi$ $\Delta\theta$ and invert the correction.
  3. Cleaner still, and the form to prefer once Idea J-14 lands: store `_prev_estimates` in the **world** frame (J-14 already derives `vx_global`/`vy_global` from the same `cos_r`/`sin_r`), clamp there — which is where the physical $3\text{ m/s}^2$ bound actually lives — and rotate back only for the body-frame export. Ego-rotation then cannot enter the limiter by construction, and `max_delta` means what its comment says.
  4. This is the fourth independent defect in the same eleven lines and overlaps none of the others: Idea J-11 corrects the clamp's *geometry* (Euclidean norm, true $\Delta t$), Ideas J-22 and A-36 keep *confidence* out of it, Idea A-32 keeps *gate-emitted zeros* out of it, and this keeps *ego-rotation* out of it. All four are required before the clamp bounds only physical acceleration.
- **Expected Benefit:** Removes up to $0.21\text{ m/s}$ of fabricated "acceleration" per frame at $\omega = 1.5\text{ rad/s}$ on a walking pedestrian — 70% of the limiter's budget — and eliminates the case where a runner saturates the limiter on ego-rotation alone, restoring the full $3\text{ m/s}^2$ allowance for representing real human acceleration exactly when the robot is turning. Also removes a direction warp of the exported velocity vector during turns, which today propagates into bypass-direction selection and the CBF drift term. Cost is one cached float and five lines, with no new per-track state and no additional per-cycle work.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-46`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
