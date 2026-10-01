---
id: A-57
title: "The Depth Frame Is From `t−τ` and the Pose Is From `t` — the `twist` the Docstring Reserves for Ego-Motion Compensation Is Fetched Every Cycle and Never Read"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-57"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-57 — The Depth Frame Is From `t−τ` and the Pose Is From `t` — the `twist` the Docstring Reserves for Ego-Motion Compensation Is Fetched Every Cycle and Never Read

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **High ROI** (Two lines that back-date a pose using a field the callback already returns and the estimator already receives; removes the only error term in the pipeline that grows with the *robot's* rotation rather than the pedestrian's motion)
- **Problem:** `VelocityEstimator.__init__`'s docstring reserves the robot twist for exactly this purpose — `'twist': {'vx': m/s, 'vy': m/s, 'wz': rad/s}} for EKF slip & ego-motion compensation` (`src/velocity_estimator.py:146–148`). `get_robot_pose_and_twist` in `src/server_x3.py:1136–1145` duly takes `ros_bridge._lock`, copies `_twist`, and returns `{"pose": pose, "twist": twist}` on every call. And a grep for `twist` across `src/velocity_estimator.py` returns **exactly one hit: line 147, the docstring.** The value is fetched under a lock at 10 Hz and discarded at line 452, where only `robot_data["pose"]` is unpacked.

  What it was reserved for is a real, uncorrected error. `_inference_loop` reads the depth frame at line 430 and the pose at line 448 **in the same iteration, and treats them as simultaneous.** They are not. The frame's capture lag $\tau$ is the sum of on-device stereo compute, XLink transfer, and the up-to-$1/f_d$ wait in the `maxSize=1, blocking=False` queue (`src/oakd_driver.py:322`) — and Idea A-43 establishes there is no capture timestamp anywhere in the path, so $\tau$ is not merely uncorrected, it is unmeasured. A conservative $\tau = 50\text{ ms}$ is half a poll interval.

  Lines 458–469 then project the centroid into the map frame with the **wrong pose**:
  $$g_{\text{used}} = R(\theta_t)\,c_{t-\tau} + p_t \qquad\text{instead of}\qquad g_{\text{true}} = R(\theta_{t-\tau})\,c_{t-\tau} + p_{t-\tau}$$
  giving a global-frame bias
  $$b_t = \big[R(\theta_t) - R(\theta_{t-\tau})\big]c_{t-\tau} + \big(p_t - p_{t-\tau}\big) \;\approx\; \omega\tau\,R(\theta_t)Jc \;+\; v_{\text{rob}}\tau$$
  with $J$ the $90°$ rotation. The magnitude of the rotational term is $\omega\tau\|c\|$: at $\omega = 1.0\text{ rad/s}$ — a routine in-place heading correction — and a pedestrian at $\|c\| = 2.0\text{ m}$, that is $\mathbf{0.10\ m}$ of phantom lateral displacement in the stored global position, perpendicular to the line of sight.

  What reaches the MLP is the *difference* of that bias, and the honest accounting has three regimes:
  1. **Straight, constant speed.** $b$ is a constant global offset, so it cancels in `dx`/`dy` entirely. It does **not** cancel in the exported position: at $0.3\text{ m/s}$ the reported obstacle sits $0.015\text{ m}$ off, which the CBF (`src/cbf_filter.py:45–61`) and the proximity gate at line 485 consume as truth. Small, but it is pure bias, not noise.
  2. **Steady turn.** Both $\theta$ and $c$ rotate, so $b$ rotates with them and the per-frame change is $\approx \omega^2\tau\|c\|\Delta t = 1.0^2 \times 0.05 \times 2.0 \times 0.1 = \mathbf{0.010\ m/frame}$ — a **sustained $0.10\text{ m/s}$ phantom lateral velocity** for as long as the turn lasts. Against the `dy` channel's training spread of $0.0739\text{ m}$ (`scaler_X.scale[3::4]`) that is $14\%$ of the entire range the model was fit on, and roughly a quarter of the $0.042\text{ m}$ sensor-noise term Idea A-39's smoother is costed against.
  3. **Turn onset and offset — the expensive one.** Nav2 and manual driving start and stop rotations constantly. Stepping $\omega$ from $0$ to $1.0\text{ rad/s}$ over two frames builds the full $0.10\text{ m}$ bias inside the window, injecting $\approx 0.05\text{ m}$ into a *single* `dy` sample — **68% of the whole $0.0739\text{ m}$ `dy` training spread in one frame**, reported as roughly $0.5\text{ m/s}$ of lateral velocity. Nothing in the pipeline catches it: it is far under the $\pm 0.25\text{ m}$ displacement clamp at line 414, it is under the $0.3\text{ m/s}$ per-frame acceleration limiter at line 590, and it is not high-frequency, so Idea A-39's Savitzky–Golay operator passes it through as signal.

  Two properties make this worth ranking high. The sign is **opposite the turn direction**, so a robot rotating to face a pedestrian sees that pedestrian appear to slide *away* — precisely the wrong input for the avoidance decision the estimator exists to inform. And the error is a function of the *robot's* angular rate, so it is identically zero in every stationary-robot bench test and maximal during the manoeuvre where the velocity estimate is actually used.

  This is a distinct locus from what is already logged. Idea A-35 is the pose being **absent** (`None` → identity transform); this is the pose being **present and from the wrong instant**. Idea A-43 is the interval **between** two samples; this is the alignment of **one** sample to its pose. Idea A-46 is the accelerator limiter differencing across frames; this is the projection into the frame in the first place.
- **Proposed Solution:** Back-date the pose to the frame's capture time using the twist that is already on the wire.
  1. Unpack what is already returned, at line 452: `tw = robot_data.get("twist") or {}`, giving body-frame $v_x, v_y, \omega_z$.
  2. Form the capture-time pose before the projection at lines 458–469:
     $$\theta_{t-\tau} = \theta_t - \omega_z\tau, \qquad p_{t-\tau} = p_t - R(\theta_t)\begin{bmatrix}v_x\\v_y\end{bmatrix}\tau$$
     and use $(p_{t-\tau},\ \theta_{t-\tau})$ — i.e. a second `cos_r`/`sin_r` pair — for `centroids_g`.
  3. **Keep the current pose at lines 529–531.** The global $\to$ local re-reference produces the position the CBF and speed scaler consume *now*, so it must use the pose *now*. The two transforms are deliberately on different clocks; today they are accidentally on the same one, which is the whole defect. Idea A-47 fixes what that output *is*; this fixes what it is measured *from*.
  4. **Source of $\tau$.** Once Idea A-43 lands, $\tau = t_{\text{host}} - t_{\text{capture}}$ exactly, per frame. Until then, `OakDCamera.get_depth_frame_age()` already exists at `src/oakd_driver.py:597–600`, is already fed at line 397, and **the estimator has never once called it** — use it as a measured lower bound plus a one-off calibrated constant for the stereo+XLink leg. Clamp $\tau$ to $[0,\ 0.15]\text{ s}$ so a stalled driver cannot extrapolate wildly.
  5. **Regression guard, three lines.** When $|\omega_z| > 0.2\text{ rad/s}$ and *every* live track reports lateral velocity of the same sign exceeding $0.4\text{ m/s}$, that is the signature of this bug rather than of a crowd walking in formation. Log it once per occurrence; it is also the cheapest acceptance test for the fix.
- **Expected Benefit:** Removes a coherent, robot-correlated error that is invisible to both the acceleration limiter and the Savitzky–Golay smoother: $\approx 0.05\text{ m}$ injected into a single `dy` sample at every turn onset — 68% of that channel's entire training spread, worth $\approx 0.5\text{ m/s}$ of phantom lateral velocity — and a sustained $0.10\text{ m/s}$ phantom through steady turns at $1.0\text{ rad/s}$. Corrects the exported obstacle position by up to $0.10\text{ m}$ during rotation and $0.015\text{ m}$ in straight driving, on the number `HolonomicCBFFilter` treats as ground truth. Cost is one dictionary lookup, two multiplications and a second sine/cosine pair per cycle, using data the pose callback already builds under a lock and then throws away.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-57`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
