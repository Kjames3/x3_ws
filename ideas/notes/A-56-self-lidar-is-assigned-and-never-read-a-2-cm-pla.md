---
id: A-56
title: "`self.lidar` Is Assigned and Never Read — a 2 cm Planar Ranger Is Wired In While the Radial Channel Runs on 12 cm Stereo Quantisation Steps"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-56"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-56 — `self.lidar` Is Assigned and Never Read — a 2 cm Planar Ranger Is Wired In While the Radial Channel Runs on 12 cm Stereo Quantisation Steps

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **High ROI** (One bearing-sorted linear scan per cycle over a sensor that is already spinning, already publishing, and already passed into the constructor — attacking the single noisiest input the MLP receives)
- **Problem:** The module docstring promises it on line 6: *"`YDLidarDriver.get_points_xy()` for LiDAR cluster features."* The constructor takes it as the **second positional argument** (line 142), documents it (line 145), and stores it (line 154). `src/server_x3.py:1152` passes the live driver. Then a grep for `self.lidar` across the whole file returns **one hit — the assignment at line 154.** The sensor is never read. The only thing wearing the "fusion" name in the file is Idea 146's *visual* gate at lines 290–315, which uses YOLO boxes, not LiDAR, and which Idea A-34 shows is disabled by a `pass` at line 313 anyway.

  Meanwhile the channel that fusion would fix is the worst input in the pipeline. The radial coordinate $r_x = Z$ (line 398) comes from passive stereo configured at `THE_400_P` with `stereo.setSubpixel(False)` (`src/oakd_driver.py:221–232`), so **disparity is integer-valued** and depth advances in steps of
  $$\Delta Z = \frac{Z^2}{f\,B}$$
  With the OAK-D's $B \approx 0.075\text{ m}$ baseline and $f \approx 442\text{ px}$ at $640\times400$ (a $71.9°$ horizontal FOV), that is:

  | $Z$ | $1.0\text{ m}$ | $1.5\text{ m}$ | $2.0\text{ m}$ | $3.0\text{ m}$ |
  | :--- | :---: | :---: | :---: | :---: |
  | $\Delta Z$ | $0.030\text{ m}$ | $0.068\text{ m}$ | $\mathbf{0.121\ m}$ | $0.272\text{ m}$ |

  And the median at line 280 does **not** average this away: a median is an order statistic, so it returns one of the actual samples — a value on the disparity grid. The blob's depth reference therefore *staircases*. At $2.0\text{ m}$ one step is $0.121\text{ m}$, which is **61% of the `dx` channel's entire training spread** of $0.198\text{ m}$ (`scaler_X.scale[2::4]`). A pedestrian walking at $1.4\text{ m/s}$ covers $0.14\text{ m}$ per frame, so the true displacement and the quantisation step are the *same size*: the observed `dx` sequence is not $[0.14, 0.14, 0.14, \ldots]$ but something like $[0.12, 0.00, 0.24, 0.12, 0.12, 0.24, 0.00, \ldots]$. That is broadband, sample-correlated, and structurally identical to the signal — no smoother separates it, which is exactly why Idea A-39's Savitzky–Golay operator is costed against a $0.042\text{ m}$ noise term it does not include.

  The YDLidar X3 is running the whole time: $8\text{ Hz}$, full $360°$, $\pm 2\%$-of-range accuracy — $\approx 0.02\text{ m}$ at $1\text{ m}$, $\approx 0.04\text{ m}$ at $2\text{ m}$ — on a **direct time-of-flight measurement with no dependence on disparity, texture, or baseline**. At $2\text{ m}$ that is $3\text{–}6\times$ better than a single stereo quantisation step, on the axis carrying essentially all of the approach-speed signal. It is also uncorrelated with every stereo failure mode already logged: the floor merging into the blob (Idea A-38), the depth reference migrating between torso and legs (Idea A-54), the range-adaptive area gate inverting (Idea A-31).
- **Proposed Solution:** Fuse the LiDAR range onto the camera's bearing, inverse-variance weighted, and take the free existence check that falls out.
  1. **Poll and cluster.** After the centroid extraction at line 442, call `self.lidar.get_points_xy()` (already implemented in `src/drivers_x3.py`). Sort the $\approx 360$ returns by bearing once and split into clusters wherever consecutive ranges differ by more than $0.15\text{ m}$ — a single linear pass, no DBSCAN, well under $100\ \mu\text{s}$.
  2. **Associate by bearing.** For a depth centroid at body-frame $(r_x, r_y)$ with $\theta_c = \operatorname{atan2}(r_y, r_x)$ and $\rho_C = \lVert(r_x, r_y)\rVert$, accept the LiDAR cluster whose mean bearing is within $\pm 3°$ of $\theta_c$ **and** whose mean range is within $\pm 0.4\text{ m}$ of $\rho_C$ — a window wide enough to survive the staircase being corrected and narrow enough to reject a wall behind the person.
  3. **Fuse the radial term, keep the camera's bearing.** The camera's angular measurement is good ($\approx 1\text{ px}$ at $f_x = 277$ is $\approx 0.2°$); its range is not. Weight by variance rather than replacing outright:
     $$\hat\rho = \frac{\sigma_L^{-2}\rho_L + \sigma_C^{-2}\rho_C}{\sigma_L^{-2} + \sigma_C^{-2}}, \qquad \sigma_L \approx 0.02\rho_L, \qquad \sigma_C(Z) = \max\!\left(0.03,\ \frac{Z^2}{fB\sqrt{12}}\right)$$
     then $(\hat r_x, \hat r_y) = \hat\rho\,(\cos\theta_c,\ \sin\theta_c)$. The $\sqrt{12}$ is the standard-deviation of a uniform quantisation step; the $\max$ keeps the stereo floor honest. The crossover falls near $1.2\text{ m}$: below it the camera keeps most of the weight (where stereo is genuinely good and the LiDAR is close to its self-return band), above it the LiDAR dominates (where the staircase explodes as $Z^2$).
  4. **Exclude the robot's own body.** `LIDAR_MOUNT_PLAN.md` §1 documents fixed self-returns at $+24°$ and $-18°$, spanning $\approx 0.25\text{–}0.43\text{ m}$ — reject clusters under $0.45\text{ m}$ until the bracket lands, at which point the whole exclusion drops out.
  5. **Take the free existence check.** A depth blob that finds *no* LiDAR cluster at its bearing for $K = 3$ consecutive frames is almost certainly not a standing person — it is the floor (Idea A-38) or a wall fragment (Idea A-27). Demote it before the `MAX_OBSTACLES = 5` cut at line 319, which directly relieves the slot scarcity both of those ideas have to ration.
- **Expected Benefit:** Replaces a $\pm 0.121\text{ m}$ quantisation staircase at $2\text{ m}$ — 61% of the `dx` channel's whole training spread, and the same magnitude as a walking pedestrian's true per-frame displacement — with a $\approx 0.04\text{ m}$ measurement, a $3\text{–}6\times$ improvement on the input that carries the approach-speed signal the safety case rests on. Just as important, it changes the *character* of the dominant error: stereo quantisation is broadband and correlated with the signal, so no filter removes it, whereas the residual after fusion is the LiDAR's gait ripple — narrowband at the $\approx 1\text{ Hz}$ step rate and exactly what Idea A-39's smoother is designed to take out.

  **Stated caveat:** the X3 sits at `laser_joint` $z = 0.11\text{ m}$ (`LIDAR_MOUNT_PLAN.md` §1), so it returns shins and ankles, and a walking person's shin-cluster centroid oscillates by $\approx \pm 0.07\text{ m}$ at the step frequency. That ripple is real and must not be sold as zero — but it is periodic and band-limited, unlike the staircase it replaces, and it shrinks further once `LIDAR_MOUNT_PLAN.md` is executed and the beam clears the chassis. The fusion should therefore be gated on a config flag and A/B'd against the camera-only path using the harness already in `src/ab_comparison_test.py` before it becomes the default.

---

## 5. Log & Prioritization Guidelines


To maintain document readability and prevent log bloat:
1. **Categorize Immediately**: Place new ideas under their respective domain section (Sections 2–4).
2. **Assign an ROI Tier**: Grade each idea based on implementation effort vs. runtime impact:
   - **High ROI**: Low investment (<2 hours), significant gains in CPU/RAM, safety, or accuracy.
   - **Medium ROI**: Moderate effort (half-day to 1 day), solid architecture or navigation benefits.
   - **Low ROI**: High effort or minor edge-case improvements.
3. **Monthly Archiving**: At the end of each month, move completed or historical ideas to the `ideas/` archive folder.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-56`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
