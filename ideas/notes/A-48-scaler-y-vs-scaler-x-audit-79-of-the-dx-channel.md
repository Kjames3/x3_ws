---
id: A-48
title: "`scaler_y` vs `scaler_X` Audit — 79% of the `dx` Channel Is Noise, and Nothing Cross-Checks the MLP Against the Free Finite-Difference Estimate"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-48"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-48 — `scaler_y` vs `scaler_X` Audit — 79% of the `dx` Channel Is Noise, and Nothing Cross-Checks the MLP Against the Free Finite-Difference Estimate

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Four arithmetic operations per track for a runtime residual that makes every silent contract defect observable, plus a fallback estimator that is competitive with the model it checks)
- **Problem:** Ideas A-30 and A-45 audited `scaler_X` against the feature builder. Nobody has audited `scaler_X` against **`scaler_y`** — and the two artifacts, read together, are strongly informative about what the model can and cannot be extracting.

  `_load_model` reads both from the same file (`src/velocity_estimator.py:196–200`). The label scaler gives the training velocity spread directly:
  $$\sigma(v_x) = 0.90467\text{ m/s}, \qquad \sigma(v_y) = 0.45487\text{ m/s}$$
  The feature scaler gives the displacement spread, and it is remarkably uniform across the window — `scale[2::4]` runs $0.1984, 0.1985, \ldots, 0.1984$ (mean $0.1983$) and `scale[3::4]` runs $0.0739 \ldots 0.0736$ (mean $0.0737$). At the $\Delta t = 0.1\text{ s}$ that `INFER_HZ = 10` and `WINDOW_SIZE = 10  # matches training T=10` (lines 33–34) bake into the weights, true pedestrian motion can account for only
  $$\sigma(v_x)\,\Delta t = 0.0905\text{ m}, \qquad \sigma(v_y)\,\Delta t = 0.0455\text{ m}$$
  of those spreads. The observed spreads are $2.19\times$ and $1.62\times$ larger. Expressed as a variance fraction, **motion accounts for $20.8\%$ of the training variance of every `dx` channel and $38.1\%$ of every `dy` channel** — the remaining $79\%$ / $62\%$ is something else. Attributing it to per-sample position error (two independent samples per difference, $\sigma_{\text{tot}}^2 = (\sigma_v\Delta t)^2 + 2\sigma_{\text{pos}}^2$) gives
  $$\sigma_{\text{pos},x} = 0.125\text{ m}, \qquad \sigma_{\text{pos},y} = 0.041\text{ m}$$
  which is internally consistent and physically sensible: three times more error along the depth axis than across it, the exact signature of a stereo range measurement versus a pixel-quantised lateral one.

  Two conclusions follow, and both are actionable.
  1. **Any single displacement channel is nearly useless, and the model must be reading the long baseline.** Classical errors-in-variables attenuation on one channel is $\lambda = \sigma_{\text{sig}}^2/(\sigma_{\text{sig}}^2 + \sigma_{\text{noise}}^2) = \mathbf{0.208}$ — a regressor leaning on one `dx` would shrink its output by $5\times$. Across the window the noise telescopes: the mean of $dx_1 \ldots dx_9$ **is** the endpoint difference $(r_{x,9} - r_{x,0})/9$, whose noise variance is $2\sigma_{\text{pos}}^2/81$, giving $\lambda = \mathbf{0.955}$. So the trained network can only be well-calibrated by exploiting the $0.9\text{ s}$ baseline across the window — which is exactly the structure Idea A-30 destroys (the twenty position channels pinned to a constant) and Idea A-45 corrupts (a fabricated zero at the head). This is independent evidence that those two are the dominant accuracy defects, and it explains the reported symptom that the model "responds only to instantaneous per-frame motion".
  2. **That same long baseline is a free estimator, and it is never computed.** After line 531 the loop holds `local_xy`, a $(10, 2)$ array of the track's positions in the current body frame. The endpoint difference costs two subtractions and two multiplies:
     $$\hat{v}_x = \frac{r_{x,9} - r_{x,0}}{9\Delta t}, \qquad \hat{v}_y = \frac{r_{y,9} - r_{y,0}}{9\Delta t}$$
     At the OAK-D Lite's actual $\sigma_{\text{pos}} \approx 0.03\text{ m}$ this has $\text{sd} = \sqrt{2}\,\sigma_{\text{pos}}/0.9 = \mathbf{0.047\text{ m/s}}$ — versus $0.42\text{ m/s}$ for a single-frame difference, the figure Idea A-39 is costed against. Nothing anywhere in `velocity_estimator.py` compares the model's output to it. The MLP output is inverse-transformed at line 568, clipped at line 569, and published, with **no consistency check of any kind** between the network and the kinematics of its own input window. Every contract defect logged this month — A-30's pinned position channels, A-45's fabricated frame-0 step, A-43's unmeasured $\Delta t$, and whatever the $\sigma$ ratio above turns out to reflect — is therefore *silent* at runtime.
- **Proposed Solution:** Compute the kinematic estimate the window already contains, publish it, and use the residual as both a monitor and a fallback.
  1. **Compute it.** Immediately after line 531:
     ```python
     n  = len(local_xy)
     inv = 1.0 / ((n - 1) * dt)
     vx_fd =  float(local_xy[-1, 0] - local_xy[0, 0]) * inv
     vy_fd =  float(local_xy[-1, 1] - local_xy[0, 1]) * inv
     ```
     Use the *unpadded* sample count so a fresh track's duplicated head (lines 375–376) cannot flatten it, and take $\Delta t$ from Idea A-43's per-sample capture stamps once those exist rather than assuming $0.1$.
  2. **Export it.** Add `'vx_fd'`, `'vy_fd'` and the residual `'v_res': hypot(vx - vx_fd, vy - vy_fd)` to the estimate dict at lines 579–587. It costs three floats in the readout payload and turns the A/B logs into a direct measurement of model-versus-kinematics agreement — which is the number `VELOCITY_SELF_TRAINING_PLAN.md` §3 needs anyway to score a retrain, now collected live instead of offline.
  3. **Gate on it.** If `v_res` exceeds a threshold (start at $3\times$ the fallback's own $0.047\text{ m/s}$ sd, i.e. $0.15\text{ m/s}$) for $K = 5$ consecutive cycles on the same track, log once and publish $\hat{v}_{\text{fd}}$ in place of the model output for that track. This is not a downgrade: with $\lambda = 0.955$ the endpoint estimator is *nearly unbiased*, and it is unconditionally immune to every serving/training contract drift, because it never touches the scaler or the network. The model's advantage over it is trajectory shape — acceleration, curvature — which is exactly the advantage it currently cannot express while A-30 holds.
  4. **Make the ratio an offline check.** Add the audit above to the load-time contract assertion Idea A-30 introduces: assert `scaler_X.scale[2] >= scaler_y.scale[0] * DT` — the displacement spread can never be *smaller* than the motion it must contain — and log the implied noise $\sigma_{\text{pos}}$ so a regenerated `scaler_params.json` that silently changes sampling rate or sensor is visible in `journalctl` rather than discovered as a velocity gain error on the robot.
  5. Ordering: this composes with Idea A-39 rather than competing. A-39 attenuates the *per-frame* channels ($0.42 \to 0.19\text{ m/s}$); the endpoint estimator bypasses them entirely ($0.047\text{ m/s}$). Both feed the same network, and the residual computed here is the instrument that will show whether A-39, A-30 and A-45 actually moved the model's output toward the kinematics.
- **Expected Benefit:** Establishes, from the shipped artifacts alone, that a single displacement channel carries only $20.8\%$ signal ($\lambda = 0.208$) while the $0.9\text{ s}$ window baseline carries $95.5\%$ ($\lambda = 0.955$) — which independently ranks A-30 and A-45 as the highest-value accuracy fixes, because both destroy precisely the long-baseline structure the model must depend on. Delivers a $0.047\text{ m/s}$-sd velocity estimate for four arithmetic operations per track, converts every silent training/serving contract defect in this log into an observable per-cycle residual, and gives the estimator a fallback that cannot be corrupted by a scaler or model mismatch. Costs no new state and no new full-frame work.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-48`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
