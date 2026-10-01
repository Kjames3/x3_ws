---
id: A-36
title: "Confidence-Scaled Velocity Export Compounds the Padding Under-Report and Inverts the Braking Response"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-36"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-36 — Confidence-Scaled Velocity Export Compounds the Padding Under-Report and Inverts the Braking Response

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Two-line change plus one downstream weighting, removes a $3.3\times$ TTC inflation on every newly eligible track)
- **Problem:** `_inference_loop` multiplies the MLP's physical output by the tracking confidence before publishing — `conf = min(1.0, visible_count / WINDOW_SIZE)` at `src/velocity_estimator.py:577–578`, applied as `'vx': round(vx * conf, 3)` at lines 584–586. Idea 236 justified this damping on the premise that a padded window "systematically overestimates pedestrian speed". The padding is a **zero-order hold** — `hist.insert(0, hist[0])` at lines 375–376 — which duplicates the oldest sample, so every padded step contributes $dx = dy = 0$ (lines 405–414). A fresh window is therefore mostly *zero* displacement and under-reports, which is exactly what Idea J-10 independently quantifies at 60–80%. The damping premise is inverted with respect to the code it damps, and the two attenuations multiply.

  At the moment a track first clears the initiation gate, `visible_count = 3` so $\text{conf} = 0.3$, and the reported speed is
  $$v_{\text{rep}} \approx v_{\text{true}} \cdot \underbrace{(1 - 0.7)}_{\text{padding}} \cdot \underbrace{0.3}_{\text{conf}} \approx 0.09\, v_{\text{true}}$$
  i.e. $0.13\text{ m/s}$ for a pedestrian walking at $1.4\text{ m/s}$. Reaching $\text{conf} = 1.0$ requires ten *consecutive* matched frames (line 92 increments, line 100 decrements on any miss), so a track that misses one frame in three never exceeds $\approx 0.5$ at all.

  The safety consequence is the sign of the response. `_get_speed_scaling` in `src/ab_comparison_test.py` computes $\text{TTC} = -d^2/(\mathbf{r}\cdot\mathbf{v})$, which scales as $1/v$: a $0.3\times$ velocity inflates TTC by $3.33\times$. A pedestrian at $d = 1.8\text{ m}$ closing at $1.4\text{ m/s}$ has a true $\text{TTC} = 1.29\text{ s}$, well inside the $3.0\text{ s}$ brake threshold — but is reported at $\approx 4.3\text{ s}$, above it, so **no predictive braking occurs at all** for the $0.7\text{ s}$ it takes `conf` to climb, during which the pedestrian closes $0.98\text{ m}$. Low confidence therefore makes the robot *less* cautious, because the only channel confidence flows through is the magnitude that TTC divides by. The same scaling corrupts every other physical consumer of the vector: J-15's forward-projected point cloud places predicted positions at 30% of the true stride, J-13's obstacle-drift CBF term is scaled to 30%, and the A/B logs record a number that is not a velocity.
- **Proposed Solution:** Stop scaling the physical quantity; publish the uncertainty beside it and let each consumer decide.
  1. Emit unscaled `vx`, `vy`, `speed` at lines 584–586, and export `'conf': round(conf, 2)` and `'visible_count': visible_count` — which is precisely Idea J-08's export, and is a prerequisite for this change rather than a duplicate of it.
  2. Address the real defect at its source with Idea J-10's backward linear extrapolation padding, which removes the 60–80% under-report instead of compounding it.
  3. Move the confidence weighting to *brake severity* in `_get_speed_scaling`, per J-08's $s_{\text{obs,weighted}} = 1 - \text{conf}\,(1 - s_{\text{obs}})$. This is conservative in the correct direction — low confidence means less aggressive braking on a possibly-spurious track — rather than the current behaviour, where low confidence means a lower apparent speed and therefore *later* braking.
  4. Ordering with the limiter: clamp the raw MLP output, store the raw clamped value in `_prev_estimates`, then publish raw plus a separate `conf` field. Combined with Idea J-22 (clamp before confidence) and Idea A-32 (do not clamp against gate-emitted zeros), the limiter, the exported vector and the consumer's weighting become three independent mechanisms that are each individually correct.
- **Expected Benefit:** Restores full velocity magnitude on newly eligible tracks — the $0.09\times$ to $0.3\times$ attenuation disappears — removing the $3.33\times$ TTC inflation and recovering $\approx 0.98\text{ m}$ of braking distance on every first encounter, which is the single largest end-to-end error in the predictive path. Also flips the confidence response from anti-conservative to conservative, and makes the published `vx`/`vy` a physical velocity that Nav2, the CBF and the A/B logs can use without a hidden 0.3–1.0 gain.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-36`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
