---
id: A-30
title: "Translation-Normalized Serving Features Contradict the Absolute-Coordinate Scaler the Model Was Fit On"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-30"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-30 — Translation-Normalized Serving Features Contradict the Absolute-Coordinate Scaler the Model Was Fit On

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (One-line change plus a load-time assertion, restores 20 of the model's 40 inputs to the distribution it was trained on)
- **Problem:** Idea 48's translation normalization was implemented in the **serving** feature builder — `rx0 = hist[0][2]`, `ry0 = -hist[0][0]` (`src/velocity_estimator.py:381–382`), applied as `rx_norm = rx - rx0` / `ry_norm = ry - ry0` at lines 402–403 and emitted at line 415 — but `src/scaler_params.json`, which `_load_model` reads at lines 196–200 and `_inference_loop` applies at line 557, was fit on **absolute** coordinates. The JSON proves it three ways:
  1. `scaler_X.mean[0] = 2.29463`, `scaler_X.scale[0] = 6.24044`. Frame 0's `rel_x_norm` is $r_{x,0} - r_{x,0} \equiv 0$ **by construction**, so a scaler fit on normalized features must show `mean[0] = 0` and `scale[0] = 0` (sklearn substitutes $1.0$ for a zero-variance column). Neither holds.
  2. `scale[0::4]` is flat across the window — $6.24044,\ 6.24038,\ 6.24032,\ \ldots,\ 6.23981$ — a $0.010\%$ **decrease** from frame 0 to frame 9. Coordinates measured from a fixed window origin necessarily have variance that *grows* with frame index; flat-and-marginally-decreasing is the signature of absolute range, whose spread is set by the scene rather than by the window.
  3. `mean[0::4]` drifts only $2\text{ mm}$ across ten frames ($2.29463 \to 2.29250$) and `mean[1::4]` likewise ($0.9394 \to 0.9403$) — these are the dataset-mean obstacle range and lateral offset, i.e. absolute-position statistics. By contrast the `dx`/`dy` columns carry mean $\approx -0.0003 / +0.0001$ with scale $0.198 / 0.074$: genuine zero-mean displacement statistics, untouched by the normalization and therefore still in-distribution.

  Git corroborates: `scaler_params.json` was last written by commit `5fe320b`, the same commit that introduced `rx_norm = rx - rx0`, and the later retrain `3a7a650` ("updated model to in attempt to remove phantom points") changed `velocity_mlp.torchscript` **alone** — the scaler was never regenerated. Because `dx` is clamped to $\pm 0.25\text{ m}$ per frame (line 413), a real window's $r_{x,\text{norm}}$ has spread $\sigma \approx 0.3\text{ m}$, so after standardization every one of the ten `rel_x` channels sits at
  $$\tilde{r}_x = \frac{r_{x,\text{norm}} - 2.2946}{6.2404} \approx -0.368 \pm 0.048$$
  against a training distribution of $\mathcal{N}(0,1)$ — a fixed $-0.37\sigma$ bias with variance collapsed by $(6.24/0.30)^2 \approx 430\times$. The `rel_y` channels sit at $-0.441 \pm 0.056$, variance down $\approx 1400\times$. **Half of the model's 40 inputs are pinned to a near-constant value the training set never contained**, contributing a fixed bias to every hidden unit and carrying essentially none of the trajectory information they were fit to carry. The network is effectively reduced to a function of the 20 `dx`/`dy` channels evaluated at a constant out-of-distribution offset — which is also why the model appears to respond only to instantaneous per-frame motion and not to trajectory shape.
- **Proposed Solution:** Whichever feature mode is chosen, the artifact and the builder must agree; today they do not. Three steps, in order of urgency:
  1. **Immediate, one line:** restore parity with the shipped artifact — emit `features.extend([rx, ry, dx, dy])` at line 415 (drop the `- rx0` / `- ry0`). Verify offline before it touches the robot by replaying a recorded `history_global` deque through both variants and scoring predicted speed against the finite-difference ground truth recipe already specified in `VELOCITY_SELF_TRAINING_PLAN.md` §3.
  2. **Durable guard:** make the training plan's §2 "feature/label contract" executable. Introduce a module constant `TRANSLATION_NORMALIZE` that selects the branch at lines 402–403, and assert it against the artifact in `_load_model()`:
     ```python
     scaler_is_normalized = abs(self.scaler_X_mean[0]) < 1e-3 and self.scaler_X_scale[0] < 1e-2
     if scaler_is_normalized != TRANSLATION_NORMALIZE:
         logger.error("scaler/feature contract mismatch: scaler_X[0] mean=%.4f scale=%.4f",
                      self.scaler_X_mean[0], self.scaler_X_scale[0])
         self._model = None
     ```
     This also closes a live hazard in the model hot-swap path (`src/server_x3.py:1720–1726`): `velocity_mlp_finetuned.torchscript` can be swapped in at runtime while a **single** `scaler_params.json` is silently reused for every model.
  3. **Long term:** when the OAK-D retrain in `VELOCITY_SELF_TRAINING_PLAN.md` runs, regenerate `scaler_params.json` from the *normalized* features and flip `TRANSLATION_NORMALIZE = True`, earning Idea 48's translation invariance legitimately instead of asserting it against a scaler that contradicts it.
- **Expected Benefit:** Returns 20 of the model's 40 inputs from a pinned $-0.37\sigma$ / $-0.44\sigma$ constant to the distribution the network was fit on, restoring the position-channel signal the estimator currently cannot use at all. Largest single accuracy defect in the pipeline: on any window where trajectory *shape* rather than per-frame delta carries the information — curved approaches, a pedestrian decelerating, oblique crossings — this is the difference between a trained regressor and a 20-input stub. Cost is one line plus a five-line load-time assertion, and it is fully verifiable offline against finite-difference labels.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-30`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
