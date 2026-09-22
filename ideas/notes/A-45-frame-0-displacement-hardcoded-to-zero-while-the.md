---
id: A-45
title: "Frame-0 Displacement Hardcoded to Zero While the Scaler Shows `dx₀` Was a Real Displacement in Training"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-45"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-45 — Frame-0 Displacement Hardcoded to Zero While the Scaler Shows `dx₀` Was a Real Displacement in Training

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (One changed constant and one slice, restores 2 dead model inputs and deletes a fabricated "stopped" frame from the head of every window)
- **Problem:** `_build_window_features` emits a hard zero for the oldest window sample's displacement — `if i == 0: dx, dy = 0.0, 0.0` (`src/velocity_estimator.py:405–406`), written into feature columns 2 and 3 by `features.extend([rx_norm, ry_norm, dx, dy])` at line 415. The shipped `src/scaler_params.json` — read by `_load_model` at lines 196–200 and applied at line 557 — says the training set contained no such thing.

  Columns 2/3 carry `mean = -0.000281 / +0.000094` and `scale = 0.198414 / 0.073921`. Those are statistically indistinguishable from every other `dx`/`dy` pair in the window (cols 6/7: $-0.000280 / +0.000091$, $0.198522 / 0.074023$; cols 38/39: $-0.000176 / +0.000123$, $0.198428 / 0.073619$). Had the training feature builder also hardcoded zero there, sklearn's `StandardScaler` would have reported `mean = 0` exactly and `scale = 1.0` — its zero-variance substitution, the same diagnostic Idea A-30 applies to the `rel_x` columns. Instead frame 0's displacement carries the *full* displacement spread, which is only possible if the training window was differenced against an **eleventh, earlier sample** that the serving path does not keep: `history_global` is bounded at `deque(maxlen=WINDOW_SIZE)` (line 116).

  At serving time those two inputs standardise to fixed constants,
  $$\tilde{d}_{x,0} = \frac{0 - (-0.000281)}{0.198414} = +0.0014, \qquad \tilde{d}_{y,0} = \frac{0 - 0.000094}{0.073921} = -0.0013$$
  against a training distribution of $\mathcal{N}(0,1)$. For a pedestrian walking at $1.4\text{ m/s}$ the true value is $0.14 / 0.198 = +0.71\sigma$ — the network is handed $+0.001$ where it expects $+0.71$. Two of the 40 inputs are pinned dead, and the pinning is **biased rather than merely uninformative**: a zero displacement is the model's learned signature for *stopped*, so every window is prefixed with one frame of fabricated stillness. Across the ten `dx` channels that carry the forward-speed evidence, that is a systematic $\approx 10\%$ pull toward zero speed, and — worse — it imposes a fixed *deceleration* shape on the window (frame 0 still, frames 1–9 moving), which is exactly the cue a trajectory regressor uses to distinguish an accelerating pedestrian from a decelerating one.

  The defect compounds with padding: for a fresh track the zero-order hold at lines 375–376 duplicates `hist[0]`, so `dx[1]` is also structurally zero and the model sees **two** consecutive fabricated stops during the first $0.7\text{ s}$ of every new detection.
- **Proposed Solution:** Recover the eleventh sample the training builder differenced against.
  1. Widen the deque to `deque(maxlen=WINDOW_SIZE + 1)` at line 116 — the only place `history_global` is sized — so it holds $s_{-1}, s_0, \ldots, s_9$. Cost is one extra tuple per track ($5 \times 24\text{ B}$).
  2. `_inference_loop` needs no change: `hist_g_arr` (line 526), `dxy` (line 529) and `local_xy` (line 531) are all shape-agnostic and simply carry 11 rows through the inverse rotation.
  3. In `_build_window_features`, difference over the eleven and emit the last ten:
     ```python
     hist = hist[-(WINDOW_SIZE + 1):]
     for i in range(1, len(hist)):        # i-1 == 0 is the pre-window sample
         dx = hist[i][2] - hist[i-1][2]
         dy = -hist[i][0] - (-hist[i-1][0])
     ```
     with the position features still taken from `hist[1:]` so the emitted vector stays $(1, 40)$ and the scaler alignment is unchanged. Pad to $\text{WINDOW\_SIZE} + 1$ when fewer samples exist — or, better, with Idea J-10's backward linear extrapolation, which makes the pre-window sample a physically meaningful extrapolant instead of a duplicate and removes the compounded `dx[1]` zero at the same time.
  4. Make it non-regressable with one more clause in the load-time contract check Idea A-30 introduces: assert `self.scaler_X_scale[2] > 0.05` (a genuine displacement spread) rather than sklearn's $1.0$ zero-variance substitute, and disable the model if it fails.
  5. Ordering note for Idea A-39: its Savitzky–Golay operator would become $(11, 11)$ and smooth the pre-window sample too, which is strictly better — the newest-sample endpoint row is unchanged and the oldest gains a real neighbour.
- **Expected Benefit:** Returns 2 of the model's 40 inputs from a pinned constant to live signal and removes a fabricated "stopped" frame from the head of every window — worth $\approx 10\%$ of the forward-speed evidence in magnitude, and more than that in *shape*, since it deletes a systematic deceleration bias from a window whose whole purpose is to encode how a pedestrian's motion is trending. Doubles on fresh tracks, where padding currently fabricates two consecutive stops. Cost is one changed constant, one slice, and one extra tuple per track. Independent of Idea A-30 — that repairs the 20 position columns, this repairs 2 of the 20 displacement columns — but both are the same serving/training contract drift and belong under the same load-time assertion.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-45`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
