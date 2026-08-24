---
id: A-39
title: "Zero-Phase Savitzky–Golay Window Smoothing Before Differencing"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-39"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-39 — Zero-Phase Savitzky–Golay Window Smoothing Before Differencing

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (One precomputed $10\times10$ matrix and one matmul per track, cuts displacement-channel noise 2.2× with zero added lag)
- **Problem:** `_build_window_features` derives every displacement feature as an **adjacent-frame first difference** — `dx = rx - rx_prev`, `dy = ry - ry_prev` at `src/velocity_estimator.py:410–411`, computed over the local trajectory reconstructed at lines 526–533. A first difference is a high-pass operator: if each position sample carries independent noise of standard deviation $\sigma$, then $\mathrm{Var}(dx) = 2\sigma^2$ — differencing *amplifies* noise by $\sqrt{2}$ while the signal it extracts is one frame of true motion.

  The OAK-D Lite is passive stereo at 400P with `stereo.setSubpixel(False)` (`src/oakd_driver.py:232`), giving $\sigma_z \approx 0.03\text{ m}$ per frame at $2.0\text{ m}$ — the same noise floor Ideas A-26 and A-29 are costed against. So
  $$\mathrm{sd}(dx) = \sqrt{2}\,\sigma_z = 0.042\text{ m} \quad\text{per } \Delta t = 0.1\text{ s} \;\Longrightarrow\; 0.42\text{ m/s}$$
  in **every one of the ten `dx` columns**. The shipped artifact says exactly how large that is relative to the channel: `scaler_params.json` gives `scaler_X.scale` $= 0.198$ for the `dx` columns and $0.074$ for the `dy` columns. Sensor noise is therefore **21% of the entire training spread of the forward-displacement channel and 57% of the lateral one** — more than half the dynamic range of `dy` is noise, on the channel that carries crossing-pedestrian motion.

  Nothing in the current pipeline attenuates it. The $\pm 0.25\text{ m}$ clamp at line 413 sits $6\times$ above the noise, so it never engages on noise — only on the discontinuity spikes of A-35/A-40. The depth EMA (`alpha_z = 0.7`, line 50) never reaches the features at all (A-26), and even once A-26 lands it is a **causal one-pole** filter that buys only a $27\%$ noise reduction and pays for it with lag on a moving target. That trade is unnecessary here: by the time `_build_window_features` runs, all ten samples are already in hand, so a **non-causal** filter is available at zero latency cost — an option a per-sample EMA structurally cannot take.
- **Proposed Solution:** Smooth the reconstructed local trajectory with a Savitzky–Golay polynomial fit **before** differencing, applied as a single precomputed linear operator so it costs one small matmul per track.
  1. Precompute a $(10, 10)$ operator `self._SG` in `__init__` from the 5-point quadratic SG smoother, interior weights
     $$\mathbf{c} = \tfrac{1}{35}\,[-3,\; 12,\; 17,\; 12,\; -3]$$
     with the standard **asymmetric endpoint rows** for indices $0,1,8,9$, so the newest sample (index 9 — the one that dominates the freshest `dx`) is fit from a one-sided quadratic rather than left unfiltered.
  2. Apply it at line 531, immediately after `local_xy = dxy @ R.T`:
     ```python
     local_xy = self._SG @ local_xy          # (10,10) @ (10,2)
     ```
     Because an order-2 SG filter reproduces any quadratic **exactly**, a constant-velocity or constant-acceleration trajectory passes through unchanged: zero bias, zero group delay. This is the property the EMA lacks.
  3. Noise reduction, exactly: with $\gamma_0 = \sum_k c_k^2 = 595/1225 = 0.4857$ and $\gamma_1 = \sum_k c_k c_{k+1} = 336/1225 = 0.2743$, the smoothed first difference has
     $$\mathrm{Var}(\Delta s) = 2(\gamma_0 - \gamma_1)\,\sigma^2 = 0.423\,\sigma^2 \quad\text{versus}\quad 2\sigma^2 \text{ raw}$$
     a variance factor of $0.211$, i.e. **sd reduced by $\sqrt{0.211} = 0.46$**. Cost is $10\times10\times2 = 200$ MACs per track, $\approx 2\ \mu\text{s}$ at batch 5 — well below the $0.5\text{–}1.5\text{ ms}$ A-37 recovers.
  4. **Feature/label contract:** this attenuates the *noise* component of `dx`/`dy` while leaving the signal untouched, so served features sit slightly inside the spread the scaler was fit on ($0.198 / 0.074$, measured on unsmoothed data). The shift is toward lower input variance, which is conservative, but the correct closure is to apply the identical operator in the training feature builder when the OAK-D retrain in `VELOCITY_SELF_TRAINING_PLAN.md` §2 runs and regenerate `scaler_params.json` — the same executable contract A-30 argues for, and the natural place to land both.
- **Expected Benefit:** Cuts per-step displacement noise from $0.042\text{ m}$ to $0.019\text{ m}$ ($0.42 \to 0.19\text{ m/s}$), a **2.2× reduction across all twenty displacement features**, with no lag penalty — which is the difference between this and any output-side EMA or median filter that would trade jitter for delay on exactly the accelerating pedestrian the estimator exists to catch. Recovers the lateral channel in particular, where sensor noise currently occupies 57% of the training spread. Also widens the margin on A-29's net-displacement stop gate, whose $0.03\text{ m}$ threshold is set directly by this noise floor.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-39`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
