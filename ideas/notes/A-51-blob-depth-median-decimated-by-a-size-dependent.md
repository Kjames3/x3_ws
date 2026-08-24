---
id: A-51
title: "Blob Depth Median Decimated by a Size-Dependent Stride — a Discontinuous Subsample That Steps `Z` and Buys ~10 µs"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-51"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-51 — Blob Depth Median Decimated by a Size-Dependent Stride — a Discontinuous Subsample That Steps `Z` and Buys ~10 µs

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Delete two lines; removes a recurring 1–3 cm step in the depth reference in exchange for a saving that is under 0.01% of the cycle budget)
- **Problem:** Each blob's depth reference — the single number that becomes $r_x$ and therefore every `dx` the MLP reads — is a median over a **subsample chosen by a stride that is a step function of the blob's own pixel count** (`src/velocity_estimator.py:277–280`):
  ```python
  # Decimate large depth arrays to speed up sorting (Idea 52)
  if len(valid_depths) > 200:
      valid_depths = valid_depths[::len(valid_depths) // 200]
  Z = float(np.median(valid_depths))
  ```
  `valid_depths` comes from a boolean gather at lines 273–274, `depth_vals = depth_slice[cnt_mask == 255]`, which returns the masked pixels in **row-major raster order** — top of the blob first, bottom last. Three things follow, and the first is the one that costs velocity accuracy.

  1. **The retained sample changes discontinuously with blob area.** The stride is $s = \lfloor n/200 \rfloor$, so $s = 1$ for $n \in [200, 399]$, $s = 2$ for $n \in [400, 599]$, $s = 3$ for $n \in [600, 799]$, and so on. A **one-pixel** change in the mask that carries $n$ from $399$ to $400$ switches the estimator from *all 399 valid depths* to *the 200 even-indexed ones* between two consecutive frames. The blob's silhouette breathes by far more than one pixel per frame — morphology at lines 239–241 opens and closes it, the range band at line 274 admits and drops edge pixels as depth flickers — so for a pedestrian at $2\text{ m}$ (a few hundred to a few thousand valid pixels at the $320\times200$ working grid) $n$ crosses a multiple of $200$ **several times per second**, and each crossing swaps the estimator underneath the median.

  2. **Raster-order striding aliases onto vertical stripes.** Consecutive rows of the blob contribute roughly $w$ masked pixels each, so advancing by $s$ moves the sampled column by $s \bmod w$ per row. For $s$ commensurate with $w$ the sample degenerates toward a small set of near-vertical stripes rather than a uniform sample of the body. At the working resolution a pedestrian is $\approx 20\text{–}40\text{ px}$ wide and $s$ reaches $3\text{–}10$ on large near-field blobs, so the surviving set is a systematically biased slice of the silhouette — and *which* slice depends on $n \bmod w$, i.e. it re-randomises every frame.

  3. **It buys essentially nothing, because the premise is wrong.** The comment says "to speed up sorting," but `np.median` does not sort: NumPy's `_median` calls `np.partition`, an $O(n)$ average-case introselect. Partitioning $n = 2{,}000$ float32 values costs $\approx 2\text{–}4\ \mu\text{s}$; decimating to $200$ first saves perhaps $2\ \mu\text{s}$ per blob, or **$\le 10\ \mu\text{s}$ per cycle** at the $\texttt{MAX\_OBSTACLES} = 5$ ceiling — $0.01\%$ of the $100\text{ ms}$ budget. Note the stride is also applied *before* the slice is even bounded correctly: it caps $n$ into $[200, 400)$, not at $200$, so the "optimization" does not do what its own comment claims either.

  The cost side is not small. $Z$ is written straight into $r_x = c_z$ (line 398) and differenced at line 410, so a median displacement of $\delta Z$ in one frame **is** a `dx` of $\delta Z$, read by the model as $\delta Z / 0.1\text{ s}$. The depth spread across a pedestrian blob is $0.2\text{–}0.4\text{ m}$ (torso to arms to silhouette edge, before A-38's floor merging is even counted), and swapping between two different subsamples of that population moves the median by a few percent of the spread — **$1\text{–}3\text{ cm}$, i.e. $0.1\text{–}0.3\text{ m/s}$ of phantom velocity in a single frame.**

  Critically, this is a **step between two estimators, not zero-mean measurement noise**, so the smoothing already logged does not remove it: Idea A-39's Savitzky–Golay operator reproduces low-order polynomials exactly and therefore *spreads* a step across its 5-point kernel rather than deleting it, and Idea A-26's causal EMA merely lags it. It also lands directly on Idea A-29's repaired stop gate, whose net-displacement threshold is $0.03\text{ m}$ over three frames — a single stride flip is enough to un-stop a genuinely stationary pedestrian.
- **Proposed Solution:** Delete the decimation; if a bound is still wanted for very large near-field blobs, make the retained set vary *continuously* with $n$ instead of snapping between strides.
  1. **Remove lines 278–279** and take `Z = float(np.median(valid_depths))` over the full valid set. `np.median` is already $O(n)$ introselect, so Idea 52's stated motivation does not apply, and the result becomes a deterministic function of the blob rather than of $n \bmod 200$.
  2. If a ceiling is desired — $n$ can reach tens of thousands at the $0.5\text{ m}$ near clip — use a fixed-count index set whose members move smoothly as the blob grows:
     ```python
     if valid_depths.size > 512:
         idx = np.linspace(0, valid_depths.size - 1, 512).astype(np.intp)
         valid_depths = valid_depths[idx]
     ```
     512 samples bound the median's own sampling standard deviation at $\approx 1.25\,\sigma/\sqrt{512} = 0.055\,\sigma$, i.e. **under $2\text{ cm}$** for a $0.35\text{ m}$ blob spread, and — unlike the stride — a one-pixel change in $n$ perturbs each index by at most one element. Cost is a single `linspace` ($\approx 1\ \mu\text{s}$).
  3. Ordering with Idea J-20: J-20 replaces the median with a 1-D histogram modal peak for occlusion robustness. That is a *different* estimator, and it inherits this defect wholesale if it is fed the decimated array — a histogram built from a stride-aliased vertical stripe is not the blob's depth mode. Whichever estimator wins, it must consume the full valid set, so this lands first.
  4. Composes with, and is not covered by, Idea A-31 (which fixes `z_ref`, the *area-gate* depth reference at line 256) and Idea A-02 (`connectedComponentsWithStats` supplies the bounding box but does not change the gather order — the fix is deleting the stride, not reordering the pixels).
- **Expected Benefit:** Removes a $1\text{–}3\text{ cm}$ discontinuous step in each blob's depth reference — $0.1\text{–}0.3\text{ m/s}$ of single-frame phantom velocity, recurring several times per second per track — in exchange for giving back at most $10\ \mu\text{s}$ per cycle. Because the artefact is a step rather than white noise, it is invisible to every smoother in this log (A-39, A-26) and is precisely the kind of transient that trips Idea A-29's $0.03\text{ m}$ stop gate and saturates Idea 152's acceleration clamp. Two deleted lines, no new state, and it makes $Z$ a deterministic function of the blob instead of of its pixel count modulo 200.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-51`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
