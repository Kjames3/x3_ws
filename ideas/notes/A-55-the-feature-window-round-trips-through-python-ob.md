---
id: A-55
title: "The Feature Window Round-Trips Through Python Objects Twice Between Two NumPy Arrays — 90 Scalar `np.clip` Dispatches per Cycle to Fill a Pre-Allocated Tensor"
status: Logged
domain: Performance
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-55"
session: "3. Performance & Execution Efficiency"
tags: [idea]
---

# A-55 — The Feature Window Round-Trips Through Python Objects Twice Between Two NumPy Arrays — 90 Scalar `np.clip` Dispatches per Cycle to Fill a Pre-Allocated Tensor

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 8)
- **ROI Tier:** **High ROI** (Rewrite one 50-line function as batched array code; removes ~95% of the feature stage's cost and turns three already-logged ideas from cross-cutting edits into one-liners)
- **Problem:** The window arrives as a NumPy array and leaves as a NumPy array, and in between it is disassembled into Python floats and reassembled — twice.

  Trace one track through one cycle of `src/velocity_estimator.py`:
  1. **Line 526** — `hist_g_arr = np.array(list(track['history_global']), dtype=np.float32)`. The deque holds 10 Python tuples of Python floats (built as scalars at lines 89–93 and 118), so `list()` copies the deque and `np.array` parses **30 boxed floats** into a $(10,3)$ array.
  2. **Lines 529–531** — the only genuinely vectorised step: `dxy`, a $2\times2$ rotation, `local_xy = dxy @ R.T`. Roughly $1\ \mu\text{s}$ of real work.
  3. **Line 533** — `hist_local = [(-float(local_xy[i, 1]), 0.0, float(local_xy[i, 0])) for i in range(len(local_xy))]`. The $(10,2)$ result is **taken straight back apart** into 10 Python tuples via 20 NumPy-scalar `float()` unboxes, with a literal `0.0` in the middle slot that no consumer ever reads (`_build_window_features` uses only `hist[i][0]` and `hist[i][2]`).
  4. **Line 373** — `hist = list(history_local)` copies the list a third time.
  5. **Lines 388–393** — the `is_stopped` check: a Python loop over two frames with `math.hypot`, re-deriving displacements that step 7 is about to compute again.
  6. **Lines 397–415** — a 10-iteration Python loop with tuple unpacking, and inside it the expensive part: **`np.clip(dx, -0.25, 0.25)` and `np.clip(dy, …)` called on Python scalars** (lines 413–414). Scalar `np.clip` is the slow path — ufunc dispatch, argument parsing, 0-d array construction, and a NumPy scalar back out, roughly $1.3\ \mu\text{s}$ against $\approx 40\ \text{ns}$ for `min(max(...))` or a single batched call. That is **18 dispatches per track**, and at the 5-track ceiling **90 per cycle, 900 per second**, purely to bound two numbers.
  7. **Line 415** — `features.extend([...])`, building a 40-element Python list of boxed floats.
  8. **Line 417** — `np.array(features, dtype=np.float32).reshape(1, -1)` parses those 40 boxed floats back into an array.
  9. **Line 554** — `np.vstack(features_list)` copies all $N$ of the $(1,40)$ arrays into one $(N,40)$ buffer.

  Cost at the 5-track ceiling, per cycle: $\approx 0.12\text{ ms}$ in `np.clip` dispatch alone, $\approx 0.05\text{ ms}$ in the five `np.array`-from-list parses, $\approx 0.08\text{ ms}$ in the line-533 comprehension and the line-397 loop, plus the deque copies and the `vstack` — call it $\mathbf{\approx 0.30\ ms}$ against $\approx 15\ \mu\text{s}$ for the equivalent batched array code. The $(5,40)$ MLP forward it feeds is $\approx 0.1\text{ ms}$, so **the marshalling costs about three times the inference it exists to serve**, and all of it is interpreted bytecode holding the GIL in a process whose asyncio loop must also encode and ship a JPEG inside the same window.

  The irony is structural: `self.x_tensor_preallocated` exists (line 175, Idea 116) to avoid an $800$-byte tensor allocation, and the path that fills it allocates roughly 250 Python float objects and five intermediate arrays to get there. The last 1% was optimised and the first 99% was not.

  This is not covered by anything logged. Idea A-37 is the *shape* the traced graph sees; Idea A-52 is what the graph *contains*; Idea A-41 is OpenCV contiguity in the extraction stage. None of them touch the feature builder.
- **Proposed Solution:** Keep the window in arrays end to end and build all $N$ rows in one pass.
  1. **Store the window as an array, not tuples.** Replace each track's `deque` of tuples with a fixed $(\texttt{WINDOW\_SIZE}, 2)$ `float32` ring buffer plus a write index, holding global $x, y$ only (the third slot is the EMA'd $z$, which Idea A-26 shows never reaches the features anyway). Steps 1 and 4 disappear; the tracker writes two floats instead of allocating a tuple.
  2. **Gather once, transform once.** Build $G$ of shape $(N, T, 2)$ by stacking the live rings, then re-reference every track and every frame in a single matmul:
     $$L = \big(G - p_{\text{rob}}\big)\,R^{\top}, \qquad R = \begin{bmatrix}\cos\theta & \sin\theta\\ -\sin\theta & \cos\theta\end{bmatrix}$$
     replacing the per-track version at lines 529–531 with one $(N\cdot T, 2)$ operation.
  3. **Difference and clamp in bulk.**
     $$D = \operatorname{clip}\!\big(\operatorname{diff}(L,\ \text{axis}=1,\ \text{prepend}=L[:,:1,:]),\ -0.25,\ +0.25\big)$$
     — **one** `np.clip` call on an $(N,T,2)$ array in place of 90 scalar dispatches. Apply it *after* Idea A-43's $\Delta t$ rescaling, per that idea's step 3.
  4. **Interleave with strided writes.** Keep a preallocated `self._feat_buf` of shape $(\texttt{MAX\_OBSTACLES}, 40)$ and fill the four interleaved channels directly — `self._feat_buf[:N, 0::4] = rel[..., 0]`, `[..., 1::4] = rel[..., 1]`, `[..., 2::4] = D[..., 0]`, `[..., 3::4] = D[..., 1]`. Steps 7, 8 and the `vstack` at line 554 all vanish.
  5. **Vectorise the stop gate.** `is_stopped` becomes a boolean vector over tracks, computed from $D$ that already exists:
     $$\texttt{stopped} = \bigwedge_{k \in \{T-2,\,T-1\}} \sqrt{D_{:,k,0}^2 + D_{:,k,1}^2} < 0.01$$
     one `np.hypot` and one `np.all(axis=1)`, replacing the loop at lines 388–393. (Idea A-29 replaces the *criterion*; this replaces the *loop* — they compose, and A-29 becomes a two-line change once the window is an array.)
  6. **Scale in place and hand the buffer to Torch directly.** `np.subtract(buf, mean, out=buf); np.multiply(buf, inv_scale, out=buf)` removes the two temporaries at line 557, and `torch.from_numpy(self._feat_buf[:N])` then wraps the buffer with **zero copy** — so `self.x_tensor_preallocated` and the `copy_` at line 561 can be deleted outright rather than merely justified.
  7. Retire `_build_window_features`'s tuple contract, or keep it as a thin shim over the array path for `src/ab_comparison_test.py`.
- **Expected Benefit:** Cuts the feature stage from $\approx 0.30\text{ ms}$ to $\approx 15\ \mu\text{s}$ per cycle — a **~95% reduction, $\approx 3\text{ ms/s}$ of GIL-held interpreted bytecode removed** from the process that also runs the asyncio broadcast loop — and takes the 900 scalar `np.clip` dispatches per second down to 10. Removes roughly 250 Python object allocations and five array parses per cycle, and lets the `copy_` into the pre-allocated tensor be deleted rather than kept. The larger payoff is structural: once the window is an $(N, T, 2)$ array, **Idea A-39** (Savitzky–Golay) is one `savgol_filter(L, …, axis=1)`, **Idea A-43** (per-pair $\Delta t$ rescaling) is one broadcast divide, and **Idea A-45** (the eleventh history sample) is a ring one slot wider — each currently a change threaded through three separate Python loops.

---

## 4. Sensor Fusion & Hardware Integration
*Focus areas: OAK-D vs. Astra Pro depth calibration, YDLidar X3 mounting/scan-matching, IMU slip compensation, and motor driver telemetry.*

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-55`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
