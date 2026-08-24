---
id: A-41
title: "Strided `[::2, ::2]` Views Force Hidden OpenCV Contiguity Copies and 2× Cache-Line Overfetch"
status: Logged
domain: Performance
roi_tier: Medium-High
source: august_improvement_ideas.md
source_id: "A-41"
session: "3. Performance & Execution Efficiency"
tags: [idea]
---

# A-41 — Strided `[::2, ::2]` Views Force Hidden OpenCV Contiguity Copies and 2× Cache-Line Overfetch

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **Medium-High ROI** (Two lines, but it is the precondition that makes Ideas A-28 and A-02 deliver the savings they are costed at rather than displacing them into OpenCV's internal copy)
- **Problem:** `_extract_depth_centroids` decimates by rebinding **strided views** — `raw_depth_frame = raw_depth_frame[::2, ::2]` at `src/velocity_estimator.py:228` and `mask = mask_orig[::2, ::2]` at line 229. Neither is a copy, which is why Ideas 81, 136 and 234 — and Idea A-28, which writes `ds = raw_depth_frame[::2, ::2]  # a view, zero cost` — all treat the decimation as free. It is not free; the cost is simply paid somewhere the source does not show.
  1. **Every OpenCV call on such a view silently copies it.** For a $(400, 640)$ uint8 `mask_orig`, the view `mask_orig[::2, ::2]` has shape $(200, 320)$ and strides $(1280, 2)$. OpenCV's `pyopencv_to` requires `strides[-1] == elemsize`; when that fails it sets `needcopy` and calls `PyArray_GETCONTIGUOUS`, allocating a fresh $64\text{ kB}$ buffer and running a strided gather **before** the operator executes. So `cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)` at line 240 pays an allocation plus a full gather that the code believes it avoided. (Line 241's second call is safe — it consumes line 240's contiguous *output*. And the hard failure path, `"Layout of the output array ... is incompatible"`, fires only for arrays passed as `dst`, which is why this has never surfaced as an error.)
  2. **A-28 as written inherits the same cost, on a bigger array.** A-28 proposes `cv2.inRange(ds, 0.5, 4.0, dst=self._mask_buf)` where `ds` is the float32 view with strides $(5120, 8)$. `inRange` will `GETCONTIGUOUS` it into a **$256\text{ kB}$** temporary every cycle — reintroducing exactly the per-cycle allocation A-28 exists to remove. The pre-allocated `dst` is contiguous and therefore accepted, so A-28 as specified saves the $64\text{ kB}$ output allocation and silently re-spends four times that on the input.
  3. **Pure-NumPy passes overfetch $2\times$.** DRAM is delivered in 64-byte lines. A stride-2 float32 read yields 8 useful values per line instead of 16 (skipped *rows* are never fetched, so the waste is $2\times$, not $4\times$). The far-range mask at line 235 and the write at line 236 therefore move $512\text{ kB}$ of bus traffic to touch $256\text{ kB}$ of data. The per-contour gathers at lines 272–274 — `depth_slice = raw_depth_frame[y_b:y_b+h_b, x_b:x_b+w_b]` and `depth_slice[cnt_mask == 255]` — inherit the parent's strides and additionally drop off NumPy's contiguous fast path onto the generic strided iterator.
- **Proposed Solution:** Decimate **once**, into a contiguous pre-allocated buffer, and let every downstream stage — mask build, morphology, contour extraction, per-blob depth gather — run on contiguous memory.
  ```python
  # __init__ (size lazily on the first frame; the OAK-D output shape can change — see A-42)
  self._ds_buf = None
  # _extract_depth_centroids, replacing line 228
  if self._ds_buf is None or self._ds_buf.shape != raw_depth_frame[::2, ::2].shape:
      self._ds_buf = np.empty(raw_depth_frame[::2, ::2].shape, dtype=np.float32)
  np.copyto(self._ds_buf, raw_depth_frame[::2, ::2])
  ds = self._ds_buf
  ```
  `np.copyto` performs the same strided gather OpenCV would have performed internally — but exactly **once** per cycle, into a buffer that is never reallocated, instead of once per `cv2` call plus implicitly inside every NumPy pass. It is also bit-identical to today's behaviour, so nothing downstream changes. `cv2.resize(raw_depth_frame, (w//2, h//2), dst=self._ds_buf, interpolation=cv2.INTER_NEAREST)` is the SIMD alternative and is faster, but note that at an exact $0.5$ scale `INTER_NEAREST` samples $(2i{+}1,\,2j{+}1)$ rather than $(2i,\,2j)$ — a one-pixel offset, immaterial for a $0.5\text{–}4.0\text{ m}$ range mask but not bit-identical, so `np.copyto` is the safer default and the `resize` form is the follow-up once A-28's single mask has been validated. Build the uint8 mask directly at the decimated resolution (which is what A-28's single `cv2.inRange` does, correctly, once `ds` is contiguous) so line 229's strided mask view disappears entirely.
- **Expected Benefit:** Removes one $\approx 256\text{ kB}$ and one $\approx 64\text{ kB}$ hidden allocation-plus-gather per cycle ($\approx 3\text{ MB/s}$ of heap churn that no profiler attributes to this file, because it happens inside `pyopencv_to`), and roughly halves the DRAM traffic of every subsequent full-frame pass — $\approx 0.5\text{ MB}$ per cycle of overfetch, $\approx 5\text{ MB/s}$ off a bus already shared with TensorRT YOLO. Direct saving is an estimated $0.4\text{–}0.8\text{ ms}$ of the $100\text{ ms}$ budget. The larger payoff is corrective: A-28 and A-02 are both costed on the assumption that the decimated frame is free to touch, and neither delivers its full estimate until that assumption is actually true.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-41`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
