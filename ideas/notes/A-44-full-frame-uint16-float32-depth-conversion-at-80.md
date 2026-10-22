---
id: A-44
title: "Full-Frame `uint16`→`float32` Depth Conversion at 80 fps Capture for a 10 Hz Consumer"
status: Logged
domain: Performance
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-44"
session: "3. Performance & Execution Efficiency"
tags: [idea]
---

# A-44 — Full-Frame `uint16`→`float32` Depth Conversion at 80 fps Capture for a 10 Hz Consumer

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Ten lines following a lazy-cache pattern already present in the same file, deletes ~164 MB/s of heap churn and ~290 MB/s of DRAM traffic)
- **Problem:** `OakDCamera._process_depth` (`src/oakd_driver.py:392–397`) runs once per frame the device delivers and does this unconditionally:
  ```python
  raw_m = depth_mm.astype(np.float32) / 1000.0
  ```
  `inDepth.getFrame()` at line 332 returns the device's **native uint16 millimetres** — $640 \times 400 = 256{,}000\text{ px} = 512\text{ kB}$. `.astype(np.float32)` allocates a $1.02\text{ MB}$ array, and the `/ 1000.0` is **not in place**, so it allocates a *second* $1.02\text{ MB}$ array. Each frame therefore costs $\approx 2.05\text{ MB}$ of fresh heap and $\approx 3.6\text{ MB}$ of memory traffic (read $0.5$, write $1.0$, read $1.0$, write $1.0$).

  The driver is constructed with `mono_fps=80` (`src/server_x3.py:1074`) and `_run` blocks on `qDepth.get()` at line 330, so this executes at the achieved capture rate. At $80\text{ fps}$ that is **$164\text{ MB/s}$ of allocation and $\approx 290\text{ MB/s}$ of LPDDR5 traffic** on a bus already shared with TensorRT YOLO — and Idea A-33 charges a further $\approx 100\text{ MB/s}$ to colourising the very same array. Even at a conservative achieved $30\text{ fps}$ it is $61\text{ MB/s}$ / $108\text{ MB/s}$.

  Almost none of that output is read. The float32-metres array has exactly three consumers: `VelocityEstimator._inference_loop` at $10\text{ Hz}$ (`src/velocity_estimator.py:430`), `OakDCamera._locate` at the NN rate `nn_fps = 12` (line 521), and `oakd_ros_publisher` only when `--oak-ros-publish` is set (line 170). At $80\text{ fps}$ capture that is at most $\approx 22$ of every $80$ frames, i.e. **$\ge 70\%$ of the conversion is computed and discarded before anything looks at it**. The file already knows this pattern is wrong and already solved it once: `_latest_depth_color` is explicitly set to `None` on every incoming frame (line 396) and colourised lazily on demand with a validity check (lines 579–591), for exactly this reason. The metric conversion sits on the same object, is just as deferrable, and was not given the same treatment.
- **Proposed Solution:** Store the native array; convert once, on demand, and only for consumers that need metres.
  1. Keep `self._latest_raw_depth_mm = depth_mm` in `_process_depth` and set `self._latest_raw_depth = None` beside the existing `_latest_depth_color = None` at line 396 — the invalidate-and-fill-lazily pattern already in the file. Fill it inside `get_raw_depth_frame()` with a single pass into a lazily-sized pre-allocated buffer, which also removes the second temporary permanently:
     ```python
     np.multiply(mm, 0.001, out=self._m_buf, dtype=np.float32)
     ```
     **Tearing caveat:** a shared `_m_buf` reintroduces exactly the hazard Idea J-24 describes, since a consumer may still hold the buffer when the next `get_raw_depth_frame()` rewrites it. Use J-24's two- or three-deep rotation rather than a single scratch buffer — the correctness requirement is the same, and this makes the buffer worth having.
  2. **Better for the hot path:** add `get_raw_depth_mm()` and let the estimator's obstacle mask work in millimetres directly. Every threshold in `_extract_depth_centroids` is a literal constant — `0.5 / 1.5 / 4.0` at lines 224, 235 and 274 become `500 / 1500 / 4000` — and the only place metres are genuinely needed is the single `Z` per blob at line 280, one scalar multiply. uint16 also **halves the bytes touched** by every full-frame pass in that function: $1.02\text{ MB} \to 512\text{ kB}$ before decimation and $256\text{ kB} \to 128\text{ kB}$ after it. `cv2.inRange` is fully typed for `CV_16U`, so this stacks directly on Idea A-28's single-mask pass and Idea A-41's contiguous decimation buffer instead of competing with them.
  3. Note this **inverts** June performance item #80, which proposed *adding* a float32→uint16 quantisation pass inside the estimator: the frame is natively uint16 and the correct move is to stop destroying that at the source, not to reconstruct it downstream. `oakd_ros_publisher` also benefits — a `16UC1` republish is the natural encoding for a mm frame and skips a conversion of its own.
  4. Second-order payoff: `_run`'s inner loop has a $12.5\text{ ms}$ budget at $80\text{ fps}$ and currently spends a substantial fraction of it here, on top of two `getCvFrame()` copies, the IMU drain and the $85 \times 6300$ NN decode — with `maxSize=1, blocking=False` silently dropping whatever it cannot keep up with. Removing this raises the *achieved* depth rate, which is precisely the quantity Idea A-43 shows the estimator's feature $\Delta t$ is a silent function of.
- **Expected Benefit:** Removes $\approx 164\text{ MB/s}$ of heap churn and $\approx 290\text{ MB/s}$ of DRAM traffic at the configured $80\text{ fps}$ ($\approx 61 / 108\text{ MB/s}$ at $30\text{ fps}$) from a thread that runs on every captured frame. Deferring the conversion cuts its execution count by $\ge 3.6\times$ immediately; moving the estimator's mask to millimetres removes it from the $10\text{ Hz}$ hot path entirely and halves the bytes touched by every full-frame pass in `_extract_depth_centroids`. Cost is about ten lines that follow a lazy-cache pattern already written, twice, in the same file.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-44`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
