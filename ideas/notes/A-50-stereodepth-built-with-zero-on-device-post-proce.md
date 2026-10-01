---
id: A-50
title: "StereoDepth Built With Zero On-Device Post-Processing — Every Depth Cleanup Pass Is Paid on the Jetson at 10 Hz"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-50"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-50 — StereoDepth Built With Zero On-Device Post-Processing — Every Depth Cleanup Pass Is Paid on the Jetson at 10 Hz

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Six lines in `_build_pipeline` that move the entire depth-cleanup stage onto the OAK's VPU, delete two host morphology passes, halve XLink payload, and attack the one error term no host smoother can)
- **Problem:** `OakDCamera._build_pipeline` configures `StereoDepth` with exactly three calls (`src/oakd_driver.py:230–232`):
  ```python
  stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
  stereo.setLeftRightCheck(self.left_right_check)   # True (line 107)
  stereo.setSubpixel(False)                          # Idea J-27's target
  ```
  `stereo.initialConfig` is **never touched anywhere in the file** — grep returns zero hits. So the device ships with the `DEFAULT` preset's permissive confidence threshold (245 of 255; the `HIGH_ACCURACY` preset uses 200) and with `postProcessing.speckleFilter`, `thresholdFilter`, `spatialFilter`, `temporalFilter` and `decimationFilter` all disabled. `_process_depth` (lines 392–397) then hands the raw device output straight to the host.

  Every cleanup stage the raw map needs is consequently paid on the Jetson, at $10\text{ Hz}$, inside `_extract_depth_centroids`:
  - the $0.5\text{–}4.0\text{ m}$ range tests at `src/velocity_estimator.py:224`, `235` and `274`;
  - **two full-frame $5\times5$ `MORPH_OPEN` / `MORPH_CLOSE` passes** at lines 239–241, whose entire purpose is to erase isolated invalid-disparity blobs;
  - the per-blob decimated median at lines 276–280 and the range-adaptive area gate at line 258, both of which exist to survive a noisy mask.

  The device can do all of it, on the Myriad-X, for zero Jetson cycles — and the confidence threshold does something the host provably **cannot**. Passive stereo with no IR projector (Idea A-31 established this camera has none) does not return holes on untextured lab walls; at a $245/255$ threshold it returns a *confident, wrong* disparity. Those values land inside the $[0.5, 4.0]$ acceptance band, survive the morphology because they are spatially coherent, and become blobs and tracks. That is a **bias**, not zero-mean noise, so Idea A-39's Savitzky–Golay smoother, Idea A-26's EMA and Idea 218's z-score rejection are all structurally unable to remove it — they attenuate variance around a mean that is already in the wrong place. Only a match-cost threshold at the source converts a wrong-but-confident disparity into a zero, which the range mask then rejects for free.
- **Proposed Solution:** Configure the stereo node's post-processing block in `_build_pipeline`, immediately after line 232.
  ```python
  stereo.initialConfig.setConfidenceThreshold(200)      # DEFAULT preset ships 245
  cfg = stereo.initialConfig.get()
  cfg.postProcessing.speckleFilter.enable      = True
  cfg.postProcessing.speckleFilter.speckleRange = 50
  cfg.postProcessing.thresholdFilter.minRange  = 500    # mm — the estimator's own 0.5 m
  cfg.postProcessing.thresholdFilter.maxRange  = 4000   # mm — the estimator's own 4.0 m
  cfg.postProcessing.decimationFilter.decimationFactor = 2
  cfg.postProcessing.decimationFilter.decimationMode   = \
      dai.RawStereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN
  stereo.initialConfig.set(cfg)
  stereo.setPostProcessingHardwareResources(3, 3)       # shaves/slices for the filter chain
  ```
  Taken in order of what each buys:
  1. **Threshold filter** performs on-device precisely what lines 224/235/274 do on the host, using the same two constants. Semantically a no-op downstream; it simply means the frame arrives already restricted to the band the estimator accepts.
  2. **Speckle filter** removes the isolated invalid-disparity islands that the host $5\times5$ `MORPH_OPEN` at line 240 exists to erase — the same pass Idea A-41 shows is silently paid *twice*, once inside `pyopencv_to`'s contiguity copy of the strided view and once in the operator itself. With speckle handled at the source, both morphology calls can be dropped or reduced to a $3\times3$ `MORPH_CLOSE`.
  3. **Confidence threshold at 200** is the accuracy item, and it is the only one that attacks a bias rather than a variance. Note the second-order payoff: fewer garbage blobs means fewer contours reaching `findContours` (line 243), which directly relieves the `MAX_OBSTACLES = 5` scarcity that Idea A-27 has to ration and Idea A-38 shows the floor already consumes.
  4. **Decimation filter, factor 2, `NON_ZERO_MEDIAN`** replaces the host `raw_depth_frame[::2, ::2]` at line 228 with an on-device **median of each $2\times2$** — strictly better than the host's point-sample, since a median rejects an invalid neighbour instead of possibly picking it. It halves the XLink depth payload ($512\text{ kB} \to 128\text{ kB}$ per frame at `THE_400_P`), which raises the *achieved* depth rate — exactly the quantity Idea A-43 shows the feature $\Delta t$ silently depends on — and it deletes Idea A-41's hidden contiguity copy and half of Idea A-44's conversion cost at the source.
     **Two hard preconditions.** (a) The estimator's own `[::2, ::2]` at line 228 must be *removed*, not stacked, or the working grid collapses to $160\times100$ and the adaptive area gate (line 258, already calibrated against $\text{MIN\_BLOB\_AREA}/4$) is off by another $4\times$. (b) The emitted resolution and therefore the intrinsics change, so this must land with or after **Idea A-42**, whose `get_intrinsics() -> (fx, fy, cx, cy, w, h)` self-describing accessor and shape assertion are what make a resolution change detectable instead of silent. Landing decimation before A-42 would create exactly the transposition A-42 was written to prevent.
  5. **Deliberately not proposed: the temporal filter.** DepthAI's `temporalFilter` is a causal IIR with persistency modes; on a walking pedestrian it smears the leading edge and biases depth toward the previous frame — the identical lag-for-jitter trade Idea A-39 rejects for the EMA, and worse here because it happens before the host can see it. Recording the exclusion explicitly so a later iteration does not add it as an apparent free win.
  6. Independent of Idea J-27 (`setSubpixel(True)`), which improves disparity *resolution*; this improves disparity *validity*. They stack, though subpixel raises the on-device compute budget, so if both land, check the achieved `get_depth_fps()` (`src/oakd_driver.py:602–605`) rather than assuming the requested `mono_fps = 80` survives.
- **Expected Benefit:** Moves the entire depth-cleanup stage from the Jetson's $10\text{ Hz}$ hot path onto the OAK's VPU: deletes two full-frame $5\times5$ morphology passes plus the strided-view contiguity copy Idea A-41 measures behind them, and halves the XLink depth payload from $512\text{ kB}$ to $128\text{ kB}$ per frame, which raises the achieved capture rate that Idea A-43 shows the feature $\Delta t$ silently tracks. On accuracy it removes the one error class no downstream filter can touch — confident-but-wrong disparities on untextured surfaces, which currently enter the acceptance band as spatially coherent phantom blobs, consume the $\text{MAX\_OBSTACLES} = 5$ budget, and are indistinguishable from an obstacle to every stage after them. Cost is six lines in `_build_pipeline` and one deleted line in `_extract_depth_centroids`.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-50`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
