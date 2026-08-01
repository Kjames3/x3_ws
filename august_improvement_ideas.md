# August 2026 Improvement Ideas & Enhancement Log

This document serves as the active improvement ideas log and architectural roadmap for the EE244 Computational Learning Project (Predictive Local Planning via Onboard Velocity Estimation) for **August 2026**.

---

## 1. Prioritization & ROI Summary Matrix

| Idea ID | Date Logged | Domain | Title | ROI Tier | Status |
| :---: | :---: | :--- | :--- | :---: | :---: |
| **A-01** | 2026-08-01 | Architecture | Global Optimal Bipartite Matching for Cross-Path Tracking | **High** | Logged (Iter 1) |
| **A-02** | 2026-08-01 | Performance | Connected Components for Zero-Overhead Depth Blob Extraction | **High** | Logged (Iter 1) |
| **A-03** | 2026-08-01 | Performance | Chunked Serial Payload Reading for Reduced Syscall Overhead | **High** | Logged (Iter 1) |
| **A-04** | 2026-08-01 | Performance | Vectorized Struct Unpacking for Telemetry Parsing | **High** | Logged (Iter 1) |
| **A-05** | 2026-08-01 | Architecture | Depth EMA Applied After Global Projection Makes `alpha_z` a No-Op on MLP Features | **High** | Logged (Iter 1) |
| **A-06** | 2026-08-01 | Architecture | Proximity-Ranked Centroid Truncation Before the `MAX_OBSTACLES` Cutoff | **High** | Logged (Iter 1) |
| **A-07** | 2026-08-01 | Performance | Single-Pass Downsampled `cv2.inRange` Mask Shared by the Empty-Frame Fast Path | **High** | Logged (Iter 1) |
| **A-08** | 2026-08-01 | Architecture | Net-Displacement Stop Gate to Restore Kinematic Stop Detection Above the Depth Noise Floor | **High** | Logged (Iter 1) |

---

## 2. Architecture & Algorithmic Enhancements

### Idea A-05: Depth EMA Applied After Global Projection Makes `alpha_z` a No-Op on MLP Features
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Three-line fix, restores an already-written depth filter that currently never reaches the model input)
- **Problem:** `ObstacleTracker` was given a depth EMA (`self.alpha_z = 0.7`, `src/velocity_estimator.py:50`) that computes `cz_filtered = alpha_z * cz + (1 - alpha_z) * prev_cz` at lines 86–87. But the global centroid coordinates are projected **before** the tracker ever runs: `_inference_loop` builds `local_coords = [[cz, -cx_l] ...]` from the **raw** median depth at line 463 and produces `global_xy` at line 467, appending `(gx, gy, cz_raw)` at line 469. The tracker then stores `history_global.append((cx_g, cy_g, cz_filtered))` (line 93) — where `cx_g, cy_g` were already computed from the unfiltered `cz`. Feature construction consumes only `hist_g_arr[:, :2]` (line 529) and rebuilds `hist_local` from those two columns (line 533), so the third element `cz_filtered` is **discarded outright**. The net effect: the forward-range feature $r_x$ fed to the MLP is built from raw, unsmoothed depth, while the exported `z` and the 1.8 m proximity gate (line 485) use the filtered value — two inconsistent depth signals from the same track. On the OAK-D Lite at 400P, per-frame depth RMS noise at $2.0\text{ m}$ is $2\text{–}3\text{ cm}$; a $3\text{ cm}$ raw excursion over one $0.1\text{ s}$ frame enters `dx` as an apparent $0.3\text{ m/s}$ of pedestrian motion, which is 30% of a typical walking speed and the dominant jitter term in the MLP output for a standing person. Idea 284 (range-adaptive `alpha_z`) tunes a filter that is presently dead code with respect to the feature vector.
- **Proposed Solution:** The global projection is linear in depth, so the filtered global position is available in closed form without re-running the transform. Pass `cos_r`/`sin_r` (already computed at `_inference_loop` lines 460–461) into `ObstacleTracker.update()` and correct the global XY by the same rotation applied to the depth delta:
  $$\Delta z = cz_{\text{filt}} - cz_{\text{raw}}, \qquad g_x' = g_x + \Delta z \cos\theta_{\text{rob}}, \qquad g_y' = g_y + \Delta z \sin\theta_{\text{rob}}$$
  Store `(gx', gy', cz_filtered)` in `history_global` at line 93 so the smoothed depth propagates into the $r_x$ column the MLP actually reads. Apply the identical correction in the new-track branch (lines 111–118) so a track's first sample is consistent with its successors.
- **Expected Benefit:** Attenuates raw depth noise in the MLP forward-range feature by $\sqrt{\alpha/(2-\alpha)} = 27\%$ at the current $\alpha_z = 0.7$ (rising to ~42% if Idea 284's far-range $\alpha = 0.5$ is adopted), removing roughly $0.08\text{ m/s}$ of RMS velocity jitter on stationary targets at $2\text{ m}$, and eliminates the filtered/unfiltered depth inconsistency between the proximity gate and the model input.

### Idea A-06: Proximity-Ranked Centroid Truncation Before the `MAX_OBSTACLES` Cutoff
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (One-line sort, closes a silent detection-loss path on the nearest and most dangerous obstacle)
- **Problem:** `_extract_depth_centroids` returns `centroids[:MAX_OBSTACLES]` at `src/velocity_estimator.py:319` (raw-depth path) and `:361` (colorised fallback), truncating to 5 detections in **`cv2.findContours` emission order** — raster order over the mask, which has no relationship to range or collision risk. `ObstacleTracker.update()` then applies a second order-dependent cap, `if len(self.tracks) >= MAX_OBSTACLES: break` (lines 103–105). In a cluttered lab the $0.5\text{–}4.0\text{ m}$ band routinely yields 6–10 blobs (desk edges, chair legs, wall segments, doorframes), so a pedestrian at $1.2\text{ m}$ can be discarded while four wall fragments at $3.5\text{–}4.0\text{ m}$ are retained. Worse, contour emission order permutes frame to frame as blobs merge and split, so *which* five survive flips continuously: the pedestrian's track is destroyed and recreated, `visible_count` resets to 1 (line 115), and the `visible_count < 3` initiation gate at line 481 then suppresses inference entirely. The pedestrian is reported with $v_x = v_y = 0$ — or not reported at all — for as long as the clutter persists, and downstream TTC scaling sees no hazard.
- **Proposed Solution:** Rank candidates by collision relevance before truncating. The cheapest correct ordering is nearest-first on depth, `centroids.sort(key=lambda c: c[2])`; a slightly better one uses the lateral-offset-aware range $d = \sqrt{x_m^2 + Z^2}$, which prefers a target in the robot's corridor over an equidistant one at the FOV edge:
  $$\text{rank}(c) = \sqrt{x_m^2 + Z^2} \quad \text{(ascending)}$$
  Apply the same ordering guarantee in the tracker by sorting the `unmatched` index list by detection range before the new-track creation loop at line 103, so the `MAX_OBSTACLES` break drops the farthest candidate rather than the last-enumerated one.
- **Expected Benefit:** Guarantees that the five nearest obstacles are always the five that get tracked, eliminating the clutter-induced dropout of the closest pedestrian and the track-churn loop that pins `visible_count` below the inference gate. Removes a failure mode in which the robot is provably blind to its most imminent hazard while reporting five well-behaved zero-velocity background tracks.

### Idea A-08: Net-Displacement Stop Gate to Restore Kinematic Stop Detection Above the Depth Noise Floor
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Four-line change, revives an existing safety gate that fires ~14% of the time it should)
- **Problem:** The Kinematic Stop-Trigger gate in `_build_window_features` (`src/velocity_estimator.py:384–394`) declares a track stopped only when **every** consecutive step in the last three frames satisfies `d < 0.01` — an AND over two independent noisy measurements, with a $1\text{ cm}$ threshold that sits well below the sensor's own noise floor. Per-frame depth RMS on the OAK-D Lite at $2\text{ m}$ is $2\text{–}3\text{ cm}$, so for a genuinely motionless pedestrian each step has only $P(d < 0.01) \approx 0.38$ of clearing the threshold, and the conjunction fires with probability $\approx 0.38^2 \approx 0.14$. The gate is therefore effectively dead beyond about $1\text{ m}$: a standing person is passed to the MLP as a moving target, and the residual $\pm 0.03\text{ m}$ per-frame noise is regressed into $0.1\text{–}0.3\text{ m/s}$ of phantom velocity that feeds TTC speed scaling and produces intermittent, unexplained braking in front of stationary people. Because the test is a conjunction, it also *worsens* as the window ages — a single noisy frame anywhere in the last three vetoes the stop.
- **Proposed Solution:** Test net displacement across the window instead of requiring each step to be individually quiet. Over the last $K = 3$ frames spanning $(K-1)\Delta t = 0.2\text{ s}$, uncorrelated depth noise partially cancels in the endpoint difference while true motion accumulates linearly:
  $$\|\mathbf{s}_{-1} - \mathbf{s}_{-K}\| < v_{\text{stop}} \cdot (K-1)\Delta t, \qquad v_{\text{stop}} = 0.15\text{ m/s}$$
  i.e. `math.hypot(rx[-1] - rx[-3], ry[-1] - ry[-3]) < 0.03`. This is a strictly better estimator of "is this person moving" at the same cost, and its threshold is expressed in physical units ($\text{m/s}$) rather than an implicit per-frame distance. Guard it with `if len(history_local) >= 3` on the **unpadded** history so a freshly padded track (whose duplicated frames are trivially identical, line 375–376) cannot be misclassified as stopped.
- **Expected Benefit:** Raises the stop-gate hit rate on genuinely stationary pedestrians from ~14% to >95% at $2\text{ m}$ range, hard-zeroing $0.1\text{–}0.3\text{ m/s}$ of phantom velocity that currently reaches TTC scaling, and removes the corresponding intermittent braking events in front of standing people without adding any latency for pedestrians who actually start walking (a $0.15\text{ m/s}$ threshold is crossed within one frame of a real gait onset).

### Idea A-01: Global Optimal Bipartite Matching for Cross-Path Tracking
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Moderate coding effort, significantly improves tracking during crowded scenes)
- **Problem:** `ObstacleTracker.update()` currently uses greedy nearest-neighbor matching by iterating over active tracks and popping the closest unmatched detection. In crowded scenes where pedestrians cross paths, greedy matching often results in identity swaps or lost tracks because the match order depends on dictionary iteration order rather than globally minimizing the total distance across all track-detection pairs. Furthermore, `det_coords` is repeatedly converted to a NumPy array inside the track loop.
- **Proposed Solution:** Compute a full pairwise distance matrix between all active tracks and all new detections simultaneously. Use `scipy.optimize.linear_sum_assignment` (the Hungarian algorithm) to find the globally optimal bipartite matching that minimizes the sum of squared distances.
- **Expected Benefit:** Eliminates tracking ID swaps during cross-path scenarios, stabilizes velocity histories for closely interacting pedestrians, and reduces CPU overhead by vectorizing the distance matrix computation.

---

## 3. Performance & Execution Efficiency

### Idea A-07: Single-Pass Downsampled `cv2.inRange` Mask Shared by the Empty-Frame Fast Path
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, removes ~5 MB/s of redundant memory traffic and ~1.5 MB/cycle of heap churn)
- **Problem:** The OAK-D depth frame is $640 \times 400$ float32 = $1.02\text{ MB}$ (`monoLeft.setResolution(THE_400_P)`, `src/oakd_driver.py:223`, converted at `:393`). Every $10\text{ Hz}$ cycle it is scanned at **full resolution** three separate times before any useful work happens:
  1. `_inference_loop:434` — `np.any((raw_depth_frame >= 0.5) & (raw_depth_frame <= 4.0))` materialises **two** $256\text{ kB}$ boolean temporaries and evaluates the entire frame; `np.any` cannot short-circuit a comparison that has already been fully computed.
  2. `_extract_depth_centroids:221` — `mask_orig = np.zeros((480, 640), uint8)` allocates and zero-fills another $256\text{ kB}$ every call.
  3. `_extract_depth_centroids:224–225` — `close_mask = (raw >= 0.5) & (raw < 1.5)` builds two more full-resolution boolean temporaries, then `mask_orig[close_mask] = 255` performs a boolean fancy-index write over the whole frame.

  The close-range mask is then **immediately decimated** at line 229 (`mask = mask_orig[::2, ::2]`), discarding 75% of the pixels that steps 2 and 3 just paid full price to compute. The "stride 1, higher resolution" close-range branch documented in the Idea 143 comment (lines 217–225) is therefore illusory: the surviving mask is bit-identical to one computed directly on the already-downsampled frame. Idea 234 fixed exactly this pattern for the **far**-range grid; the close-range half and the emptiness probe were left in place. Total waste: ~6 full-resolution passes and ~1.5 MB of per-cycle allocations, on a platform where LPDDR5 bandwidth is shared with the GPU running TensorRT YOLO.
- **Proposed Solution:** Downsample once, mask once, and reuse the result. In `__init__`, pre-allocate `self._mask_buf = np.empty((200, 320), dtype=np.uint8)`. In `_extract_depth_centroids`, take the strided view **first** (`ds = raw_depth_frame[::2, ::2]` — a view, zero cost), then produce the entire $0.5\text{–}4.0\text{ m}$ obstacle mask with one SIMD C call writing into the pre-allocated destination:
  ```python
  cv2.inRange(ds, 0.5, 4.0, dst=self._mask_buf)
  ```
  Hoist the same call above the fast-path check in `_inference_loop` and replace lines 433–439 with `if cv2.countNonZero(self._mask_buf) == 0:` — `countNonZero` is a single-pass SIMD reduction over $\tfrac{1}{4}$ the pixels and genuinely short-circuits nothing it does not need. The close/far split collapses entirely, since both branches now operate on the same working-resolution grid.
- **Expected Benefit:** Cuts the pre-contour stage from ~6 full-resolution passes to a single quarter-resolution SIMD pass — an ~$24\times$ reduction in bytes touched before `findContours` — saving an estimated $2\text{–}4\text{ ms}$ of the $100\text{ ms}$ inference budget per cycle and eliminating $\sim1.5\text{ MB}$ of per-cycle allocation, i.e. $15\text{ MB/s}$ of GC pressure removed from the Jetson's shared memory bus. Composes cleanly with Idea A-02 (connected components), which consumes the same single mask.

### Idea A-02: Connected Components for Zero-Overhead Depth Blob Extraction
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, reduces image processing latency)
- **Problem:** In `_extract_depth_centroids`, `cv2.findContours` is used to identify obstacle blobs, followed by `cv2.moments` and `cv2.boundingRect` in a Python loop to find the centroid. For complex or noisy depth masks, contour tracing is CPU-intensive. Additionally, cropping the contour mask (`cv2.drawContours`) allocates a new NumPy array and performs rasterization for every detected object, driving up garbage collection overhead.
- **Proposed Solution:** Replace `cv2.findContours` and the contour-masking loop with `cv2.connectedComponentsWithStats`. This single highly-optimized C++ call directly returns the bounding boxes, areas, and centroids for all blobs in one pass without tracing perimeters. Use the bounding box directly to slice the depth frame.
- **Expected Benefit:** Reduces vision processing latency by 30-50% during crowded scenes and eliminates per-obstacle NumPy array allocations and rasterization calls.

### Idea A-03: Chunked Serial Payload Reading for Reduced Syscall Overhead
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, frees up kernel resources)
- **Problem:** The serial receive thread `__receive_data` in `src/Rosmaster_Lib.py` reads data one byte at a time using `self.ser.read()`. Each call triggers a kernel syscall and Python GIL context switch. Worse, each byte is wrapped in a new `bytearray(self.ser.read())[0]` object. For a 20-byte payload, this causes 20 syscalls and 20 temporary object allocations per packet, capping the maximum effective telemetry parsing rate and wasting CPU cycles.
- **Proposed Solution:** Once the header and `ext_len` are parsed, read the entire remaining payload in a single block: `payload = self.ser.read(ext_len - 2)`. Alternatively, read all available bytes into a circular bytearray buffer and parse packets out of the buffer synchronously.
- **Expected Benefit:** Drastically reduces kernel syscall overhead and Python object allocations, allowing the Jetson's CPU to spend more time on Nav2 and inference rather than serial I/O.

### Idea A-04: Vectorized Struct Unpacking for Telemetry Parsing
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding effort, accelerates UART data ingestion)
- **Problem:** In `__parse_data` of `src/Rosmaster_Lib.py`, data extraction is performed by repeatedly slicing the `ext_data` list and constructing a new `bytearray` for every single field: `struct.unpack('h', bytearray(ext_data[0:2]))[0]`. For IMU data (`FUNC_REPORT_ICM_RAW`), this happens 9 times per packet, creating massive object churn and slowing down telemetry ingestion.
- **Proposed Solution:** Convert the entire payload to `bytes` once: `data_bytes = bytes(ext_data)`. Then unpack all related fields simultaneously using `struct.unpack_from`. For example, `vx, vy, vz, voltage = struct.unpack_from('<hhhB', data_bytes, 0)`.
- **Expected Benefit:** Accelerates UART parsing by over 5x, eliminates hundreds of temporary `bytearray` allocations per second, and prevents the serial thread from becoming a bottleneck during high-frequency IMU telemetry (100Hz).
