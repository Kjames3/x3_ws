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

---

## 2. Architecture & Algorithmic Enhancements

### Idea A-01: Global Optimal Bipartite Matching for Cross-Path Tracking
- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Moderate coding effort, significantly improves tracking during crowded scenes)
- **Problem:** `ObstacleTracker.update()` currently uses greedy nearest-neighbor matching by iterating over active tracks and popping the closest unmatched detection. In crowded scenes where pedestrians cross paths, greedy matching often results in identity swaps or lost tracks because the match order depends on dictionary iteration order rather than globally minimizing the total distance across all track-detection pairs. Furthermore, `det_coords` is repeatedly converted to a NumPy array inside the track loop.
- **Proposed Solution:** Compute a full pairwise distance matrix between all active tracks and all new detections simultaneously. Use `scipy.optimize.linear_sum_assignment` (the Hungarian algorithm) to find the globally optimal bipartite matching that minimizes the sum of squared distances.
- **Expected Benefit:** Eliminates tracking ID swaps during cross-path scenarios, stabilizes velocity histories for closely interacting pedestrians, and reduces CPU overhead by vectorizing the distance matrix computation.

---

## 3. Performance & Execution Efficiency

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
