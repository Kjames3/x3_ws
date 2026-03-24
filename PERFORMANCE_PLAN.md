# X3 Server Performance Optimization Plan

**Hardware:** Jetson Orin Nano, JetPack 6.1
**Constraints:** CPU-only PyTorch (libcusparseLt missing), TensorRT GPU inference via pycuda, asyncio WebSocket server

---

## Phase 1 — Event Loop Fixes (highest ROI, ~1 hr)

| # | Status | Issue | Fix | Est. Gain |
|---|--------|-------|-----|-----------|
| P1 | [x] | `cv2.imencode()` + `base64.b64encode()` block the event loop (~6–15ms) | Moved into executor closure alongside YOLO (and encode-only path) | 6–15ms/frame freed |
| P2 | [x] | `cv2.rectangle()` races with JPEG encode — both access `frame` from different threads simultaneously | Draws on `annotated = frame.copy()` inside `_run_yolo` | Correctness fix + stability |
| P9 | [x] | Battery voltage read at 20Hz (only meaningful at 1Hz) | `_batt_cache_v` refreshed only when `now - _batt_cache_time >= 1.0` | Minor CPU savings |
| P10 | [x] | ~150 bytes of always-`None`/`False` fields sent every frame (`target_pose`, `is_demo_mode`, `latest_log`) | Removed from per-frame payload | ~3KB/s/client saved |

---

## Phase 2 — Memory Allocation Reduction (~2 hrs)

| # | Status | Issue | Fix | Est. Gain |
|---|--------|-------|-----|-----------|
| P5 | [x] | `TRTDetector._preprocess()` did 3–4 intermediate numpy allocations per inference | `cv2.dnn.blobFromImage` single C++ pass → `np.copyto` into pre-allocated `_h_input` | 1–3ms/inference, less GC |
| P8 | [x] | 3 numpy allocations per depth frame (`normalize`, `applyColorMap`, `flip`) | Pre-allocated `_depth_buf_8` / `_depth_buf_col` in `_open_depth()`; passed as `dst=` | ~2ms/depth frame |
| P6 | [x] | `xyxy.tolist()` / `confs.tolist()` in `NMSBoxes` — wrong coordinate format + Python→C++ round-trip | Converted xyxy→xywh; pass numpy arrays directly | <1ms + correct IoU |

---

## Phase 3 — Throughput Architecture (~3–4 hrs)

| # | Status | Issue | Fix | Est. Gain |
|---|--------|-------|-----|-----------|
| P4 | [x] | Lidar `_scan_loop` did per-point Python `math.cos`/`math.sin`, built list-of-lists | Vectorised with numpy (`np.fromiter` + masked cos/sin); stores flat `[x0,y0,…]` float32 array; `drawLidar` updated to stride by 2 | 1–3ms/scan + ~33% smaller JSON |
| P3 | [x] | `json.dumps()` on ~50KB payload (base64 image + telemetry) blocked event loop 2–5ms | Camera frame sent as binary WebSocket message (raw JPEG bytes); GUI `onmessage` routes `Blob` directly to `img.src` via `createObjectURL`; JSON payload has no `"image"` key | 2–5ms/frame + ~25% bandwidth |
| P7 | [x] | `_capture_loop` did lock+copy even with no clients connected | `_has_clients` flag (set by server on connect/disconnect); capture loop skips lock+copy when `False` — buffer still drained | Idle CPU reduction |

---

## Bonus — GPU Inference Pipelining (~4–6 hrs)

`TRTDetector.__call__()` calls `stream.synchronize()` which stalls the CPU until the GPU finishes.
Use two alternating CUDA streams so GPU processes frame N while CPU captures frame N+1.
Eliminates GPU wait stall from the critical path entirely.

---

## Key Files

| File | Relevant Lines |
|------|----------------|
| `src/server_x3.py` | `broadcast_loop` (~400–500), `_run_yolo` closure, JSON payload assembly |
| `src/trt_detector.py` | `_preprocess()` (151–155), `_postprocess()` NMSBoxes (188–189) |
| `src/drivers_x3.py` | `_capture_loop` (238–244), `get_depth_frame` (301–319), `_scan_loop` (470–491) |
