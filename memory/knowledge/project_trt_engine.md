---
name: project_trt_engine
description: "Working TensorRT engine for yolo26n on the robot — how it was built, the trt_detector.py fixes, and when it actually runs"
metadata: 
  node_type: memory
  type: project
  originSessionId: af77d7db-7b2b-46e1-9815-f5c7278e3b0d
---

Built a working FP16 TensorRT engine for the Jetson-side YOLO on 2026-07-25: `src/yolo_models/default/yolo26n.engine` (plus `yolo26n.onnx`, `bus.jpg` test image — none in git). Build path (all ON the robot, engines are device-specific): `yolo export model=yolo26n.pt format=onnx imgsz=640 opset=13 simplify=True` → `/usr/src/tensorrt/bin/trtexec --onnx=yolo26n.onnx --saveEngine=... --fp16` (~8 min build on Orin, ~4.2 ms raw GPU compute, ~38 fps end-to-end incl. pre/post). trtexec + `tensorrt 10.3.0` + `pycuda` are all present on the robot. Server auto-prefers `.engine` over `.pt` (server_x3.py ~L1062) and logs "YOLO running on: TensorRT GPU". Depends on the CUDA torch env — see [[project_robot_deploy]].

**KEY: yolo26 is NMS-free / end-to-end.** Its ONNX/engine output is `(1, 300, 6)` = `[x1,y1,x2,y2,conf,cls]` in 640-input space, already NMS-filtered on-device — NOT the classic YOLOv8 `(1, 4+C, A)` cxcywh+NMS layout. The OLD broken "296-class" engine in the notes was never a different model; it was yolo26 end-to-end output being misread by `trt_detector.py` (`num_classes = out_shape[1]-4` = 300-4 = 296). Confirmed by matching engine output to `.pt` ground truth on bus.jpg (person/bus boxes align to ±2px, same confs).

Three fixes were required in `src/trt_detector.py` (edited locally + synced to robot) to make it work — WITHOUT these the engine loads but returns garbage/crashes:
1. **Output adapter**: added `self._end2end = (len(out_shape)==3 and out_shape[2]==6)`; `_postprocess` branches to just threshold+rescale (no transpose, no NMS) for end2end; `num_classes` taken from `.pt` names (80) not `out_shape[1]-4`.
2. **CUDA context mismatch → "Cask convolution execution" errors**: original `__init__` popped its context then called `torch.load()` (for class names) which left torch's primary context current, so the engine deserialize + `create_execution_context()` bound to torch's context while inference ran under `self._ctx`. Fix: load names FIRST, then `make_context()`, then do ALL engine/exec-context/buffer creation while `self._ctx` is current (single try/finally with `.pop()`).
3. **Shutdown segfault ("Error 709 destroying stream")**: `cleanup()` detached the context before destroying the TRT execution context/engine/stream. Fix: null out `_context/_engine/_stream` and free buffers while context is current, THEN pop+detach; made idempotent; added `__del__`.

**WHERE IT ACTUALLY RUNS (important):** the Jetson `model` (TRT or `.pt`) only feeds the GUI's 2D detection overlay in `broadcast_loop` (server_x3.py ~L1842), fed by `camera.get_frame()`. But the robot's systemd service runs with `--webrtc-camera`, and in that mode `camera` stays `None` (the `elif WEBRTC_CAMERA` branch ~L1028 never assigns it) → `frame` is always None → the overlay YOLO loop NEVER executes. So with `--webrtc-camera` the engine is loaded but idle (GR3D ~0, fps_detection None). It's only exercised when the server runs WITHOUT `--webrtc-camera` (Astra base64 camera mode). The OAK-D VPU does the real 3D spatial detection either way, independent of this engine.
