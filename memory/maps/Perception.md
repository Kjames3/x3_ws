---
title: Perception
description: Map of content — cameras, YOLO, TensorRT, velocity estimation
tags: [moc, perception]
---

# Perception

Back to [[Home]].

## Cameras

- **Orbbec Astra Pro** — RGB + depth, driven through `src/drivers_x3.py`.
- **OAK-D Lite** — `oakd_camera_viewer.py`. USB3 + 1080P works; USB2 fallback needed when
  it crashes on 9001. FPS is loop-bound, not sensor-bound. YOLO already runs on the OAK's
  VPU, which is why moving it to the Jetson GPU would be a regression.
  → [[project_oakd_lite]]

## YOLO / TensorRT

Models live in `src/yolo_models/` subdirectories (`cans_models/`, `default/`); the active one
is resolved by `find_model_path()`, which searches recursively for `<name>.pt`.

`src/trt_detector.py` wraps `.engine` files and mimics the Ultralytics API. A working FP16
`yolo26n` engine exists. Hard-won details: the end2end output is shaped `(1, 300, 6)`, the
CUDA context needs explicit handling, and cleanup order matters. It only runs in the
non-WebRTC camera mode. → [[project_trt_engine]]

## Velocity estimation

The `velocity_MLP` work is the largest single thread in the idea logs — feature/scaler
contracts, translation normalization, fine-tune artifacts. Start at
`VELOCITY_SELF_TRAINING_PLAN.md`, then `august_improvement_ideas.md` (A-30, A-48 and
neighbours) and `august_roi_analysis.md`. → [[Ideas-and-Planning]]

Recurring theme worth internalizing: a `StandardScaler` is a property of the **dataset**, not
the architecture. Any fine-tune that touched the data needs its own exported scaler, or
serving silently applies the wrong transform on both ends.

## Cost

The OAK driver and the velocity estimator are the top CPU consumers on the Jetson.
There is no hardware video encoder on the Orin Nano. → [[project_jetson_cpu_profile]]

## Related

- [[Hardware]], [[Navigation-and-SLAM]], [[Data-and-Bags]]
