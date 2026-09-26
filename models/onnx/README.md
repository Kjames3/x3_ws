# Pose ONNX exports for OAK (RVC2) blobs

Exported 2026-09-26 from `models/yolo11n-pose.pt` and `models/yolo26n-pose.pt`
(Ultralytics 8.4.26, onnx 1.17, opset 12, onnxslim, static shape). The venv
is project-local: `.cache/onnx-export` (system site-packages + onnx/onnxslim/onnxruntime).

| File | Input | Output | sha256 |
|---|---|---|---|
| `yolo11n-pose-640x480.onnx` | `images` 1x3x640x480 f32 | `output0` 1x56x6300 | `a143022b…` |
| `yolo26n-pose-640x480.onnx` | `images` 1x3x640x480 f32 | `output0` 1x56x6300 | `be5d903d…` |

- **Same input and anchor grid as the deployed `src/blobs/yolo26n` detector:**
  640 high × 480 wide (80×60 + 40×30 + 20×15 = 6300 anchors), so it drops into
  the same OAK preview size.
- **Output rows:** 0–3 box cx,cy,w,h (pixels), 4 person score, 5–55 = 17 COCO
  keypoints × (x, y, visibility). No NMS in the graph; decode + NMS on the host
  as `oakd_driver._process_nn` does for the detector (which has 85 rows, not 56).
- **yolo26n-pose was exported with `end2end=False`** (the raw one-to-many head),
  matching how the detector blob is decoded. Ultralytics' default `.pt`
  prediction uses the NMS-free head instead and differs by a few pixels; with
  the head matched, both ONNX files reproduce their `.pt` keypoints to <0.001 px
  (checked on `.cache/pose-lite/sample-person.jpg`, CPU).

## Blob conversion

Use the same settings as `src/blobs/yolo26n/buildinfo.json` (Luxonis
modelconverter 0.5.4, OpenVINO 2022.3, MYRIAD):

```
mo --input_model <file>.onnx --output output0 --compress_to_fp16 \
   --input "images[1 3 640 480]{f32}" \
   --mean_values "images[0.0,0.0,0.0]" --scale_values "images[255.0,255.0,255.0]" \
   --reverse_input_channels
compile_tool -d MYRIAD -ip U8 -m <file>.xml -o <file>.blob   # 8 shaves, as the detector
```

`oakd_driver` would need a 56-row decode path before a pose blob can replace
the detector; nothing loads these yet. Check close-range keypoint visibility
from the low mount first (MediaPipe found a pose in only 30–55 % of frames
under 1.8 m, `artifacts/c3-eval-2026-09-25/pose_compare.json`).
