# Smaller portrait pose latency experiment

Built locally 2026-10-02 from models/yolo26n-pose.pt using the existing
.cache/onnx-export environment (Ultralytics 8.4.26). Export arguments:
format=onnx, imgsz=(512,384), opset=12, simplify=True, dynamic=False,
end2end=False, device=cpu. Input images float32 NCHW 1x3x512x384;
output0 1x56x4032, unchanged COCO-17 raw pose head and host parser.

Compilation follows ../README-4shave.md, substituting input dimensions
512,384, using the same local ModelConverter Docker image, OpenVINO 2022.3,
FP16 weights, U8 input, reverse channels, scale 255, and four SHAVEs/CMX slices.
Working files and scripts: .cache/pose-512. Checksums are recorded alongside.
Base .blob is a symlink to the four-SHAVE blob (no eight-SHAVE variant here).

CPU OpenVINO IR versus ONNX on resized sample-person.jpg: mean absolute
output difference 0.0272, maximum 2.303; output shape verified. Robot DepthAI
2.32 blob inspection confirms input [384,512,3,1], output [4032,56,1]. This
is conversion evidence, not live foot-localization accuracy validation.

Select --oak-model yolo26n-pose-512 with X3_OAK_BLOB_SHAVES=4 and
X3_OAK_NN_THREADS=2. Preserve latest-output setting for comparison. Same
portrait aspect ratio; driver also reduces aligned depth dimensions, so
this is a pipeline-resolution comparison, not isolated NN timing.
Robot test selection: 99-oak-small-pose.conf systemd drop-in. Remove that
drop-in and reload systemd to restore the prior full-size model selection;
restart remains user-owned. Two threads are restored separately in
98-oak-nn-threads.conf. Capture label: foot-small-pose.
