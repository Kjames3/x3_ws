# OAK colored Surroundings: source audit and initial baseline

2026-10-30. Implementation remains gated on current-camera registration evidence.
No application source, camera configuration, service, or GUI changes made.

## Verified live source contract

The Orin's `/oak/rgbd/metadata` is arriving. Current device is Pro W IMX378,
MxId 14442C103181D7D600. Actual pair dimensions are **384x512**, not the historical
480x640. Use each pair's calibration, never hardcode the remembered dimensions.
Sample pair device timestamp skew is -3.281 ms; one sample is not a skew bound.
Depth is `raw_uncorrected`, with inverse-depth correction coefficient 0.0821/m.
`same_pixel_grid_verified` is false. Full metadata is saved in
`artifacts/oak-colored-cloud-2026-10-30/live-rgbd-metadata.yaml`.

Existing `RGBDPairer` is one-to-one and bounded, with a 20 ms skew threshold,
sequence/reset tracking and stream counters. Driver pairs are owned numpy arrays
from the capture path; verify immutability/ownership before any new fan-out.
`pop_rgbd_pairs()` is destructive and consumed by OakRosPublisher. A viewer must
use a separate latest-pair reference or fan-out, not steal this queue. Existing
pair calibration includes RGB distortion and separate RGB/depth K matrices.
`c1_dataset.rgb_on_depth_grid` supplies a nominal rectification and support mask;
this is not proof of physical registration. Reuse its math with cached/sampled
maps for eventual live work after qualification.

The planned architecture still applies: existing camera owner -> synchronized
pair -> latest-only optional worker -> colored cloud -> separate browser layer.
No camera SDK migration, new camera process, or pipeline bootstrap is required.

## Initial resource sample

18 saved one-second tegrastats samples during the existing workload: mean CPU
across cores 61.1%; individual core samples 47–73%, clocks varying. RAM about
1793–1803 MB of 7620 MB, GPU activity 0%, no swap in these samples. This is an
observational baseline only: browser visibility was uncontrolled and no matched
hidden/visible or latency test has been completed. Raw evidence:
`artifacts/oak-colored-cloud-2026-10-30/baseline-tegrastats.txt`.

## Registration gate

The 2026-09-23 rigid-board recordings belong to the retired IMX214 Lite, not the
current IMX378 Pro W. Their geometry/correction findings cannot qualify this
camera. The Pro W crop model is implemented and supported by later measurements,
but current pair metadata still does not assert same-pixel alignment.

Next: stationary flat target with colored regions and visible edges/background,
measured width and camera-to-front-face distance (~1–1.5 m). User was asked to
confirm placement. Capture a short synchronized dataset using the existing
`src/tools/perception/c1_dataset.py record` path, audit it, annotate the ROI from
RGB independently of depth, and score rectified edge alignment. Reuse the existing
owner/ROS streams; do not open another device connection. A second placement and
motion check follow if the first is sound. Do not fit a pixel shift to one target.

After geometry validation, implement the bounded display stream and compare
matched baseline/on trials. Full plan remains in
`2026-10-30-oak-colored-surroundings.md`.

## Offline filtering evaluation

Current Pro W purple-box captures now exist, with user width~25cm,
camera distance~124cm and raised support7.5cm. Both have60 complete pairs and
no dataset audit failures. Raised corrected interior depth~1.234m; perimeter
registration still fails. Offline 8/12px boundary exclusion tested on10 frames;
12px removes most spill-proxy points but retains only21% of RGB border region.
Actual6000-point rendering loses outline/print detail. See
artifacts/oak-colored-cloud-2026-10-30/raised/filter-evaluation.md and
cloud-detail.png. Do not deploy current filter as a successful recognition upgrade.
Next proposed offline refinement: neutral uncertain geometry plus confident
interior color, with spatially balanced sampling at unchanged bandwidth.
