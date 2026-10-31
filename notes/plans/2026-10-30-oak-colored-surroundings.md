# OAK colored cloud for Surroundings

Status: proposed; planning only. No application changes or deployment authorized by this plan.

## Outcome and scope

Show recognizable, RGB-colored surfaces in the camera's observed region while
retaining the Livox's surrounding geometry. First release is a live, bounded
point cloud, not a persistent reconstruction or mesh. It is display-only: no
changes to /scan, CBF, Nav2, /oak/points, person tracking, camera model, subpixel
settings, IR emitters, or existing recording outputs.

Keep the existing camera pipeline and SDK version. Do not open a second camera
connection or migrate DepthAI for this feature. The current portrait crop limits
coverage; do not promise the full Pro W lens field of view. Wider RGB coverage is
a separate later experiment with its own calibration and bandwidth budget.

## 1. Establish the baseline and source contract

Inspect the deployed revision as well as local source. Relevant code:
`src/oakd_driver.py`, `src/rgbd_pairing.py`, `src/c1_camera_geometry.py`,
`src/oakd_ros_publisher.py`, `src/server_x3.py`, `src/web/main.js`, and
`src/web/scene-view.js`.

The existing record_rgbd path produces synchronized pairs with calibration and
metadata; `pop_rgbd_pairs()` drains its queue. Add a separate latest-pair snapshot
or fan-out after pairing, not a second draining consumer. Neither recording nor
existing ROS publication may lose pairs to the viewer. Avoid copying full frames
until needed; maintain explicit ownership so capture buffers cannot change under
the worker. Latest-frame queue capacity is one, dropping obsolete work.

Measure repeated runs with Surroundings hidden and visible, camera/perception
active, and Livox enabled. Record CPU, RAM, GPU, temperatures/clocks, camera and
scan rates, frame age, event-loop stalls, and WebSocket bandwidth. The earlier
single ~60% CPU sample is context, not a capacity benchmark. Compare identical
scenes/workloads and device power modes; measure tail latency as well as means.

## 2. Build and validate colored geometry offline

Reuse an existing suitable synchronized capture, or provide a short capture
procedure if no suitable recording exists. Validate calibration before live work:

- Use each pair's actual dimensions, crop, intrinsics, distortion model and
  acquisition timestamps. Do not assume RGB pixels and aligned-depth pixels
  correspond until the pairing path's geometry is verified.
- Use the existing crop-aware camera geometry, not a generic CAM_A intrinsics
  query. Handle RGB distortion/rectification consistently with aligned depth.
- Paired recording depth is raw. Apply the MxId-specific correction in
  `config/oak_depth_correction.json` exactly once (current unit uses
  Z_corrected = Z_raw / (1 + 0.0821 * Z_raw), with Z in metres). Never correct an
  already-corrected frame again.
- Back-project valid depths, sample the matching RGB pixels, and explicitly
  convert the current BGR representation into RGB channel order.
- Reject zero/nonfinite depths and large discontinuities; start with a
  configurable 0.4–3 m display range. This is a visualization cutoff, not a
  claimed sensor accuracy range. Keep filtering light and preserve thin objects.
- Transform through the verified optical-frame-to-base transform. Compare walls,
  furniture edges and floor against Livox to detect duplicated/misaligned surfaces.
  Do not adjust settled hardware measurements simply to hide an alignment error.

Verify with known-color objects, vertical edges and objects of known dimensions
at several ranges. Include a moving hand/person to expose temporal misalignment.
Record error versus distance; agree tolerances from the camera's measured noise
rather than forcing millimetre agreement. Any unexplained constant offset, scale
error or color displacement blocks live acceptance.

## 3. Publish a bounded display stream

Generate cloud data in a worker outside the motion/event-loop and capture threads.
Use vectorized NumPy initially; profile before considering GPU offload.
Reuse current pairs rather than adding full RGB/depth ROS streams just for display.

Proposed initial limits (targets to validate, not measured results):

- 3 Hz, at most 6,000 XYZRGB points/update; optional 5 Hz only after profiling.
- Sample the depth image spatially before back-projection; preserve representative
  coverage rather than truncate one image region. Add voxel filtering only if it
  improves quality enough to justify its sorting/allocation cost.
- Binary PointCloud2 on a separate `/surroundings/oak/points` topic, with explicit
  XYZ and packed RGB layout, little-endian contract, and acquisition timestamp.
  At 16 bytes/point this is at most 288 kB/s at 3 Hz or 480 kB/s at 5 Hz of point
  payload alone; ROS/WebSocket overhead and other streams are additional.
- Best-effort, bounded queue; no accumulating backlogs. Produce only when there
  is a subscriber. A disconnected browser must stop optional cloud work.
- Reuse the existing device-to-ROS timestamp mapping; reject/reset stale work
  across clock jumps or camera reconnects.

## 4. Add the browser layer

Offer an OAK color overlay alongside existing Livox Clean/Raw modes. Render
colored points with GPU vertex colors, not a mesh or one object per point.
Subscribe only while visible and enabled; release buffers/subscriptions on hide.

Start without OAK temporal accumulation. Place each cloud using robot pose at
its acquisition time; do not attach old measurements to the robot's current pose.
If pose is unavailable/stale, hide or clearly mark the cloud rather than displaying
false alignment. Reuse the existing pose machinery only after checking its history
covers the actual camera latency. Fade and remove clouds on timeout.

Keep Livox as a distinct layer, optionally dimmed where OAK color is shown. Do not
claim unseen space is clear, color the robot's rear from front-camera images, or
silently fuse contradictory surfaces. Reuse buffers and bound browser memory.

## 5. Resource and regression gates

Suggested initial acceptance budgets, to be revised against the baseline:

- Added mean Jetson CPU <=0.25 of one core (25 percentage points in process CPU
  reporting, not 25% of the whole six-core machine).
- Added steady RAM <=100 MiB and no growth across a 15-minute visible/hidden test.
- Cloud construction p95 <=15 ms at 3 Hz; end-to-end cloud age p95 <=500 ms,
  with timestamp validity checked. Record baseline camera age separately.
- Browser remains responsive, targeting >=30 FPS on the actual viewing device.
- Scan rate stays near its baseline ~10 Hz; no new >500 ms scan gaps, no material
  increase in motion-loop latency or perception dropouts, and no thermal throttling
  attributable to the feature. Compare paired baseline/on trials.

If budgets fail: reduce point count, range, then update rate; disable optional
work when hidden. If still over budget, defer deployment and consider browser-side
projection or a measured GPU implementation. Idle GPU usage alone is insufficient
justification for a new dependency or pipeline rewrite.

## 6. Validation and release

Automated checks: calibration/back-projection fixtures; exactly-once depth
correction; RGB order and binary layout; invalid depths; point budget; latest-only
queue/fan-out; acquisition-time pose placement; stale data and clock-jump handling.

Live validation proceeds stationary first: known objects, visibility toggling,
multiple browser clients, disconnect/reconnect, and camera reconnect. Then a
user-controlled short translation/turn to check registration and latency. Keep
moving people out of persistent layers. Compare screenshots and metrics against
Livox-only at the same viewpoint. Perform a planned cold boot to confirm startup
and subscriptions recover.

Release behind an opt-in flag/toggle initially. Record results and defaults before
making it generally enabled. Rollback disables the new publisher/overlay and
returns to the existing Livox view; no safety consumers depend on it.

Deferred: wider RGB stream, RGB projection onto Livox, persistent colored maps,
TSDF/mesh generation, and using OAK display points for navigation.

References: Luxonis RGBD documentation describes synchronized color/depth clouds
and host compute options: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/rgbd
This is conceptual guidance, not authorization to migrate the existing SDK.
