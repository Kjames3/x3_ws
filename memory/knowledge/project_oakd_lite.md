---
name: project_oakd_lite
description: "OAK-D Lite on the X3 — how it is driven, the USB-C cable trap, measured hardware ceilings, and the custom yolo26 host-decode path"
metadata:
  node_type: memory
  type: project
  originSessionId: 9e1d02c6-85e6-45b4-8439-a2b19bca5dd4
  modified: 2026-08-08T06:25:17.680Z
---

The OAK-D Lite (MxId 1944301081C69F5A00) is mounted on the robot and driven **in-process** by
`src/oakd_driver.py` inside `server_x3.py`: mono L/R + on-device StereoDepth + BMI270 IMU + a
spatial-detection NN. **No color** — the Astra keeps RGB, freeing OAK bandwidth. DepthAI opens the
device exclusively, so **only one process can own the OAK**; anything else wanting OAK streams
(e.g. WebRTC stereo) must be folded into `server_x3.py`'s pipeline, not run as a separate script.
It is the depth source for `broadcast_loop` and VelocityEstimator, falling back to the Astra when
`oak.available` is False; `--no-oak` disables it. `src/oakd_camera_viewer.py` is the standalone
laptop viewer (it shows color; its FPS is capped by its single-threaded imshow loop, not the link).

**depthai is pinned to v2** (`2.32.0.0`, `python3 -m pip install --user "depthai<3"`). The driver
and viewer use the v2 XLinkOut/`getOutputQueue` API — v3 would break them.

**udev rule is REQUIRED** on any host (needs sudo, user runs it), or you get
`Insufficient permissions ... Failed to boot device`:
`echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules && sudo udevadm trigger`,
then replug. Device enumerates as `03e7:2485` (bootloader), `03e7:f63b` booted.

**USB-C cable trap — the single biggest time-waster here.** The bundled cable has ONE dead
SuperSpeed lane-set, so it enumerates at USB2 ("HIGH") on every host until you **flip the USB-C
plug orientation** (USB-C carries SS on two lane-sets, one per flip). Also plug **directly** into a
blue SS port — behind a USB2 hub the SS link never trains. This is not a Jetson, host, or code
issue. Headless speed test:
`python3 -c "import depthai as dai; d=dai.Device(dai.Pipeline(), maxUsbSpeed=dai.UsbSpeed.SUPER); print(d.getUsbSpeed().name); d.close()"`.
On USB2 the full pipeline runs ~3 fps vs ~35 at SUPER. The driver has `auto_economy=True`: if the
link isn't SUPER it rebuilds without the mono host streams (~15 MB/s) to keep depth+detection
alive, and `get_stereo_frames()` returns None (GUI stereo panels blank).

**Measured hardware ceiling (SUPER):** mono L/R + depth + IMU together = 45/45 fps mono, 38 fps
depth, 256 Hz IMU. Depth compute paces everything to ~40-45 fps; mono alone hits 100 **only with
depth off** — it is depth *or* 100 Hz stereo, never both. With the detection NN and CAM_A-aligned
depth running on the robot, expect ~20-37 fps depth depending on Jetson load.

**Spatial detections use a hand-rolled decode.** depthai v2's `YoloSpatialDetectionNetwork`
**cannot** decode the yolo26 head (it floods `Mask is not defined for output layer with width '85'`
every frame and pegs the host CPU). So `src/blobs/yolo26n/yolo26n.blob` runs as a plain
`dai.node.NeuralNetwork` and `oakd_driver._process_nn` decodes in host numpy. Locked decode mode
`('am', False)`: output is anchor-major `[6300,85]`, box cxcywh in **pixels** at 0:3, 80 class
scores at 4:83, ch84 unused, **no objectness** (score = max class). XYZ comes from CAM_A-aligned
depth (`setDepthAlign(CAM_A)`, `setOutputSize(480,640)`, intrinsics via `readCalibration`), sampled
at the 20th percentile with the box clamped to frame; `conf_threshold` 0.35. The blob is 8-shave
(4-shave may be faster). A `.superblob` cannot be loaded by depthai 2.32 — carve the base blob out
with `src/blobs/extract_superblob.py`.

**Never put a numpy scalar in a readout field** — `round(np.float64, 3)` crashed the broadcast with
`Type is not JSON serializable: numpy.float64` and put the service in a crash-loop that presented
as "works 5s then freezes". Always cast to native `float()`/`int()`.

Known accuracy limit: the NN input is 480x640 **portrait, stretched** from the landscape sensor
(`keepAspectRatio=False`), which distorts off-center subjects — letterboxing plus recomputed
intrinsics (`fx*=scale`, `cy+=pad`) is the main reliability lever left, and needs measured ground
truth to attempt.

Low-latency video: `src/webrtc_poc/` (OAK H.264 → ffmpeg → RTSP → mediamtx → browser) measures
**~172 ms glass-to-glass**, vs the GUI's stuttery 10 fps base64-JPEG-over-TCP path. Color is
IMX214-capped at 1080p/35 fps. **ffmpeg is not installed on the robot** (`sudo apt install -y ffmpeg`).
Sub-100 ms would need GStreamer webrtcbin to drop the RTSP hop.

History, benchmarks and superseded plans: `notes/oakd_lite_history.md`.
See [[project_oak_rosbag_recording]], [[project_robot_deploy]], [[project_jetson_cpu_profile]].
