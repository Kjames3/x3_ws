"""
oakd_driver.py — Headless DepthAI driver for the Luxonis OAK-D Lite.

Streams the OAK-D Lite's stereo (mono L/R), on-device metric depth, and 6-axis
IMU (BMI270), and — optionally — runs a YOLO detector on the device as a plain
NeuralNetwork whose raw output is decoded on the host (the color camera is used
as the NN input only; it is never streamed). 3D positions come from sampling the
metric depth map inside each box. The Orbbec Astra still supplies the RGB view.

Why host-side decode: depthai v2's on-device YoloSpatialDetectionNetwork cannot
parse the newer "yolo26" head (single [85, 6300] output). Feeding it there floods
the device with "Mask is not defined for output layer" errors every frame, which
pegs the host CPU. A plain NeuralNetwork just returns the raw tensor, and we do
NMS + back-projection in numpy here.

Depth getters mirror the AstraCamera / ROS2Bridge API so an OakDCamera instance
is a drop-in depth source for server_x3.py:

    get_depth_frame()       -> colourised BGR uint8 (white = near), or None
    get_raw_depth_frame()   -> float32 ndarray in METRES (0 = no return), or None
    get_depth_frame_age()   -> seconds since last depth frame (inf if never)
    get_frame()             -> None (no color camera on this path)

OAK-specific getters:

    get_stereo_frames()     -> (left, right) grayscale uint8, or (None, None)
    get_imu()               -> {"accel": {x,y,z}, "gyro": {x,y,z}, "ts": float} | None
    get_spatial_detections()-> [ {label, conf, bbox:[x1,y1,x2,y2],
                                   xyz_m:{x,y,z},        # camera optical frame
                                   xyz_base_m:{x,y,z}},  # base_link frame
                                  ... ]

Resilience: missing depthai / no device / a dropped link leave the getters
returning None/empty while the worker retries with exponential backoff.
"""
import json
import logging
import os
import threading
import time
import uuid
from collections import deque
from rgbd_pairing import RGBDPairer, timedelta_ns
from person_box_merge import group_overlapping, union_box

import numpy as np
import cv2

logger = logging.getLogger("x3_server")

try:
    import depthai as dai
    _DEPTHAI_AVAILABLE = True
except Exception as _exc:  # pragma: no cover - import guard
    dai = None
    _DEPTHAI_AVAILABLE = False
    logger.warning(f"OakDCamera: depthai import failed ({_exc}); OAK-D disabled")

DEPTH_MIN_M = 0.3
DEPTH_MAX_M = 5.0
MONO_W = 640
MONO_H = 400

# Static transform oak_rgb_camera_optical_frame -> base_link (from the measured
# X3 Plus URDF): the OAK-D Pro W sits above the Astra at x=0.107815 and its
# optical centre is z=0.1315 above base_link (0.213 m above the floor; lens
# centre measured 21.2-21.4 cm, housing bottom 19.4-19.5 cm, 2026-09-25). Optical
# convention is X right, Y down, Z forward, so base_x = MOUNT_X + z,
# base_y = -x, base_z = MOUNT_Z - y.
NN_STALL_S = 5.0          # no NN packet for this long while depth flows = stalled
OAK_MOUNT_X = 0.107815
OAK_MOUNT_Z = 0.1315
# Pose keypoint visibility threshold; same cut oak_pose_summary.py uses.
KPT_VIS = 0.5

DEPTH_CORRECTION_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                     "config", "oak_depth_correction.json")


def load_depth_correction(mxid, path=DEPTH_CORRECTION_PATH):
    """Return this device's inverse-depth offset (1/m), or 0.0 if it has none.

    Stereo on the Pro W reads long by a constant disparity offset (2.04 m reads
    2.44 m), so Z_true = Z / (1 + c * Z). See config/oak_depth_correction.json.
    """
    try:
        with open(path) as f:
            entry = json.load(f)["devices"].get(str(mxid))
    except (OSError, ValueError, KeyError) as e:
        logger.warning("OakDCamera: no depth correction loaded (%s)", e)
        return 0.0
    return float(entry["inv_depth_offset_per_m"]) if entry else 0.0

_COCO80 = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _nms(boxes, scores, iou_thres):
    """Plain numpy NMS. boxes = (N,4) xyxy. Returns kept indices."""
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thres]
    return keep


class OakDCamera:
    def __init__(self, mono_fps=30, nn_fps=12, usb2_mode=False, align_depth_to_left=True,
                 accel_hz=250, gyro_hz=200, left_right_check=True, sim_mode=False,
                 spatial_blob=None, spatial_config=None, auto_economy=True,
                 conf_threshold=None, speckle_filter=True, speckle_range=50,
                 record_rgbd=False, subpixel=False):
        self.record_rgbd = record_rgbd
        self.subpixel = bool(subpixel)
        self._rgbd_pairs = deque(maxlen=16)
        self._rgbd_drops = 0
        self._rgbd_calibration = None
        self._latest_detection_meta = None
        self._pairer = RGBDPairer(capacity=64)
        self._capture_session = str(uuid.uuid4())
        self.mono_fps = min(mono_fps, 30) if record_rgbd else mono_fps
        self.nn_fps = nn_fps
        self.usb2_mode = usb2_mode
        self._auto_economy = auto_economy   # on a USB2 link, stop streaming mono L/R
        self.align_depth_to_left = align_depth_to_left
        self.accel_hz = accel_hz
        self.gyro_hz = gyro_hz
        self.left_right_check = left_right_check
        self.speckle_filter = speckle_filter
        self.speckle_range = speckle_range
        self.sim_mode = sim_mode

        self.spatial_blob = spatial_blob
        self._want_spatial = bool(spatial_blob)
        self._spatial_ok = self._want_spatial
        self.labels = _COCO80
        self.nn_conf = 0.5
        self.nn_iou = 0.5
        self.nn_classes = 80
        self.nn_kpts = 0                  # >0 for a YOLO pose head (17 COCO keypoints)
        self.nn_masks = 0                 # >0 for a YOLO seg head (32 mask coefficients)
        self.nn_proto_name = None
        self.nn_proto_shape = None        # (n_masks, H, W) of the prototype output
        self._latest_masks = []           # per latest detection: bool (nn_h, nn_w) or None
        self._fast_layers = {}            # layer name -> True/False once the raw read is checked
        self.nn_w, self.nn_h = 480, 640
        self.nn_out_name = None
        if spatial_config:
            self._load_nn_config(spatial_config)
        if conf_threshold is not None:
            self.nn_conf = conf_threshold   # override config (lower = catches more)

        self._lock = threading.Lock()
        self._latest_left = None
        self._latest_right = None
        self._latest_depth_color = None   # lazily colourised from raw
        self._latest_raw_depth = None     # float32, metres
        self._last_depth_time = 0.0
        self._latest_imu = None
        self._latest_detections = []
        self._latest_detections_t = 0.0   # monotonic, last NN packet decoded
        self._latest_detections_capture_t = None  # monotonic capture time of its frame
        # (capture monotonic s, metres) of recent depth frames, so a detection can
        # be paired with the depth taken with its image, not the newest one. NN
        # packets arrive a few hundred ms after capture; at ~30 fps this is ~0.6 s.
        self._depth_history = deque(maxlen=20)
        self._det_log = None              # open NDJSON file (set_detection_log)
        self._det_log_name = None
        self.depth_fps = 0.0              # live depth/stereo capture rate (~1s window)
        self.nn_stalls = 0                # pipeline rebuilds forced by the NN watchdog

        # CAM_A intrinsics at (nn_w, nn_h), filled once the device is up.
        self._fx = self._fy = self._cx = self._cy = None
        self._inv_depth_offset = 0.0
        # Intrinsics for the stream emitted by the active depth pipeline. Spatial
        # mode aligns to CAM_A at the NN size; stereo-only mode aligns to CAM_B/C
        # at the native 400p mono size.
        self._depth_fx = self._depth_fy = None
        self._depth_cx = self._depth_cy = None
        self._depth_intr_size = None
        self._logged_nn = False
        self._decode_mode = None          # locked (reshape, has_obj) once identified

        self.available = False
        self.spatial_active = False
        self.economy = False              # True when running the USB2-lean pipeline
        self.usb_speed = None
        self._running = False
        self._thread = None

    def _load_nn_config(self, config_path):
        try:
            with open(config_path) as f:
                cfg = json.load(f)
            model = cfg.get("model", {})
            inp = (model.get("inputs") or [{}])[0]
            shape = inp.get("shape")
            if shape and len(shape) == 4:
                self.nn_h, self.nn_w = int(shape[2]), int(shape[3])
            outs = model.get("outputs") or [{}]
            self.nn_out_name = outs[0].get("name")
            # Segmentation heads add a prototype output (1, n_masks, H/4, W/4).
            if len(outs) > 1:
                self.nn_proto_name = outs[1].get("name")
                self.nn_proto_shape = tuple(int(v) for v in outs[1].get("shape", [])[1:])
            head = (model.get("heads") or [{}])[0]
            meta = head.get("metadata", {})
            if meta.get("classes"):
                self.labels = list(meta["classes"])
            self.nn_classes = int(meta.get("n_classes", len(self.labels)))
            self.nn_kpts = int(meta.get("n_keypoints") or 0)   # the detector config has null
            self.nn_masks = int(meta.get("n_masks") or 0)
            self.nn_conf = float(meta.get("conf_threshold", self.nn_conf))
            self.nn_iou = float(meta.get("iou_threshold", self.nn_iou))
            logger.info(f"OakDCamera: NN config — {self.nn_classes} classes, input "
                        f"{self.nn_w}x{self.nn_h}, out '{self.nn_out_name}', "
                        f"conf {self.nn_conf}, iou {self.nn_iou}"
                        + (f", {self.nn_kpts} keypoints" if self.nn_kpts else "")
                        + (f", {self.nn_masks} mask coefficients" if self.nn_masks else ""))
        except Exception as e:
            logger.error(f"OakDCamera: failed to read NN config {config_path}: {e}")

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        if self.sim_mode:
            logger.info("OakDCamera: sim_mode — driver not started")
            return
        if not _DEPTHAI_AVAILABLE:
            logger.error("OakDCamera: depthai not installed — OAK-D disabled")
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="oakd", daemon=True)
        self._thread.start()
        logger.info("OakDCamera: worker thread started"
                    + (" (spatial detection ON)" if self._want_spatial else ""))

    def set_detection_log(self, path, model_name=None):
        """Append one NDJSON line per decoded NN packet (empty ones too, so
        false detections can be counted per minute). Diagnostic; off by default."""
        self._det_log_name = model_name
        self._det_log = open(path, "a", buffering=1)
        logger.info(f"OakDCamera: logging detections to {path}")

    def _write_det_log(self):
        with self._lock:
            dets = list(self._latest_detections)
        try:
            self._det_log.write(json.dumps({
                "t_mono": round(self._latest_detections_t, 4), "t_wall": round(time.time(), 3),
                "model": self._det_log_name, "dets": dets}) + "\n")
        except (OSError, ValueError) as e:
            logger.warning(f"OakDCamera: detection log disabled: {e}")
            self._det_log = None

    def cleanup(self):
        self._running = False
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
        if self._det_log is not None:
            self._det_log.close()
            self._det_log = None
        self.available = False
        logger.info("OakDCamera: stopped")

    stop = cleanup

    # ------------------------------------------------------------------ pipeline
    def _build_pipeline(self, with_spatial, economy=False):
        pipeline = dai.Pipeline()

        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        imu = pipeline.create(dai.node.IMU)

        xoutDepth = pipeline.create(dai.node.XLinkOut); xoutDepth.setStreamName("depth")
        xoutImu = pipeline.create(dai.node.XLinkOut);   xoutImu.setStreamName("imu")
        if self.record_rgbd:
            xoutDepth.input.setBlocking(False)
            xoutDepth.input.setQueueSize(2)
        # Economy (USB2): don't ship mono L/R to the host — those XLink streams eat
        # ~15 MB/s, which starves depth on a 480 Mbit link. get_stereo_frames() then
        # returns None (GUI stereo panels stay blank); depth + detection are unaffected.
        if not economy and not self.record_rgbd:
            xoutLeft = pipeline.create(dai.node.XLinkOut);  xoutLeft.setStreamName("left")
            xoutRight = pipeline.create(dai.node.XLinkOut); xoutRight.setStreamName("right")

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoLeft.setFps(self.mono_fps)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoRight.setFps(self.mono_fps)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(self.left_right_check)
        stereo.setSubpixel(self.subpixel)
        if self.subpixel:
            stereo.initialConfig.setSubpixelFractionalBits(3)
            # DEFAULT reserves three post-processing SHAVEs/CMX slices.
            # With subpixel that leaves only seven SHAVEs for our 8-SHAVE NN.
            # Reserve two instead; keep filter settings intact and audit timing.
            stereo.setPostProcessingHardwareResources(2, 2)

        # Speckle removal. The pipeline previously ran with NO post-processing at
        # all, so isolated mismatched-disparity blobs reached consumers as real
        # geometry -- the exact shape that becomes a phantom obstacle. This runs
        # on-device and is the least invasive filter available: it removes small
        # disconnected regions and leaves genuine surfaces untouched.
        #
        # NOT enabling the temporal filter on purpose. It blends across frames,
        # which suppresses noise but also smears genuinely moving objects -- the
        # one signal the velocity estimator exists to measure.
        #
        # This feed is shared: /oak/points, the Nav2 oak_voxel_layer and octomap
        # all consume the same depth, so verify 3D mapping after changing it.
        if self.speckle_filter:
            try:
                cfg = stereo.initialConfig.get()
                cfg.postProcessing.speckleFilter.enable = True
                cfg.postProcessing.speckleFilter.speckleRange = self.speckle_range
                stereo.initialConfig.set(cfg)
                logger.info("OakDCamera: speckle filter on (range=%d)",
                            self.speckle_range)
            except Exception as exc:
                # An older depthai may not expose postProcessing; depth without
                # the filter is still correct, so do not take the camera down.
                logger.warning("OakDCamera: speckle filter unavailable (%s); "
                               "continuing without post-processing", exc)

        # Optional spatial (edge-preserving, small hole filling) + range threshold,
        # for A/B testing: X3_OAK_DEPTH_FILTERS=1. Still no temporal filter (see
        # above). Shared feed: check 3D mapping before making it the default.
        if os.environ.get("X3_OAK_DEPTH_FILTERS") == "1":
            try:
                cfg = stereo.initialConfig.get()
                sf = cfg.postProcessing.spatialFilter
                sf.enable = True
                sf.holeFillingRadius = 2
                sf.numIterations = 1
                sf.alpha = 0.5
                sf.delta = 0
                cfg.postProcessing.thresholdFilter.minRange = 200
                cfg.postProcessing.thresholdFilter.maxRange = 8000
                stereo.initialConfig.set(cfg)
                logger.info("OakDCamera: spatial filter + 0.2-8 m threshold ON (X3_OAK_DEPTH_FILTERS)")
            except Exception as exc:
                logger.warning("OakDCamera: depth filters unavailable (%s)", exc)

        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, self.accel_hz)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, self.gyro_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        if not economy and not self.record_rgbd:
            monoLeft.out.link(xoutLeft.input)
            monoRight.out.link(xoutRight.input)
        stereo.depth.link(xoutDepth.input)
        imu.out.link(xoutImu.input)

        if with_spatial:
            # Color (CAM_A) feeds the NN only — not streamed. Depth is aligned to
            # CAM_A and output at the NN resolution so a detection's box pixels
            # index the metric depth map directly for 3D back-projection.
            camRgb = pipeline.create(dai.node.ColorCamera)
            camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setPreviewSize(self.nn_w, self.nn_h)
            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            # Centre crop, NOT a stretch. The depth aligned to CAM_A below is the
            # centre 810x1080 of the 1080P frame at uniform scale (fx == fy ~677,
            # checked against the lidar 2026-09-28). With False the preview was the
            # full 1920 width squeezed into 480 px (horizontal fx ~285), so boxes
            # and depth used different columns: bearings came out 2.37x too small
            # (0.593 / 0.25) and off-centre boxes read the wall behind the person.
            camRgb.setPreviewKeepAspectRatio(True)
            camRgb.setFps(self.nn_fps)

            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(self.nn_w, self.nn_h)

            nn = pipeline.create(dai.node.NeuralNetwork)
            nn.setBlobPath(self.spatial_blob)
            nn.setNumInferenceThreads(2)
            nn.input.setBlocking(False)
            if self.record_rgbd:
                # NN queue references must not exhaust the RGB preview pool.
                nn.input.setQueueSize(1)
                camRgb.setPreviewNumFramesPool(8)
            camRgb.preview.link(nn.input)
            if self.record_rgbd:
                xoutRgb = pipeline.create(dai.node.XLinkOut)
                xoutRgb.setStreamName("record_rgb")
                xoutRgb.input.setBlocking(False)
                xoutRgb.input.setQueueSize(4)
                camRgb.preview.link(xoutRgb.input)

            xoutDet = pipeline.create(dai.node.XLinkOut)
            xoutDet.setStreamName("det")
            nn.out.link(xoutDet.input)
        else:
            align_socket = (dai.CameraBoardSocket.CAM_B if self.align_depth_to_left
                            else dai.CameraBoardSocket.CAM_C)
            stereo.setDepthAlign(align_socket)
            stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

        return pipeline

    def _run(self):
        backoff = 1.0
        spatial_failures = 0
        economy = False
        while self._running:
            with_spatial = self._want_spatial and self._spatial_ok
            try:
                pipeline = self._build_pipeline(with_spatial, economy)
                max_speed = dai.UsbSpeed.HIGH if self.usb2_mode else dai.UsbSpeed.SUPER
                with dai.Device(pipeline, maxUsbSpeed=max_speed) as device:
                    self.usb_speed = device.getUsbSpeed().name
                    # Auto-economy: match the pipeline to the link. On USB2 drop the mono
                    # streams so depth/detection keep the bandwidth; on USB3 restore full.
                    _want_eco = self._auto_economy and self.usb_speed not in ("SUPER", "SUPER_PLUS")
                    if _want_eco != economy:
                        economy = _want_eco
                        if economy:
                            logger.warning(f"OakDCamera: USB {self.usb_speed} (USB2 link) — economy "
                                           "mode ON: mono L/R not streamed so depth + detection keep "
                                           "the bandwidth (GUI stereo blank). USB3 restores full mode.")
                        else:
                            logger.info("OakDCamera: USB3 link — economy mode OFF (full stereo restored)")
                        continue    # exits `with` (closes device), rebuilds to match the link
                    self.economy = economy
                    self.available = True
                    self.spatial_active = with_spatial
                    backoff = 1.0
                    self._capture_session = str(uuid.uuid4())
                    self._pairer = RGBDPairer(capacity=64)
                    with self._lock:
                        self._rgbd_pairs.clear()
                        self._latest_detection_meta = None
                    self._read_intrinsics(device, with_spatial)
                    if self.record_rgbd and not with_spatial:
                        logger.error("C1 RGB-D unavailable: CAM_A/NN pipeline inactive")
                    logger.info(f"OakDCamera: connected (USB {self.usb_speed}) — depth + imu"
                                + ("" if economy else " + stereo")
                                + (" + host-decoded detections" if with_spatial else "")
                                + (" [ECONOMY/USB2]" if economy else ""))

                    # Live consumers get newest frames. C1 retains a bounded depth
                    # history so later-delivered RGB can find its temporal neighbour;
                    # non-blocking queues never backpressure the device.
                    if not economy and not self.record_rgbd:
                        qLeft = device.getOutputQueue("left", maxSize=1, blocking=False)
                        qRight = device.getOutputQueue("right", maxSize=1, blocking=False)
                    else:
                        qLeft = qRight = None
                    qDepth = device.getOutputQueue("depth", maxSize=32 if self.record_rgbd else 1, blocking=False)
                    qImu = device.getOutputQueue("imu", maxSize=20, blocking=False)
                    qDet = device.getOutputQueue("det", maxSize=1, blocking=False) if with_spatial else None

                    qRgb = (device.getOutputQueue("record_rgb", maxSize=4, blocking=False)
                            if self.record_rgbd and with_spatial else None)
                    fps_n, fps_t = 0, time.monotonic()
                    # NN stall watchdog. The NN node has been seen to stop emitting
                    # (2026-09-28, yolo11n-pose + C1 recording) while depth and the
                    # shared RGB preview kept flowing, silently. Every NN packet is
                    # normally a fresh answer (empty or not), so a gap means a stall.
                    det_last_rx = None
                    fps_win_n, fps_win_t = 0, fps_t   # ~1s window for the live get_depth_fps() value
                    # Block on the depth queue (paces the loop at the depth rate, no busy-spin).
                    while self._running:
                        inDepth = qDepth.get()          # blocks until the next (freshest) depth frame
                        if inDepth is not None:
                            if qRgb is not None:
                                # Retain device-time history for slower RGB delivery.
                                # The live consumer still gets only the newest depth.
                                batch = [inDepth] + qDepth.tryGetAll()
                                for depth_packet in batch:
                                    depth_mm = depth_packet.getFrame()
                                    self._enqueue_pairs(self._pairer.add('depth',
                                        {'image': depth_mm, 'meta': self._packet_meta(depth_packet)}))
                            else:
                                depth_packet = inDepth
                                depth_mm = inDepth.getFrame()
                            # getTimestamp() is the host-synced capture time on the
                            # same clock as time.monotonic() (checked: 0.001 ms).
                            self._process_depth(depth_mm, depth_packet.getTimestamp().total_seconds())
                            fps_n += 1
                            fps_win_n += 1
                            _now = time.monotonic()
                            # Live depth FPS on a ~1s window (one division/sec — negligible cost)
                            if _now - fps_win_t >= 1.0:
                                self.depth_fps = round(fps_win_n / (_now - fps_win_t), 1)
                                fps_win_n, fps_win_t = 0, _now
                            if _now - fps_t >= 5.0:
                                logger.info(f"OakDCamera: depth {fps_n / (_now - fps_t):.1f} fps "
                                            f"(USB {self.usb_speed}, mono {self.mono_fps} req"
                                            f"{', economy' if economy else ''})")
                                fps_n, fps_t = 0, _now
                        if qRgb is not None:
                            for inRgb in qRgb.tryGetAll():
                                meta = self._packet_meta(inRgb)
                                self._enqueue_pairs(self._pairer.add('rgb',
                                    {'image': inRgb.getCvFrame(), 'meta': meta}))
                        if qLeft is not None:
                            inLeft = qLeft.tryGet()
                            if inLeft is not None:
                                with self._lock:
                                    self._latest_left = inLeft.getCvFrame()
                        if qRight is not None:
                            inRight = qRight.tryGet()
                            if inRight is not None:
                                with self._lock:
                                    self._latest_right = inRight.getCvFrame()
                        inImu = qImu.tryGet()
                        if inImu is not None:
                            self._process_imu(inImu)
                        if qDet is not None:
                            inDet = qDet.tryGet()
                            if inDet is not None:
                                det_last_rx = time.monotonic()
                                detection_meta = self._packet_meta(inDet)
                                self._process_nn(inDet, detection_meta)
                            elif (det_last_rx is not None
                                  and time.monotonic() - det_last_rx > NN_STALL_S):
                                self.nn_stalls += 1
                                logger.error(f"OakDCamera: NN stalled ({NN_STALL_S:.0f} s without a "
                                             f"packet while depth flows); rebuilding the pipeline "
                                             f"(stall #{self.nn_stalls})")
                                break   # leaves `with`, closes the device; the outer loop rebuilds
            except Exception as e:
                self.available = False
                self.spatial_active = False
                if not self._running:
                    break
                if with_spatial:
                    spatial_failures += 1
                    if spatial_failures >= 2:
                        logger.error(f"OakDCamera: spatial pipeline failed twice ({e}); "
                                     "falling back to stereo+depth+imu only")
                        self._spatial_ok = False
                        continue
                logger.error(f"OakDCamera: device error ({e}); retrying in {backoff:.0f}s")
                time.sleep(backoff)
                backoff = min(backoff * 2, 10.0)
        self.available = False
        self.spatial_active = False

    def _read_intrinsics(self, device, with_spatial=True):
        mxid = device.getMxId() if hasattr(device, "getMxId") else None
        self._inv_depth_offset = load_depth_correction(mxid)
        logger.info("OakDCamera: %s depth correction 1/Z offset %.4f 1/m",
                    mxid, self._inv_depth_offset)
        self._rgbd_calibration = None
        # A reconnect may change pipeline mode. Invalidate first so callers can
        # never consume calibration left over from the previous stream geometry.
        with self._lock:
            self._depth_fx = self._depth_fy = None
            self._depth_cx = self._depth_cy = None
            self._depth_intr_size = None
        try:
            calib = device.readCalibration()
            if with_spatial:
                socket = dai.CameraBoardSocket.CAM_A
                width, height = self.nn_w, self.nn_h
            else:
                socket = (dai.CameraBoardSocket.CAM_B if self.align_depth_to_left
                          else dai.CameraBoardSocket.CAM_C)
                width, height = MONO_W, MONO_H

            M = calib.getCameraIntrinsics(socket, width, height)
            if with_spatial:
                # The 480x640 grid is a centre crop of the 16:9 ISP output at
                # uniform scale, which M above does not model (fx, fy ~2.4x too
                # small). Map the EEPROM K through the real sensor crop instead.
                from c1_camera_geometry import rgb_1080_preview_crop, rgb_1080_preview_intrinsics
                native_k, native_w, native_h = calib.getDefaultIntrinsics(socket)
                sensor_name = next(f.sensorName for f in device.getConnectedCameraFeatures()
                                   if f.socket == socket)
                try:
                    M = rgb_1080_preview_intrinsics(native_k,
                        (native_w, native_h), (width, height), sensor_name)
                except ValueError as exc:
                    logger.error("OakDCamera: %s; falling back to uniform-scale "
                                 "intrinsics, which are WRONG on a cropped preview", exc)
                else:
                    if self.record_rgbd:
                        crop_xywh = [round(v, 1) for v in rgb_1080_preview_crop(
                            sensor_name, (native_w, native_h), (width, height))]
                        with self._lock:
                            self._rgbd_calibration = dict(width=width, height=height,
                                requested_mono_fps=self.mono_fps, requested_rgb_fps=self.nn_fps,
                                stereo_subpixel=self.subpixel,
                                nn_inference_threads=2,
                                stereo_postprocessing_shaves=2 if self.subpixel else 3,
                                stereo_postprocessing_memory_slices=2 if self.subpixel else 3,
                                subpixel_fractional_bits=3 if self.subpixel else 0,
                                left_right_check=self.left_right_check,
                                speckle_filter=self.speckle_filter, speckle_range=self.speckle_range,
                                rgb_k=np.asarray(M).reshape(-1).tolist(),
                                rgb_d=list(calib.getDistortionCoefficients(socket)),
                                rgb_sensor_name=sensor_name,
                                rgb_native_k=np.asarray(native_k).reshape(-1).tolist(),
                                rgb_native_size=[native_w,native_h],
                                rgb_sensor_mode='1080P', rgb_crop_xywh=crop_xywh,
                                rgb_binning=2,
                                depth_k=np.asarray(M).reshape(-1).tolist(),
                                rgb_geometry=f'{sensor_name}_1080P_bin2_preview_centre_crop',
                                depth_geometry='CAM_A_aligned_stereo_depth',
                                # Recorded depth stays RAW; replay applies
                                # Z/(1 + c*Z) with this per-device c.
                                device_mxid=mxid,
                                depth_inv_offset_per_m=self._inv_depth_offset,
                                depth_recorded='raw_uncorrected',
                                same_pixel_grid_verified=False)
                        logger.info("C1 RGB calibration: %s native %sx%s, preview crop %s -> %sx%s",
                                    sensor_name,native_w,native_h,crop_xywh,width,height)
            fx, fy = float(M[0][0]), float(M[1][1])
            cx, cy = float(M[0][2]), float(M[1][2])
            if not np.isfinite((fx, fy, cx, cy)).all() or fx <= 0.0 or fy <= 0.0:
                raise ValueError("invalid camera intrinsic matrix")

            with self._lock:
                self._depth_fx, self._depth_fy = fx, fy
                self._depth_cx, self._depth_cy = cx, cy
                self._depth_intr_size = (width, height)
                # Host-side NN localisation always operates in the CAM_A-aligned
                # spatial stream, so retain its existing short field names.
                if with_spatial:
                    self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy

            socket_name = getattr(socket, "name", str(socket))
            logger.info(f"OakDCamera: {socket_name} depth intrinsics @ {width}x{height} "
                        f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
        except Exception as e:
            logger.error(f"OakDCamera: readCalibration failed: {e}")

    # ------------------------------------------------------------------ processing
    def _packet_meta(self, packet):
        receipt_mono = time.monotonic_ns()
        receipt_unix = time.time_ns()
        before = time.monotonic_ns()
        sdk_now = timedelta_ns(dai.Clock.now())
        after = time.monotonic_ns()
        source_host = timedelta_ns(packet.getTimestamp())
        return dict(session_id=self._capture_session, seq=int(packet.getSequenceNum()),
                    device_ns=timedelta_ns(packet.getTimestampDevice()),
                    sdk_host_ns=source_host,
                    host_monotonic_estimate_ns=(before + after) // 2 + source_host - sdk_now,
                    host_clock_sample_span_ns=after - before,
                    receipt_monotonic_ns=receipt_mono, receipt_unix_ns=receipt_unix,
                    timestamp_kind='depthai_message_timestamp',
                    clock_sync='depthai_sdk', hard_clock_error_bound_ns=None)

    def _enqueue_pairs(self, pairs):
        with self._lock:
            for pair in pairs:
                if len(self._rgbd_pairs) == self._rgbd_pairs.maxlen:
                    self._rgbd_drops += 1
                pair['calibration'] = self._rgbd_calibration
                pair['publisher_queue_drops'] = self._rgbd_drops
                self._rgbd_pairs.append(pair)

    def pop_rgbd_pairs(self):
        with self._lock:
            pairs = list(self._rgbd_pairs)
            self._rgbd_pairs.clear()
        return pairs

    def get_detection_observation(self):
        with self._lock:
            return self._latest_detections, self._latest_detection_meta

    def _process_depth(self, depth_mm, capture_t=None):
        raw_m = depth_mm.astype(np.float32) / 1000.0
        if self._inv_depth_offset:
            raw_m /= 1.0 + self._inv_depth_offset * raw_m   # 0 (invalid) stays 0
        with self._lock:
            self._latest_raw_depth = raw_m
            if capture_t is not None:
                self._depth_history.append((capture_t, raw_m))
            self._latest_depth_color = None   # invalidate; colourise lazily on demand
            self._last_depth_time = time.monotonic()

    def _colourise(self, raw_m):
        clean = np.where(raw_m <= 0.1, DEPTH_MAX_M, raw_m)
        clean = np.clip(clean, DEPTH_MIN_M, DEPTH_MAX_M)
        min_d = float(clean.min())
        max_d = float(clean.max())
        if max_d > min_d:
            norm = (255.0 * (1.0 - (clean - min_d) / (max_d - min_d))).astype(np.uint8)
        else:
            norm = np.zeros_like(clean, dtype=np.uint8)
        return cv2.applyColorMap(norm, cv2.COLORMAP_BONE)

    def _process_imu(self, imu_data):
        packets = imu_data.packets
        if not packets:
            return
        last = packets[-1]
        accel = last.acceleroMeter
        gyro = last.gyroscope
        sample = {
            "accel": {"x": float(accel.x), "y": float(accel.y), "z": float(accel.z)},
            "gyro":  {"x": float(gyro.x),  "y": float(gyro.y),  "z": float(gyro.z)},
            "ts": time.monotonic(),
        }
        with self._lock:
            self._latest_imu = sample

    # Candidate interpretations of the yolo26 [85, 6300] tensor. The RVC2 compiler's
    # memory order (channel- vs anchor-major) and whether the head carries an
    # objectness channel aren't documented for this model, so we auto-pick the one
    # that yields sane, SPARSE detections (a real scene has few boxes; a wrong layout
    # scrambles into thousands). The pick is logged once, then locked.
    # A pose head ([4 box, n_classes, 3 * n_keypoints] rows) has no objectness.
    _DECODE_MODES = [("cm", False), ("cm", True), ("am", False), ("am", True)]
    _N_ANCHORS = 6300

    @property
    def _nn_rows(self):
        if self.nn_kpts:
            return 4 + self.nn_classes + 3 * self.nn_kpts
        if self.nn_masks:
            return 4 + self.nn_classes + self.nn_masks
        return 85

    def _decode(self, raw, mode):
        """Returns (box cxcywh, scores, cls_id, extra): extra is keypoints
        (N, K, 3) for a pose head, mask coefficients (N, M) for a seg head,
        else None."""
        reshape_mode, has_obj = mode
        rows, n = self._nn_rows, self._N_ANCHORS
        base = raw[:rows * n]
        A = base.reshape(rows, n).T if reshape_mode == "cm" else base.reshape(n, rows)
        box = np.ascontiguousarray(A[:, 0:4])
        kpts = None
        if self.nn_kpts:
            obj = None
            cls = A[:, 4:4 + self.nn_classes]
            # Ultralytics' exported pose head already decodes keypoints to
            # input pixels and applies the visibility sigmoid.
            kpts = A[:, 4 + self.nn_classes:].reshape(n, self.nn_kpts, 3)
        elif self.nn_masks:
            obj = None
            cls = A[:, 4:4 + self.nn_classes]
            kpts = A[:, 4 + self.nn_classes:4 + self.nn_classes + self.nn_masks]
        elif has_obj:
            obj = A[:, 4]
            cls = A[:, 5:85]
        else:
            obj = None
            cls = A[:, 4:84]
        if cls.max() > 1.0 or cls.min() < 0.0:
            cls = _sigmoid(cls)
        cls_id = cls.argmax(axis=1)
        cls_sc = cls[np.arange(cls.shape[0]), cls_id]
        if obj is not None:
            if obj.max() > 1.0 or obj.min() < 0.0:
                obj = _sigmoid(obj)
            scores = obj * cls_sc
        else:
            scores = cls_sc
        return box, scores.astype(np.float32), cls_id, kpts

    def _mode_plausible(self, box, scores):
        """Plausible = boxes lie inside the frame and detections are sparse."""
        cx, cy = box[:, 0], box[:, 1]
        sx = self.nn_w if cx.max() > 2.0 else 1.0
        sy = self.nn_h if cy.max() > 2.0 else 1.0
        in_range = float(np.mean((cx >= -0.2 * sx) & (cx <= 1.2 * sx) &
                                 (cy >= -0.2 * sy) & (cy <= 1.2 * sy)))
        n_hits = int((scores >= self.nn_conf).sum())
        return (in_range > 0.9 and 1 <= n_hits <= 300), n_hits

    def _process_nn(self, nndata, metadata=None):
        before = self._latest_detections_t
        self._decode_nn(nndata, metadata)
        if self._det_log is not None and self._latest_detections_t != before:
            self._write_det_log()

    def _decode_nn(self, nndata, metadata=None):
        """Host-side decode of the YOLO head (detect [85, 6300] or pose [56, 6300])
        + depth back-projection."""
        try:
            raw = self._read_layer(nndata, self.nn_out_name)
        except Exception:
            return
        # Stamp every decoded packet, including ones with no detections: "no
        # person this frame" is a fresh answer, not a missing one.
        try:
            self._latest_detections_capture_t = nndata.getTimestamp().total_seconds()
        except Exception:
            self._latest_detections_capture_t = None
        self._latest_detections_t = time.monotonic()
        if raw.size < self._nn_rows * self._N_ANCHORS:
            with self._lock:
                self._latest_detections = []
                self._latest_detection_meta = metadata
            return

        if self._decode_mode is None:
            best, report = None, []
            for mode in self._DECODE_MODES:
                if (self.nn_kpts or self.nn_masks) and mode[1]:
                    continue   # pose / seg heads have no objectness row
                b, s, _, _ = self._decode(raw, mode)
                ok, n_hits = self._mode_plausible(b, s)
                report.append(f"{mode[0]}{'+obj' if mode[1] else ''}={n_hits}{'*' if ok else ''}")
                if ok and (best is None or n_hits < best[1]):
                    best = (mode, n_hits)
            if not self._logged_nn:
                self._logged_nn = True
                logger.info("OAK NN decode candidates (hits@%.2f): %s"
                            % (self.nn_conf, " ".join(report)))
            if best is None:
                with self._lock:
                    self._latest_detections = []
                self._latest_detection_meta = metadata
                return
            self._decode_mode = best[0]
            logger.info(f"OAK NN: locked decode mode {self._decode_mode}")

        box, scores, cls_id, kpts = self._decode(raw, self._decode_mode)
        coef = None
        if self.nn_masks:
            kpts, coef = None, kpts
        keep = scores >= self.nn_conf
        if not keep.any():
            with self._lock:
                self._latest_detections = []
                self._latest_masks = []
                self._latest_detection_meta = metadata
            return
        box, scores, cls_id = box[keep], scores[keep], cls_id[keep]
        if kpts is not None:
            kpts = kpts[keep].copy()
        if coef is not None:
            coef = coef[keep]
        # Cap work before the pure-python NMS (O(n^2)): keep only the top-100 by score.
        if scores.shape[0] > 100:
            top = np.argpartition(scores, -100)[-100:]
            box, scores, cls_id = box[top], scores[top], cls_id[top]
            if kpts is not None:
                kpts = kpts[top]
            if coef is not None:
                coef = coef[top]
        # Box coords: normalised (<=~1) -> scale to pixels; else already pixels.
        if box.max() <= 2.0:
            box[:, [0, 2]] *= self.nn_w
            box[:, [1, 3]] *= self.nn_h
            if kpts is not None:
                kpts[:, :, 0] *= self.nn_w
                kpts[:, :, 1] *= self.nn_h
        # cxcywh -> xyxy
        xyxy = np.empty_like(box)
        xyxy[:, 0] = box[:, 0] - box[:, 2] / 2.0
        xyxy[:, 1] = box[:, 1] - box[:, 3] / 2.0
        xyxy[:, 2] = box[:, 0] + box[:, 2] / 2.0
        xyxy[:, 3] = box[:, 1] + box[:, 3] / 2.0

        kept = _nms(xyxy, scores, self.nn_iou)
        depth = self.get_raw_depth_frame()   # (nn_h, nn_w) metres, CAM_A-aligned
        protos = self._read_protos(nndata) if coef is not None else None
        # Candidates first, then (seg only) merge the person boxes that cover
        # one person, then locate: the 3D point must come from the merged
        # silhouette, not from one of its halves.
        cands = []
        for i in kept[:20]:
            label = self.labels[cls_id[i]] if 0 <= cls_id[i] < len(self.labels) else str(cls_id[i])
            mask = None
            if protos is not None and label == "person":
                mask = self._person_mask(protos, coef[i], tuple(xyxy[i]))
            cands.append({"label": label, "conf": float(scores[i]),
                          "box": [float(v) for v in xyxy[i]], "mask": mask,
                          "kpts": kpts[i] if kpts is not None else None})
        if protos is not None:
            cands = self._merge_people(cands)
        out, masks = [], []
        for c in cands:
            x1, y1, x2, y2 = c["box"]
            det = {
                "label": c["label"],
                "conf": c["conf"],
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "xyz_m": None,
                "xyz_base_m": None,
            }
            if c["kpts"] is not None:
                # COCO-17 order; [u, v, visibility] in NN/depth-grid pixels.
                det["keypoints"] = [[round(float(u), 1), round(float(v), 1), round(float(k), 3)]
                                    for u, v, k in c["kpts"]]
            mask = c["mask"]
            if protos is not None and c["label"] == "person":
                det["mask_px"] = int(mask.sum()) if mask is not None else 0
            if c.get("merged_from", 1) > 1:
                det["merged_from"] = c["merged_from"]
            masks.append(mask)
            xyz = self._locate(depth, x1, y1, x2, y2, mask)
            if xyz is not None:
                x, y, z = xyz
                det["xyz_m"] = {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)}
                det["xyz_base_m"] = {"x": round(OAK_MOUNT_X + z, 3),
                                     "y": round(-x, 3),
                                     "z": round(OAK_MOUNT_Z - y, 3)}
                if c["kpts"] is not None:
                    det["keypoints_xyz"] = self._keypoints_xyz(c["kpts"], z)
            out.append(det)
        with self._lock:
            self._latest_detections = out
            self._latest_masks = masks
            self._latest_detection_meta = metadata

    @staticmethod
    def _merge_people(cands):
        """Seg heads split a close person into overlapping partial boxes; make
        each group one detection (union box, max confidence, OR of silhouettes).
        Same rule as the offline scorer (person_box_merge). Other labels and
        candidate order are kept."""
        people = [k for k, c in enumerate(cands) if c["label"] == "person"]
        groups = group_overlapping([cands[k]["box"] for k in people])
        if len(groups) == len(people):
            return cands
        drop = set()
        for g in groups:
            if len(g) == 1:
                continue
            members = [cands[people[j]] for j in g]
            first = members[0]
            first["box"] = union_box([m["box"] for m in members])
            first["conf"] = max(m["conf"] for m in members)
            sils = [m["mask"] for m in members if m["mask"] is not None]
            first["mask"] = np.logical_or.reduce(sils) if sils else None
            first["merged_from"] = len(members)
            drop.update(people[j] for j in g[1:])
        return [c for k, c in enumerate(cands) if k not in drop]

    def _keypoints_xyz(self, kpts, z):
        """COCO-17 keypoints -> CAM_A optical-frame points, all at the box depth z.

        Per-keypoint depth is deliberately not sampled: on thin limbs the depth
        window mostly hits the background. Flattening onto the torso plane loses
        reach toward/away from the camera but keeps limb directions stable.
        Keypoints below KPT_VIS are None.
        """
        out = []
        for u, v, c in kpts:
            if c < KPT_VIS:
                out.append(None)
            else:
                out.append([round(float((u - self._cx) * z / self._fx), 3),
                            round(float((v - self._cy) * z / self._fy), 3),
                            round(float(z), 3)])
        return out

    def _read_layer(self, nndata, name):
        """One NN output layer as float32 (name None = first layer).

        getLayerFp16 hands back a Python list: 15 ms to convert for a pose head
        and 55 ms for a seg head (1.3 M values per packet) on the Orin, on top
        of DepthAI building the list, all holding the GIL. That halved the GUI
        telemetry rate with seg (2026-10-01). The fast path views the packet's
        raw FP16 bytes instead. It is checked against the list path on the
        first packet of each layer and used only if the two agree exactly.
        """
        def slow():
            vals = nndata.getLayerFp16(name) if name else nndata.getFirstLayerFp16()
            return np.array(vals, dtype=np.float32)

        key = name or ""
        state = self._fast_layers.get(key)
        if state is False or not hasattr(nndata, "getData"):
            return slow()
        try:
            layers = nndata.getAllLayers()
            info = next(t for t in layers if t.name == name) if name else layers[0]
            if "FP16" not in str(info.dataType):
                raise ValueError(f"layer is {info.dataType}, not FP16")
            count = int(np.prod([d for d in info.dims if d > 0]))
            buf = np.asarray(nndata.getData(), dtype=np.uint8)
            fast = np.frombuffer(buf, dtype="<f2", count=count,
                                 offset=int(info.offset)).astype(np.float32)
        except Exception as e:
            if state is None:
                logger.warning(f"OakDCamera: fast NN read unavailable for '{key or 'first layer'}' ({e}); "
                               "using the list path")
            self._fast_layers[key] = False
            return slow()
        if state is None:
            ref = slow()
            ok = ref.shape == fast.shape and np.array_equal(ref, fast)
            self._fast_layers[key] = ok
            logger.info(f"OakDCamera: fast NN read for '{key or 'first layer'}' "
                        + ("verified against the list path" if ok else "DISAGREES with the list path; disabled"))
            return ref
        return fast

    def _read_protos(self, nndata):
        """Seg prototypes as (n_masks, H, W) float32, or None."""
        try:
            p = self._read_layer(nndata, self.nn_proto_name)
            return p[:int(np.prod(self.nn_proto_shape))].reshape(self.nn_proto_shape)
        except Exception as e:
            if not getattr(self, "_logged_proto", False):
                self._logged_proto = True
                logger.warning(f"OakDCamera: seg prototypes unreadable ({e}); boxes only")
            return None

    def _person_mask(self, protos, coef, box):
        """Silhouette as bool (nn_h, nn_w): sigmoid(coef . protos) > 0.5, inside the box."""
        m, ph, pw = protos.shape
        sy, sx = self.nn_h // ph, self.nn_w // pw
        x1, y1, x2, y2 = (int(round(v)) for v in box)
        y1c, y2c = max(0, y1), min(self.nn_h, y2)
        x1c, x2c = max(0, x1), min(self.nn_w, x2)
        if y2c <= y1c or x2c <= x1c:
            return None
        # Only the prototype cells the box touches: the mask is zero elsewhere.
        py1, py2 = y1c // sy, -(-y2c // sy)
        px1, px2 = x1c // sx, -(-x2c // sx)
        sub = protos[:, py1:py2, px1:px2]
        small = np.tensordot(coef[:m], sub, axes=1) > 0.0   # sigmoid(x) > 0.5  <=>  x > 0
        if not small.any():
            return None
        up = np.repeat(np.repeat(small, sy, axis=0), sx, axis=1)
        crop = np.zeros((self.nn_h, self.nn_w), dtype=bool)
        crop[y1c:y2c, x1c:x2c] = up[y1c - py1 * sy:y2c - py1 * sy, x1c - px1 * sx:x2c - px1 * sx]
        return crop if crop.any() else None

    def get_detections_with_masks(self):
        """(detections, masks) from the same packet, read atomically."""
        with self._lock:
            return list(self._latest_detections), list(self._latest_masks)

    def get_detection_masks(self):
        """Masks aligned with get_detection_observation()[0] (None where unavailable).
        Kept apart from the detection dicts: those are broadcast as JSON."""
        with self._lock:
            return list(self._latest_masks)

    def _locate(self, depth, x1, y1, x2, y2, mask=None):
        """Depth of the subject -> 3D point in the CAM_A optical frame.

        With a seg mask: median depth over the silhouette (no background by
        construction), at the silhouette's centroid. Without: 20th percentile in
        the inner half of the box.
        """
        if depth is None or None in (self._fx, self._fy, self._cx, self._cy):
            return None
        if mask is not None and mask.shape == depth.shape[:2]:
            # The mask is the prototype grid upscaled by s, so every s-th pixel
            # keeps its shape at 1/s^2 of the work (this ran on ~150k pixels per
            # close person). Small silhouettes keep full resolution.
            # Go to full resolution when too few samples have valid depth
            # (a far person, mostly past the depth limit).
            s0 = max(1, mask.shape[0] // self.nn_proto_shape[1]) if self.nn_proto_shape else 1
            for s in (s0, 1):
                d = depth[::s, ::s]
                sel = mask[::s, ::s] & (d > DEPTH_MIN_M) & (d < DEPTH_MAX_M)
                vs, us = np.nonzero(sel)
                if s == 1 or vs.size >= 200:
                    break
            if vs.size >= 30:
                vals = d[vs, us]
                z0 = float(np.median(vals))
                near = np.abs(vals - z0) < max(0.3, 3 * float(np.median(np.abs(vals - z0))))
                z = float(np.median(vals[near]))
                # Sample (i, j) stands for the s x s block starting at (i*s, j*s).
                u = float(us[near].mean()) * s + (s - 1) / 2.0
                v = float(vs[near].mean()) * s + (s - 1) / 2.0
                x = float((u - self._cx) * z / self._fx)
                y = float((v - self._cy) * z / self._fy)
                return x, y, z
        h, w = depth.shape[:2]
        # Clamp the box to the frame first: a detection that runs off-frame (common with
        # the distorted NN input) otherwise puts the sample point off the subject and
        # onto background, inflating Z (and the X/Y that scale with it).
        x1 = min(max(x1, 0.0), w - 1.0); x2 = min(max(x2, 0.0), w - 1.0)
        y1 = min(max(y1, 0.0), h - 1.0); y2 = min(max(y2, 0.0), h - 1.0)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        bw = (x2 - x1) * 0.25
        bh = (y2 - y1) * 0.25
        u0 = max(0, int(cx - bw)); u1 = min(w, int(cx + bw) + 1)
        v0 = max(0, int(cy - bh)); v1 = min(h, int(cy + bh) + 1)
        if u1 <= u0 or v1 <= v0:
            return None
        roi = depth[v0:v1, u0:u1]
        valid = roi[(roi > DEPTH_MIN_M) & (roi < DEPTH_MAX_M)]
        if valid.size < 4:
            return None
        # 20th percentile, not median: bias to the nearer surface (the subject) so a
        # box that also sees the wall/floor behind them doesn't push Z too far.
        z = float(np.percentile(valid, 20))
        # Native python floats — numpy scalars aren't JSON-serialisable and would
        # crash the readout broadcast (orjson/json TypeError).
        x = float((cx - self._cx) * z / self._fx)
        y = float((cy - self._cy) * z / self._fy)
        return x, y, z

    # ------------------------------------------------------------------ getters
    def get_frame(self):
        return None

    def get_depth_frame(self):
        with self._lock:
            if self._latest_depth_color is not None:
                return self._latest_depth_color
            raw = self._latest_raw_depth
        if raw is None:
            return None
        coloured = self._colourise(raw)
        with self._lock:
            # Only cache if the raw frame hasn't been replaced meanwhile.
            if self._latest_raw_depth is raw:
                self._latest_depth_color = coloured
        return coloured

    def get_detection_capture_time(self):
        """Monotonic capture time of the frame behind the latest detections."""
        return self._latest_detections_capture_t

    def get_depth_near(self, capture_t, max_gap_s=0.05):
        """(capture_t, depth metres) of the stored frame nearest capture_t, or None."""
        with self._lock:
            hist = list(self._depth_history)
        if not hist or capture_t is None:
            return None
        t, d = min(hist, key=lambda h: abs(h[0] - capture_t))
        return (t, d) if abs(t - capture_t) <= max_gap_s else None

    def get_raw_depth_frame(self):
        with self._lock:
            return self._latest_raw_depth

    def get_depth_frame_age(self) -> float:
        with self._lock:
            t = self._last_depth_time
        return float("inf") if t == 0.0 else time.monotonic() - t

    def get_depth_fps(self) -> float:
        """Live depth/stereo capture rate (Hz), updated on a ~1s window by the
        worker thread. Cheap read of a cached float — no capture-path impact."""
        return self.depth_fps

    def get_depth_intrinsics(self):
        """Return ``(fx, fy, cx, cy, width, height)`` for the emitted depth map.

        The width and height identify the resolution at which calibration was
        queried; consumers can safely rescale it if an actual frame differs.
        """
        with self._lock:
            if self._depth_intr_size is None:
                return None
            width, height = self._depth_intr_size
            return (self._depth_fx, self._depth_fy,
                    self._depth_cx, self._depth_cy, width, height)

    def get_stereo_frames(self):
        with self._lock:
            return self._latest_left, self._latest_right

    def get_imu(self):
        with self._lock:
            return self._latest_imu

    def get_detection_age(self):
        """Seconds since the last NN packet was decoded (inf if never)."""
        t = self._latest_detections_t
        return time.monotonic() - t if t else float("inf")

    def get_spatial_detections(self):
        with self._lock:
            return list(self._latest_detections)
