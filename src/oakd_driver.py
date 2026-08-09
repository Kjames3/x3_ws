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
import threading
import time

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

# Static transform oak_rgb_camera_optical_frame -> base_link (from the URDF):
# OAK mounts at base_link x=0.0435, z=0.185; optical convention X right, Y down,
# Z forward. So base_x = MOUNT_X + z, base_y = -x, base_z = MOUNT_Z - y.
OAK_MOUNT_X = 0.0435
OAK_MOUNT_Z = 0.185

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
                 conf_threshold=None):
        self.mono_fps = mono_fps
        self.nn_fps = nn_fps
        self.usb2_mode = usb2_mode
        self._auto_economy = auto_economy   # on a USB2 link, stop streaming mono L/R
        self.align_depth_to_left = align_depth_to_left
        self.accel_hz = accel_hz
        self.gyro_hz = gyro_hz
        self.left_right_check = left_right_check
        self.sim_mode = sim_mode

        self.spatial_blob = spatial_blob
        self._want_spatial = bool(spatial_blob)
        self._spatial_ok = self._want_spatial
        self.labels = _COCO80
        self.nn_conf = 0.5
        self.nn_iou = 0.5
        self.nn_classes = 80
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
        self.depth_fps = 0.0              # live depth/stereo capture rate (~1s window)

        # Intrinsics of the socket the depth map is aligned to, at the size the
        # stereo node actually outputs. Both depend on whether the spatial NN
        # branch is built (CAM_A @ nn_w x nn_h) or not (CAM_B/CAM_C @ mono res),
        # so _build_pipeline records them and _read_intrinsics reads that pair.
        # Getting this wrong silently skews every back-projected point.
        self._fx = self._fy = self._cx = self._cy = None
        self._intr_socket = None
        self._intr_size = None      # (w, h) the intrinsics above are valid for
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
            out = (model.get("outputs") or [{}])[0]
            self.nn_out_name = out.get("name")
            head = (model.get("heads") or [{}])[0]
            meta = head.get("metadata", {})
            if meta.get("classes"):
                self.labels = list(meta["classes"])
            self.nn_classes = int(meta.get("n_classes", len(self.labels)))
            self.nn_conf = float(meta.get("conf_threshold", self.nn_conf))
            self.nn_iou = float(meta.get("iou_threshold", self.nn_iou))
            logger.info(f"OakDCamera: NN config — {self.nn_classes} classes, input "
                        f"{self.nn_w}x{self.nn_h}, out '{self.nn_out_name}', "
                        f"conf {self.nn_conf}, iou {self.nn_iou}")
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

    def cleanup(self):
        self._running = False
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
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
        # Economy (USB2): don't ship mono L/R to the host — those XLink streams eat
        # ~15 MB/s, which starves depth on a 480 Mbit link. get_stereo_frames() then
        # returns None (GUI stereo panels stay blank); depth + detection are unaffected.
        if not economy:
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
        stereo.setSubpixel(False)

        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, self.accel_hz)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, self.gyro_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        if not economy:
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
            camRgb.setPreviewKeepAspectRatio(False)
            camRgb.setFps(self.nn_fps)

            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(self.nn_w, self.nn_h)
            self._intr_socket = dai.CameraBoardSocket.CAM_A
            self._intr_size = (self.nn_w, self.nn_h)

            nn = pipeline.create(dai.node.NeuralNetwork)
            nn.setBlobPath(self.spatial_blob)
            nn.setNumInferenceThreads(2)
            nn.input.setBlocking(False)
            camRgb.preview.link(nn.input)

            xoutDet = pipeline.create(dai.node.XLinkOut)
            xoutDet.setStreamName("det")
            nn.out.link(xoutDet.input)
        else:
            align_socket = (dai.CameraBoardSocket.CAM_B if self.align_depth_to_left
                            else dai.CameraBoardSocket.CAM_C)
            stereo.setDepthAlign(align_socket)
            stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
            self._intr_socket = align_socket
            self._intr_size = (monoLeft.getResolutionWidth(),
                               monoLeft.getResolutionHeight())

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
                    if with_spatial:
                        self._read_intrinsics(device)
                    logger.info(f"OakDCamera: connected (USB {self.usb_speed}) — depth + imu"
                                + ("" if economy else " + stereo")
                                + (" + host-decoded detections" if with_spatial else "")
                                + (" [ECONOMY/USB2]" if economy else ""))

                    # maxSize=1 + non-blocking = always the freshest frame, old ones
                    # dropped (low latency, no device backpressure). get() still blocks
                    # the host until the next frame, so the loop paces without busy-spin.
                    if not economy:
                        qLeft = device.getOutputQueue("left", maxSize=1, blocking=False)
                        qRight = device.getOutputQueue("right", maxSize=1, blocking=False)
                    else:
                        qLeft = qRight = None
                    qDepth = device.getOutputQueue("depth", maxSize=1, blocking=False)
                    qImu = device.getOutputQueue("imu", maxSize=20, blocking=False)
                    qDet = device.getOutputQueue("det", maxSize=1, blocking=False) if with_spatial else None

                    fps_n, fps_t = 0, time.monotonic()
                    fps_win_n, fps_win_t = 0, fps_t   # ~1s window for the live get_depth_fps() value
                    # Block on the depth queue (paces the loop at the depth rate, no busy-spin).
                    while self._running:
                        inDepth = qDepth.get()          # blocks until the next (freshest) depth frame
                        if inDepth is not None:
                            self._process_depth(inDepth.getFrame())
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
                                self._process_nn(inDet)
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

    def _read_intrinsics(self, device):
        socket = self._intr_socket or dai.CameraBoardSocket.CAM_A
        w, h = self._intr_size or (self.nn_w, self.nn_h)
        try:
            calib = device.readCalibration()
            M = calib.getCameraIntrinsics(socket, w, h)
            self._fx, self._fy = float(M[0][0]), float(M[1][1])
            self._cx, self._cy = float(M[0][2]), float(M[1][2])
            logger.info(f"OakDCamera: {socket.name} intrinsics @ {w}x{h} "
                        f"fx={self._fx:.1f} fy={self._fy:.1f} cx={self._cx:.1f} cy={self._cy:.1f}")
        except Exception as e:
            logger.error(f"OakDCamera: readCalibration failed: {e}")

    def get_intrinsics(self, shape=None):
        """(fx, fy, cx, cy) for the depth map, or None if the device isn't up.

        `shape` is an optional (h, w) of the frame being back-projected. The
        intrinsics are rescaled to it, so a depth map that arrives at a size
        other than the one the pipeline requested still projects correctly
        instead of producing a quietly stretched cloud.
        """
        if None in (self._fx, self._fy, self._cx, self._cy):
            return None
        fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        if shape is not None and self._intr_size is not None:
            iw, ih = self._intr_size
            h, w = shape[:2]
            if (w, h) != (iw, ih) and iw > 0 and ih > 0:
                sx, sy = w / float(iw), h / float(ih)
                fx, cx = fx * sx, cx * sx
                fy, cy = fy * sy, cy * sy
        return fx, fy, cx, cy

    # ------------------------------------------------------------------ processing
    def _process_depth(self, depth_mm):
        raw_m = depth_mm.astype(np.float32) / 1000.0
        with self._lock:
            self._latest_raw_depth = raw_m
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
    _DECODE_MODES = [("cm", False), ("cm", True), ("am", False), ("am", True)]

    def _decode(self, raw, mode):
        reshape_mode, has_obj = mode
        base = raw[:85 * 6300]
        A = base.reshape(85, 6300).T if reshape_mode == "cm" else base.reshape(6300, 85)
        box = np.ascontiguousarray(A[:, 0:4])
        if has_obj:
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
        return box, scores.astype(np.float32), cls_id

    def _mode_plausible(self, box, scores):
        """Plausible = boxes lie inside the frame and detections are sparse."""
        cx, cy = box[:, 0], box[:, 1]
        sx = self.nn_w if cx.max() > 2.0 else 1.0
        sy = self.nn_h if cy.max() > 2.0 else 1.0
        in_range = float(np.mean((cx >= -0.2 * sx) & (cx <= 1.2 * sx) &
                                 (cy >= -0.2 * sy) & (cy <= 1.2 * sy)))
        n_hits = int((scores >= self.nn_conf).sum())
        return (in_range > 0.9 and 1 <= n_hits <= 300), n_hits

    def _process_nn(self, nndata):
        """Host-side decode of the yolo26 [85, 6300] output + depth back-projection."""
        try:
            if self.nn_out_name:
                raw = np.array(nndata.getLayerFp16(self.nn_out_name), dtype=np.float32)
            else:
                raw = np.array(nndata.getFirstLayerFp16(), dtype=np.float32)
        except Exception:
            return
        if raw.size < 85 * 6300:
            with self._lock:
                self._latest_detections = []
            return

        if self._decode_mode is None:
            best, report = None, []
            for mode in self._DECODE_MODES:
                b, s, _ = self._decode(raw, mode)
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
                return
            self._decode_mode = best[0]
            logger.info(f"OAK NN: locked decode mode {self._decode_mode}")

        box, scores, cls_id = self._decode(raw, self._decode_mode)
        keep = scores >= self.nn_conf
        if not keep.any():
            with self._lock:
                self._latest_detections = []
            return
        box, scores, cls_id = box[keep], scores[keep], cls_id[keep]
        # Cap work before the pure-python NMS (O(n^2)): keep only the top-100 by score.
        if scores.shape[0] > 100:
            top = np.argpartition(scores, -100)[-100:]
            box, scores, cls_id = box[top], scores[top], cls_id[top]
        # Box coords: normalised (<=~1) -> scale to pixels; else already pixels.
        if box.max() <= 2.0:
            box[:, [0, 2]] *= self.nn_w
            box[:, [1, 3]] *= self.nn_h
        # cxcywh -> xyxy
        xyxy = np.empty_like(box)
        xyxy[:, 0] = box[:, 0] - box[:, 2] / 2.0
        xyxy[:, 1] = box[:, 1] - box[:, 3] / 2.0
        xyxy[:, 2] = box[:, 0] + box[:, 2] / 2.0
        xyxy[:, 3] = box[:, 1] + box[:, 3] / 2.0

        kept = _nms(xyxy, scores, self.nn_iou)
        depth = self.get_raw_depth_frame()   # (nn_h, nn_w) metres, CAM_A-aligned
        out = []
        for i in kept[:20]:
            x1, y1, x2, y2 = xyxy[i]
            label = self.labels[cls_id[i]] if 0 <= cls_id[i] < len(self.labels) else str(cls_id[i])
            det = {
                "label": label,
                "conf": float(scores[i]),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                "xyz_m": None,
                "xyz_base_m": None,
            }
            xyz = self._locate(depth, x1, y1, x2, y2)
            if xyz is not None:
                x, y, z = xyz
                det["xyz_m"] = {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)}
                det["xyz_base_m"] = {"x": round(OAK_MOUNT_X + z, 3),
                                     "y": round(-x, 3),
                                     "z": round(OAK_MOUNT_Z - y, 3)}
            out.append(det)
        with self._lock:
            self._latest_detections = out

    def _locate(self, depth, x1, y1, x2, y2):
        """Median depth in the inner half of the box -> 3D point in the CAM_A optical frame."""
        if depth is None or None in (self._fx, self._fy, self._cx, self._cy):
            return None
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

    def get_stereo_frames(self):
        with self._lock:
            return self._latest_left, self._latest_right

    def get_imu(self):
        with self._lock:
            return self._latest_imu

    def get_spatial_detections(self):
        with self._lock:
            return list(self._latest_detections)
