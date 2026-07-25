"""
TensorRT YOLO inference wrapper.

Loads a raw .engine file produced by trtexec and exposes the same
call interface as the Ultralytics YOLO class so the rest of the server
needs no changes:

    results = model(frame, verbose=False, conf=0.25, imgsz=640)
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls[0])]
            conf  = float(box.conf[0])
"""

import logging
import os

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Minimal result objects that match the Ultralytics box iteration interface
# ---------------------------------------------------------------------------

class _TRTBox:
    """Single detection — mimics a one-row Ultralytics Boxes object."""
    __slots__ = ("xyxy", "conf", "cls")

    def __init__(self, xyxy, conf, cls_id):
        self.xyxy = [xyxy]     # box.xyxy[0] → [x1, y1, x2, y2]
        self.conf = [conf]     # box.conf[0] → float
        self.cls  = [cls_id]   # box.cls[0]  → int/float


class _TRTBoxes:
    """Collection of detections for one frame."""
    def __init__(self, xyxy_arr, conf_arr, cls_arr):
        self._xyxy = xyxy_arr   # [N, 4] np.float32
        self._conf = conf_arr   # [N]    np.float32
        self._cls  = cls_arr    # [N]    np.float64

    def __iter__(self):
        for i in range(len(self._xyxy)):
            yield _TRTBox(self._xyxy[i], self._conf[i], self._cls[i])

    def __len__(self):
        return len(self._xyxy)


class _TRTResult:
    """Single-frame result — mirrors ultralytics.engine.results.Results."""
    def __init__(self, boxes: "_TRTBoxes"):
        self.boxes = boxes


# ---------------------------------------------------------------------------
# Main detector
# ---------------------------------------------------------------------------

class TRTDetector:
    """
    TensorRT YOLO detector built from a trtexec-generated .engine file.

    Parameters
    ----------
    engine_path : str
        Path to the .engine file produced by trtexec.
    pt_path : str | None
        Path to the original .pt file — used only to read class names.
        If None or missing, names fall back to "class_N" strings.
    conf_thres : float
        Confidence threshold for detections.
    iou_thres : float
        IoU threshold for NMS.
    """

    def __init__(self, engine_path: str, pt_path: str | None = None,
                 conf_thres: float = 0.25, iou_thres: float = 0.45):
        try:
            import pycuda.driver as cuda
            import tensorrt as trt
        except ImportError as e:
            raise ImportError(f"TRTDetector requires pycuda and tensorrt: {e}") from e

        self.conf_thres = conf_thres
        self.iou_thres  = iou_thres
        self.names: dict = {}
        import threading
        self._lock = threading.Lock()

        # ── Class names from .pt checkpoint ────────────────────────────────
        #   Done BEFORE creating our CUDA context: torch.load() initialises CUDA
        #   and leaves torch's primary context current. If we deserialized the
        #   engine / created the execution context after that, they'd bind to
        #   torch's context while inference runs under self._ctx — a mismatch
        #   that surfaces as "Cask convolution execution" errors at enqueue.
        if pt_path and os.path.exists(pt_path):
            try:
                import torch
                ckpt = torch.load(pt_path, map_location="cpu", weights_only=False)
                raw  = ckpt.get("model", ckpt)
                if hasattr(raw, "names"):
                    self.names = raw.names
                elif isinstance(ckpt, dict) and "names" in ckpt:
                    self.names = ckpt["names"]
                logger.info(f"TRTDetector: loaded {len(self.names)} class names from {pt_path}")
            except Exception as exc:
                logger.warning(f"TRTDetector: could not read names from {pt_path}: {exc}")

        # ── CUDA context (explicit management for thread-pool safety) ──────
        cuda.init()
        self._cuda = cuda
        self._ctx  = cuda.Device(0).make_context()   # now current

        # All engine/context/buffer creation must happen while self._ctx is the
        # current context, so those resources bind to THIS context (see above).
        try:
            self._trt_logger = trt.Logger(trt.Logger.WARNING)
            with open(engine_path, "rb") as f:
                runtime      = trt.Runtime(self._trt_logger)
                self._engine = runtime.deserialize_cuda_engine(f.read())

            self._context = self._engine.create_execution_context()

            # Tensor names & shapes
            self._in_name  = self._engine.get_tensor_name(0)
            self._out_name = self._engine.get_tensor_name(1)
            in_shape       = tuple(self._engine.get_tensor_shape(self._in_name))   # (1,3,H,W)
            out_shape      = tuple(self._engine.get_tensor_shape(self._out_name))  # (1,4+C,A) or (1,N,6)

            self.input_h     = in_shape[2]
            self.input_w     = in_shape[3]

            # Detect output layout:
            #   classic YOLOv8 : (1, 4+C, A) → transpose + cxcywh decode + NMS
            #   end-to-end (YOLO26/v10): (1, N, 6) → [x1,y1,x2,y2,conf,cls] in input
            #     space, already NMS-filtered on-device (no post-NMS needed).
            self._end2end = (len(out_shape) == 3 and out_shape[2] == 6)
            if self._end2end:
                # Class count comes from the .pt names; the output's second dim is
                # the detection cap (e.g. 300), NOT 4+num_classes.
                self.num_classes = len(self.names) or 80
            else:
                self.num_classes = out_shape[1] - 4

            # Fill any missing names with generic labels
            for i in range(self.num_classes):
                self.names.setdefault(i, f"class_{i}")

            # ── GPU buffers (allocated in this context) ─────────────────────
            self._d_input  = cuda.mem_alloc(int(np.prod(in_shape))  * 4)  # float32
            self._d_output = cuda.mem_alloc(int(np.prod(out_shape)) * 4)
            self._stream   = cuda.Stream()
        finally:
            self._ctx.pop()   # don't leave it current; push/pop around inference

        # Pre-allocated host buffers — reused every inference call to avoid per-frame allocation
        self._h_input  = np.empty(in_shape,  dtype=np.float32)   # (1,3,H,W) — P5
        self._h_output = np.empty(out_shape, dtype=np.float32)

        logger.info(
            f"TRTDetector: ready — {self.num_classes} classes, "
            f"input {self.input_h}×{self.input_w}, engine {engine_path}"
        )

    # ── Internal helpers ───────────────────────────────────────────────────

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """BGR uint8 → normalised NCHW float32 written into the pre-allocated host buffer.

        cv2.dnn.blobFromImage does resize + BGR→RGB + /255 normalisation in a
        single optimised C++ pass (replaces resize → cvtColor → astype → transpose
        → ascontiguousarray chain that caused 3-4 intermediate allocations).
        """
        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (self.input_w, self.input_h), swapRB=True, crop=False
        )
        np.copyto(self._h_input, blob)   # write into the stable, reused buffer
        return self._h_input

    def _postprocess(
        self,
        raw: np.ndarray,
        orig_w: int,
        orig_h: int,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Decode engine output [1, 4+C, A] → filtered xyxy boxes.

        Returns (xyxy [N,4], confs [N], cls_ids [N]).
        """
        # ── End-to-end head (YOLO26): [N, 6] = x1,y1,x2,y2,conf,cls, input-space,
        #    already NMS-filtered on-device. Just threshold and rescale. ────────
        if self._end2end:
            pred  = raw[0]                                       # [N, 6]
            confs = pred[:, 4]
            keep  = confs >= self.conf_thres
            pred  = pred[keep]
            if len(pred) == 0:
                return np.empty((0, 4)), np.empty(0), np.empty(0, dtype=float)
            sx = orig_w / self.input_w
            sy = orig_h / self.input_h
            xyxy = pred[:, :4].copy()
            xyxy[:, 0] *= sx; xyxy[:, 2] *= sx
            xyxy[:, 1] *= sy; xyxy[:, 3] *= sy
            return xyxy, pred[:, 4], pred[:, 5].astype(float)

        pred     = raw[0].T                                      # [A, 4+C]
        boxes    = pred[:, :4]                                   # cx cy w h
        cls_sc   = pred[:, 4:]                                   # [A, C]
        cls_ids  = np.argmax(cls_sc, axis=1)
        confs    = cls_sc[np.arange(len(cls_sc)), cls_ids]

        mask  = confs >= self.conf_thres
        boxes = boxes[mask]; confs = confs[mask]; cls_ids = cls_ids[mask]
        if len(boxes) == 0:
            return np.empty((0, 4)), np.empty(0), np.empty(0, dtype=float)

        # xywh (input-space) → xyxy (original-image-space)
        sx = orig_w / self.input_w
        sy = orig_h / self.input_h
        x1 = (boxes[:, 0] - boxes[:, 2] / 2) * sx
        y1 = (boxes[:, 1] - boxes[:, 3] / 2) * sy
        x2 = (boxes[:, 0] + boxes[:, 2] / 2) * sx
        y2 = (boxes[:, 1] + boxes[:, 3] / 2) * sy
        xyxy = np.stack([x1, y1, x2, y2], axis=1)

        # NMSBoxes expects [x, y, w, h] (top-left corner), not xyxy.
        # Pass numpy arrays directly — OpenCV 4.5+ accepts them without .tolist().
        xywh = np.stack([xyxy[:, 0], xyxy[:, 1],
                         xyxy[:, 2] - xyxy[:, 0],
                         xyxy[:, 3] - xyxy[:, 1]], axis=1)
        idx = cv2.dnn.NMSBoxes(xywh, confs, self.conf_thres, self.iou_thres)
        if len(idx) == 0:
            return np.empty((0, 4)), np.empty(0), np.empty(0, dtype=float)

        idx = np.array(idx).flatten()
        return xyxy[idx], confs[idx], cls_ids[idx].astype(float)

    # ── Public interface ───────────────────────────────────────────────────

    def __call__(
        self,
        frame: np.ndarray,
        verbose: bool = False,
        conf: float | None = None,
        imgsz: int | None = None,
    ) -> list[_TRTResult]:
        """
        Run inference on a single BGR frame.
        Signature matches: model(frame, verbose=False, conf=..., imgsz=...)
        """
        with self._lock:
            if getattr(self, "_ctx", None) is None:
                return [_TRTResult(_TRTBoxes(np.empty((0, 4)), np.empty(0), np.empty(0, dtype=float)))]

            if conf is not None:
                self.conf_thres = conf

            orig_h, orig_w = frame.shape[:2]
            inp = self._preprocess(frame)

            self._ctx.push()
            try:
                self._cuda.memcpy_htod_async(self._d_input, inp, self._stream)
                self._context.set_tensor_address(self._in_name,  int(self._d_input))
                self._context.set_tensor_address(self._out_name, int(self._d_output))
                self._context.execute_async_v3(stream_handle=self._stream.handle)
                self._cuda.memcpy_dtoh_async(self._h_output, self._d_output, self._stream)
                self._stream.synchronize()
            finally:
                self._ctx.pop()

            xyxy, confs, cls_ids = self._postprocess(self._h_output, orig_w, orig_h)
            return [_TRTResult(_TRTBoxes(xyxy, confs, cls_ids))]

    def cleanup(self):
        """Release GPU buffers and TRT/CUDA resources.

        Order matters: the TRT execution context, engine and stream must be
        destroyed while self._ctx is still current — otherwise their C++
        destructors later run against an already-detached context, raising
        "Error 709 destroying stream" and segfaulting at shutdown. Idempotent.
        """
        with self._lock:
            ctx = getattr(self, "_ctx", None)
            if ctx is None:
                return
            ctx.push()
            try:
                for name in ("_d_input", "_d_output"):
                    buf = getattr(self, name, None)
                    if buf is not None:
                        try:
                            buf.free()
                        except Exception:
                            pass
                        setattr(self, name, None)
                # Drop TRT + stream refs while this context is current so their
                # CUDA resources release against the correct context.
                self._context = None
                self._engine  = None
                self._stream  = None
            finally:
                ctx.pop()
            ctx.detach()
            self._ctx = None

    def __del__(self):
        # Best-effort teardown if the owner never called cleanup() (e.g. the
        # detector is dropped on a model swap).
        try:
            self.cleanup()
        except Exception:
            pass
