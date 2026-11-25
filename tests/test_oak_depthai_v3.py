"""OakDCamera on the depthai v3 API (the AGX ships 3.11).

Device nodes can't be created without a device, so the pipeline build itself
is checked on the robot (src/tools/hw_tests/oak_v3_smoke.py). Here:
  * every v3 name the driver relies on exists (skipped unless depthai >= 3),
  * the NN read works on a real v3 NNData (no getLayerFp16 there),
  * the v3 session opens device -> builds -> makes queues -> starts, and stops
    the pipeline on every exit, using a stand-in dai module.
"""
import contextlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import oakd_driver  # noqa: E402
from oakd_driver import OakDCamera  # noqa: E402

try:
    import depthai as _real_dai
    _REAL_V3 = oakd_driver._depthai_major(_real_dai.__version__) >= 3
except Exception:  # pragma: no cover
    _real_dai, _REAL_V3 = None, False

needs_v3 = pytest.mark.skipif(not _REAL_V3, reason="needs depthai >= 3")


def test_major_version_parse():
    assert oakd_driver._depthai_major("3.11.0") == 3
    assert oakd_driver._depthai_major("2.32.0.0") == 2
    assert oakd_driver._depthai_major("dev") == 0


@needs_v3
def test_v3_api_surface_the_driver_uses():
    d = _real_dai
    n = d.node
    for cls, names in [
        (d.Pipeline, ["create", "start", "stop"]),
        (d.Device, ["getUsbSpeed", "readCalibration", "getConnectedCameraFeatures",
                    "getDeviceId", "__enter__"]),
        (n.MonoCamera, ["setResolution", "setBoardSocket", "setFps", "out",
                        "getResolutionWidth", "getResolutionHeight"]),
        (n.ColorCamera, ["setBoardSocket", "setResolution", "setPreviewSize", "setInterleaved",
                         "setColorOrder", "setPreviewKeepAspectRatio", "setFps",
                         "setPreviewNumFramesPool", "preview"]),
        (n.StereoDepth, ["setDefaultProfilePreset", "setLeftRightCheck", "setSubpixel",
                         "setPostProcessingHardwareResources", "setDepthAlign",
                         "setOutputSize", "initialConfig", "depth", "left", "right"]),
        (n.NeuralNetwork, ["setBlobPath", "setNumInferenceThreads", "input", "out"]),
        (n.IMU, ["enableIMUSensor", "setBatchReportThreshold", "setMaxBatchReports", "out"]),
        (d.Node.Output, ["createOutputQueue", "link"]),
        (d.Node.Input, ["setBlocking", "setMaxSize"]),
        (d.MessageQueue, ["get", "tryGet", "tryGetAll"]),
        (d.CalibrationHandler, ["getCameraIntrinsics", "getDefaultIntrinsics",
                                "getDistortionCoefficients"]),
        (d.NNData, ["getAllLayers", "getData", "getTensor", "getFirstTensor"]),
        (d.ImgFrame, ["getFrame", "getCvFrame", "getSequenceNum", "getTimestamp",
                      "getTimestampDevice"]),
        (d.IMUData, ["packets"]),
        (d.IMUPacket, ["acceleroMeter", "gyroscope"]),
    ]:
        missing = [x for x in names if not hasattr(cls, x)]
        assert not missing, f"{cls.__name__} lacks {missing}"
    cfg = _real_dai.StereoDepthConfig()
    cfg.postProcessing.speckleFilter.enable = True
    cfg.postProcessing.speckleFilter.speckleRange = 50
    cfg.postProcessing.thresholdFilter.minRange = 200
    cfg.setSubpixelFractionalBits(3)
    assert cfg.postProcessing.speckleFilter.enable is True          # nested writes stick
    assert cfg.postProcessing.speckleFilter.speckleRange == 50
    assert hasattr(d.MonoCameraProperties.SensorResolution, "THE_400_P")
    assert hasattr(d.ColorCameraProperties.SensorResolution, "THE_1080_P")
    assert hasattr(n.StereoDepth.PresetMode, "DEFAULT")
    assert callable(d.Clock.now)


def test_stereo_config_edit_v3_writes_the_live_object(monkeypatch):
    monkeypatch.setattr(oakd_driver, "_DAI_V3", True)
    live = SimpleNamespace(postProcessing=SimpleNamespace(
        speckleFilter=SimpleNamespace(enable=False, speckleRange=0)))
    stereo = SimpleNamespace(initialConfig=live)
    OakDCamera._edit_stereo_config(stereo, lambda c: setattr(c.postProcessing.speckleFilter,
                                                             "enable", True))
    assert live.postProcessing.speckleFilter.enable is True


@needs_v3
def test_nn_read_on_a_real_v3_nndata():
    rng = np.random.default_rng(0)
    a = rng.normal(size=(1, 6, 50)).astype(np.float16)
    b = rng.normal(size=(1, 4, 8, 6)).astype(np.float16)
    nn = _real_dai.NNData()
    nn.addTensor("output0", a)
    nn.addTensor("output1", b)
    cam = OakDCamera.__new__(OakDCamera)
    cam._fast_layers = {}
    first = cam._read_layer(nn, "output1")                 # list path + verification
    assert cam._fast_layers == {"output1": True}
    np.testing.assert_array_equal(first, b.astype(np.float32).ravel())
    np.testing.assert_array_equal(cam._read_layer(nn, "output1"), b.astype(np.float32).ravel())
    np.testing.assert_array_equal(cam._read_layer(nn, None), a.astype(np.float32).ravel())


# ---------------------------------------------------------------- v3 session lifecycle
class _FakeQueue:
    def __init__(self, name, maxSize, blocking):
        self.name, self.maxSize, self.blocking = name, maxSize, blocking


class _FakeOutput:
    def __init__(self, name):
        self.name = name

    def createOutputQueue(self, maxSize=16, blocking=False):
        return _FakeQueue(self.name, maxSize, blocking)


class _FakeDevice:
    events = []

    def __init__(self, speed):
        self.speed = speed

    def __enter__(self):
        self.events.append("open")
        return self

    def __exit__(self, *a):
        self.events.append("close")

    def getUsbSpeed(self):
        return SimpleNamespace(name="SUPER")


class _FakePipeline:
    def __init__(self, ev):
        self.ev = ev

    def start(self):
        self.ev.append("start")

    def stop(self):
        self.ev.append("stop")


def _v3_cam(monkeypatch, record=False):
    ev = []
    _FakeDevice.events = ev
    fake = SimpleNamespace(Device=_FakeDevice,
                           UsbSpeed=SimpleNamespace(HIGH="HIGH", SUPER="SUPER"))
    monkeypatch.setattr(oakd_driver, "dai", fake)
    monkeypatch.setattr(oakd_driver, "_DAI_V3", True)
    cam = OakDCamera.__new__(OakDCamera)
    cam.usb2_mode, cam._auto_economy, cam.record_rgbd = False, True, record
    cam.device_id = None

    def build(with_spatial, economy, device=None):
        assert device is not None, "v3 must build against the open device"
        ev.append(f"build eco={economy}")
        names = ["left", "right", "depth", "imu"] + (["det"] if with_spatial else [])
        return _FakePipeline(ev), {n: _FakeOutput(n) for n in names}
    cam._build_pipeline = build
    return cam, ev


def test_v3_session_order_and_queue_sizes(monkeypatch):
    cam, ev = _v3_cam(monkeypatch)
    with cam._session(True, False) as (device, queues, eco):
        assert eco is False and cam.usb_speed == "SUPER"
        assert set(queues) == {"left", "right", "depth", "imu", "det"}
        assert queues["depth"].maxSize == 1 and queues["imu"].maxSize == 20
        assert not any(q.blocking for q in queues.values())
        assert ev == ["open", "build eco=False", "start"]
    assert ev[-2:] == ["stop", "close"]


def test_v3_session_stops_pipeline_on_error(monkeypatch):
    cam, ev = _v3_cam(monkeypatch, record=True)
    with pytest.raises(RuntimeError):
        with cam._session(False, False) as (_, queues, _eco):
            assert queues["depth"].maxSize == 32        # C1 recording keeps depth history
            raise RuntimeError("X_LINK_ERROR")
    assert ev[-2:] == ["stop", "close"]
