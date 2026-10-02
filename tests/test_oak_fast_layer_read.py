"""The driver's raw-byte NN layer read: used only after it matches the list path."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from oakd_driver import OakDCamera  # noqa: E402


class _Packet:
    """Two FP16 layers packed back to back, like a DepthAI NNData."""

    def __init__(self, layers, lie_offset=0, raw=True):
        self.values = {k: np.asarray(v, dtype=np.float16) for k, v in layers.items()}
        self.infos, blob, off = [], b'', 0
        for name, v in self.values.items():
            self.infos.append(SimpleNamespace(name=name, offset=off + lie_offset,
                                              dims=list(v.shape), dataType='DataType.FP16'))
            blob += v.tobytes()
            off += v.nbytes
        self.blob = np.frombuffer(blob, dtype=np.uint8)
        self.list_calls = 0
        if not raw:
            self.getData = None
            del self.getData

    def getData(self):
        return self.blob

    def getAllLayers(self):
        return self.infos

    def getLayerFp16(self, name):
        self.list_calls += 1
        return [float(x) for x in self.values[name].ravel()]

    def getFirstLayerFp16(self):
        return self.getLayerFp16(next(iter(self.values)))


def _cam():
    cam = OakDCamera.__new__(OakDCamera)
    cam._fast_layers = {}
    return cam


def _layers(seed=0):
    rng = np.random.default_rng(seed)
    return {'output0': rng.normal(size=(1, 6, 50)), 'output1': rng.normal(size=(1, 4, 8, 6))}


def test_fast_read_is_verified_once_then_skips_the_list_path():
    cam = _cam()
    first = _Packet(_layers(0))
    a = cam._read_layer(first, 'output1')
    assert first.list_calls == 1 and cam._fast_layers == {'output1': True}
    assert np.array_equal(a, first.values['output1'].ravel().astype(np.float32))
    second = _Packet(_layers(1))
    b = cam._read_layer(second, 'output1')
    assert second.list_calls == 0
    assert b.dtype == np.float32
    assert np.array_equal(b, second.values['output1'].ravel().astype(np.float32))
    # The unnamed (first) layer is tracked separately.
    c = cam._read_layer(second, None)
    assert np.array_equal(c, second.values['output0'].ravel().astype(np.float32))


def test_wrong_layout_is_caught_and_disabled():
    cam = _cam()
    bad = _Packet(_layers(2), lie_offset=2)      # offsets shifted by one value
    a = cam._read_layer(bad, 'output0')
    assert cam._fast_layers == {'output0': False}
    assert np.array_equal(a, bad.values['output0'].ravel().astype(np.float32))
    again = _Packet(_layers(3), lie_offset=2)
    b = cam._read_layer(again, 'output0')
    assert again.list_calls == 1
    assert np.array_equal(b, again.values['output0'].ravel().astype(np.float32))


def test_packets_without_raw_access_use_the_list_path():
    cam = _cam()

    class Plain:
        def getLayerFp16(self, name):
            return [1.0, 2.0, 3.0]

        def getFirstLayerFp16(self):
            return [4.0, 5.0]
    assert cam._read_layer(Plain(), 'x').tolist() == [1.0, 2.0, 3.0]
    assert cam._read_layer(Plain(), None).tolist() == [4.0, 5.0]


def test_depth_fps_override_only_accepts_a_sane_number():
    from oakd_driver import _depth_fps_override as f
    assert f(30, {}) == 30
    assert f(30, {'X3_OAK_DEPTH_FPS': '15'}) == 15.0
    assert f(30, {'X3_OAK_DEPTH_FPS': '60'}) == 30     # never above the default
    assert f(30, {'X3_OAK_DEPTH_FPS': '3'}) == 30
    assert f(30, {'X3_OAK_DEPTH_FPS': 'abc'}) == 30
