"""CPU-side parity and server wiring with mocked PyCUDA and TensorRT."""

import ast
import logging
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from trt_detector import TRTDetector


@pytest.fixture
def mock_engine(tmp_path, monkeypatch):
    output = SimpleNamespace(raw=np.zeros((1, 300, 6), dtype=np.float32))
    context = Mock()
    stream = Mock(handle=12)
    device = Mock()
    device.make_context.return_value = context
    cuda = SimpleNamespace(
        init=Mock(), Device=Mock(return_value=device),
        ctx_flags=SimpleNamespace(SCHED_BLOCKING_SYNC=4),
        mem_alloc=Mock(return_value=1), Stream=Mock(return_value=stream),
        memcpy_htod_async=Mock(),
        memcpy_dtoh_async=lambda dest, *_: np.copyto(dest, output.raw),
    )
    monkeypatch.setitem(sys.modules, "pycuda", SimpleNamespace(driver=cuda))
    monkeypatch.setitem(sys.modules, "pycuda.driver", cuda)
    engine = Mock()
    engine.get_tensor_name.side_effect = ("input", "output")
    engine.get_tensor_shape.side_effect = lambda name: (
        (1, 3, 640, 640) if name == "input" else output.raw.shape
    )
    runtime = Mock()
    runtime.deserialize_cuda_engine.return_value = engine
    trt = SimpleNamespace(Logger=Mock(WARNING=1), Runtime=Mock(return_value=runtime))
    monkeypatch.setitem(sys.modules, "tensorrt", trt)
    path = tmp_path / "model.engine"
    path.write_bytes(b"engine")
    return SimpleNamespace(path=path, device=device, context=context, stream=stream,
                           engine=engine, output=output)


@pytest.mark.parametrize("height,width", [(800, 1280), (400, 640), (640, 640), (900, 480)])
def test_fast_preprocess_matches_blob_and_reuses_buffers(mock_engine, height, width):
    det = TRTDetector(str(mock_engine.path), fast=True)
    rng = np.random.default_rng(42)
    frames = [rng.integers(0, 256, (height, width, 3), dtype=np.uint8) for _ in range(2)]
    resized = det._resized
    for frame in frames:
        # Include a negative-stride view of a side-camera image.
        frame = frame[:, ::-1]
        old = cv2.dnn.blobFromImage(frame, 1 / 255.0, (640, 640), swapRB=True, crop=False)
        assert det._preprocess(frame) is det._h_input
        assert det._resized is resized
        np.testing.assert_allclose(det._h_input, old, atol=1e-7, rtol=0)


@pytest.mark.parametrize("end2end", [True, False])
def test_class_zero_output_matches_filtering_legacy_after_nms(mock_engine, end2end):
    if end2end:
        raw = np.array([[
            [10, 20, 50, 60, 0.4, 0],     # Threshold equality
            [11, 21, 51, 61, 0.7, 1],     # Overlap is already resolved by the engine
            [100, 200, 150, 260, 0.9, 0],
            [150, 250, 160, 270, 0.39, 0],
        ]], dtype=np.float32)
    else:
        raw = np.array([[
            [30, 40, 20, 20, 0.8, 0.9, 0.1],    # Class 1 suppresses next box
            [30, 40, 20, 20, 0.7, 0.1, 0.1],
            [120, 140, 20, 20, 0.6, 0.6, 0.2],  # Equal scores favor class 0
            [120, 140, 20, 20, 0.5, 0.1, 0.1],  # Suppressed by class 0
            [150, 170, 20, 20, 0.3, 0.1, 0.1],  # Below threshold
            [210, 230, 20, 20, 0.95, 0.1, 0.1],
            [250, 260, 20, 20, 0.97, np.nan, 0.1],
        ]], dtype=np.float32).transpose(0, 2, 1).copy()
    mock_engine.output.raw = raw
    det = TRTDetector(str(mock_engine.path), fast=True, conf_thres=0.4)
    assert det._end2end is end2end
    for threshold, expected_count in [(0.4, 2), (0.99, 0)]:
        det.conf_thres = threshold
        legacy = det._postprocess(raw, 1280, 800)
        fast = det._postprocess(raw, 1280, 800, class_zero_only=True)
        people = legacy[2] == 0
        assert people.sum() == expected_count
        for expected, actual in zip(legacy, fast):
            np.testing.assert_array_equal(expected[people], actual)


@pytest.mark.parametrize("fast", [False, True])
def test_server_side_detect_uses_opt_in(mock_engine, monkeypatch, fast):
    if fast:
        monkeypatch.setenv("X3_TRT_FAST", "1")
    else:
        monkeypatch.delenv("X3_TRT_FAST", raising=False)
    path = Path(__file__).resolve().parents[1] / "src" / "server_x3.py"
    functions = [
        node for node in ast.parse(path.read_text()).body
        if isinstance(node, ast.FunctionDef) and node.name in {"_load_side_detector", "_side_detect"}
    ]
    ns = dict(cv2=cv2, os=os, YOLO_MODEL=str(mock_engine.path.with_suffix(".pt")),
              SIDE_PERSON_CONF=0.4, logger=logging.getLogger(__name__))
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), ns)
    det = ns["_load_side_detector"]()
    if fast:
        mock_engine.device.make_context.assert_called_once_with(flags=4)
    else:
        mock_engine.device.make_context.assert_called_once_with()
    assert det.fast is fast
    mock_engine.output.raw[0, :2] = [[10, 20, 50, 60, 0.75, 0], [1, 2, 3, 4, 0.8, 1]]
    postprocess = Mock(wraps=det._postprocess)
    det._postprocess = postprocess
    det.names.update({0: "person", 1: "bus"})
    mono = np.zeros((800, 1280), dtype=np.uint8)
    assert ns["_side_detect"](det, mono) == [(20.0, 25.0, 100.0, 75.0, 0.75)]
    assert postprocess.call_args.kwargs["class_zero_only"] is fast
    mock_engine.context.push.assert_called_once_with()
    assert mock_engine.context.pop.call_count == 2
    mock_engine.stream.synchronize.assert_called_once_with()

    det.names.update({0: "bus", 1: "person"})
    assert ns["_side_detect"](det, mono) == [(2.0, 2.5, 6.0, 5.0, pytest.approx(0.8))]
    assert postprocess.call_args.kwargs["class_zero_only"] is False
    det.cleanup()
    mock_engine.context.detach.assert_called_once_with()
