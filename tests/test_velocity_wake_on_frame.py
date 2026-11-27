"""X3_VEL_WAKE_ON_FRAME: the estimator wakes on a new OAK depth frame.

Real threads and real time: a fake producer drives the real
OakDCamera._process_depth / wait_for_depth, and the estimator's _step is
stubbed to record when each capture reaches it.
"""
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from oakd_driver import OakDCamera
from velocity_estimator import VelocityEstimator

MODEL = str(Path(__file__).resolve().parents[1] / 'src/velocity_mlp_v3.torchscript')


def make_camera():
    cam = OakDCamera.__new__(OakDCamera)
    cam._lock = threading.Lock()
    cam._depth_history = deque(maxlen=20)
    cam._inv_depth_offset = 0.
    cam._depth_intr_size = (4, 4)
    cam._depth_fx = cam._depth_fy = 100.
    cam._depth_cx = cam._depth_cy = 2.
    cam._latest_depth_observation = None
    cam._depth_seq = 0
    cam._depth_cond = threading.Condition()
    return cam


class Producer(threading.Thread):
    """Publishes a 2 m depth frame every `period` s; capture_t = publish time."""

    def __init__(self, cam, period, frames=None):
        super().__init__(daemon=True)
        self.cam, self.period, self.frames = cam, period, frames
        self.published = []
        self.stop_evt = threading.Event()

    def run(self):
        n = 0
        depth = np.full((4, 4), 2000, np.uint16)
        while not self.stop_evt.is_set() and (self.frames is None or n < self.frames):
            n += 1
            t = time.monotonic()
            self.cam._process_depth(depth, t, ('s', n), t)
            self.published.append(t)
            self.stop_evt.wait(self.period)


def run_loop(monkeypatch, wake, cam, seconds, producer=None):
    if wake:
        monkeypatch.setenv('X3_VEL_WAKE_ON_FRAME', '1')
    else:
        monkeypatch.delenv('X3_VEL_WAKE_ON_FRAME', raising=False)
    monkeypatch.delenv('DISPLAY', raising=False)
    est = VelocityEstimator(None, None, model_path=MODEL)
    assert est._wake_on_frame is wake
    est.camera = cam
    est._extract_depth_centroids = lambda *a: [(0., 0., 2.)]
    steps = []   # (capture_t, time it reached _step)

    def step(*args, capture_t=None, **kwargs):
        steps.append((capture_t, time.monotonic()))
        return [], {}
    est._step = step
    est._running = True
    loop = threading.Thread(target=est._inference_loop, daemon=True)
    loop.start()
    if producer is not None:
        producer.start()
    time.sleep(seconds)
    est._running = False
    if producer is not None:
        producer.stop_evt.set()
        producer.join(1)
    loop.join(1)
    assert not loop.is_alive()
    return est, steps


def latencies_ms(steps):
    return np.array([(t - c) * 1e3 for c, t in steps])


def test_wake_cuts_publish_to_estimate_latency(monkeypatch):
    # 130 ms period: not phase-locked to the 100 ms poll, so the old mode
    # samples the whole 0-100 ms wait.
    lat = {}
    for wake in (False, True):
        cam = make_camera()
        _, steps = run_loop(monkeypatch, wake, cam, 2.0, Producer(cam, 0.13))
        assert len(steps) >= 10
        lat[wake] = latencies_ms(steps)
    assert np.median(lat[False]) > 20
    assert np.percentile(lat[False], 90) > 50
    assert np.median(lat[True]) < 10
    assert np.percentile(lat[True], 90) < 20


def test_wake_processes_each_frame_once_and_keeps_the_rate_cap(monkeypatch):
    cam = make_camera()
    prod = Producer(cam, 0.02)   # 50 Hz, faster than the 10 Hz cap
    est, steps = run_loop(monkeypatch, True, cam, 1.5, prod)
    captures = [c for c, _ in steps]
    assert len(captures) == len(set(captures))            # never twice
    assert set(captures) <= set(prod.published)
    gaps = np.diff([t for _, t in steps])
    assert gaps.min() >= 0.095                             # still <= 10 Hz
    assert 12 <= len(steps) <= 16


def test_wake_slow_camera_each_frame_once(monkeypatch):
    cam = make_camera()
    prod = Producer(cam, 0.25)   # slower than 10 Hz: every frame is new work
    est, steps = run_loop(monkeypatch, True, cam, 1.4, prod)
    captures = [c for c, _ in steps]
    assert len(captures) == len(set(captures))
    assert len(captures) >= len(prod.published) - 1
    assert est.timing_stats['duplicates'] >= 1   # idle ticks between frames


def test_wake_timeout_path_runs_without_frames(monkeypatch):
    cam = make_camera()   # no producer: nothing is ever published
    est, steps = run_loop(monkeypatch, True, cam, 0.65)
    assert not steps
    # The invalid/dropout path still ticks at ~10 Hz instead of hanging.
    assert 4 <= est.timing_stats['invalid'] <= 8


def test_wake_timeout_after_frames_stop_goes_stale(monkeypatch):
    cam = make_camera()
    prod = Producer(cam, 0.05, frames=3)
    est, steps = run_loop(monkeypatch, True, cam, 1.2, prod)
    assert 1 <= len(steps) <= 3
    # Last frame ages past MAX_DEPTH_FRAME_AGE_S (0.5 s) while no new frame
    # arrives; only the idle timeout can get the loop there.
    assert est.timing_stats['invalid'] >= 1


def test_default_mode_never_waits_on_the_source(monkeypatch):
    cam = make_camera()
    calls = []
    cam.wait_for_depth = lambda *a: calls.append(a)
    _, steps = run_loop(monkeypatch, False, cam, 0.5, Producer(cam, 0.05))
    assert steps and not calls


def test_wake_falls_back_to_sleep_without_wait_method(monkeypatch):
    class Legacy:   # e.g. the ROS bridge: no wait_for_depth
        def __init__(self, cam):
            self.cam = cam

        def get_depth_observation(self):
            return self.cam.get_depth_observation()
    cam = make_camera()
    est, steps = run_loop(monkeypatch, True, Legacy(cam), 1.0, Producer(cam, 0.13))
    assert steps
    assert len({c for c, _ in steps}) == len(steps)


def test_wait_for_depth_returns_on_publish_and_times_out():
    cam = make_camera()
    t = time.monotonic()
    assert cam.wait_for_depth(0, 0.05) == 0
    assert time.monotonic() - t >= 0.045
    threading.Timer(0.02, cam._process_depth,
                    (np.full((4, 4), 2000, np.uint16), 1., ('s', 1), 1.)).start()
    t = time.monotonic()
    assert cam.wait_for_depth(0, 1.0) == 1
    assert time.monotonic() - t < 0.5
    assert cam.get_depth_observation()['depth_seq'] == 1
