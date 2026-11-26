"""X3_CAM_DECODE_ON_DEMAND: grab() every frame, retrieve() only when a consumer will use it."""

import threading
import time

import numpy as np
import pytest

import cam_decode
from drivers_x3 import AstraCamera


class FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeCap:
    """A VideoCapture that counts calls. Each grab()/read() advances the clock by 1/fps;
    frame k is filled with value k so tests can tell which one was decoded."""

    def __init__(self, clock, fps=30.0, n_frames=30, on_frame=None):
        self.clock, self.dt, self.n = clock, 1.0 / fps, n_frames
        self.on_frame = on_frame
        self.reads = self.grabs = self.retrieves = 0
        self.k = -1

    def _advance(self):
        self.k += 1
        self.clock.t += self.dt
        if self.on_frame is not None:
            self.on_frame(self.k)
        return self.k < self.n

    def _img(self):
        return np.full((8, 10, 3), self.k % 256, np.uint8)

    def read(self):
        self.reads += 1
        ok = self._advance()
        return ok, (self._img() if ok else None)

    def grab(self):
        self.grabs += 1
        return self._advance()

    def retrieve(self):
        self.retrieves += 1
        return True, self._img()

    def release(self):
        pass


@pytest.fixture
def clock(monkeypatch):
    c = FakeClock()
    monkeypatch.setattr(cam_decode, '_now', c)
    return c


def test_env_parsing():
    assert cam_decode.gate_from_env({}) is None
    assert cam_decode.gate_from_env({'X3_CAM_DECODE_ON_DEMAND': '0'}) is None
    g = cam_decode.gate_from_env({'X3_CAM_DECODE_ON_DEMAND': '1'})
    assert g is not None and g.min_dt == pytest.approx(0.1)
    g = cam_decode.gate_from_env({'X3_CAM_DECODE_ON_DEMAND': 'true', 'X3_CAM_DECODE_HZ': '5'})
    assert g.min_dt == pytest.approx(0.2)
    g = cam_decode.gate_from_env({'X3_CAM_DECODE_ON_DEMAND': '1', 'X3_CAM_DECODE_HZ': '0'})
    assert g.min_dt == float('inf')    # request-only


def test_legacy_next_frame_reads_every_frame(clock):
    cap = FakeCap(clock, n_frames=10)
    out = [cam_decode.next_frame(cap, None, post=lambda f: f + 1) for _ in range(10)]
    assert cap.reads == 10 and cap.grabs == 0 and cap.retrieves == 0
    assert all(ok and f is not None for ok, f, _ in out)
    assert out[3][1][0, 0, 0] == 4     # post ran


def test_gate_rate_caps_decodes(clock):
    cap = FakeCap(clock, fps=30.0, n_frames=31)
    gate = cam_decode.DecodeGate(hz=10.0)
    decoded = 0
    for _ in range(30):                # 1 s of 30 fps
        ok, f, _ = cam_decode.next_frame(cap, gate)
        assert ok
        decoded += f is not None
    assert cap.grabs == 30 and cap.reads == 0
    assert cap.retrieves == decoded
    assert 9 <= decoded <= 11


def test_gate_inactive_never_decodes_and_request_is_kept(clock):
    cap = FakeCap(clock, n_frames=20)
    gate = cam_decode.DecodeGate(hz=0)   # request-only
    for _ in range(5):
        assert cam_decode.next_frame(cap, gate)[1] is None
    gate.request()
    assert cam_decode.next_frame(cap, gate, active=False)[1] is None   # nobody listening
    ok, f, t = cam_decode.next_frame(cap, gate)                         # request survived
    assert f is not None and f[0, 0, 0] == cap.k
    assert cam_decode.next_frame(cap, gate)[1] is None                   # one decode per request
    assert cap.retrieves == 1


def test_timestamp_is_grab_time_not_decode_time(clock):
    cap = FakeCap(clock, n_frames=5)
    gate = cam_decode.DecodeGate(hz=0)
    gate.request()

    def slow_retrieve():
        clock.t += 0.05                # decode takes 50 ms
        return True, np.zeros((2, 2, 3), np.uint8)
    cap.retrieve = slow_retrieve
    t_before = clock.t
    ok, f, t = cam_decode.next_frame(cap, gate)
    assert f is not None and t == pytest.approx(t_before + cap.dt)


def test_grab_failure_reports_not_ok(clock):
    cap = FakeCap(clock, n_frames=0)
    assert cam_decode.next_frame(cap, cam_decode.DecodeGate())[0] is False
    assert cam_decode.next_frame(cap, None)[0] is False


def _run_astra(monkeypatch, clock, flag, has_clients=True, fps=30.0, n=30, consumer=None, hz='10'):
    if flag:
        monkeypatch.setenv('X3_CAM_DECODE_ON_DEMAND', '1')
        monkeypatch.setenv('X3_CAM_DECODE_HZ', hz)
    else:
        monkeypatch.delenv('X3_CAM_DECODE_ON_DEMAND', raising=False)
    cam = AstraCamera(width=1280, height=800, sim_mode=True)
    cam.sim_mode = False               # sim_mode only skipped opening the device
    cam._has_clients = has_clients

    def on_frame(k):
        if k >= n:
            cam._running = False
        elif consumer is not None:
            consumer(cam, k)
    cap = FakeCap(clock, fps=fps, n_frames=n, on_frame=on_frame)
    cam._cap = cap
    cam._capture_loop()                # runs synchronously until n frames
    return cam, cap


def test_astra_legacy_decodes_every_frame(monkeypatch, clock):
    cam, cap = _run_astra(monkeypatch, clock, flag=False)
    assert cam._decode_gate is None
    assert cap.reads == 31 and cap.grabs == 0 and cap.retrieves == 0
    frame, t = cam.get_frame_stamped()
    assert frame[0, 0, 0] == 29


def test_astra_legacy_no_clients_still_decodes(monkeypatch, clock):
    cam, cap = _run_astra(monkeypatch, clock, flag=False, has_clients=False)
    assert cap.reads == 31 and cam.get_frame() is None


def test_astra_on_demand_rate_capped(monkeypatch, clock):
    cam, cap = _run_astra(monkeypatch, clock, flag=True)
    assert cap.reads == 0 and cap.grabs == 31
    assert 9 <= cap.retrieves <= 11
    frame, t = cam.get_frame_stamped()
    # the stored frame is the last decoded one, stamped at its own grab
    assert t == pytest.approx(100.0 + (frame[0, 0, 0] + 1) * cap.dt)


def test_astra_on_demand_no_clients_never_decodes(monkeypatch, clock):
    cam, cap = _run_astra(monkeypatch, clock, flag=True, has_clients=False)
    assert cap.grabs == 31 and cap.retrieves == 0
    assert cam.get_frame() is None


def test_astra_on_demand_get_frame_requests_next_decode(monkeypatch, clock):
    calls = []

    def consumer(cam, k):              # a consumer polling every frame (e.g. a 30 Hz GUI)
        calls.append(cam.get_frame())
    cam, cap = _run_astra(monkeypatch, clock, flag=True, consumer=consumer, hz='0')
    # request-only gate: each get_frame forces the next decode, so a full-rate consumer
    # gets every frame decoded (no regression for the GUI feed)
    assert cap.retrieves == 30
    assert calls[-1][0, 0, 0] == 28    # the previous frame: one frame of lag at most


def test_astra_on_demand_5hz_detector(monkeypatch, clock):
    """Headless robot: only the 5 Hz side detector reads; 10 Hz cap + its requests."""
    def consumer(cam, k):
        if k % 6 == 0:                 # 30 fps / 6 = 5 Hz
            cam.get_frame_stamped()
    cam, cap = _run_astra(monkeypatch, clock, flag=True, consumer=consumer)
    assert cap.grabs == 31
    assert cap.retrieves <= 15         # vs 30 decodes in legacy mode


def test_astra_on_demand_concurrent_consumer(monkeypatch, clock):
    """get_frame_stamped from another thread while the loop grabs: no torn frames, fresh stamps."""
    monkeypatch.setenv('X3_CAM_DECODE_ON_DEMAND', '1')
    cam = AstraCamera(width=1280, height=800, sim_mode=True)
    cam.sim_mode = False
    cam._has_clients = True
    cap = FakeCap(clock, n_frames=400)

    def on_frame(k):
        cam._running = k < 400
        time.sleep(0.0005)             # a real grab() blocks until the next frame
    cap.on_frame = on_frame
    cam._cap = cap
    th = threading.Thread(target=cam._capture_loop)
    th.start()
    seen = []
    while th.is_alive():
        got = cam.get_frame_stamped()
        if got is not None:
            f, t = got
            assert (f == f[0, 0, 0]).all()
            seen.append(t)
    th.join()
    assert seen and all(b >= a for a, b in zip(seen, seen[1:]))
    assert cap.retrieves < cap.grabs


def test_post_runs_only_on_decoded_frames(clock):
    """The left camera's rotate is skipped together with the decode."""
    cap = FakeCap(clock, fps=30.0, n_frames=31)
    gate = cam_decode.DecodeGate(hz=10.0)
    posts = []
    for _ in range(30):
        cam_decode.next_frame(cap, gate, post=lambda f: posts.append(1) or f[::-1])
    assert len(posts) == cap.retrieves and cap.retrieves < cap.grabs


def test_fourcc_str():
    class C:
        def get(self, prop):
            return float(int.from_bytes(b'MJPG', 'little'))
    assert cam_decode.fourcc_str(C()) == 'MJPG'
