"""Decode camera frames only when someone will use them (opt-in prototype).

A V4L2 ``cap.read()`` is ``grab()`` (dequeue the next buffer) plus
``retrieve()`` (MJPEG decode / colour conversion). The capture threads call
``read()`` for every sensor frame, but the consumers want far fewer: the side
person detector samples each camera at 5 Hz, and the GUI feeds only matter
while someone is watching. Decoding 1280x800 MJPEG at the sensor rate cost the
Arducam capture thread ~14 % CPU on the robot (overnight perf 2026-11-26).

With ``X3_CAM_DECODE_ON_DEMAND=1`` the capture threads still ``grab()`` every
frame (the driver buffer never goes stale, so latency is unchanged) and only
``retrieve()`` a grabbed frame when either

* ``1 / X3_CAM_DECODE_HZ`` seconds (default 10 Hz) have passed since the last
  decode, or
* a consumer called :meth:`DecodeGate.request` since the last decode (the
  Arducam's ``get_frame*``, the GUI feed while it has viewers).

Unset or ``0`` keeps the old ``read()``-every-frame behaviour.

Keep X3_CAM_DECODE_HZ at or above 2 x SIDE_PERSON_HZ (5 Hz): a request returns
the previously decoded frame and triggers the next, so at a low or zero rate a
5 Hz consumer sees frames ~200 ms old against SIDE_FRAME_MAX_AGE_S (0.3 s).
``X3_CAM_DECODE_HZ=0`` (request-only) is for tests, not the robot.

The saving assumes the V4L2 backend, where ``retrieve()`` does the MJPEG
decode. FFmpeg and GStreamer decode inside ``grab()``; the capture code logs
the backend at open.

Timestamps: a decoded frame is stamped with the monotonic time its ``grab()``
returned, i.e. when the frame reached the host, not when it was decoded.
"""

import os
import threading
import time

_now = time.monotonic   # tests replace this with a fake clock


def on_demand_enabled(env=None):
    """True when X3_CAM_DECODE_ON_DEMAND is set to something truthy."""
    env = os.environ if env is None else env
    return env.get('X3_CAM_DECODE_ON_DEMAND', '0').strip().lower() in ('1', 'true', 'yes', 'on')


def decode_hz(env=None):
    """X3_CAM_DECODE_HZ (default 10). <= 0 means decode only on request."""
    env = os.environ if env is None else env
    try:
        return float(env.get('X3_CAM_DECODE_HZ', '10'))
    except ValueError:
        return 10.0


class DecodeGate:
    """Decides, per grabbed frame, whether to pay for the decode.

    ``request()`` may be called from any thread; ``should_decode()`` is called
    only by the capture thread that owns the VideoCapture.
    """

    def __init__(self, hz=10.0):
        self.min_dt = (1.0 / hz) if hz and hz > 0 else float('inf')
        self._next = float('-inf')     # when the rate timer next fires
        self._demand = threading.Event()

    def request(self):
        """A consumer is about to want a frame: decode the next grabbed one."""
        self._demand.set()

    def should_decode(self, now, active=True):
        if not active:
            return False
        dt = self.min_dt
        if self._demand.is_set():
            self._demand.clear()
            if dt != float('inf'):
                self._next = now + dt  # a requested decode counts against the rate budget
            return True
        if dt == float('inf') or now < self._next:
            return False
        # Advance on a fixed schedule so frames that land just short of the
        # period do not push the rate down (30 fps at a 10 Hz cap would decode
        # every 4th frame, 7.5 Hz, with a plain "dt since last" test). After a
        # gap longer than one period, resync instead of bursting.
        self._next = now + dt if now - self._next > dt else self._next + dt
        return True


def gate_from_env(env=None):
    """A DecodeGate when the opt-in flag is set, else None (legacy read())."""
    return DecodeGate(decode_hz(env)) if on_demand_enabled(env) else None


def fourcc_str(cap):
    """The capture's FOURCC as text ('MJPG', 'YUYV', ...), '?' if unreadable."""
    try:
        v = int(cap.get(6))   # cv2.CAP_PROP_FOURCC; no cv2 import needed here
        return ''.join(chr((v >> 8 * i) & 0xFF) for i in range(4)) if v > 0 else '?'
    except Exception:
        return '?'


def next_frame(cap, gate, active=True, post=None):
    """One capture step: ``(ok, frame_or_None, t)``.

    ``gate is None`` -> legacy: ``cap.read()`` every call, stamped after it
    returns. Otherwise ``cap.grab()`` every call and ``cap.retrieve()`` only
    when the gate says so; ``frame`` is None for a frame that was grabbed but
    not decoded. ``ok`` False means the device failed. ``post`` (e.g. a
    rotate) runs only on decoded frames.
    """
    if gate is None:
        ok, frame = cap.read()
        t = _now()
        if not ok:
            return False, None, t
        return True, (post(frame) if post is not None else frame), t
    if not cap.grab():
        return False, None, _now()
    t = _now()
    if not gate.should_decode(t, active):
        return True, None, t
    ok, frame = cap.retrieve()
    if not ok or frame is None:
        return False, None, t
    return True, (post(frame) if post is not None else frame), t
