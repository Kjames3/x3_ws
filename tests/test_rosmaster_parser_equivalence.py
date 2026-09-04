#!/usr/bin/env python3
"""Differential test: patched Rosmaster_Lib parser vs. the stock V3.3.1 parser.

The receive path is the one piece of this library that cannot be tested on the
bench without risking the robot, so it gets tested offline instead. We build
synthetic wire frames, feed them through both the original parser and the
patched one via a fake serial port, and require the decoded telemetry to match
byte for byte -- except where a fix deliberately changes the output, which is
asserted explicitly rather than allowed to drift.

Run:  python3 tests/test_rosmaster_parser_equivalence.py
"""

import importlib.util
import os
import random
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATCHED = os.path.join(REPO, 'src', 'Rosmaster_Lib.py')
# The stock file as it was before this change; 'main' is the pre-fix baseline.
BASELINE_REF = os.environ.get('ROSMASTER_BASELINE_REF', 'main')
BASELINE_PATH = 'src/Rosmaster_Lib.py'


# ---------------------------------------------------------------- fake serial
class FakeSerial(object):
    """Minimal pyserial stand-in that replays a fixed byte stream, then idles.

    Supports both access patterns: the stock parser's unbuffered one-byte
    ``read()`` calls and the patched parser's ``in_waiting`` + bulk ``read(n)``.
    """

    def __init__(self, *args, **kwargs):
        self.timeout = kwargs.get('timeout', None)
        self._data = bytearray()
        self._pos = 0
        self._lock = threading.Lock()
        self.is_open = True
        self.written = bytearray()
        self._exhausted = threading.Event()

    # -- test-side API
    def feed(self, data):
        with self._lock:
            self._data += data
            self._exhausted.clear()

    def wait_drained(self, timeout=5.0):
        return self._exhausted.wait(timeout)

    # -- pyserial API
    @property
    def in_waiting(self):
        with self._lock:
            return len(self._data) - self._pos

    def read(self, size=1):
        deadline = time.monotonic() + (self.timeout if self.timeout else 2.0)
        while True:
            with self._lock:
                avail = len(self._data) - self._pos
                if avail > 0:
                    n = min(size, avail)
                    out = bytes(self._data[self._pos:self._pos + n])
                    self._pos += n
                    if self._pos >= len(self._data):
                        self._exhausted.set()
                    return out
                self._exhausted.set()
            if time.monotonic() >= deadline:
                # Stock parser has timeout=None and would block forever; give it
                # a bounded stall so a buggy test cannot hang the suite.
                return b''
            time.sleep(0.001)

    def write(self, data):
        self.written += bytearray(data)
        return len(data)

    def isOpen(self):
        return self.is_open

    def close(self):
        self.is_open = False

    def cancel_read(self):
        self._exhausted.set()

    def reset_input_buffer(self):
        pass

    def flush(self):
        pass


# ------------------------------------------------------- packet construction
HEAD = 0xFF
DEVICE_ID = 0xFC


def frame(ext_type, payload):
    """Build one wire frame: HEAD, DEV-1, ext_len, ext_type, payload..., checksum.

    ext_len counts ext_type + payload + checksum, i.e. len(payload) + 3 minus
    the one byte the firmware does not count -- derived from the stock parser:
    data_len = ext_len - 2 bytes follow ext_type, the last of which is the
    checksum, so len(payload) == ext_len - 3.
    """
    payload = bytes(payload)
    ext_len = len(payload) + 3
    checksum = (ext_len + ext_type + sum(payload)) & 0xFF
    return bytes([HEAD, DEVICE_ID - 1, ext_len, ext_type]) + payload + bytes([checksum])


def speed_payload(vx, vy, vz, battery):
    return struct.pack('<hhhB', vx, vy, vz, battery)


def imu_payload(vals):
    return struct.pack('<9h', *vals)


def att_payload(roll, pitch, yaw):
    return struct.pack('<3h', roll, pitch, yaw)


def enc_payload(m1, m2, m3, m4):
    return struct.pack('<4i', m1, m2, m3, m4)


# --------------------------------------------------------------- module load
def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def load_baseline():
    """Materialise the pre-fix Rosmaster_Lib and import it.

    Normally read out of git, but ROSMASTER_BASELINE_FILE lets the suite run
    somewhere the pre-fix revision is not in local history -- e.g. on the robot,
    whose checkout has a diverged history.
    """
    explicit = os.environ.get('ROSMASTER_BASELINE_FILE')
    if explicit:
        with open(explicit, 'rb') as f:
            src = f.read()
    else:
        src = subprocess.check_output(
            ['git', '-C', REPO, 'show', '%s:%s' % (BASELINE_REF, BASELINE_PATH)])
    fd, tmp = tempfile.mkstemp(suffix='_baseline.py', prefix='rosmaster_')
    with os.fdopen(fd, 'wb') as f:
        f.write(src)
    return load_module(tmp, 'rosmaster_baseline'), tmp


def make_bot(module, fake):
    """Construct a Rosmaster with serial.Serial patched out."""
    import serial as _serial
    real = _serial.Serial
    _serial.Serial = lambda *a, **k: fake
    try:
        bot = module.Rosmaster(car_type=1, com='/dev/null', debug=False)
    finally:
        _serial.Serial = real
    return bot


def drive(module, stream, settle=0.35):
    """Feed `stream` through a fresh parser and return the decoded telemetry."""
    fake = FakeSerial(timeout=0.1)
    bot = make_bot(module, fake)
    bot.create_receive_threading()
    fake.feed(stream)
    fake.wait_drained(5.0)
    time.sleep(settle)
    snap = {
        'accel': bot.get_accelerometer_data(),
        'gyro': bot.get_gyroscope_data(),
        'mag': bot.get_magnetometer_data(),
        'att_rad': bot.get_imu_attitude_data(ToAngle=False),
        'att_deg': bot.get_imu_attitude_data(ToAngle=True),
        'motion': bot.get_motion_data(),
        'battery': bot.get_battery_voltage(),
        'encoder': bot.get_motor_encoder(),
    }
    stop = getattr(bot, 'stop', None)
    if stop:
        stop()
    return snap, bot


def approx(a, b, tol=1e-9):
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


# ---------------------------------------------------------------------- tests
class ParserEquivalence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.new = load_module(PATCHED, 'rosmaster_patched')
        cls.old, cls._tmp = load_baseline()

    @classmethod
    def tearDownClass(cls):
        try:
            os.unlink(cls._tmp)
        except OSError:
            pass

    # -- the stream used by most cases: one of every auto-report packet
    def build_stream(self, seed=1234):
        rng = random.Random(seed)
        self.imu_vals = [rng.randint(-30000, 30000) for _ in range(9)]
        self.speed = (rng.randint(-2000, 2000), rng.randint(-2000, 2000),
                      rng.randint(-2000, 2000), rng.randint(0, 255))
        self.att = (rng.randint(-30000, 30000), rng.randint(-30000, 30000),
                    rng.randint(-30000, 30000))
        self.enc = tuple(rng.randint(-2 ** 30, 2 ** 30) for _ in range(4))
        return (frame(0x0A, speed_payload(*self.speed))
                + frame(0x0B, imu_payload(self.imu_vals))
                + frame(0x0C, att_payload(*self.att))
                + frame(0x0D, enc_payload(*self.enc)))

    def test_clean_stream_matches_baseline(self):
        """Gyro, accel, speed, attitude and encoders must decode identically."""
        stream = self.build_stream()
        new, _ = drive(self.new, stream)
        old, _ = drive(self.old, stream)
        for key in ('accel', 'gyro', 'att_rad', 'att_deg', 'motion', 'encoder'):
            for n, o in zip(new[key], old[key]):
                self.assertTrue(approx(n, o),
                                "%s: patched=%r baseline=%r" % (key, new[key], old[key]))
        self.assertAlmostEqual(new['battery'], old['battery'], places=9)

    def test_gyro_sign_convention_unchanged(self):
        """The MPU gy/gz negation is preserved -- this is a refactor, not a recal."""
        stream = self.build_stream()
        new, _ = drive(self.new, stream)
        ratio = 1 / 3754.9
        self.assertTrue(approx(new['gyro'][0], self.imu_vals[0] * ratio))
        self.assertTrue(approx(new['gyro'][1], self.imu_vals[1] * -ratio))
        self.assertTrue(approx(new['gyro'][2], self.imu_vals[2] * -ratio))

    def test_magnetometer_now_in_tesla(self):
        """The one intended numeric change: raw counts -> tesla (AK8963 0.15 uT/LSB)."""
        stream = self.build_stream()
        new, _ = drive(self.new, stream)
        old, _ = drive(self.old, stream)
        for i in range(3):
            self.assertTrue(approx(old['mag'][i], float(self.imu_vals[i + 6])),
                            "baseline should publish raw counts")
            self.assertTrue(approx(new['mag'][i], self.imu_vals[i + 6] * 0.15e-6),
                            "patched should publish tesla")

    def test_struct_alignment_servo_and_pid(self):
        """'<Bh' / '<Bhhh' -- a native 'Bh' pads to 4 bytes and shifts the value."""
        self.assertEqual(struct.calcsize('<Bh'), 3)
        self.assertEqual(struct.calcsize('<Bhhh'), 7)
        stream = (frame(0x20, struct.pack('<Bh', 3, 2048))
                  + frame(0x13, struct.pack('<Bhhh', 1, 1234, -567, 890)))
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        fake.feed(stream)
        fake.wait_drained(5.0)
        time.sleep(0.3)
        self.assertEqual(bot._Rosmaster__read_id, 3)
        self.assertEqual(bot._Rosmaster__read_val, 2048)
        self.assertEqual(bot._Rosmaster__pid_index, 1)
        self.assertEqual(bot._Rosmaster__kp1, 1234)
        self.assertEqual(bot._Rosmaster__ki1, -567)
        self.assertEqual(bot._Rosmaster__kd1, 890)
        bot.stop()

    # -- the framing fixes, which the baseline cannot pass
    def test_resync_preserves_following_frame(self):
        """0xFF 0xFF <frame>: the stock parser ate two bytes and lost the frame."""
        stream = self.build_stream()
        good = frame(0x0D, enc_payload(11, 22, 33, 44))
        # A stray 0xFF immediately before a real header. Resyncing by two bytes
        # consumes the real HEAD; resyncing by one recovers.
        new, _ = drive(self.new, stream + b'\xff' + good)
        self.assertEqual(new['encoder'], (11, 22, 33, 44))

    def test_garbage_between_frames_is_skipped(self):
        stream = (b'\x00\x11\x22' + frame(0x0D, enc_payload(1, 2, 3, 4))
                  + b'\xfe\xff\x01' + frame(0x0D, enc_payload(5, 6, 7, 8)))
        new, _ = drive(self.new, stream)
        self.assertEqual(new['encoder'], (5, 6, 7, 8))

    def test_bad_checksum_does_not_eat_next_frame(self):
        bad = bytearray(frame(0x0D, enc_payload(9, 9, 9, 9)))
        bad[-1] ^= 0xFF          # corrupt the checksum only
        good = frame(0x0D, enc_payload(7, 7, 7, 7))
        new, bot = drive(self.new, bytes(bad) + good)
        self.assertEqual(new['encoder'], (7, 7, 7, 7))

    def test_truncated_length_byte_does_not_kill_thread(self):
        """ext_len < 3 made the stock parser raise struct.error and die."""
        evil = bytes([HEAD, DEVICE_ID - 1, 0x01, 0x0D, 0x0E])
        good = frame(0x0D, enc_payload(4, 3, 2, 1))
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        fake.feed(evil + good)
        fake.wait_drained(5.0)
        time.sleep(0.3)
        self.assertEqual(bot.get_motor_encoder(), (4, 3, 2, 1))
        self.assertTrue(bot._Rosmaster__rx_thread.is_alive(),
                        "receive thread must survive a malformed length byte")
        bot.stop()

    def test_frame_split_across_reads(self):
        """A frame arriving in two chunks must not be dropped."""
        f = frame(0x0D, enc_payload(100, 200, 300, 400))
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        fake.feed(f[:5])
        time.sleep(0.15)
        fake.feed(f[5:])
        fake.wait_drained(5.0)
        time.sleep(0.3)
        self.assertEqual(bot.get_motor_encoder(), (100, 200, 300, 400))
        bot.stop()

    def test_no_frames_lost_over_a_long_stream(self):
        """Every frame in a dense stream must be counted -- this is the frame-loss fix."""
        frames = b''.join(frame(0x0D, enc_payload(i, i, i, i)) for i in range(500))
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        fake.feed(frames)
        fake.wait_drained(5.0)
        time.sleep(0.4)
        stats = bot.rx_stats()
        self.assertEqual(stats['frames'], 500, stats)
        self.assertEqual(stats['checksum_err'], 0, stats)
        self.assertEqual(stats['malformed'], 0, stats)
        self.assertEqual(bot.get_motor_encoder(), (499, 499, 499, 499))
        bot.stop()

    # -- health / lifecycle
    def test_rx_healthy_reports_staleness(self):
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        self.assertFalse(bot.rx_healthy(), "no frame yet -> not healthy")
        fake.feed(frame(0x0D, enc_payload(1, 1, 1, 1)))
        fake.wait_drained(5.0)
        time.sleep(0.2)
        self.assertTrue(bot.rx_healthy())
        self.assertFalse(bot.rx_healthy(max_age=0.0))
        bot.stop()

    def test_snapshot_is_internally_consistent(self):
        stream = self.build_stream()
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        fake.feed(stream)
        fake.wait_drained(5.0)
        time.sleep(0.3)
        s = bot.get_imu_sample()
        self.assertEqual((s.gx, s.gy, s.gz), bot.get_gyroscope_data())
        self.assertEqual((s.ax, s.ay, s.az), bot.get_accelerometer_data())
        self.assertEqual((s.mx, s.my, s.mz), bot.get_magnetometer_data())
        self.assertEqual(s.seq, 1)
        self.assertGreater(s.t, 0.0)
        self.assertEqual(bot.get_imu_kind(), 'mpu9250')
        bot.stop()

    def test_stop_is_idempotent_and_joins_thread(self):
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        t = bot._Rosmaster__rx_thread
        bot.stop()
        bot.stop()
        self.assertFalse(t.is_alive(), "receive thread must exit on stop()")
        self.assertFalse(fake.is_open)

    def test_receive_thread_survives_port_errors(self):
        class FlakySerial(FakeSerial):
            def __init__(self, *a, **k):
                FakeSerial.__init__(self, *a, **k)
                self.fail_count = 0

            @property
            def in_waiting(self):
                if self.fail_count < 3:
                    self.fail_count += 1
                    raise OSError("simulated port error")
                return FakeSerial.in_waiting.fget(self)

        fake = FlakySerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        time.sleep(0.5)          # ride out the 0.05/0.10/0.20 s backoff
        fake.feed(frame(0x0D, enc_payload(6, 6, 6, 6)))
        fake.wait_drained(5.0)
        time.sleep(0.3)
        self.assertEqual(bot.get_motor_encoder(), (6, 6, 6, 6))
        self.assertGreaterEqual(bot.rx_stats()['errors'], 3)
        bot.stop()

    def test_get_version_does_not_busy_poll_repeatedly(self):
        """Without a version reply, repeated calls must not re-request every time."""
        fake = FakeSerial(timeout=0.1)
        bot = make_bot(self.new, fake)
        bot.create_receive_threading()
        fake.written.clear()
        t0 = time.monotonic()
        for _ in range(10):
            self.assertEqual(bot.get_version(), -1)
        elapsed = time.monotonic() - t0
        # Stock behaviour: 10 requests and ~220 ms of sleeping.
        self.assertLess(elapsed, 0.15, "get_version still busy-polls on every call")
        self.assertLessEqual(len(fake.written), 7,
                             "get_version re-requested more than once in 5 s")
        bot.stop()


if __name__ == '__main__':
    unittest.main(verbosity=2)
