# Rosmaster_Lib.py — Latency, Accuracy & Robustness Review

**Date:** 2026-08-07
**Target file:** `src/Rosmaster_Lib.py` (Yahboom official lib, v3.3.1, 1314 lines)
**Status:** Analysis only — **no code has been changed**. This is a backlog document.

Review of the low-level Rosmaster board serial library for anything that improves
execution, accuracy, or latency without degrading performance. Findings are ranked
by real-world impact on *this* workspace, not by generic code-quality severity.

## Scope note — the file exists twice

`src/Rosmaster_Lib.py` and `src/yahboomcar_bringup/yahboomcar_bringup/Rosmaster_Lib.py`
are **byte-identical** (verified: md5 `8dedae2406e593bffacef5336b448461`). Every fix below
must be applied to both copies, or one should be reduced to a shared import / symlink.
The copy actually loaded at runtime by the live driver is the `yahboomcar_bringup` one.

## What is actually on the hot path

Worth recording, because it corrects an intuitive but wrong assumption:
`set_car_motion()` is **not** called from the asyncio event loop.

- `SIM_MODE` and `ROS2_MODE` are strict complements (`src/server_x3.py:125-126`), so the
  `else` branch at `src/server_x3.py:1030-1043` that constructs `Rosmaster(sim_mode=False)`
  + `MecanumDrive` **can never execute**. `drive` is always a `ROS2Bridge`.
- `ROS2Bridge.move()` (`src/server_x3.py:546`) publishes a `Twist` — non-blocking, no serial.
- The real serial writer is `Mcnamu_driver_X3.py:139` `set_motor(...)`, running in the
  **single-threaded rclpy executor**, driven at 30 Hz by `motion_loop`
  (`src/server_x3.py:2202`, `await asyncio.sleep(0.033)`).
- That same executor thread also runs the 10 Hz `pub_data` timer that stamps and publishes
  IMU / `vel_raw`. So anything that blocks in `set_motor` directly jitters IMU timestamps.

Also note: the comment describing a "100 Hz drain" motion watchdog queue is stale —
`motion_loop` runs at 30 Hz with a maxsize-2 drop-oldest queue and a 0.50 s watchdog.

Data-rate ceiling: the MCU auto-reports 4 packet types, one per 10 ms, so **each type
refreshes every 40 ms (25 Hz)**. `pub_data` samples at 10 Hz — the stack currently discards
about 60% of the available sensor rate, at a non-integer ratio (2.5) that causes beat aliasing.

---

## A. Correctness & Safety

### 1. The receive thread can die silently and permanently
**Target:** `Rosmaster_Lib.py:257-281` (`__receive_data`), `:344-355` (`create_receive_threading`)
**Severity:** Critical

`__receive_data` has **zero exception handling**; `create_receive_threading`'s `try` only wraps
`.start()`. Any exception inside the loop kills the daemon thread with no message.

Reachable triggers:
- `bytearray(self.ser.read())[0]` → `IndexError` on a zero-byte read.
- `serial.SerialException` on USB re-enumeration (the CH341 does this on brownout).
- A corrupt frame with `ext_len < 2` makes `data_len <= 0`, so the payload loop never runs and
  `ext_data` stays `[]` with `rx_check_num = 0`. If `(ext_len + ext_type) % 256 == 0` the frame
  is **accepted as valid**, and `__parse_data` then calls
  `struct.unpack('h', bytearray([]))` → `struct.error` → thread dead.
- A false frame with `ext_len = 255` consumes 253 bytes, swallowing ~10 real packets.

After the thread dies, `__uart_state` remains `1`, so `create_receive_threading()` refuses to
restart it, and **every getter returns its last value forever with no error**. Frozen encoders
mean the EKF integrates zero motion — the robot believes it is stationary while driving.

Fix: wrap the loop body, bound `ext_len`, count consecutive errors, and expose health:

```python
def __receive_data(self):
    errors = 0
    while not self.__rx_stop.is_set():
        try:
            b = self.ser.read()
            if not b or b[0] != self.__HEAD:
                continue
            b = self.ser.read()
            if not b or b[0] != self.__DEVICE_ID - 1:
                continue
            ext_len = self.ser.read()[0]
            if ext_len < 3 or ext_len > 64:          # sanity bound
                continue
            ext_type = self.ser.read()[0]
            payload = self.ser.read(ext_len - 2)     # exact count, one syscall
            if len(payload) != ext_len - 2:
                continue
            if (ext_len + ext_type + sum(payload[:-1])) & 0xFF == payload[-1]:
                self.__parse_data(ext_type, payload)
                self.__last_rx = time.monotonic()
            errors = 0
        except Exception as e:
            errors += 1
            print(f"[Rosmaster] RX error #{errors}: {e!r}")
            if errors > 50:
                self.__rx_healthy = False
                return
            time.sleep(0.05)
```

Expose `rx_healthy` / `last_rx_time` so the server can e-stop rather than drive blind.

**The checksum arithmetic itself is correct.** Sender computes
`sum([0xFF, 0xFC, len, func, ...]) + COMPLEMENT`, which reduces mod 256 to
`len + func + payload` — exactly what the receiver accumulates. Resync after a bad frame is a
plain byte-scan for `0xFF`; a `0xFF 0xFB` sequence inside a payload can produce a false frame
with a 1/256 chance of false-accept. The `ext_len` bound above is what limits the damage.

### 2. No lock around `self.ser.write()`
**Target:** every setter method
**Severity:** High

pyserial's POSIX write is `while len(d): n = os.write(fd, d)`, and `os.write` releases the GIL.
Frames are 7–13 bytes against a ~4 KB tty buffer, so the common case is one `os.write` and
interleaving is rare. But at 11.5 kB/s, sustained cmd_vel plus RGB/beep/telemetry writes will
eventually fill the buffer, `os.write` returns short, and a second thread's frame is injected
mid-frame. The MCU then fails the checksum and **drops the motion command** — the robot holds
its previous velocity until the watchdog fires. A truncated frame can also leave the MCU parser
mid-packet so the *next* good frame is lost too.

Minimal fix — the lock must cover only the write, never the sleep:

```python
self._tx_lock = threading.Lock()      # in __init__
...
with self._tx_lock:
    self.ser.write(bytes(cmd))        # bytes(), not list
time.sleep(self.__delay_time)         # OUTSIDE the lock
```

### 3. Unconditional 2 ms `time.sleep()` after every write
**Target:** `Rosmaster_Lib.py:521` (`set_motor`), `:575` (`set_car_motion`), ~20 other sites
**Severity:** High

The sleep lands in the rclpy executor thread (see "hot path" above), not the event loop.
At 30 Hz that is **60 ms/s (6%) of that thread**, and each `set_motor` can delay the 10 Hz
`pub_data` timer by up to 2 ms, jittering the IMU/`vel_raw` timestamps that feed the EKF.
Wire time for the 12-byte MOTION frame is only ~1.04 ms.

The guarantee the sleep provides is "≥2 ms between successive writes" — identical to a
*pre-write deadline*, which costs **zero** when callers are already >2 ms apart (30 Hz = 33 ms
apart) and charges genuinely back-to-back bursts once rather than once per caller:

```python
# __init__:
self.__write_lock = threading.Lock()
self.__next_write = 0.0

def __send(self, cmd):                    # cmd = list WITHOUT checksum
    cmd.append(sum(cmd, self.__COMPLEMENT) & 0xff)
    buf = bytes(cmd)
    with self.__write_lock:
        now = time.monotonic()
        wait = self.__next_write - now
        if wait > 0:
            time.sleep(wait)
            now += wait
        self.ser.write(buf)
        self.__next_write = now + self.__delay_time
```

Every writer then collapses to `self.__send([...])`, which also removes ~20 copies of the
duplicated header/checksum/write/sleep boilerplate and folds in fix #2 for free.

Do **not** blind-lower `delay` below 2 ms — the MCU's RX ring depth is undocumented. Test first.

### 4. Torn multi-field reads
**Target:** `Rosmaster_Lib.py:1096-1148` (all getters)
**Severity:** High

Under CPython each attribute *load* is atomic, but
`m1, m2, m3, m4 = self.__encoder_m1, self.__encoder_m2, ...` is four separate `LOAD_ATTR`
opcodes with bytecode boundaries between them — the RX thread can complete a whole packet in
the gap. You can read m1 from packet N and m3 from packet N+1, producing a phantom per-wheel
jump that dead-reckoning reports as lateral slip.

Worse across calls: `Mcnamu_driver_X3.py:177-178` builds a single `Imu` message from
`get_accelerometer_data()` and `get_gyroscope_data()` — two separate calls that can straddle
packets ≥10 ms apart.

Zero-cost fix — publish one immutable tuple per packet, a single atomic `STORE_ATTR`:

```python
# writer, in __parse_data:
self.__encoders = struct.unpack('<4i', ext_data[0:16]) + (time.monotonic(),)
# reader:
def get_motor_encoder(self):
    return self.__encoders[:4]
```

Same treatment for the IMU triples and `get_motion_data`. This pairs naturally with #9.

### 5. `__uart_state` is a class attribute written as an instance attribute
**Target:** `Rosmaster_Lib.py:12` vs `:346`, `:352`
**Severity:** Medium

Declared on the class, read as `self.__uart_state`, but *written* as an instance attribute.
The first instance shadows it; the class value stays `0`. A second `Rosmaster` instance (test
script, stale node) will therefore happily start a **second RX thread on the same port** — two
threads racing `ser.read()`, each grabbing half of every packet, so nothing ever checksums.
`__del__` (`:136`) sets the *instance* attribute, so it never resets anything either.
Make it a plain instance attribute in `__init__`.

### 6. Blanket `try/except: print; pass` on ~20 public methods
**Target:** all setters
**Severity:** Medium

`set_car_motion` swallows `SerialException` on unplug, prints once per call, and returns `None`
— indistinguishable from success. Callers have no way to learn the motors were never commanded;
mid-Nav2-goal this means the stack keeps planning against a bus that is gone.

Also latent: `set_car_motion(v_x=40, ...)` → `int(40*1000)` overflows `'h'` → `struct.error` →
command silently discarded while the robot keeps moving. Clamp before packing.

Fix: return `True`/`False`, catch `serial.SerialException` specifically, and let
`struct.error`/`TypeError` propagate — those are programmer bugs, not I/O conditions.

### 7. `__del__` is the only cleanup path
**Target:** `Rosmaster_Lib.py:133-137`
**Severity:** Medium

`__del__` is not guaranteed to run (reference cycles, interpreter exit, tracebacks holding
frames). Motors retain their last commanded velocity when the port closes, so a crash can leave
the robot driving. Add an explicit `close()` that sends `set_car_motion(0,0,0)`, stops the RX
thread, then closes the port — plus `__enter__`/`__exit__`, wired into the server shutdown path.

---

## B. Accuracy

### 8. IMU axis signs are inconsistent — two separate problems
**Target:** `Rosmaster_Lib.py:154-161` (MPU9250), `:169-183` (ICM20948)
**Severity:** High — **needs hardware to verify which convention is correct**

**(a) Within the MPU9250 branch:** `gy` and `gz` are negated (`*-gyro_ratio`) but `ay`/`az` are
**not**. No rigid transform flips two gyro axes without flipping the matching accel axes (a 180°
roll about X flips `gy,gz` *and* `ay,az`). So the accelerometer gravity vector and the gyro rates
are expressed in different frames. `imu_filter_madgwick` fuses exactly these two — the gravity
correction term fights gyro integration, degrading roll/pitch.

**(b) Across branches:** the ICM20948 path negates nothing, while
`Mcnamu_driver_X3.py:197,213` unconditionally publishes `-gz`. Net result:

| Board IMU | Lib | Driver | Published gz |
|---|---|---|---|
| MPU9250  | negates | negates | **+raw** |
| ICM20948 | passthrough | negates | **−raw** |

Yaw-rate sign therefore depends on which chip is populated. `vel_raw.angular.z` feeds
`base_node_X3.cpp:86,94` (`heading_ += angular_velocity_z_ * dt`), and `imu/data_raw` feeds
Madgwick → EKF → `/odom` + TF. A flip means the map rotates the wrong way under SLAM.
Separately, `drivers_x3.py:107` applies **no** compensation, so its telemetry gz has the opposite
sign from the ROS topic on at least one of the two boards.

Fix: normalize the sign once in the library so both branches emit the same body frame, then drop
the `-gz` hack in `Mcnamu_driver_X3.py`. **Verify against the physical board before changing** —
this cannot be settled from the code alone.

### 9. No receive timestamps — published stamps are up to 40 ms late
**Target:** `__parse_data`, all getters
**Severity:** High — best effort-to-benefit ratio in this document

Nothing records when a packet arrived. The board refreshes each type every 40 ms; `pub_data`
polls at 10 Hz and applies `Clock().now()` at publish time (`Mcnamu_driver_X3.py:160`). The
stamp is therefore up to **40 ms newer than the data it labels**, and that lie propagates
straight into `imu/data_raw` → Madgwick → EKF → `/odom` → SLAM/Nav2. The 2.5 sampling ratio
also causes beat aliasing, so consecutive samples are alternately 40 or 80 ms stale.

Stamp `time.monotonic()` per packet type (fold it into the tuple from #4) and expose
`get_imu_snapshot() -> (ax..gz, t_rx)`. Costs nothing at runtime. Consumers should reject
samples older than ~200 ms rather than integrating stale deltas.

### 10. Push instead of poll — 2.5× odometry rate for free
**Target:** `Rosmaster_Lib.py` (new callback hooks), `Mcnamu_driver_X3.py` (drop the timer)
**Severity:** High value, moderate effort

Add per-packet callbacks (`on_imu`, `on_speed`, `on_encoder`) invoked from the receive thread.
This lets the driver publish at the **native 25 Hz instead of 10 Hz**, eliminates the aliasing
in #9 entirely, and removes the 0.1 s timer. Note the callback runs on the RX thread, so it must
stay cheap or hand off to a queue.

### 11. Accelerometer scale constant is 0.068% off
**Target:** `Rosmaster_Lib.py:158`
**Severity:** Low

`accel_ratio = 1 / 1671.84` assumes g = 9.8. The correct value is `1 / 1671.9827`
(`32768 / (2 * 9.80665)`) — a 0.068% scale error on all three axes.
By contrast `gyro_ratio = 1/3754.9` vs exact `3754.9403` is 1.2e-5, negligible.

### 12. Battery is quantized to 0.1 V
**Target:** `Rosmaster_Lib.py:148`, `:1139`
**Severity:** Low — not fixable client-side

`'B'` ÷ 10.0 → 0–25.5 V in 0.1 V steps. On a 3S pack that's ~4% SOC per LSB, making any
low-voltage threshold or discharge-slope estimate jittery. The firmware sends one byte, so
mitigate with an EMA in `drivers_x3.py` rather than pretending to more precision.

### 13. ICM20948 branch has 3.8× coarser gyro resolution
**Target:** `Rosmaster_Lib.py:170`
**Severity:** Informational

ICM gyro LSB is 0.001 rad/s vs MPU's 0.000266 rad/s — measurably more quantization noise into
Madgwick on ICM boards. Worth knowing when comparing filter tuning across two robots.

### 14. Attitude and encoder representation limits
**Target:** `Rosmaster_Lib.py:187-189`, `:192-196`
**Severity:** Informational

- Attitude `/10000.0`: int16 → ±3.2768 rad at 1e-4 rad (0.0057°) resolution. Adequate, but yaw
  range **barely exceeds ±π**, so it cannot represent a full unwrapped heading.
- Encoders are cumulative signed int32 with **no wraparound handling anywhere**. Only
  `server_x3.py:1996` reads them (display) and that path is currently dead, so impact today is
  nil — but any future odometry off `get_motor_encoder` must unwrap deltas via
  `((d + 2**31) % 2**32) - 2**31`.

---

## C. Performance

### 15. Per-byte `ser.read()` in the receive loop
**Target:** `Rosmaster_Lib.py:257-281`
**Severity:** Medium

Auto-report frames are `ext_len + 2` bytes: SPEED 12 B, MPU/ICM_RAW 23 B, IMU_ATT 11 B,
ENCODER 21 B. One frame per 10 ms → **100 frames/s, 67 B per 40 ms cycle, 1675 B/s** (15% of
the 11.5 kB/s link — throughput is a non-issue; latency and CPU are).

pyserial's `read()` is 1 `select()` + 1 `os.read()` per call, plus a `bytes()`+`bytearray()`
allocation per byte here. That's **~1675 read calls = ~3350 syscalls and ~1675 RX-thread
wakeups per second**, each taking the GIL. The wakeups matter more than the CPU, because they
inject scheduling jitter into a process also running camera and lidar.

Measured over a pty on a dev box: `ser.read()` **1.93 µs/byte** vs chunked read
**0.009 µs/byte** (~200×). Estimated on Orin Nano (A78AE, ~3–4× slower): 6–8 µs/byte →
**10–13 ms CPU/s (~1% of a core) → under 1 ms/s**, with wakeups dropping 1675/s → ~100/s.

Two viable shapes: the exact-count `read(ext_len - 2)` in #1 (simplest, one syscall per frame),
or a full accumulator + state machine that drains `in_waiting` per wakeup:

```python
def __receive_data(self):
    HEAD, DEV = self.__HEAD, self.__DEVICE_ID - 1
    ser, buf = self.ser, bytearray()
    while True:
        try:
            n = ser.in_waiting
            chunk = ser.read(n if n else 1)   # block for >=1 byte, then drain
        except Exception as e:
            if self.__debug: print("serial rx error:", e)
            time.sleep(0.5); continue
        if not chunk:
            continue                          # timeout tick
        buf += chunk
        i, n = 0, len(buf)
        while True:
            if i + 4 > n: break
            if buf[i] != HEAD:   i += 1; continue
            if buf[i+1] != DEV:  i += 2; continue
            ext_len = buf[i+2]
            if ext_len < 2:      i += 1; continue
            end = i + 2 + ext_len
            if end > n: break                 # partial frame, wait for more
            ext_type = buf[i+3]
            payload  = buf[i+4:end]           # last byte is the checksum
            if (ext_len + ext_type + sum(payload[:-1])) & 0xFF == payload[-1]:
                self.__parse_data(ext_type, payload)
            elif self.__debug:
                print("check sum error:", ext_len, ext_type, list(payload))
            i = end
        del buf[:i]
        if len(buf) > 4096: del buf[:-256]    # runaway guard
```

### 16. Nine `struct.unpack` calls + nine `bytearray()` allocations per IMU packet
**Target:** `Rosmaster_Lib.py:141-196` (`__parse_data`)
**Severity:** Low absolute, but free

Measured: **2.36 µs → 0.137 µs, ~17× faster**, and 9 fewer allocations per packet. At 100 Hz
aggregate that's only ~0.2 ms/s, but it costs nothing to take:

```python
_IMU9 = struct.Struct('<9h')      # module level, precompiled
_ENC4 = struct.Struct('<4i')
_SPD  = struct.Struct('<3hB')
...
gx, gy, gz, ax, ay, az, mx, my, mz = _IMU9.unpack_from(ext_data)
```

`unpack_from` accepts `bytes`, so combined with #15 (where `ext_data` is already `bytes`) the
`bytearray()` copies vanish entirely. Pairs with the atomic-tuple fix in #4.

### 17. Native-endian format strings should be pinned to `'<'`
**Target:** all `struct.pack`/`unpack` calls
**Severity:** Low now, **blocking for #16**

Verified on this machine: `sys.byteorder == 'little'`, `calcsize('h') == 2`, `calcsize('i') == 4`
— identical to `'<'`. aarch64 Linux is always little-endian in practice (aarch64_be exists but
is not a JetPack target), so nothing miscomputes today. Two reasons to pin `'<'` anyway:

1. **Native format inserts alignment padding** the moment you use a multi-field format:
   `calcsize('Bh')` is 4 native but 3 with `'<'`. The `'B'`+`'h'` fields at `:200-201` and
   `:222-234` would silently misparse if merged into one native call — so this must be fixed
   *before* doing #16, and `'<3hB'` is mandatory over native `'3hB'`.
2. It documents the firmware's wire order instead of inheriting the host's. The `pack()` setters
   (`:392, 540, 564-566, 600-602, 689-690, 820-827`) put bytes on the wire and deserve the same.

### 18. `get_version()` is called every `pub_data` tick
**Target:** `Rosmaster_Lib.py:1203-1216`, called from `Mcnamu_driver_X3.py:174`
**Severity:** Latent

Harmless once `__version_H` populates (it then returns a cached value). But if it never
populates, each call burns 2 ms of `__request_data` plus up to 20 ms of `time.sleep` **inside
the 100 ms timer** — a latent 22% duty-cycle stall on the executor thread. Make it a one-shot
non-blocking read at init and cache the result in the driver.

### 19. Port configuration
**Target:** `Rosmaster_Lib.py:20`
**Severity:** Low, but `exclusive` is worth it independently

```python
self.ser = serial.Serial(com, 115200, timeout=0.5, write_timeout=0.2, exclusive=True)
try:
    self.ser.set_low_latency_mode(True)
except Exception:
    pass          # ch341 likely ENOTTY
```

`exclusive=True` matters here because this repo already fights over `/dev/ttyCH341USB0` between
the server and orphaned Mcnamu/base_node processes (see the comment at `server_x3.py:2190`) —
`O_EXCL` converts silent frame corruption into a clean `SerialException`.
`timeout` also removes the forever-hang on unplug.

**Flagged as unverified:** `/sys/bus/usb-serial/devices/*/latency_timer` is created by
**ftdi_sio only**; ch341 does not export it, so there is no latency-timer knob on this hardware.
`set_low_latency_mode` issues `TIOCSSERIAL` + `ASYNC_LOW_LATENCY`; ch341 may not implement
`.set_serial` (→ ENOTTY), and the tty `low_latency` flag has been largely inert since ~Linux
3.12. **Expected gain on CH341: ~0.** The RTT floor is set by USB full-speed 1 ms frames plus
CH341 internal packetization (~1–2 ms), not by anything tunable from Python. Confirm with
`ls /sys/bus/usb-serial/devices/*/` on the Jetson before assuming otherwise.
(Could not be checked during this review — the device was not attached.)

### 20. Minor cleanups
**Target:** various
**Severity:** Cosmetic

- `:349` `setDaemon()` is deprecated on Python 3.10.12 (confirmed version). Use
  `threading.Thread(..., daemon=True)`.
- `:145-147` `int(struct.unpack('h', ...)[0])` — `'h'` already yields `int`. The cast is a no-op.
- `:1118` hardcoded `RtA = 57.2957795` → use `math.degrees` (5e-9 relative difference; clarity
  only).
- `ser.write(list)` → `bytes(cmd)`: 0.377 µs → 0.089 µs, i.e. ~9 µs/s at 30 Hz. Free inside
  #3's `__send`, but claim no real saving. Do **not** add `ser.flush()` — it's `tcdrain` and
  blocks ~1 ms+.
- `__arm_convert_value` (`:296-310`) has four identical branches for ids 1–4 and uses bare
  `int()` (truncate-toward-zero, biased low by up to 1 LSB ≈ 0.08°), while
  `__arm_convert_angle` (`:314-328`) uses `int(x + 0.5)` — correct only for positives. The
  round-trip is not identity. Replace with a table + `round()`:

```python
_ARM_RANGE = {1: (900, 3100, 180, 0), 2: (900, 3100, 180, 0), 3: (900, 3100, 180, 0),
              4: (900, 3100, 180, 0), 5: (380, 3700, 0, 270), 6: (900, 3100, 0, 180)}

def __arm_convert_value(self, s_id, a):
    if s_id not in _ARM_RANGE:
        return -1
    p0, p1, a0, a1 = _ARM_RANGE[s_id]
    return int(round((p1 - p0) * (a - a0) / (a1 - a0) + p0))
```

---

## D. Dead code and library defects worked around downstream

These are places where callers reimplement or paper over a library limitation. Each is evidence
of a defect worth fixing at the source so the workaround can be deleted.

- **`set_car_motion` only applies the first non-zero axis** (firmware limitation, priority
  vx > vy > omega), so combined vx+omega is broken. `Mcnamu_driver_X3.py:92-95, 111-139`
  reimplements mecanum IK in Python and uses `set_motor` instead. Document this in the library.
- **`get_motion_data()`'s `vz` is wrong** — M2/M3 encoder cables are swapped on the board, so
  the firmware computes ~0 angular velocity during pure rotation. `Mcnamu_driver_X3.py:206-212`
  discards it and substitutes `-gz`; `:137-139` swaps M2/M3 in the `set_motor` argument order.
- **No velocity→PWM calibration**, so `Mcnamu_driver_X3.py:119-127` hand-rolls a `min_pwm = 28`
  deadband plus proportional normalization.
- **No ramping or command timeout**, so there are now *two* watchdogs:
  `Mcnamu_driver_X3.py:88-89, 153-157` and `server_x3.py:2147-2202`.
- **Commands are unacked**, so `Mcnamu_driver_X3.py:141-147` fires `for i in range(3)` retry
  loops around `set_colorful_effect` / `set_beep` — 3× redundant writes.
- **`server_x3.py:1991/1997`** caches battery at 1 Hz ("Idea 67") because `get_battery_voltage`
  *looks* expensive but is actually a free cached attribute read — cargo-culted around an
  opaque API.
- **Unreachable branch:** `server_x3.py:1030-1043` and `drivers_x3.py:39-127` never execute.
  `drivers_x3.py:56` is the **only** caller of `set_auto_report_state` anywhere in the repo, so
  the live system relies purely on the firmware default. Either delete or gate this branch.
- `Mcnamu_driver_x1.py` and `Ackman_driver_R2.py` are not launched by `x3_bringup.launch.py` or
  `x3_slam.launch.py` — dead for the X3.

---

## Suggested sequencing

1. **RX robustness** (#1) — safety. Nothing else matters if telemetry can freeze undetected.
2. **Snapshot tuples + per-packet timestamps** (#4, #9) — biggest accuracy win, nearly free.
3. **`__send()` helper: write lock + monotonic deadline** (#2, #3) — removes 60 ms/s of executor
   stall and ~20 copies of boilerplate in one refactor.
4. **Chunked RX parser** (#15) — ~10× RX CPU, ~16× fewer GIL handoffs.
5. **Pin `'<'` then merge unpacks** (#17, #16) — in that order; #17 is a prerequisite.
6. **Port config** (#19) — `exclusive`/`timeout`/`write_timeout`.
7. **Push callbacks at 25 Hz** (#10) — larger change, touches the driver.
8. **IMU sign audit** (#8) — **requires the physical board**; cannot be settled from code.
9. Cleanups (#5, #6, #7, #11, #18, #20) and dead-code removal (section D) as they come up.

Throughout: apply every change to **both copies** of the file, or collapse them first.
