---
name: project-rosmaster-lib
description: "Rosmaster_Lib.py hardware-measured facts — 2ms gap is load-bearing, MPU9250 not ICM, mag scale wrong, baud can't be raised"
metadata: 
  node_type: memory
  type: project
  originSessionId: e9d76d65-23d3-42a9-8fb4-0136bfc58cab
  modified: 2026-08-08T08:06:54.339Z
---

Measured on the live X3 robot (10.13.245.167, firmware V2.4) on 2026-08-07/08.
Full reports in `analysis/` on branch `worktree-rosmaster-perf` (commits f2cf6fb, 1b5ddb6).

**FIXES LANDED 2026-08-08 on branch `worktree-rosmaster-lib-fixes` (46e14c5, 0fba30b),
pushed, NOT merged. Deployed to the robot 2026-08-08 via scp + colcon build; the
pre-fix files are backed up at `~/x3_ws/.rosmaster_backup_20260808_004849/` there.**
Deploy gotcha confirmed live: the robot's git history has diverged, so sync by scp,
not git pull — and `colcon build` alone is a NO-OP if the source was never copied
over. Always grep the `build/` copy for a marker string afterwards.
`sudo` on the jetson is NOT passwordless, so a session cannot restart x3_server
itself; the user must run `ssh -t jetson@... 'sudo systemctl restart x3_server'`. Covers: buffered frame reader
with one-byte resync, RX error handling + `rx_healthy()`/`rx_stats()`, atomic
per-packet snapshots (`get_imu_sample()` etc.), `'<'` struct prefixes, magnetometer
→ tesla, unified `_GYRO_SIGNS`, `stop()`/`__del__` lifecycle, rate-limited
`get_version()`, plus a staleness gate in `Mcnamu_driver_X3.pub_data`.
Offline proof: `tests/test_rosmaster_parser_equivalence.py` (15 cases, differential
vs. the stock parser through a fake serial port). Both library copies patched.
The 2 ms write delay and the gz double-negation were deliberately NOT touched.

**The 2 ms `time.sleep(__delay_time)` after every `ser.write()` is REAL and load-bearing
— do NOT delete it.** Removing it costs 35–60% frame loss. Minimum safe gap is
command-dependent: >=1.25 ms for a 7-byte command, >=2.25 ms for a 9-byte RGB write
(the stock 2.00 ms is marginal, 94.2%). Frames are lost whole; no checksum failures.
`set_motor` was never tested (motion safety) so the motion-path minimum is unknown.
The correct fix is NOT removal but a monotonic minimum-spacing *deadline* (sleep the
residual gap before the write): at the production 30 Hz cmd_vel rate commands are
already 33 ms apart, so the sleep becomes a no-op and recovers 62 ms/s of blocking in
Mcnamu_driver_X3's single-threaded rclpy executor.

**The IMU is an MPU9250, not an ICM20948.** The board emits ext_type 0x0B only
(732 frames vs 0 of 0x0E in 30 s); the ICM branch in `__parse_data` is dead code.
The MPU branch negates gy/gz and `pub_data` negates gz *again*, so the two cancel and
published gz carries the raw sensor sign. Which negation to delete is **still
UNRESOLVED** — needs someone physically present to rotate the chassis by hand and
watch `/imu/data_raw.angular_velocity.z` against REP-103 (CCW from above = positive).
Both negations were kept as a matched pair in the fix branch for exactly this reason.
Magnetometer was published ~6.7 million× too large (`mag_ratio=1`; AK8963 is
~0.15 µT/LSB) — **fixed** to tesla on the fix branch and verified live 2026-08-08.
But `/imu/mag` then reads a steady **~10 µT**, well below Earth's ~48 µT, and the
vector swings wildly when the motors are energized. The units are now dimensionally
correct; the sensor is still **uncalibrated (hard-iron / motor interference) and
must not be used for heading** without a calibration pass. Pre-existing, not caused
by the unit fix.
Accel and gyro-at-rest both check out.

**Baud cannot be raised**: 230400 and 460800 yield 0 valid frames, and no baud command
exists in the protocol — it would need a reflash. Pointless anyway, the link is only
11.86% saturated (73.19 frames/s, 1366.5 B/s of the 11520 B/s ceiling).

**0x0C attitude is never sent by this firmware**, so `get_imu_attitude_data()` returns
zeros forever. Anything relying on it is silently dead.

Perf reality check: the RX thread costs 2.4% of one core (0.35% of the box). The
struct/checksum micro-optimizations logged as A-04/A-19 save ~0.02% of a core — take
them for `struct` alignment safety (`calcsize('Bh')==4`; needs a `<` prefix or the PID
and servo branches silently corrupt), not for speed.

No pip/egg shadow exists; colcon uses `--symlink-install`, so editing
`src/Rosmaster_Lib.py` does reach the running driver. See [[project_robot_deploy]].
