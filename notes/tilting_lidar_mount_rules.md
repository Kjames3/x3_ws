# Tilting lidar mount rules (hardware removed 2026-10-29)

The tilt mount, its Dynamixel XL430 and the OpenRB-150 were taken off the robot on
2026-10-29 when the Livox Mid-360S replaced the YDLidar X3. The text below was moved
word for word from `CLAUDE.md` on 2026-11-11. It applies again if the mount goes back on.

## Hardware list entry (was under `## Hardware`)

- **Lidar**: YDLidar X3 on `/dev/ttyUSB0` (TOF, 512000 baud; the driver comments
  say 8 Hz / ~1000 beams, the hardware actually does **7.2 Hz / 2795 beams**),
  on a Dynamixel-tilted mount driven via `/dev/openrb150`
  (`deploy/udev/64-openrb150.rules`)

## Tilting Lidar Mount

The tilt axis is a **Dynamixel XL430-W250-T (id 1, fw 50)** on an
**OpenRB-150** (`2f5d:2202` → `/dev/openrb150`). It has its own supply; XL430
stall current exceeds 800 mA, so it is never on the Orin 40-pin 5 V rail.

> **The LX-16A is RETIRED.** It, `/dev/lx16a`, `deploy/udev/62-lx16a.rules`,
> `src/lidar_tilt.py`, `x3_lidar_home.service` and
> `config/lidar_tilt_calibration.json` are all historical. Do not reason about
> the tilt axis from them — the counts scale, the port, the units and the
> calibration file are all different now. `src/lx16a_servo.py` is kept only for
> the bench. Background in the memory note `project_historical_superseded.md` (`notes/dynamixel_servo_history.md` never existed).

```bash
python3 src/dynamixel_tilt.py --status      # read-only (does NOT drop torque)
python3 src/dynamixel_tilt.py --calibrate   # store current position as level
python3 src/dynamixel_tilt.py --home        # drive back to level
python3 src/tools/calibration/dynamixel_setup.py --commission # bench tool: identify/set-id/zero/sweep
```

Calibration lives in `config/lidar_tilt_calibration_dynamixel.json`.
X-series position is **0–4095 over 360° (11.378 counts/deg)** — neither the
LX-16A's 4.167 nor the YB-SD15M scale `Rosmaster_Lib.__arm_convert_angle`
assumes. Zero is the user's **hand-levelled 2032, not 2048**; do not "fix" it.
`tolerance_counts` is 12 because the residual ~10-count error is **backlash,
not PID droop** — do not tune gains, approach from a consistent direction.
Never set the EEPROM Homing Offset: Goal Position stays clamped to 0..4095 *in
the offset frame*, silently halving the reachable arc.

`x3_dynamixel_tilt_home.service` homes the mount at boot, ordered before
`x3_server` (`x3_lidar_home.service` is disabled). `x3_server` also homes on
startup, because a restart while tilted used to gate `/scan` off forever.

Two traps for any new X-series code:
- **The Moving flag (addr 122) lags the Goal Position write**, so "poll until
  not moving" returns before motion starts and looks exactly like a stalled
  servo. Judge arrival on position error with a grace period.
- The GUI 3D sweep is **ping-pong, not sawtooth** (`lidar_scan_loop` reverses
  `sweep_dir` at the ends). `tilt_direction` is **+1** (increasing counts =
  nose DOWN). A -1 was briefly recorded on 2026-09-04 from a forward-sector
  floor-range test that mistook raw laser +X for robot forward (`laser_joint`
  has yaw=pi); it is **wrong**. The settling evidence is a single-run A/B in
  `artifacts/deskew-2026-09-04/sign_comparison.npz`, which stores the same
  returns transformed both ways: -1 puts 13.7% of returns up to **2.4 m
  underground** and finds no ceiling, while +1 puts the floor at z~0 as the
  scene's tightest plane (std 0.041 m) and a ceiling at +2.97 m. Do not
  re-derive this sign by comparing a +30 station against a -30 one — a global
  flip only swaps which physical station gets the label, so both look
  sensible. The +1 recorded for the retired LX-16A is a coincidence, not
  evidence; it does not carry over.
