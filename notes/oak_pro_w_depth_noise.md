# OAK-D Pro W depth noise model (2026-09-25)

Camera: OAK-D-PRO-W, MxId 14442C103181D7D600, CAM_A-aligned 480x640 depth,
fx=fy=676, mono 640x400 (fB = 21.6 px*m). All numbers are on depth corrected by
`config/oak_depth_correction.json` (Z / (1 + 0.0821 Z)), projector off unless noted.

## Floor (grazing incidence) — production stereo config

Exact `oakd_driver` settings: DEFAULT preset, LR check, speckle filter 50, 80 fps
mono. Lab carpet, 4.5 m clear, 100–150 frames. "Per-pixel" = per-row line fit
removed, so pitch/roll/bias are excluded. This is what ground removal sees.

| range (m) | temporal sd, sp off | fixed per-pixel sd, sp off | total per-frame sd, sp off | total, sp on |
|---|---|---|---|---|
| 0.50–0.75 | 0.9 mm | 4.3 mm | 4.5 mm | 5.1 mm |
| 1.00–1.25 | 2.7 | 11.6 | 12.2 | 13.0 |
| 1.50–1.75 | 4.7 | 21.9 | 22.4 | 24.5 |
| 2.00–2.25 | 11.5 | 38.9 | 41.3 | 42.2 |
| 2.50–2.75 | 27.4 | 56.6 | 60.0 | 64.7 |
| 2.75–3.00 | 19.4 | 69.2 | 71.8 | 107.0 |

- Fits: **total per-frame sd ≈ 0.009·Z² (subpixel off), 0.011·Z² (on)**; temporal
  only ≈ 0.0024·Z² (off), 0.0041·Z² (on).
- The error is dominated by a **fixed per-pixel** term. Subpixel does not shrink it,
  so it is not disparity quantization — it is matching error on a surface seen at a
  grazing angle. A per-row quadratic removes only 5–15%, so it is not a distortion
  bow either.
- Temporal sd alone understates the error badly: with subpixel off a pixel can sit
  on one disparity level for the whole capture (sd 0) while being ~half a step wrong.
- Bias vs flat-floor geometry: +7..+25 mm at 0.5–2 m, −20..−65 mm at 2.3–3 m;
  within what a 0.1 deg pitch error produces at those ranges.

## Fronto-parallel board (30.1 cm) — HIGH_DENSITY preset, not production

| range | mode | temporal sd | plane RMS / p95 |
|---|---|---|---|
| 0.57 m | sp off | 1.6 mm | 4.4 / 9.2 mm |
| 1.1 m | sp off | 22.3 mm (2 levels, 57 mm apart) | 11.1 / 19.7 mm |
| 1.1 m | sp on | 5.4 mm | 4.5 / 9.8 mm |

- Subpixel **does** help on a surface facing the camera (people, walls, obstacles),
  unlike on the floor. Not yet measured in the production preset or beyond 1.1 m.
- Disparity step with subpixel off: **0.046·Z² m** (5 cm at 1 m, 19 cm at 2 m,
  42 cm at 3 m); subpixel (3 bits) divides it by 8.

## Projector

- Plain board at >= 2 m: without the projector, matching fails (patches at 5–7 m).
- Board at 0.6 m: **with** the projector, the repeating dot pattern ghosts large
  patches to 0.4 m and 1.9 m. Off, the board is clean (0.62–0.65 m).
- It degrades the upper VL53L5CX array's far zones (valid 50.7 -> 46.3%).
- Production keeps it off.

## Use

- Ground removal / floor-height gating: use **0.011·Z²** per pixel (subpixel on, production; 0.009·Z² with it off).
- `c3_person_tracker.measurement_cov_camera` assumes the Lite's 0.0025·Z². It is
  clamped by `meas_sigma_floor_m` 0.25 m, so it only matters beyond ~5 m (0.0025) or
  ~5.3 m (0.009); not urgent, but the constant is the Lite's.
- Subpixel is **ON in production** since 2026-09-25 (see below).

## Subpixel and the velocity estimator (decided 2026-09-25)

With subpixel off, the disparity step is ~0.5 m at 3–3.5 m. A person standing at the
3.5 m tape mark read torso 3.19 m and legs 3.72 m — two adjacent disparity levels —
and a steadily walking person became a staircase. Parked robot, lane 1.0–3.5 m,
300 s walks, `phantom_baseline.py` + `score_velocity_models.py` replay:

| | subpixel off (1.34 m/s) | subpixel on (1.25 m/s) |
|---|---|---|
| live p95 error | −19 / −20% | −8 / −9% (gate 1.8 / 4.0) |
| replay v1 / v2 / v3 / compress25 | −27 / −22 / −21 / −22% | −17 / −13 / **−10** / −11% |
| person at 3.5 m (legs) | 3.72 m | 3.48 m |

Subpixel cost nothing measurable: `/oak/detections` 5.1 Hz (4.9 before), depth
16–25 fps either way. The floor noise table above does not show the benefit because
floor error is per-pixel matching error; a blob-median centroid cannot average away
quantization, since all its pixels share one disparity level.

### How it is enabled (interim)

`/etc/systemd/system/x3_server.service.d/90-c1-recording.conf` on the robot:
`SERVER_ARGS=--domain-id 42 --webrtc-camera --c1-recording --c1-subpixel`
(backup of the previous file: `~/wip-backup-2026-09-25/90-c1-recording.conf.bak`).
This only works because `server_x3.py` passes
`subpixel=args.c1_recording and args.c1_subpixel` and `--c1-subpixel` requires
`--c1-recording`. **Dropping `--c1-recording` silently turns subpixel off** and brings
back the −20%.

### TODO — standalone subpixel option (do with the C1 commit)

- `server_x3.py`: add `--oak-subpixel/--no-oak-subpixel`, default ON, independent of
  `--c1-recording`; pass `subpixel=args.oak_subpixel` to `OakDCamera`. Keep
  `--c1-subpixel` as a deprecated alias (or drop it) so the drop-in keeps working.
- `oakd_driver.OakDCamera`: default `subpixel=True`; the
  `setSubpixelFractionalBits(3)` + `setPostProcessingHardwareResources(2, 2)` block
  already exists and is what was tested.
- Record the stereo mode in the C1 calibration metadata (`stereo_subpixel` already is).
- Then remove `--c1-subpixel` from `90-c1-recording.conf`, restart, and check the
  depth has fine steps (>100 distinct values between 2.5 and 4.5 m in one frame).
- The depth correction (`config/oak_depth_correction.json`) was fitted with subpixel
  ON; refit if subpixel is ever turned off.

Scripts used (robot `/tmp`, not in repo): `floor_noise.py`, `board_noise.py`.
