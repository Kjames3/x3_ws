# REAL lab forward odometry baseline — 2026-09-10

User measured all four endpoints at 4.000 m forward along the existing taped
line, with right offsets 150–152, 148, approximately 145, and 75 mm.
User observed a small right-facing arc. Physical final heading was not measured.

Bags are on x3 under `/home/jetson/bags/`:

| Bag | Start averaging window (s) | End averaging window (s) | /odom forward (m) | /odom right (mm) | Heading change (degrees, CCW positive) |
| --- | --- | --- | --- | --- | --- |
| odom_forward_1_20260910_172859 | 5–9 | 28–31 | 4.1362 | 117.9 | -2.640 |
| odom_forward_2_20260910_173154 | 8–12 | 30–35 | 4.1060 | 66.7 | -2.363 |
| odom_forward_3_20260910_173445 | 6–10 | 30–33 | 4.1764 | 108.1 | -4.093 |
| odom_forward_4_20260910_173655 | 1–4 | 25–30 | 4.1397 | 4.1 | -1.278 |

Windows are relative to the earliest bag message receipt timestamp. Means use
stationary pose samples; yaw uses a circular mean. Endpoint displacement is
rotated into the starting chassis frame. Comparing this frame with the tape
assumes the robot initially faced along the tape. Initial alignment error is
particularly important for interpreting lateral residuals.

All four SQLite databases passed integrity_check. Motion intervals were
approximately 11–25, 14–28, 11–28, and 6–22 s. The later handling in bags 2
and 4 was excluded. Bag 4 has a stationary interval before handling at ~55 s.

Forward overestimate is 106–176 mm (2.65–4.41%), mean 139.5 mm (3.49%).
Filtered and raw odometry endpoints agree within 0.4 mm in these calculations.
Odometry reports clockwise heading change in every run, consistent in direction
with the observed arc, but there is no independent angular ground truth.
Lateral offsets are under-reported by approximately 33, 81, 37, and 71 mm.
These runs establish endpoint error, not proof that wheel slip alone caused it.
No odometry scale or EKF parameters changed. Strafe baseline remains pending.

## Strafe diagnostic and velocity trace

Bag `odom_strafe_1_20260910_174525` on x3 passed SQLite integrity_check.
User confirms 4.000 m along the tape, 0.970 m left viewed start-to-finish,
starting chassis rotated 90 degrees CCW. Thus measured displacement in initial
body coordinates is approximately x=+0.970 m, y=-4.000 m. CBF intervened twice;
this is not a clean single-axis scale calibration.

Stationary averages at 5–9 and 44–46 s give /odom displacement
x=-2.3811 m, y=+7.1413 m, heading change +28.3219 degrees.
/odom_raw is effectively identical. Physical final heading is unmeasured.

Read-only live trace, 2026-09-10:

* `/base_node` parameters: linear_scale_x=-0.686, linear_scale_y=+1.0,
  angular_scale=1.0, pub_odom_tf=false.
* `/driver_node`: use_external_imu_yaw=true, publish_imu=false;
  FL/RL gains 1.0, FR/RR gains 0.95; wheel_separation_factor=0.165.
* Robot and laptop driver/base-node source SHA256 hashes match.
* Rosmaster_Lib decodes signed firmware speed words /1000; get_motion_data
  returns them unchanged. Mcnamu_driver_X3 publishes vx and vy unchanged to
  /vel_raw. Base node scales these once and integrates normal planar body-to-
  odom rotation. EKF consumes that pose and twist. No downstream lateral sign
  correction exists. The driver already remaps M2/M3 on motor output; the exact
  firmware/hardware reason for the feedback sign is not established here.

For 14–28 s, before observed command cross-axis interventions:

| Topic | mean vx (m/s) | mean vy (m/s) | mean wz (rad/s) |
| --- | --- | --- | --- |
| /cmd_vel | 0 | -0.07368 | 0 |
| /vel_raw | -0.01617 | +0.28464 | +0.02503 |
| /odom_raw | +0.01109 | +0.28464 | +0.02503 |
| /odom | +0.01146 | +0.28570 | +0.02671 |

This isolates the sign mismatch upstream of the integrator/EKF. Command
magnitude is not a ground-truth speed: motor control is open-loop PWM with a
28-unit additive deadband, SCALE=200, gains, and saturation normalization.
Do not derive an odometry scale from the command/feedback ratio.

Offline replay of recorded raw-odom increments, decomposed into body x/y using
the preceding heading, over 9–44 s reproduces the original endpoint exactly.
Changing only the lateral sign gives x=+2.2533 m, y=-7.3409 m. A least-squares
one-run lateral multiplier -0.53003 gives x=+1.1643 m, y=-3.9378 m, still missing
the measured endpoint by ~0.204 m. This value is diagnostic only, NOT a
deployable calibration (arc, CBF, single direction/run, alignment uncertainty).

Next: user-operated short 1 m unobstructed robot-right and robot-left runs,
recorded separately with stationary endpoints and independent tape measurements.
Keep current parameters for these baseline measurements. No live parameters,
motor commands, services, or estimator code were changed during this audit.

## Short strafe pair

Both new SQLite bags passed integrity_check:

* `odom_strafe_right_1m_20260910_175808`: user measured 1 m along tape,
  3 cm left deviation. Raw-odom stationary windows 5–8 and 19–22 s:
  initial-body x=+0.06446 m, y=+1.97372 m, yaw +2.3623 degrees.
  Command vy roughly -0.065 m/s, but at ~16 s there is a brief vx-positive
  component and increased negative vy. Whether this was CBF or manual input
  remains unanswered.
* `odom_strafe_left_1m_20260910_180205`: user reports finish facing right,
  approximately 36 cm right of target line; user subsequently explicitly
  confirmed the endpoint was aligned with the 1 m finish mark. Raw-odom stationary
  windows 6–9 and 18–21 s: x=-0.37387 m, y=-1.94687 m, yaw -21.9727 degrees.
  Commands are pure positive vy, approximately +0.110 m/s; vx and wz remain
  zero throughout recorded commands. This does not rule out CBF scaling vy.

Opposite lateral feedback sign occurs in BOTH directions. Candidate lateral
scale -0.5 replay (same increment decomposition as above) gives:
right x=+0.16745, y=-0.97859 m; left x=+0.14015, y=+0.98241 m.
This supports a provisional sign-and-scale correction, not validation of full
trajectory or cross-track accuracy. Tape-to-body cross-track signs must account
for the starting orientation for each run; do not equate odom x with tape right
without that check. The two command speeds differ, so the large difference in
yaw drift cannot be attributed to direction alone. No parameters changed.

## Provisional correction deployment

User authorized deployment after confirming the left endpoint. On 2026-09-10,
changed only `linear_scale_y` from +1.0 to -0.5 in x3_bringup.launch.py and
x3_slam.launch.py on laptop and robot, including the robot's installed launch
copies. AST parsing passed. Forward scale stays -0.686 and yaw scale stays 1.0.
Motor mixing, wheel gains, CBF and EKF configuration are unchanged.
Robot backup directory: `/home/jetson/x3_ws/artifacts/lateral-scale-20260910_180935`.
Restart requested through systemd; physical acceptance requires new short bags
with measured endpoints. This corrects estimated lateral motion, not commanded
motion or the physical yaw drift.

Post-restart parameter service verified active scales [-0.686, -0.5, 1.0].
An 8 s subscriber check received 400 /odom, 80 /odom_raw, 38 /scan, and 1600
/imu/data_raw messages. Physical post-change verification is still pending.

## Corrected physical runs

User recorded right/left corrected 1 m runs and reported tape deviations of
9 cm left and 36 cm right respectively. User described clockwise rotation
in both directions. Treat 1 m along-line travel as the test intent pending
explicit confirmation if precision calibration is needed.

Both databases passed integrity_check. Stationary windows 6–9 and 17–20 s:

| Bag | /odom initial-body x (m) | /odom initial-body y (m) | Heading change |
| --- | --- | --- | --- |
| odom_strafe_right_corrected_1m_20260910_181230 | +0.17394 | -0.98332 | +12.4497 deg CCW |
| odom_strafe_left_corrected_1m_20260910_181610 | +0.18998 | +0.90178 | -27.2769 deg CW |

Raw and filtered endpoints agree within 0.1 mm. The sign fix is physically
supported; right along-axis distance is ~1.7% low and left ~9.8% low if both
ended exactly at 1 m. Full cross-track/heading accuracy is not established.
No command yaw or forward command was present during either main drive;
steady lateral commands were approximately -0.118 and +0.120 m/s. Recorded
heading therefore indicates opposite-sign yaw drift at similar command speeds,
contrary to the user's visual clockwise description of the right run. Resolve
this observation/frame discrepancy before choosing a yaw compensation sign.
No further calibration or motor-control changes made following these bags.
