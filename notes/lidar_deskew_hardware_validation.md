# Deskew deployment and physical validation — 2026-09-04

Deployed to /home/jetson/x3_ws on x3: sampler timestamp correction, acquisition
ordered timed-point publisher, buffered deskew processor, ROS dependencies and
processor parameters. Both ROS packages built on Jetson; all 18 geometry,
ROS/TF/message, invalid-input and sampler tests passed there.

Rollback copies: /tmp/x3-deskew-rollback/before.tar.gz plus calibration_before.json
on the robot. Changes remain uncommitted, including the driver submodule edit.

## Calibration correction discovered by physical testing

The first parked sweep produced 243 clouds but reconstructed the ceiling near
z=-2.1 m and the floor near z=+0.6 m. User confirmed the robot was on the floor.
This was not a deskew-only artifact: deskew versus rigid projection differed by
7.3 mm at p95. On identical captured returns, reversing only pitch reconstructed
floor near +0.03 m and ceiling near +2.7 m. The earlier -1 verification had
confused raw laser +X with robot +X despite laser_joint yaw=pi.

Corrected tilt_direction to +1 on both machines, retained zero 2032 and archived
the prior direction-verification block in calibration JSON. Repeated physical
sweep with the corrected live pipeline. This overturns the earlier sign claim;
it does not imply that all older +1 maps were mirrored.

## Corrected 60-second parked sweep

- 511 raw scans and 511 timed clouds observed during warmup, sweep and homing.
- 240 deskewed clouds, all successfully transformed with acquisition-time TF.
- Tilt coverage -44.12 to +44.30 degrees.
- Zero nonzero chassis commands; zero odometry translation.
- Zero /scan messages observed while tilt exceeded the test's 0.07 rad threshold.
- Floor-candidate median height +0.013 m; upper-surface z p95/p99 2.64/2.79 m.
- Deskew vs rigid point difference p50 0.23 mm, p95 6.64 mm, p99 25.0 mm.
  This is agreement with the stationary baseline, NOT absolute spatial error.
- Joint gaps p50 20.1 ms, p95 56.8 ms, max 206 ms. The processor's 120 ms
  gap rejection remains enabled; scans overlapping motion are also rejected.
- A robust floor fit on preselected near-floor candidates had a 4 mm intercept
  and 23 mm p95 residual among its selected inliers. This excludes outliers and
  cannot be interpreted as whole-cloud accuracy. Floor slope remains ~1.8 deg;
  mounting/level and ranging calibration need further investigation before
  claiming precise 3D localization.

## OctoMap and performance

A separate 10-second parked sweep fed an isolated /deskew_validation OctoMap
instance: 37 map messages, final binary payload 19,222 bytes. Temporary mapper
was stopped afterward; no existing saved map was overwritten.

A short /proc sample measured ~44.3% of one CPU core for the processor during
sweeping. A later py-spy sample found substantial rclpy subscription handling and
Point32 object creation cost. This is not an A/B baseline and not an isolated
numpy-kernel measurement. The prior 0.75 ms microbenchmark must not be presented
as end-to-end deployed cost. Packed PointCloud2 input is a likely next
optimization, to be measured rather than assumed.

## Final state and limits

x3_server active; sweep off; mount returned within horizontal tolerance;
/scan flowing; motors intentionally left disabled. require_settled remains
true. No continuous-sweep or moving-chassis validation performed. No virtual
horizontal scan or 3D localization added.

Evidence: artifacts/deskew-2026-09-04/corrected_result.json, corrected_points.npz,
floor_fit.json, sign_comparison.npz/png, octomap result, logs and test scripts.
The first and paired-sign captures are retained separately from corrected data.

Operational note: the second service restart waited on an orphan ros2 CLI daemon
inside its cgroup after server and hardware processes had exited. Stopping that
specific daemon completed the restart; no robot power cycle was needed.
