# Lidar deployment reconciliation — 2026-09-06

Deployed the local processor, processor YAML and OctoMap launch file after
comparing each with the robot. Remote originals are backed up under
`/home/jetson/x3_ws/artifacts/deploy-2026-09-06/before/`.
Built yahboomcar_bringup and yahboomcar_nav successfully and restarted x3_server.
No server code, calibration, viewer changes or interlocks were overwritten.

The timed driver source already matched byte-for-byte. Its submodule remains
modified, but `patches/ydlidar_timed_points.patch` is tracked and reverse-checks
successfully against the installed source version; a fork is not required for
this deployment. Preserve/apply that patch when recreating the checkout.

Verification:
- 22 tests passed on both laptop and Jetson, including the new behavioral
  regression for live publish_cloud_when_level, cloud_max_range_m and
  tilted_min_range_m changes.
- Installed processor SHA256 matches local source:
  bb52b0f2bb8669b0b2c903c8d488adb1855c38ddbcb8ad965f4c45c06359f063.
- Live processor starts at 6.0 m, require_settled=true, parked cloud publishing
  false. Temporary enabling produced 57 clouds in 8 s; a live 1 m limit kept
  maximum range at 0.998 m. Restoring 6 m admitted returns to 4.575 m.
- /scan continued at roughly 7 Hz throughout. Timed input contains
  acquisition_time and scan_duration. Every observed /cmd_vel was zero.
- Restored cloud_max_range_m=6.0 and publish_cloud_when_level=false; final 8 s
  window had 57 scans and no clouds. Motors disabled; no sweep requested.
- A temporary OctoMap launch using an unused input topic reported
  sensor_model.max_range=6.0. It was shut down, with no orphan server/throttle.
  This verifies configuration, not insertion throughput at six metres.

Raw verification/build logs and probe are in artifacts/deploy-2026-09-06 on
both machines. The room only supplied returns to 4.575 m, so this test does not
measure far-range accuracy or performance in a larger space.

The robot git history remains older/divergent; this was a scoped file deployment,
not a repository reset. The processor parameter fix, its regression test and
other sessions' working-tree changes are not committed by this deployment.
