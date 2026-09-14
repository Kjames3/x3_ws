# Diagnostic 3D-lidar localization — 2026-09-06

Implemented and installed an opt-in ROS localizer on the Jetson. It estimates
x/y/yaw using point-to-plane matches against 3D surfaces. It does not estimate
z/roll/pitch, publish TF, command motion, or replace AMCL. No chassis movement,
lidar sweep, or robot-service restart was needed for this work.

## Map and input contract

`src/build_lidar_localization_map.py` rebuilds the five-station graph using
only alternating complete clouds. The other clouds are held out of BOTH edge
fitting and map construction. A connectivity check admits only the component
anchored at station 0. Stations 0, 1, 2 and 4 form that component; station 3 is
excluded, not silently inserted at its drifted odometry pose. Its area requires
new overlapping observations before inclusion. The generated map has 36,968
points/normals at 5 cm voxels. Reverse yaw consistency supplements the existing
edge overlap/residual/reverse-translation checks.

This resolves map contamination by exclusion, not by claiming station 3 was
successfully localized. The builder still uses the experimental graph code;
it is an offline preparation tool, not an online map optimizer.

The diagnostic node consumes deskewed PointCloud2 at the original scan-start
stamp, transforms it into base_footprint using TF at that stamp, and uses
odometry plus the last accepted map-to-odom correction as its prior. It needs
an explicit map-aligned initial pose. A fresh odometry origin after reboot is
NOT automatically aligned to this map. Only accepted results update that
internal correction, and only accepted results publish a PoseStamped.

## Conservative admission checks

- Finite input, minimum 150 usable points/correspondences, maximum 3,000 query
  points; initial pose constrained to planar SE(2).
- At least 60% query overlap inside 15 cm; point-to-plane RMSE <=6 cm and
  Euclidean nearest-neighbor RMSE <=10 cm.
- Convergence within 30 iterations; normalized information-matrix condition
  <=1,000 with nonzero information for every estimated DOF.
- Correction relative to prior <=30 cm and <=15 degrees.
- 120 ms matcher budget; input AND completed result age <=500 ms.
- Missing TF, out-of-order timestamps, and stale clouds produce status only.
  One pending cloud is retained; newer input replaces pending older input.

These are provisional diagnostic gates. They reject the recorded low-overlap
case but do NOT prove immunity to repeated-geometry false matches. There is no
global relocalization, automatic initialization, or calibrated covariance.

## Results

137 held-out clouds from connected stations were tested with an explicitly
map-aligned prior perturbed by 10 cm / 2 degrees. Both machines accepted 136;
one failed convergence. Reference poses come from the training graph, not
independent ground truth.

| Measurement | Laptop | Jetson |
|---|---:|---:|
| Matcher runtime median | 12.7 ms | 34.0 ms |
| Matcher runtime p95 | 26.4 ms | 75.7 ms |
| Matcher runtime maximum | 41.4 ms | 107.3 ms |
| Accepted graph disagreement median / p95 / maximum | 6.8 / 48.0 / 112.1 mm | same |
| Accepted heading disagreement median / p95 / maximum | 0.122 / 0.583 / 1.711 deg | same |

Jetson benchmark ran with the normal robot service active and single-threaded
OpenBLAS. It did not include simultaneous continuous sweeping and OctoMap
insertion. Matcher timing excludes ROS transport and TF; it is not an
end-to-end latency guarantee. Held-out clouds share the same room, stationary
stations and session as training. This is repeatability/consistency evidence,
not a 6.8 mm absolute localization-accuracy claim.

The actual station 3-to-4 low-overlap regression was rejected (28% overlap,
plane residual 6.08 cm, NN residual 10.02 cm). Empty, nonfinite and unmapped
queries were also rejected. Synthetic tests cover single-wall degeneracy,
known-transform recovery, correction limits and runtime rejection.

34 relevant tests passed on laptop and Jetson. The installed ROS node also
passed a Jetson replay in isolated ROS_DOMAIN_ID=43: 12/12 clouds produced
poses with original input stamps; no poses before initialization; a stale
cloud was rejected; loss of input produced a watchdog status. Publisher
inspection confirmed no /tf, /tf_static or /cmd_vel publisher. This replay used
current synthetic stamps and identity odometry, not recorded sensor latency.

The first replay exposed an overly short watchdog assertion window and noisy
SIGINT shutdown; the test now allows the 1 Hz watchdog's full scheduling window
and the node shuts down cleanly. All temporary replay processes were stopped.

Existing Jetson SciPy 1.8.0 warns that installed NumPy 1.26.4 is outside its
supported version range. The numerical tests and replay passed; system Python
was not modified. Resolve/pin that environment before production qualification.

## Reproduce and run

Derived map/fixture NPZ files are ignored by git and kept on both machines.
The raw input capture is tracked. Regenerate the derived files with:

```bash
OPENBLAS_NUM_THREADS=1 python3 src/build_lidar_localization_map.py
OPENBLAS_NUM_THREADS=1 python3 src/bench_lidar_localization.py \
  --map artifacts/localization-2026-09-06/map.npz \
  --capture artifacts/registration-2026-09-05/drive_capture.npz \
  --out artifacts/localization-2026-09-06/benchmark.json
```

On the robot, start the installed diagnostic node explicitly:

```bash
cd /home/jetson/x3_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=42
ros2 launch yahboomcar_bringup lidar_localization_diagnostic.launch.py \
  map_path:=/home/jetson/x3_ws/artifacts/localization-2026-09-06/map.npz
```

The launch constrains BLAS/OMP threads to one. It is NOT included in normal
bringup and is not left running after validation.

Initialize on `/lidar_localizer/initial_pose` with geometry_msgs/PoseStamped
in frame `lidar_map`, specifying a known map-aligned base pose. Use a recent
nonzero timestamp for which odom-to-base TF is available (within 0.5 s).
Listen for `initialized`; a rejected initialization requires correcting the
reported cause. Never substitute an old boot's raw odometry coordinates.
Output topics are `/lidar_localizer/pose` and `/lidar_localizer/status`.
At the normal parked configuration, 3D cloud publishing is off; the node will
report no recent clouds until an authorized capture provides them.

Full numerical results, map manifest, build log and ROS replay report are in
`artifacts/localization-2026-09-06/` on laptop and robot. Deployment backed up
setup.py/package.xml before editing. `artifacts/COLCON_IGNORE` prevents colcon
from treating backup package.xml files as duplicate packages.

## Remaining qualification

1. Additional stop-and-go overlapping captures to connect station 3 and test
   genuinely new viewpoints against a frozen map. Preserve acquisition stamps,
   cloud IDs and same-clock odometry. Use independent position references for
   any absolute accuracy claim.
2. Test initialization error, prolonged rejection/recovery, corridors, repeated
   geometry and dynamic scenes. Establish gates on separate validation data.
3. Measure end-to-end age, queue replacement, CPU and rejection rate with the
   full parked sweep/OctoMap workload before a higher-load rollout.
4. Once the obstacle-feed prerequisites pass, test combined driving/sweeping.
5. Only then design navigation integration: transform ownership, confidence,
   lost-localization behavior and controlled AMCL handover.

## 2026-09-07 update

Temporal consistency and explicit local recovery are now integrated into the
diagnostic node. Single-match publication described above is historical; see
`notes/lidar_temporal_recovery.md` for the current policy, pause API, new-data
results, and remaining qualification limits.
