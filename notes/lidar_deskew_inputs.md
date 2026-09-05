# Deskew input audit — 2026-09-04

Robot inspected over SSH; no hardware commands or deployment performed.

## Servo timestamps

Initial inspection found direction -1 and horizontal zero 2032. Hardware
validation below overturned the -1 sign; the corrected value is +1.
The sampler previously calculated a monotonic read midpoint but stamped both
JointState messages at publication. The local fix anchors ROS time before the
position read and adds half its monotonic duration. The optional moving-state
read and later scheduling delays no longer shift the published measurement.
This remains an estimated measurement instant, not a hardware timestamp.

`tests/test_tilt_sampler_timestamps.py` exercises the actual sampler function
with an 8 ms position read and both zero and 80 ms moving-read delays.
Both topics must carry the 4 ms midpoint in each case.

## Beam ordering blocks naive LaserScan deskew

Verified in local and robot source:

- `YDLidar-SDK/src/CYdLidar.cpp` assigns scan.stamp from the first raw node.
  It transforms angles (including inversion and wrapping), then appends points
  in acquisition order. It can omit points outside the configured angular span.
- `src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp` stores each return
  at `ceil((angle-min_angle)/angle_increment)`, not its acquisition index.
- Consequently `scan.header.stamp + ranges_index*time_increment` is not a
  justified measurement time, despite the scan-start timestamp being valid.
- The existing legacy PointCloud preserves point order and a `stamps` channel,
  but its index-based offsets need checking against SDK filtering and the live
  angular configuration before using it as an authoritative timing source.

Implemented locally: the driver now publishes `/lidar/points_timed`, an ordered
PointCloud with `acquisition_time` and `scan_duration` channels. It requires a
full angular span, fixed resolution disabled, positive finite SDK timing, and
point count matching scan_time/time_increment. Thus SDK cropping/padding cannot
silently invalidate the acquisition index. Range rejection happens after the
index is converted to an offset, preserving holes rather than compressing time.
These are uniform SDK-derived timing estimates, not hardware beam timestamps.

The processor consumes this topic for 3D output; `/scan_raw` still drives the
existing 2D gate. It buffers two seconds of stamped tilt samples and up to eight
clouds, waiting at most 0.3 seconds for data/TF. It rejects tilt gaps above 0.12
seconds, nonmonotonic history, stale cloud timestamps, invalid timing, missing
TF, or any unsettled sample bracketing a scan with require_settled enabled.
Publication/drop counters and the last rejection reason appear in diagnostics.

Geometry uses per-return Y-axis pitch interpolation and interpolated
odom->lidar_mount_link translation/quaternion at scan endpoints. Pivot
translation and the fixed laser transform come from TF. The current X3 joint
origin rotation is identity and its axis is +Y; another mount needs adapting.
Output is transformed back into the scan-start laser TF frame so OctoMap
retains the physical sensor origin. Base motion between endpoints is modeled
as linear translation and shortest-arc SLERP; acceleration within a revolution
is not separately estimated.

Also account for the full URDF chain: tilt pivot translation, Y-axis pitch,
22 mm laser offset and 180 degree laser yaw. For octomap, preserve a physical
sensor origin: simply publishing points in odom makes the cloud-frame origin
the odom origin for raycasting. A scan-start sensor reference frame, or an
explicitly supported sensor-origin path, is needed. One origin per scan is an
approximation once the robot or sensor origin moves during acquisition.

Earlier 13.5 cm figures describe angular displacement across a sample gap,
not measured interpolation residuals. Old Dynamixel 3D maps with the reversed
tilt sign cannot serve as unquestioned geometry references.

## Validation and rollout

18 tests pass using sourced ROS Humble and `/usr/bin/python3 -m pytest -q
tests/test_lidar_deskew.py tests/test_lidar_deskew_ros.py
tests/test_tilt_sampler_timestamps.py`. They cover static geometry, moving-wall
reconstruction, antipodal quaternions, physical reference origin, actual ROS
PointCloud2/TF integration, missing data/TF rejection and sampler timestamps.
Both ydlidar_ros2_driver and yahboomcar_bringup build in isolated /tmp build and
install directories. Driver warnings are existing unused callback arguments.

Deployed and hardware-tested on 2026-09-04. Deploy the driver, processor, parameters,
and sampler timestamp correction together; the old driver does not publish the
new timed topic. The driver is a git submodule: its source edit must be included
when committing/deploying, not just the parent repository files.

For subsequent robot validation, use a parked step-and-stare capture: compare clouds
against measured floor/wall geometry, inspect timed-topic and published/drop
rates, and measure callback cost. Preserve require_settled and the four-arm
interlock. Continuous motion, range-cap changes, and virtual `/scan` synthesis
are subsequent work.
