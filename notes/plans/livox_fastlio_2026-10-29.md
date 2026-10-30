# Livox Mid-360S: independent FAST-LIO comparison

The first integration runs alongside the existing wheel/ICM EKF. It publishes
`/fastlio/odom`, `/fastlio/cloud_world`, and `/fastlio/cloud_body`; its TF is
isolated on `/fastlio/tf`. It does not feed Nav2, the CBF, or the person tracker.
URDF changes are reserved for the user.

Validated on the mounted sensor on 2026-10-29: Release build completed in
3 min 6 s. The corrected 20-second no-command smoke capture received FAST-LIO
at 10.00 Hz, EKF at 50.01 Hz, wheel odometry at 10.00 Hz, and 3,805 IMU samples.
FAST-LIO timestamp age was 66 ms median / 78 ms p95; maximum position excursion
was 3.2 mm and yaw excursion 0.033 degrees. EKF translation stayed zero and yaw
excursion was 0.035 degrees. These are stationary stability observations, not
a moving accuracy comparison. FAST-LIO used about 57% of one CPU core and
167 MB RSS in a process snapshot; the driver used about 26% / 43 MB.

Verified FAST-LIO only publishes its isolated TF topic. A separate 11-second
bag trial contains 111 raw clouds, 2,214 Livox IMUs, 112 LIO poses, 532 EKF
poses and 108 wheel poses. The original Python monitor deserialized all points
and starved its own callbacks; its rate/latency results are invalid. The fixed
monitor uses raw serialized cloud callbacks and decodes one example only.
Local evidence: `artifacts/fastlio-2026-10-29/`. Moving comparison and person
tracking fusion remain pending; no accuracy improvement is claimed yet.
The bounded smoke launch has been stopped and no diagnostic processes remain.
On shutdown the Livox driver logged successful SDK deinitialization followed
by exit -11 (segmentation fault); runtime data collection succeeded. This
upstream shutdown issue needs investigation before unattended service use.

Source: https://github.com/Ericsii/FAST_LIO (ros2), commit
`2fffc570a25d0df172720bac034fbdb6a13d2162`, with its pinned ikd-Tree submodule.
The Jetson uses SDK2 `c0796f04c143143899c87a773d9f6b7136453c0b` and
livox_ros_driver2 `21445540f0d100dc86a7e6df312dd70bbdb4afdf` (supports type 35).
Build with `bash scripts/build_fastlio_shadow.sh`; dependencies were already
available in the Jetson ROS Humble installation. Separate workspace: ~/ws_fastlio.

## Calibration and implementation constraints

- User reports the same horizontal position as the old lidar and **0.31 m
  above the floor**. This is floor height, not base_link-relative z. With the
  current base_link height 0.0815 m, a level-frame origin would be z=0.2285 m.
  The reference point of the height and forward-arrow orientation remain to
  be checked. Do not copy the old YDLidar yaw (pi + 4.85 degrees) to the Livox.
- The standard Mid-360 internal LiDAR-to-IMU translation in the config is
  **provisional for the Mid-360S**. It is separate from the robot mounting pose.
  Online extrinsic estimation is disabled: planar robot motion is a poor way
  to identify all six extrinsic degrees of freedom.
- Use CustomMsg, which preserves per-return offset_time for deskew. The
  existing PointCloud2-only bench tool cannot subscribe to this topic type.
- Driver accelerations are in g. FAST-LIO's IMU_Processing.hpp normalizes by
  measured gravity internally. A future robot_localization input would instead
  need SI units and consistent covariance; never feed this raw IMU directly.
- Upstream /Odometry is an IMU-body pose in camera_init, not a base_link pose
  in odom. Its twist is not populated. Its pose covariance is filled after
  publication (one sample late). Do not blindly fuse its twist or covariance.
- FAST-LIO provides local odometry and mapping, not loop closure or persistent
  map relocalization. Closed-loop displacement is a test metric, not proof of
  absolute accuracy. Dynamic people can contaminate the map.

## Run and compare

On the robot, start a foreground launch and wait for initialization:

```bash
cd ~/x3_ws
bash scripts/fastlio_shadow.sh
```

In a second robot terminal:

```bash
cd ~/x3_ws
bash scripts/record_fastlio_comparison.sh 120
```

The capture prints `CAPTURE START` and its destination. Remain still for the
first 10 seconds, drive a slow closed rectangle with forward and sideways
legs, include a turn, return to the marked start pose, and remain still for
the final 10 seconds. User starts/drives the experiment; the agent does not
issue motion commands. Ctrl-C in the launch terminal stops both diagnostic
nodes. No systemd service is installed.

Metrics contain per-stream rates, gaps, timestamp age, excursion and endpoint
displacement. Missing streams fail the capture rather than appearing as zero
drift. Bags preserve raw Livox + IMU for replay. Inspect bag payload counts
before treating a longer run as valid. Point clouds are intentionally omitted
from the bag because they can be regenerated from the raw CustomMsg stream.

For a quantitative moving comparison, transform the LIO IMU pose to the robot
reference point using measured extrinsics, interpolate both trajectories to
common timestamps, then align their initial poses (no scale fit). Report
relative translation/yaw disagreement versus distance, stationary jitter,
loop endpoint error against physical start markers, dropouts, latency and CPU.
Neither estimator is ground truth. Use mocap when available to establish
which estimator is actually more accurate.

## Improve person tracking with the continuous cloud

The current C3Live code already places camera measurements using capture-time
odometry history; improving that pose reduces false person motion during robot
turns. Keep the current estimator as an A/B reference until moving validation.

1. Deskew the Livox cloud, remove the robot body and floor in a calibrated
   frame, and cluster the remaining returns. Preserve timestamps, point counts,
   dimensions and range-dependent uncertainty. Do not classify every vertical
   cluster as a person: chairs, doorframes and static objects are confounders.
2. Associate camera-confirmed people with LiDAR clusters at the same timestamp.
   Use covariance-gated association and track history to preserve identity as
   people leave camera view. LiDAR-only unmatched clusters remain unclassified
   obstacles until supported by a validated 3D classifier or other evidence.
3. Update person position/velocity from LiDAR while visible to it, retaining
   camera semantics with a finite expiry. Explicitly distinguish measured,
   predicted/occluded and lost tracks; publish measurement age and covariance.
   Continuous scanning is not continuous visibility through occlusion.
4. Keep stationary people: a static-map subtraction or motion-only detector
   alone would erase them. Use independent semantic evidence and conservative
   background-map updates. Avoid circularly masking all unexpected returns out
   of odometry before their dynamic status is established.
5. Replay side-by-side: current estimator, current estimator with LIO ego-pose,
   and camera/LiDAR association. Test near-field, behind robot, crossing people,
   standing people, robot turns and occlusions. Score time with a measured
   estimate, longest gap, ID switches, false tracks, position/velocity error,
   latency and CPU. Only then connect a qualified result to the CBF/Nav2.

Near-field coverage must be measured on this mounting: the bench vertical FOV
was approximately -7.5 to +53 degrees, so 360-degree azimuth does not mean full
floor coverage close to the bumper. Retain the ToF low-obstacle sensing.
