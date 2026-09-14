# OAK obstacle-feed feasibility — 2026-09-06

Verdict: current OAK path is NOT ready to support driving during wide lidar
sweeps. This is a stationary feasibility measurement, not a safety validation.
No sweep or chassis motion was requested; interlocks were unchanged.

## Current deployed integration

- Normal service arguments: --domain-id 42 --webrtc-camera. No /oak/points or
  /oak/depth topics were present before the probe.
- --oak-cloud is parsed but never consumed in server_x3.py. oakd_cloud.py
  contains projection helpers but no active publisher is instantiated.
- ROS2Bridge subscribes to /scan for its CBF obstacle list, not /oak/points.
  Its freshness stamp is callback time, not acquisition time.
- OakRosPublisher stamps depth at publication. The driver passes getFrame()
  into _process_depth and discards the DepthAI frame timestamp. Therefore
  publication-to-callback latency is NOT camera acquisition age.

## Measurement

Temporarily enabled --oak-ros-publish using a /run systemd override, preserving
normal arguments. Kept motors disabled, ran a 35 s subscriber, then removed
the override and restarted the normal service. No production source changes.

297 frames analyzed, sampled every fourth pixel, transformed using live TF to
base_footprint. TF camera height was 0.210 m. Depth admitted from 0.30 to 4 m;
obstacle height band was 0.12 to 0.40 m. Results and script are under
artifacts/oak-feasibility-2026-09-06 on both machines.

| Measurement | Result |
|---|---|
| Delivered depth rate | 8.72 Hz (publisher configured for 10 Hz) |
| Arrival gap p50 / p95 / max | 102 / 199 / 375 ms |
| Publication-to-callback p50 / p95 / max | 12 / 77 / 117 ms |
| Depth image | 480 wide x 640 high |
| Calibrated horizontal image angle | -21.52 to +21.00 degrees |
| Valid sampled depth fraction, median | 76.7% |
| Forward 5-degree bins containing height-band returns | 9 of 12 |
| Maximum observed chassis command | zero |

The forward bins cover -30 to +30 degrees. A partially occupied bin counts as
occupied, so 75% does not mean 75% of the sector is fully observed. Missing
returns are unknown, not clear space. This one static room is not a moving
obstacle or material/lighting test. Neither refresh tails during a sweep nor
acquisition latency have been established. The existing projection helper's
0.30 m optical-depth cutoff also requires explicit near-field testing.

## Required next work

1. Preserve an acquisition timestamp/sequence through the OAK driver and
   publisher, with an explicit clock mapping. Reject old or repeated frames;
   do not make cached data fresh by re-stamping it at publication.
2. Implement an actual cloud/obstacle publisher and source-aware controller
   fusion. A fresh front camera frame must not refresh stale side/rear lidar
   observations. Empty/invalid regions must remain unknown.
3. Resolve coverage: investigate a wider native stereo depth output separate
   from the spatial detector's aligned image, then measure it. The current
   roughly 42.5-degree image cannot cover the requested 60-degree sector;
   changing projection intrinsics cannot create missing observations.
4. Validate near-field coverage, low obstacles, moving targets, stale-feed
   stopping and then parked-sweep load. Only after those pass consider the
   controlled combined chassis-motion/tilt test.

The current camera is promising as a forward supplement, not an established
replacement for the 360-degree obstacle feed on this mecanum robot.
