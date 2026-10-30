# Surroundings display memory and coverage

The Drive view uses display-only geometry. Navigation and the CBF do not consume
this memory, and clearance measurements always use the fresh cloud.

- Environment fragments between the chase camera and the robot fade to 10%
  opacity. This applies to points, voxels and the 2D wall fallback.
- `livox_scan_node` keeps two seconds of stamped `/odom` (`odom` ->
  `base_footprint`) and interpolates planar poses at per-point acquisition times.
  Extrapolation is limited to 80 ms. It publishes the de-skewed cloud in
  `base_footprint`, paired with `/surroundings/pose` (`PoseStamped` in `odom`)
  at the frame-end timestamp. Missing/out-of-sync poses leave live rendering
  available but disable accumulation. Display errors cannot stop `/scan`.
- The browser accumulates 10 cm cells in odom, up to 12,000 cells / 8 seconds /
  7 m. Three observations promote a tentative cell to solid; tentative cells
  expire after 350 ms without another observation. Old solid cells become
  muted and fade out. Tracked people/movers use current observations only.
- Up to 512 measured 3D rays per frame test stored cells. Three contradictory
  frames clear a cell; missing returns do not clear it. A 20 cm endpoint buffer
  limits clearing due to surface sampling error. This is a display heuristic,
  not an occupancy map suitable for control.
- Large odom jumps, timestamp reversals, gaps over 2 seconds, or the Clear memory
  button reset accumulation. Map localization does not shift the odom-local grid.
- Coverage tiles mark only actual near-floor endpoints (-5 to +6 cm), on a
  25 cm grid. Fresh floor is blue, old floor is muted, unknown floor is gray.
  A ray passing above a tile never proves its floor clear. These observations
  do not prove traversability; the Livox near-floor blind region remains unknown.

Controls: Stabilize static surroundings, Observation coverage, Clear memory.
Raw points remain a view of the current cloud. Ceiling filtering affects
rendering, not the fresh-cloud clearance calculation.

Validation:

```bash
node tests/web/surroundings_geometry_check.cjs
node tests/web/surroundings_memory_check.cjs
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pytest tests/test_surroundings_cloud.py -q
```

Synthetic translation/turn tests verify stable world cells, pose interpolation,
per-point de-skew, CDR round trips and synchronized pose stamps. Browser checks
exercise both modes, missing poses, reset, shader compilation and stale hiding.
A user-driven turn remains the practical visual check for odometry/calibration
errors. Odometry drift and untracked moving objects can still affect the display.
