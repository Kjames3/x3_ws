# Out-of-tree patches

## `ydlidar_timed_points.patch`

Adds a `/lidar/points_timed` publisher to the YDLidar driver: the same
PointCloud the driver already builds, plus an `acquisition_time` channel
(per-return offset from the scan start) and a `scan_duration` channel.

**Why it lives here instead of in the submodule.** `src/ydlidar_ros2_driver` is
a submodule pointing at YDLIDAR's own repo, which we cannot push to. A local
edit there is invisible to this repo: git records only
`4ef70d3-dirty`, so a clean clone silently gets the *unpatched* driver, no
`/lidar/points_timed` topic, and a `lidar_3d_processor_node` that withholds
every cloud with "missing or invalid acquisition times". The deskew looks
broken and the cause is not in this repository. Keeping the patch tracked here
is what makes that recoverable.

**Apply:**

```bash
cd src/ydlidar_ros2_driver
git apply ../../patches/ydlidar_timed_points.patch
cd ../.. && colcon build --packages-select ydlidar_ros2_driver
```

**Verify** (the topic must exist and carry both channels):

```bash
ros2 topic echo /lidar/points_timed --field channels --once
```

**What it deliberately refuses to do.** The timed cloud is published *only*
when the SDK preserved every raw slot, because the whole point is that a
return's index maps to its acquisition time. `fixed_resolution:=true` pads the
array, and a cropped `angle_min`/`angle_max` drops slots; either destroys that
mapping. Rather than publish a plausible-looking cloud with silently wrong
per-ray times, the patch withholds it and warns. If `/lidar/points_timed` is
missing, check those driver params before suspecting the patch.

Long-term fix is to fork the driver and repoint the submodule; until then,
re-run `git apply` after any submodule update.
