# src/

Top-level `.py` files here are the **runtime**: `server_x3.py` and the modules it
imports, plus the boot-time `dynamixel_tilt.py`. systemd runs them from this
directory (`deploy/systemd/x3_server.service`), so don't move them without
updating the unit files and reinstalling them on the robot.

Everything you run by hand lives under `tools/`, grouped by purpose:

| Directory | What's in it |
|---|---|
| `tools/calibration/` | IMU, lidar yaw, footprint, ToF floor baseline, Dynamixel commissioning, web robot model bake |
| `tools/hw_tests/` | Drive and sensor tests on the robot (IMU, odometry, strafe, point-to-point, sweep bench, old ToF bench) |
| `tools/perception/` | Offline person/velocity work: C1 dataset, C3 tracker replay/score, velocity model scoring, A/B logs |
| `tools/lidar/` | Lidar localization capture, replay and validation |
| `tools/nav/` | Nav/CBF analysis: ORCA vs CBF, STL monitor, map-diff replay |
| `tools/viewers/` | Bag, octomap, OAK, ToF viewers and the mock telemetry server |

Tools import runtime modules (and siblings in their own group) by bare name.
Each one puts `src/` on `sys.path` in its header; `tests/conftest.py` adds
`src/` and every `tools/<group>/`.

The other directories:

- ROS 2 packages: `yahboomcar_*`, `ydlidar_ros2_driver` (submodule), `x3_msgs`, `nav2_mppi_dbas_controller`
- `web/`: the browser GUI
- Model files: `models/`, `checkpoints/`, `yolo_models/`, `blobs/`, and the `velocity_mlp*.torchscript` / `scaler_*` files next to the server (loaded by relative path)
- `logs/`, `fixtures/`: A/B test CSVs and test fixtures
- `archive/`: dead code and old `server_x3.py` backups
- `lidar_tilt.py`, `lx16a_servo.py`: the retired LX-16A tilt. `lidar_tilt_node` still looks for `lx16a_servo.py` here.
