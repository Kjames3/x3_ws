# Robot / laptop sync history


## 2026-10-21
Synced robot to 5bb4866 (RealLab3 map + Save Map fix). Robot copies of server_x3.py, site_maps.json and RealLab3.* were byte-identical to the pull, so no restart. Untracked map copies moved to robot backups/pre-sync-2026-10-21; stash@{0} holds the identical edits and can be dropped.

## 2026-10-21 (evening)
Synced robot to 27dfd3c (Nav2 heartbeat yield default on + DiffDrive MPPI). Robot edits to server_x3.py/nav2_client.py were older versions of the same change; stashed (stash@{0}, droppable) and fast-forwarded. install params copy refreshed from src. No restart: yield set live via set_nav_yield; the running process picks up the new default at its next restart.

## 2026-10-21 (late)
Synced robot to 9d2b15f (scan-match AMCL seeding). Robot copies of server_x3.py, GUI.html and main.js were byte-identical to the pull (stashed, droppable). Server already running the new code; no restart needed.

## 2026-10-22
Synced robot to the RotationShim commit (nav2 params; installed copy rebuilt). Earlier the same night: 4f7e1d1 (best-effort driver), dac2c98 (driver counters), 9d2b15f (auto-seed).
- 2026-11-18: AGX (jetson@orin, now the robot's compute) ff-pulled c681992 -> e6cbe53 -> a76df68 (Design_v2 URDF); clean tree, yahboomcar_{description,bringup,nav} rebuilt, x3_server restarted. Laptop keeps uncommitted WIP (OAK surroundings, GUI, FastDDS/HTTP hunks in server_x3.py). diff_robot.sh needs X3_HOST=jetson@orin.
- 2026-11-20: AGX ff-pulled 74b2733 -> 9304d92 (calibration tools/results, heading hold, wood strafe 0.98, Arducam URDF frame). Every robot edit was a byte-identical scp copy of the commits; untracked copies moved to backups/pre-sync-2026-11-20, tracked ones in stash@{0} (droppable). No restart (contents unchanged). Still differing on purpose: scene-view.js + web/models/x3_v2_robot.glb (user WIP, identical on both), robot-only yolo26n.engine (rebuilt for TRT 10.3; old one kept as .orin-nano-bak). Server args now include --floor apartment_wood (override.conf).
