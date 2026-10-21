# Robot / laptop sync history


## 2026-10-21
Synced robot to 5bb4866 (RealLab3 map + Save Map fix). Robot copies of server_x3.py, site_maps.json and RealLab3.* were byte-identical to the pull, so no restart. Untracked map copies moved to robot backups/pre-sync-2026-10-21; stash@{0} holds the identical edits and can be dropped.

## 2026-10-21 (evening)
Synced robot to 27dfd3c (Nav2 heartbeat yield default on + DiffDrive MPPI). Robot edits to server_x3.py/nav2_client.py were older versions of the same change; stashed (stash@{0}, droppable) and fast-forwarded. install params copy refreshed from src. No restart: yield set live via set_nav_yield; the running process picks up the new default at its next restart.

## 2026-10-21 (late)
Synced robot to 9d2b15f (scan-match AMCL seeding). Robot copies of server_x3.py, GUI.html and main.js were byte-identical to the pull (stashed, droppable). Server already running the new code; no restart needed.
