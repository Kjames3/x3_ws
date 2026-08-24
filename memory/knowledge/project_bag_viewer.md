---
name: project_bag_viewer
description: "src/bag_viewer.py — GUI/CLI for browsing and scrubbing the robot's rosbags"
metadata: 
  node_type: memory
  type: project
  originSessionId: bafe6300-bfd8-4706-9685-1ed2fb22cb92
  modified: 2026-07-27T04:28:45.536Z
---

`src/bag_viewer.py` (added 2026-07-26, user-requested) browses the rosbag corpus described in [[project_rosbag_corpus]]. It reads bag sqlite directly — no ROS graph, no `ros2 bag play`, no DDS — so it works regardless of the discovery mess in [[project_rviz_debugging]].

- `python3 src/bag_viewer.py` — PyQt5 GUI (bag list, depth/RGB image, laser-scan + odom-trail canvas, topic/sensor table, per-topic message dump at the cursor).
- `--list` / `--info <bag>` — text-only modes for "what was recorded".
- Keys: Space play, ←/→ step one message of the sync topic, Shift/Ctrl ±1s/±10s, n/p bag, i image topic, s sync topic, o trail, d depth range, r rescan, ? help.
- Default scan roots: `~/EE_244_Final_Project/bags`, `~/bags`, `./bags`.
- It never decompresses `.zstd` in place (cache is `~/.cache/x3_bag_viewer/`) and auto-pads truncated `.db3` files.
- `run_gui(roots, sync_topic, _hook)` takes a `_hook(app, win)` seam used for offscreen (`QT_QPA_PLATFORM=offscreen`) smoke tests — drive the `Viewer` methods and `win.grab().save(...)` to verify rendering without a display.
