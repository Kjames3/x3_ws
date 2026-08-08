---
name: project_rosbag_corpus
description: State of the EE_244_Final_Project rosbag corpus + how to read/repair these zstd bags
metadata: 
  node_type: memory
  type: project
  originSessionId: bafe6300-bfd8-4706-9685-1ed2fb22cb92
  modified: 2026-07-27T04:28:38.538Z
---

The physical-robot rosbags live in `/home/kamren/EE_244_Final_Project/bags/` (NOT `~/bags`, which is empty), 87 dirs / ~7.4 GB as of 2026-07-26. **42 of them contain zero messages** (recording aborted before any data) — a real bag is ~300 msgs/7 s up to 11.5k msgs/196 s. `bags/` itself is also a stray bag dir (empty). `domain_adapt_20260528_185711 ` (trailing space in the name) is all zero-byte files, unrecoverable. Recorded topics are `/camera/depth/image_raw` (Orbbec Astra 16UC1 mm, ~640x480 @ 5-10 Hz), `/scan`, `/odom`, `/tf`, `/tf_static` — no `/oak/*` in this corpus despite what `record_bag.sh` lists.

Reading them: `rosbag2_py.SequentialReader` FAILS on these (compression-mode FILE); you need `rosbag2_py.SequentialCompressionReader`, and that **decompresses the .db3.zstd in place, next to the original**. Prefer reading the `.db3` with plain `sqlite3` instead — schema is `topics(id,name,type,...)` + `messages(id,topic_id,timestamp,data BLOB)` with a `timestamp_idx`, so a full per-topic timestamp index is a single cheap query and random-access scrubbing is trivial. Deserialize with `rclpy.serialization.deserialize_message` + `rosidl_runtime_py.utilities.get_message` (no `rclpy.init()` needed).

Several `.db3.zstd` archives are TRUNCATED ("premature end" from `zstd -t`) — the transfer from the Jetson was cut short — so the decompressed `.db3` cannot be regenerated. When a `.db3` is short, sqlite reports "database disk image is malformed"; **the fix is to zero-pad the file to `page_size * page_count` read from the sqlite header (bytes 16:18 and 28:32, big-endian)**, which restored 6328/6329 messages on `domain_adapt_20260529_195105`. Padding only appends, so it is non-destructive. `sqlite3` CLI is NOT installed on this laptop (no `.recover`); python's `sqlite3` module works fine.

Viewer: [[project_bag_viewer]] — `src/bag_viewer.py`.
