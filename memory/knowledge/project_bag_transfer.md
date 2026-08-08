---
name: project_bag_transfer
description: "Bag auto-transfer robot↔laptop — push in record_bag.sh, pull via fetch_bag.sh; laptop sshd + non-interactive ROS gotchas"
metadata: 
  node_type: memory
  type: project
  originSessionId: dd9ea643-6012-411f-9303-2f5ec166bbf6
  modified: 2026-07-31T21:56:56.708Z
---

As of 2026-07-31, bags move between the X3 robot and the laptop two ways:

- **Push** — `record_bag.sh` scp's the bag to the SSH client at the end of a
  recording. Detects the target via `$SSH_CONNECTION` → `who` → `DEST_HOST`.
  Auth is **SSH agent forwarding** (connect with `ssh -A`), so no key lives on
  the robot. Knobs: `AUTO_SCP`, `DEST_USER` (default `kamren`), `DEST_HOST`,
  `DEST_DIR`.
- **Pull** — `fetch_bag.sh` runs on the laptop: ssh's in, runs the recorder,
  rsyncs the bag back. Needs no inbound access to the laptop. It sets
  `AUTO_SCP=false` so the push path doesn't double-transfer.

Gotchas that cost real debugging time:

1. **The laptop has no `openssh-server`** (`dpkg` shows `un`). The push
   direction cannot work until it's installed — the robot must connect *to* the
   laptop. Pull needs nothing. Laptop `~/.ssh/authorized_keys` was created
   2026-07-31 from `ssh-add -L`.
2. **`ssh host cmd` gets no ROS.** Ubuntu's `~/.bashrc` returns early for
   non-interactive shells, and the robot's `.bashrc` doesn't source ROS at all,
   so `bash -lc` still yields "ros2 not found". `fetch_bag.sh` sources
   `/opt/ros/humble/setup.bash` and `~/x3_ws/install/setup.bash` explicitly.
3. **Use `ssh -tt`, not `-t`** — without a local tty, `-t` silently skips
   allocation and Ctrl+C never reaches `ros2 bag record`, orphaning it.
4. **`StrictHostKeyChecking=accept-new`** is required on the robot→laptop hop,
   or `BatchMode=yes` fails with "Host key verification failed".
5. **`set -e` + `grep` in a command substitution** killed the script after a
   good recording; the `who` fallback needs `|| true`.
6. `rsync -a` clobbers the executable bit from the local file — re-`chmod +x`
   after syncing to the robot.

Data rate measured 2026-07-31: ~6.7 MB per 15 s with all 8 topics incl.
`/oak/detections` (~28 MB/min), so 1–2 min bags are ~30–60 MB.

Robot IP was 10.13.196.218, laptop 10.13.149.173 (both DHCP, expect drift).
See [[project_robot_deploy]] and [[project_rosbag_corpus]].
