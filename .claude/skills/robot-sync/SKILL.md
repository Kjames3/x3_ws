---
name: robot-sync
description: Find and reconcile differences between the robot's checkout and the laptop's. Use when the user says the git is dirty or inconsistent, asks what differs between robot and laptop, before deploying if the robot has local edits, or when a fix "works on the laptop but not on the robot".
---

# Reconciling robot and laptop

The two checkouts drift again and again. The usual cause is code copied to the
robot before it was committed on the laptop, so the robot shows the laptop's
commits as uncommitted edits. Direction is almost always laptop -> robot, but
verify per file: neither side is uniformly newer.

## 1. Report, do not copy

```bash
bash scripts/diff_robot.sh            # summary: HEADs, robot-only, laptop-only, differing
bash scripts/diff_robot.sh <path>     # unified diff of one file, robot -> laptop
```

Also: `git status --short` on both sides, `git log origin/main..HEAD` on the
laptop (unpushed), and `ssh x3 'cd ~/x3_ws && git log -1 --oneline'`.

Present the result as three lists: only on the robot, only on the laptop,
differing. For each differing file say which side is newer and why you think
so. Get the user's go-ahead before changing anything.

## 2. Safe sync

1. Rescue robot-only work first: copy it to the laptop and commit it there.
   Robot-only edits are usually older versions, but check before discarding.
2. Commit and push on the laptop in logical commits.
3. On the robot, confirm every locally modified file is byte-identical to, or
   older than, what the pull will bring.
4. `git stash` on the robot as a safety net.
5. Move untracked copies that the pull will supply into
   `backups/pre-sync-<date>/`, otherwise the pull refuses.
6. `git pull --ff-only`.
7. Restart only if file contents actually changed.
8. Add a line to `notes/robot-laptop-sync-history.md` and update the
   `project_robot_laptop_divergence` memory with the new synced commit.

## Never

- `git reset --hard` on the robot. `install/` is untracked and holds
  per-machine paths; a hard reset deletes `install/setup.bash`. Recovery is a
  full `colcon build`.
- `--symlink-install`. Mixing it in breaks `yahboomcar_description`. Recovery:
  `rm -rf build install log`, stop `x3_server`, clean `colcon build`.
- `sync_code` with `delete=True`. The robot has maps, logs, weights and
  captures that exist nowhere else.

## Expected to differ

- The `ydlidar_ros2_driver` submodule edit (` m` on both machines).
- `artifacts/`, laptop `images/` and `models/yolo11x.pt`, robot `backups/`.
- `install/`, `build/`, `log/`.

## A test that expects code which is not there

`tests/test_rosmaster_parser_equivalence.py` skips because the patched
`Rosmaster_Lib.py` is only on `origin/worktree-rosmaster-lib-fixes`. When a
test and its code disagree, look for an unmerged branch before assuming the
robot has it.
