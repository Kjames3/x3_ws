# Live trial: seg (merged) vs pose on the robot

**Why:** offline, merged seg ties pose while the person is fully in view and is
clearly better when they are cut by the image edge (wrong-depth track in
0 / 0 / 16 % of edge frames at 1.2 / 2.4 / 3.5 m vs 0 / 8 / 46 % for pose;
`artifacts/c3-eval-edge/`). Seg has never run live. This trial checks the
things recordings cannot: packet rate, CPU load, stalls and live coverage.

**Ready:** the merge is in the OAK driver (`7748573`) and gives the same boxes
as the offline scorer on all 6,535 recorded frames. The robot has the code but
`x3_server` has NOT been restarted, so it still runs the old driver on pose.

## Steps (about 30 min)

Robot parked, clear floor between it and you, tape marks from the scoring runs.

1. **Pose baseline** (robot as it is). On the laptop:
   `python3 scripts/c3_live_window.py --label pose-walk`
   Walk 1.5-3.5 m in view for the 60 s. Then
   `python3 scripts/c3_live_window.py --label pose-cross`
   and cross the view at the 2.4 m line, out of view each side, about 6 passes.
2. **Switch to seg** (on the robot; restarts `x3_server`, ~30 s):
   `bash scripts/pose_test.sh yolo26n-seg`
   Stand in view until it prints the decode mode; the layout locks on a person.
3. **Seg windows**, same two patterns:
   `python3 scripts/c3_live_window.py --label seg-walk`
   `python3 scripts/c3_live_window.py --label seg-cross`
4. **Close range:** `python3 scripts/c3_live_window.py --label seg-close --seconds 30`
   standing 0.8-1.2 m away. This is where seg split one person into several
   boxes; expect one person box per sample and some `merged` samples.
5. **Switch back** if anything looks wrong: `bash scripts/pose_test.sh yolo26n-pose`.

## Pass criteria for seg

| Check | Pose, 2026-10-01 | Seg must |
|---|---|---|
| Detection packets per second | 5.6 | be >= 4.5 |
| Ring when a person is in the packet (walk) | 96 % | be >= 90 % |
| Tracker resets | 0 | be 0 |
| Person boxes per sample, one person in view | 1 | be 1 (merge working) |
| CPU total / server process | measure in step 1 | not exceed pose by > 15 points |
| NN stall / pipeline rebuild in `journalctl -u x3_server` | none | none |

Crossing windows are compared pose vs seg by eye in the GUI (ring appears as
you enter, stays on you, no ring left behind at the wrong distance) and by ring
coverage; there is no live ground truth for depth at the edges.

## Known side effects of running seg

- The GUI avatars lose arm posing (no keypoints from a seg model).
- `--oak-det-log` keeps writing, now with seg detections.
