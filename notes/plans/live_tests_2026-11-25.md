# Live tests and recordings, next lab / courtyard session (planned 2026-11-24)

Needs open space (the apartment is too small).
Recorder: `ssh jetson@orin`, then `cd ~/x3_ws && bash scripts/record_person_lidar.sh <name> <s>`.
The robot stays parked unless a step says otherwise.
Check the camera-frame screws first.

## A. Tracker fixes, CBF feed OFF (`cat /tmp/x3_lidar_persons_cbf` should be absent or 0)

| name | s | what | checks |
|---|---|---|---|
| `hold45-r1` | 90 | stand ~4.5 m away for 20 s, walk out to ~6 m, come back; repeat on another side | 30 s hold (bbdc01e): one id throughout |
| `circle-r1` | 120 | circle at ~1.5 m, twice each way | no extra short ids near the front camera (1a33b96) |
| `swap-r1` | 90 | needs 2 people: A walks away, B steps into A's spot within a few seconds | do the ids swap? (risk of the 30 s hold) |
| `empty-r1` | 60 | nobody near | still 0 false tracks |

## B. CBF feed ON (`echo 1 > /tmp/x3_lidar_persons_cbf`; `echo 0` turns it off, no restart)

Feed lidar person tracks to the CBF. Simulated 2026-11-23 (`src/tools/nav/lidar_cbf_sim.py`):
the time spent closer than 0.5 m went from 12.9 to 5.8 s, mostly from side and rear approaches.

| name | s | what | checks |
|---|---|---|---|
| `cbf-side-r1` | 60 | parked: walk up slowly from the left, then from the right | robot backs off from ~1.2 m |
| `cbf-rear-r1` | 60 | parked: walk up from behind | same, rear camera / lidar only |
| `cbf-drive-r1` | 90 | drive slowly (joystick, <= 0.2 m/s) toward someone standing to the side / behind | slows earlier than with the feed off |
| `cbf-empty-r1` | 60 | nobody near, parked | robot does not move |

Watch for the camera + lidar double count (the same person counted once by each, 0.59 m apart,
outside the 0.5 m dedupe): if a walk-up pushes the robot unusually hard, note the time.

## D. Crowd re-test (needs several people)

Head-peak split + track seeds (5e51134) are live but untested with a crowd. Repeat the
2026-11-24 crowd runs: `crowd-r5`.. 120 s each, 3+ people standing close, some against the
desks, from 2-3 robot positions. Expect fewer lidar losses than crowd-r1..r4 (offline r3
17 -> 3). Also: one person leaves from beside the desks; their track should end within
30 s and not stay on the furniture.

## C. Rear OAK Livox calibration

The rear pitch/yaw in the URDF is an interim correction from the 2026-11-24 crowd runs
(fac31f6: -5.84 deg nose up, +1 deg yaw, from person feet vs Livox clusters). Do a proper
camera-to-Livox solve for the rear OAK, like the front/right/left, and replace it.

## Already done

- Lab crowd runs crowd-r1..r4 (2026-11-24); rear OAK pitch corrected from them (fac31f6).

- Left camera, new lens: recalibrated (800b510) and alignment checked (`left-check-r1`, 2026-11-24).
