# Lidar-only person classifier: first offline results (2026-11-01)

Goal: let the Livox start a person track by itself (beyond the camera's 4 m, behind and
beside the robot) instead of only where C3 lands on a cluster
(`lidar_person_tracker.py`, [2026-10-30 plan](lidar_person_tracks_2026-10-30.md)).
Offline only; nothing on the robot changed.

## Pieces
| File | What |
|---|---|
| `src/yahboomcar_bringup/yahboomcar_bringup/cluster_features.py` | crop a candidate's points + 24 shape features (sensor geometry is a parameter) |
| `src/tools/perception/lidar_person_dataset.py` | robot bags -> auto-labelled candidates |
| `src/tools/perception/sit_pcd.py`, `sit_person_dataset.py` | SiT clips (PCD/LZF reader) -> box-labelled candidates through the same extractor |
| `src/tools/perception/lidar_person_classifier.py` | gradient boosting, run-held-out scores per range band |
| `tests/test_lidar_person_dataset.py` | features, LZF/PCD, linking, labelling, camera outliers |

Data: `artifacts/lidar-person-classifier-2026-11-01/` (`own.npz`, `sit.npz` are
git-ignored; rebuild with the commands below). SiT clips unpacked (lidar, labels,
poses only) to `~/datasets/sit/`.

```bash
python3 src/tools/perception/lidar_person_dataset.py artifacts/person-lidar-2026-10-30/* \
    --overrides artifacts/lidar-person-classifier-2026-11-01/label_overrides.json -o own.npz
python3 src/tools/perception/sit_person_dataset.py $(ls -d ~/datasets/sit/* | grep -v Courtyard_2) -o sit.npz
python3 src/tools/perception/lidar_person_classifier.py --own own.npz --sit sit.npz
```

## Labels
Robot bags (13 runs + trial, ~14 min, mostly one walker): candidates linked into
tracklets; a tracklet the camera confirmed is a person on EVERY frame ('carried' where
the camera did not see it). Not-person = static, never matched, in clear camera view
('static_neg'), or fixed structure present in >= 50% of a run's frames ('persistent',
weaker). Camera positions from the pre-da7d8ae raw-odom pose bug are dropped as
outliers per camera id. Result: 6.0k person (3.4k carried, ~1.1k beyond 4 m),
4.8k + 33.7k not-person.

SiT (11 labelled clips in v1, 20 with Hallway in v2; Corridor_4/Courtyard_3 have no labels, Courtyard_2 has a
non-flat floor): boxes are WORLD frame -> ego via inv(ego_trajectory) (checked by
points-in-box). Lidar ~0.95 m above the floor vs ours 0.31. 10.4k person, 12.6k
not-person, 5.5k seated people kept apart.

## Results (`results_v1.txt`)
Average precision, held out by run:

| Training -> test | all | 0-2 m | 2-4 m | 4-6 m | 6-8 m |
|---|---|---|---|---|---|
| robot -> robot | 0.967 | 0.99 | 0.95 | 0.97 | 0.93 (<100 people) |
| SiT -> robot (no robot data) | 0.806 | 0.62 | 0.98 | 0.86 | 0.04 |
| robot + SiT -> robot | 0.973 | 0.99 | 0.97 | 0.98 | 0.93 |
| ... tracklet-averaged | 0.982 | 0.98 | 1.00 | 1.00 | 1.00 |
| robot -> SiT | 0.816 | 0.70 | 0.91 | 0.90 | 0.81 (8-12 m 0.71) |

- SiT transfers at 2-4 m with no robot data (0.98) but not near the robot (FOV
  truncation differs) or far out. Adding it to robot data helps a little and makes the
  tracklet score cleaner.
- Robot -> robot is optimistic: one walker, one lab, the same furniture in every run.
- Close-range false positives were almost all ONE static object ~0.69 m right of the
  robot in furniture-r1 / goal-on-r4 (~1.0 m tall, 0.4 m wide). The user confirmed it
  was a PERSON SITTING ON A CHAIR: the model was right, the label was wrong. In
  goal-on-r4 it had even been a camera-checked negative, i.e. C3 had them in view and
  did not detect them seated. Fixed by hand (see below).
- 7% of 'unknown' candidates (2,991 of 41.6k) score >= 0.5: where lidar-only births
  would happen. Not inspected yet.

### With the Hallway clips (`results_v2.txt`, 20 SiT clips, 17.1k person / 25.3k not)
Hallway 2-4, 6-11 added (5 and 12 ship no labels; Hallway_1-009 = Hallway_1).
More SiT does not help the robot: SiT -> robot fell 0.806 -> 0.724 (hallways now half of
SiT, further from our sensor and lab); robot + SiT unchanged at 0.973 per frame,
0.982 -> 0.990 tracklet-averaged; within SiT 0.942 -> 0.953. Data from our own sensor
matters more than more SiT. `sit.npz` in artifacts is now the 20-clip one.

### Seated person relabelled (`results_v3.txt`)
`--overrides artifacts/lidar-person-classifier-2026-11-01/label_overrides.json` marks
the candidates near the seated person as 'sitting' (label 2, left out of training like
SiT's seated people; 905 candidates). Robot + SiT: AP 0.973 -> 0.981, 0-2 m precision
0.72 -> 0.93, tracklet-averaged 0.990 -> 0.994, camera-checked negatives 0.999.
Lesson: the 'persistent' rule, and even the camera check, cannot tell a person sitting
still from furniture; other seated people may still be hiding in the persistent set.
Decided (user, 2026-11-01): a seated person counts as a person (can stand up into the path).

### Seated = person, Lobby clips added (`results_v4.txt`, 27 SiT clips)
Lobby 2-8 added (1 and 9 ship no labels): SiT is 20.5k person / 37.3k not-person.
The robot's seated person now counts as a person. SiT's 7.1k seated pedestrians do NOT:
counted as people they took SiT -> robot AP from 0.81 to 0.39 and robot + SiT 4-6 m
precision from 0.91 to 0.65 (a SiT seated pedestrian is usually one cluster with the
bench or table, which teaches 'furniture = person'). Isolated by running each change
alone: Lobby alone took SiT -> robot 0.76 -> 0.81. Final: robot only 0.975
(tracklet 0.995), robot + SiT 0.977 (tracklet 0.993), SiT -> robot 0.776.
`--sit-sitting-as-person` / `--exclude-sitting` switch the two rules.

## The extractor, not the classifier, limits crowds
SiT recall of `person_candidates.extract` for labelled standing pedestrians with >= 10
points: 75% when isolated (> 1.5 m from anyone), 53% with someone 0.6-1.0 m away, 41%
within 0.6 m. Two people that close share an 18 cm grid component (> 0.9 m span) and
the > 1.1 m split separates them only sometimes. (SiT's higher sensor sees more upper
body, so the robot's number may differ.) A crowd fix has to start in the extractor.

## Next
1. Resolve the 0.69 m object; inspect high-scoring 'unknown' candidates.
2. Recording session: several people, 4-10 m, behind the robot, a real crowd.
3. Extractor: split merged people (e.g. per-height-slice clustering or a leg/torso
   blob split) and re-measure recall on SiT.
4. Only then: lidar-only births in the tracker (lower confidence class), shadow mode.
5. JRDB (access requested 2026-11-01) as a second, robot-height dataset.
