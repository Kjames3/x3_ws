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

## Synthetic people in real frames (2026-11-02, `synthetic_v1.txt`)
`src/tools/perception/synthetic_people.py` (tests: `tests/test_synthetic_people.py`).
Only the people are simulated: backgrounds are recorded frames from the 14 runs, and the
rays are each frame's own Livox rays (direction + capture time), so the scan pattern,
clutter and floor are real. Bodies are ~15 oriented ellipsoids (walking gait or standing,
height 1.5-1.95 m, random girth/shoulders, 1 in 4 with a backpack), posed at each point's
capture time (motion smear), with range noise and edge tails. Placement needs free floor
and a line of sight (the lab is small: only ~10% of directions are open past 6 m, so
bearings are drawn from rays that reach past the chosen range). No licence-bound assets
(SMPL could replace the ellipsoids later). 4000 scenes / 6000 people run in ~13 s.

**Checked against real people in the same frames** (synthetic person at the real one's
range, other side): point count 647 vs 681, upper-body width 0.47 vs 0.49, legs while
crossing 0.38 vs 0.42, along-ray tail shares within ~3 points (after fitting: noise
0.008 + 0.0012 r, 12% of hits pushed back ~5 cm). The first body was 25% too narrow
and strode too long; both fixed. Standing upper-body depth is still ~0.05 m shallower
than real. Real camera-confirmed people are mostly face-on (walkup/idle), so compare by
orientation, not pooled.

**Extractor recall on our sensor (0.31 m), found / scored:**

| nearest other person | 0-2 m | 2-4 m | 4-6 m | 6-8 m | all |
|---|---|---|---|---|---|
| < 0.6 m | 34% | 30% | 36% | 41% | 34% |
| 0.6-1.0 m | 70% | 76% | 76% | 70% | 74% |
| 1.0-1.5 m | 100% | 98% | 94% | 92% | 96% |
| > 1.5 m (alone) | 100% | 99% | 93% | 86% | 96% |

- Isolated people: ~99% when the same person is also found alone in that frame; the
  few losses at 4-8 m are clutter/occlusion, not crowding. 8-12 m is too rarely visible
  in this lab to score.
- The SiT crowd finding holds on our sensor: 0.6-1.0 m apart loses a quarter, < 0.6 m
  two thirds. At 0.6-1.0 m the losses are 'missed' (no candidate at all), not merged:
  the pair is one component wider than 0.9 m and the split drops it. Under 0.6 m it is
  both (a third of losses merged into one candidate covering both).
- Single-knob fixes do not solve it (same 4000 scenes): isolation ring off 74->78% /
  34->41% but +13% clutter candidates on bare frames; split grid 18->9 cm 74->80% /
  34->35%, +22% clutter. A crowd fix needs a different split (e.g. head/shoulder
  peaks in a height map), measured with `score(..., extract=new_extract)` on the same
  `--seed`.

## Extractor crowd fix: head/shoulder peak split (2026-11-02, `extractor_peaks_v1.txt`)
`person_candidates.extract(..., peaks=True)`, **off by default**; nothing live changed.
For components 0.55-2.5 m wide: an 8 cm height map, local maxima above 1.2 m (lower near
the robot where the FOV top cuts people), and a peak is another person only if the
height dips >= 15 cm on the line to every accepted peak within 1.2 m. Each peak takes
the points within 0.45 m. A piece is kept if person-sized, compact (85% of its upper
points within 0.3 m of the peak), top between 1.2 m (or the FOV top) and 2.1 m, n r^2 >=
300, and isolated counting only points no other piece took. The usual split's pieces
always stay; peak pieces are added beside them.

Compared with `src/tools/perception/extractor_compare.py` (same synthetic seed, real
frames, SiT, timing):

| | base | peaks |
|---|---|---|
| synthetic, nearest other < 0.6 m | 34% | 56% |
| synthetic, 0.6-1.0 m | 74% | 83% |
| synthetic, alone / >= 6 m | 96% / 91% | 96% / 92% |
| SiT, < 0.6 m / 0.6-1.0 m | 36% / 51% | 45% / 60% |
| SiT alone | 77% | 78% |
| real labelled people still found | 100% | 100% |
| clutter candidates per real frame | 10.15 | 10.34 (+2%) |
| duplicates per person (synthetic / SiT) | 0 / 0.001 | 0.001 / 0.011 |
| Jetson ms/frame, x3_server running (median / p95) | 8.5 / 10.6 | 10.5 / 14.5 |

How it got there (each step on the same seeds): the first version split people well
(62% / 89%) but cut walls into 'people' (clutter 10 -> 28 per frame) and lost 10% of
real people, by dropping the usual split's pieces when it found others in the same
component. Fixes: keep the usual pieces; compactness; top height and n r^2 (these keep
~93% of real and synthetic people's pieces and drop 2/3 of clutter pieces). Uncapped it
reaches 62% / 89% at +10% clutter but costs 2x on the laptop (Jetson 16.7 / 29 ms); the
2.5 m component cap gives up groups merged into a long wall. `PEAK_MAX_SPAN_M` is the knob.

Not yet: the classifier was trained on base candidates only (rebuild own.npz with peaks
before trusting its scores on peak pieces); Monday's real crowd runs are the real test;
then shadow mode on the robot (both extractors, log only).

## Next
1. Resolve the 0.69 m object; inspect high-scoring 'unknown' candidates.
2. Recording session: several people, 4-10 m, behind the robot, a real crowd.
3. Extractor: peak split built and measured offline (above). Next: score it on the
   real crowd runs, rebuild own.npz with peaks and re-check the classifier, then shadow
   mode. Add seated synthetic poses (chairs next to them) for the furniture case.
4. Only then: lidar-only births in the tracker (lower confidence class), shadow mode.
5. JRDB (access requested 2026-11-01) as a second, robot-height dataset.
