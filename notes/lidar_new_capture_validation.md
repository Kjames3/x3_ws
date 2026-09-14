# New eight-station capture against frozen map

Capture received 2026-09-06; analysis continued 2026-09-07. The user subsequently
clarified that stations 5 and 8 entered different rooms, superseding the earlier
same-room interpretation. The apartment contains a restroom, living room/kitchen
and bedroom; exact station-to-room assignment and reference-map coverage remain
unconfirmed. Furniture was unchanged. They moved themselves twice;
station 8 intentionally did not return to the initial pose. No loop-closure or
absolute position-error score is therefore claimed.

## Recording passes; localization does not yet pass

609 distinct clouds, 1,658,919 finite points, eight accepted stations. Every
cloud has a matching ID and acquisition timestamp in cloud_odom, and the ROS
odometry trace brackets the capture. Counts match payload sizes. Within-sweep
odometry translation is below 0.1 mm, with heading variation below 0.83 deg;
these are odometry measurements, not an external motion reference. All stations
cover roughly -44 to +44 degrees of tilt.

Station 2 contains one maximum acquisition gap of 0.558 s; other stations'
maximum gaps are 0.142-0.144 s. The capture is usable, but the gap must not be
hidden by re-stamping observations. Saved data does not identify the cause.

Raw capture and summaries are on laptop and robot under
artifacts/localization-validation/. SHA256 of the raw NPZ:
80adfa9a857540a695047614834d983e83585332b50b096b133e032c9bd972e5.

## Frozen-map sequential evaluation

`src/evaluate_lidar_validation.py` first estimates an OFFLINE initial alignment
using only the first 35 clouds of station 1. It searches 240 map-neighborhood
pose seeds using relaxed search gates. This is a diagnostic bootstrap, not the
production initializer and not ground truth. The leading hypothesis had 93.0%
overlap / 5.0 cm NN residual; another distinct admitted cluster had 77.9%
overlap. Candidate ranking does not prove the leading hypothesis correct.

The other 574 clouds then run sequentially using the normal Matcher limits and
exact per-cloud odometry. Only accepted results update the internal correction.
There is no per-station reset, no addition of new points to the map, and no
reuse of these new observations to optimize the frozen reference map.

| Station (user numbering) | Accepted / tested | Observation |
|---|---:|---|
| 1 | 35 / 35 | 18.5 cm p95 displacement from accepted station median |
| 2 | 75 / 76 | One convergence rejection; 7.7 cm p95 spread |
| 3 | 75 / 75 | 4.6 cm p95 spread |
| 4 | 76 / 76 | 7.7 cm p95 spread |
| 5 | 0 / 79 | Every cloud exceeds prior-correction limits |
| 6 | 78 / 78 | 3.7 cm p95 spread |
| 7 | 16 / 78 | Partial admission; 23.8 cm p95 spread |
| 8 | 0 / 77 | Low overlap for all clouds; other gates also reject |

Overall admission is 355/574 (61.8%). These are admission rates, NOT localization
success rates. The substantial pose variation at parked stations 1 and 7 shows
that the gates can admit unstable results. No ground truth is available to
establish which individual poses are correct.

Laptop matcher runtime p50/p95/max: 13.5/50.0/73.7 ms. This is offline matcher
time, not sensor latency or a full Jetson sweep/OctoMap load measurement. Source
geometry and timestamp checks are separate from this solver evaluation.

## Reacquisition diagnosis

`src/diagnose_lidar_validation.py` searches whole accumulated stations 5, 7 and
8 against the SAME frozen map. This intentionally consumes all clouds from
those stations and relaxes prior gates. Its fits are hypotheses for diagnosis,
not extra passing tracking results or independently validated poses.

At station 5 the sequential per-cloud correction request is median 39.3 cm,
p95 48.8 cm, above the 30 cm tracking gate. Whole-station search finds a 77.1%
overlap hypothesis, but also a distinct 71.1% alternative with a LOWER residual.
At station 7 it finds an 85.7% overlap hypothesis plus a distinct 68.7% alternative.
Thus lowest residual alone cannot select a trustworthy reacquisition pose.
Station 8 also has competing fits: 69.0% versus 62.2% overlap. The leading
hypothesis differs by 3.83 m from the initial odometry alignment, but that
is NOT evidence of 3.83 m actual odometry error: the global match itself is
unverified. All three problematic stations have plausible alternatives.

## What follows from this test

The earlier 136/137 result came from held-out clouds within the SAME capture
session and stations as map construction. It did not establish generalization
to this new eight-station run. This result supersedes any interpretation that
per-cloud registration is already ready to replace AMCL.

Keep the diagnostic node separate from navigation. Do NOT widen the 30 cm gate
just to increase admission: the competing fits and within-station instability
show why that can admit wrong poses.

The next software experiment should use these existing recordings:

1. Add a consistency check over several clouds, explicitly compensating their
   odometry motion, before accepting a large pose correction. Evaluate both
   stationary jitter and changes across sweep pitch, not just fit residual.
2. Separate ordinary tracking from explicit lost/reacquisition states. During
   reacquisition compare multiple candidate poses and require independent
   subsequent observations to support a unique candidate before publishing.
3. Diagnose/map any persistently uncovered region using additional observations
   only AFTER frozen-map validation; do not quietly rebuild the reference with
   this validation set and call improved fit generalization.
4. Re-run frozen-map evaluation with declared gates and report rejected clouds,
   ambiguity and stationary pose spread. Keep separate hold-out observations
   for assessing any changes tuned on this run.

No repeat physical run is necessary for these next diagnostic experiments.
The user's movement may introduce transient returns; the capture does not
label those intervals, so it cannot establish that they caused the failures.

## Room-context correction (2026-09-07)

Stations 5 and 8 being in other rooms makes map coverage/doorway overlap a
priority diagnostic. The numerical results remain unchanged, but they do not
isolate an estimator-only failure. Audit reference coverage before adding
recovery complexity. Missing map geometry cannot be supplied by pose filtering.

Further physical context supplied by the user: station 5 was partially inside
the restroom, about 1.5 m from the preceding station; station 8 was in the
middle of the bedroom, about 2-2.5 m from the preceding station. Doors stayed
open. These approximate distances exceed the proposed 0.5-0.8 m capture spacing.
They support investigating reduced overlap and longer uncorrected odometry
intervals, but do not establish either as the sole cause. Original map coverage
of those rooms remains unconfirmed.

The user subsequently confirmed that the original five-station reference-map
capture stayed entirely in the living room/kitchen. No reference stations were
inside the restroom or bedroom. Any geometry captured there was incidental
visibility through the open doorways. This makes insufficient destination-room
coverage a leading explanation for stations 5 and 8, though it does not prove
that every rejection was caused solely by coverage. Prioritize a connected
multi-room mapping pass before further gate tuning. Evaluate the expanded map
on separate new observations, not the observations used to build it.
