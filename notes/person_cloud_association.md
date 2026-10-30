# GUI person/cloud association — first diagnostic version

The Surroundings overlay associates compact Livox clusters with camera-confirmed
C3 identities. It is not a lidar person detector, joint pose estimator, or a
replacement for the navigation/CBF person estimates. Existing avatar positions
and C3 rings retain their original sources.

C3 telemetry adds odometry XY, odometry velocity, source state timestamp in Unix
seconds, and packet age. The browser pairs the cloud with its exact stamped
`/surroundings/pose`. Camera state is predicted to the cloud timestamp with a
bounded age gate. No client wall-clock synchronization is required.

The browser:

- Filters out low floor returns and points above 2.2 m; builds whole-scene XY
  connected components on an 18 cm grid. A component needs eight points and
  35 cm vertical support, with maximum XY span 90 cm and minimum span 12 cm.
  Whole walls are rejected before cropping can make them appear person-sized.
- Gates components within 55 cm of a predicted camera-seeded track. Requires
  distinct nearest matches in both directions; ambiguous and merged candidates
  are left unassigned. Geometry close to walls/furniture may conservatively fail.
- Uses a bounded alpha-beta position update in odometry. Three supporting cloud
  frames establish lidar confirmation. A duplicate frame cannot add a hit.
- Allows lidar-supported continuation for at most 1.2 seconds after the most
  recent semantic camera observation, and only brief uncertain prediction
  (400 ms after the last lidar support). Stale/repeated camera packets cannot
  renew that deadline. Missing synchronized pose and stale clouds hide/reset
  the overlay; pose discontinuities reset it.

Matched returns are colored by camera ID, with a matching observed-extent box
and a position ring. The ring radius is a heuristic uncertainty estimate, not a
calibrated confidence interval. Status identifies tentative support, camera +
lidar, lidar-only continuation, camera-only fallback, and uncertain prediction.
The ring follows the observed surface cluster, not an inferred anatomical centre.

Supported lidar locations also mask people out of static scene memory during a
brief camera dropout. Raw points and the clean environment remain available.

Validation:

```bash
node tests/web/person_cloud_tracker_check.cjs
python3 -m pytest tests/test_c3_live.py tests/test_c3_person_tracker.py -q
```

Browser checks inject synthetic camera/cloud/pose packets, verify tentative ->
confirmed transitions, check rendering and exercise toggle/stale cleanup. A real
walking/crossing test is still needed to tune the conservative association gates
for this mount, range, and the 6,000-point display stream. No robot motion is
commanded by this feature.
