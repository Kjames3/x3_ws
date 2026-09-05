# Item 5 — virtual horizontal /scan during continuous sweeping (2026-09-05)

Offline study. No hardware was moved. Replays the parked 30 s / 45 deg/s
continuous sweep from `artifacts/continuous-2026-09-05/continuous_45_points.npz`
(deskewed clouds, base_footprint) through the new
`yahboomcar_bringup/virtual_scan.py`. Script and results:
`artifacts/virtual-scan-2026-09-05/replay.py` + `replay.json`.

## Method and its one reconstruction step

The npz stores concatenated points with no per-cloud boundary or timestamp, so
each return's acquisition tilt is recovered geometrically. The tilt axis is +Y,
so a beam at bearing psi leaves the laser along
`(cos psi cos th, sin psi, -cos psi sin th)`, giving `th = atan2(-vz, vx)` for
every bearing except the two exactly on-axis. Validated against the servo
record: recovered p1/p99 tilt is **-44.1 / +44.4 deg** versus the joint topic's
**-44.1 / +44.4**, with the expected density pile-ups at the ping-pong
endpoints where the mount decelerates.

Caveat carried into every number below: the climbing and descending passes are
merged, so **coverage is marginally optimistic**. The gap figures model both
visits explicitly and are not affected.

## The geometry that decides this

Because the tilt axis is +Y, beams at psi = +/-90 deg lie **on** the axis: they
are invariant under tilt and stay in the laser's horizontal plane for the whole
sweep. Beams near psi = 0 (forward) take the full pitch. So lateral bearings are
refreshed every scan and the forward sector is only inside the robot's height
band near a level crossing. This is not a tuning problem; it is the mount.

## Result: a single cloud is not an obstacle feed

Worst-case window placement, height band 0.12-0.40 m, 6 m cap:

| accumulation | all bearings | forward +/-30 deg |
|---|---:|---:|
| 0.142 s (one cloud) | 7% | **0%** |
| 0.285 s | 10% | **0%** |
| 0.500 s | 12% | **0%** |
| 1.001 s | 94% | 64% |
| 2.001 s (half sweep) | 100% | 100% |

Per-bearing refresh gap over one 4 s ping-pong cycle: all p50 1.85 s / p95
2.02 s; **forward p50 1.98 s, max 2.02 s**.

`server_x3.OBSTACLE_STALE_S` is **0.5 s**. At +/-45 deg the forward sector is
measured every ~2 s, so the CBF would discard the feed as stale at all times,
and raising the threshold to accept it means driving on 2-second-old forward
obstacles — ~0.6 m of blind travel at 0.3 m/s, against a CBF trigger distance
of 0.30 m. **The user's suspicion is confirmed: a horizontal slice alone is too
sparse, and so is the full height band.**

The band is still the right projection — it doubles per-cloud coverage over a
thin slice (7% vs 3%) and is what keeps the floor out (floor spread is p10
-0.062 to p90 +0.089 m, so a band starting under ~0.10 m re-detects the floor as
an obstacle, the same trap ground-plane removal hit). But the band does not fix
the forward sector, because the limit is illumination, not filtering.

## The lever is amplitude, and it fights the 3D map

The forward gap tracks `2 * amplitude / speed` almost exactly:

| ampl | period | fwd gap p50 | fwd gap max | low obstacles (z 0.02-0.12) | ceiling (z>1.5) |
|---:|---:|---:|---:|---:|---:|
| 5 deg | 0.45 s | 0.20 s | 0.24 s | 2.5% | 0.0% |
| 10 deg | 0.89 s | 0.42 s | 0.47 s | 6.6% | 0.0% |
| 15 deg | 1.33 s | 0.65 s | 0.69 s | 9.3% | 0.0% |
| 20 deg | 1.78 s | 0.87 s | 0.91 s | 10.2% | 1.2% |
| 30 deg | 2.67 s | 1.31 s | 1.36 s | 10.4% | 5.4% |
| 45 deg | 4.00 s | 1.98 s | 2.02 s | 10.9% | 12.4% |

The two requirements separate cleanly:

- **Safety** (low obstacles below the laser plane) saturates by +/-15-20 deg:
  9.3-10.2% of returns versus 10.9% at the full +/-45.
- **Overhang / ceiling** — the reason the sweep is +/-45 at all — collapses:
  12.4% at 45 deg, 5.4% at 30, 1.2% at 20, zero below.

So one sweep cannot serve both while moving.

## Recommendation

**Two regimes, not one compromise.**

- *Driving*: +/-20 deg at ~80 deg/s gives a forward gap of `2*20/80 = 0.50 s`,
  meeting `OBSTACLE_STALE_S` unchanged, and keeps ~94% of low-obstacle returns.
  80 deg/s is well inside the XL430's ~366 deg/s no-load speed.
- *Parked*: +/-45 deg at 45 deg/s, the already-validated configuration, for the
  full 3D map including overhangs.

Two things must be measured before this is trusted, and neither is in this data:

1. **Deskew at 80 deg/s is unvalidated.** Per-scan smear rises from 6.2 to
   11 deg. The floor p95 of 4.07 cm was measured at 45 deg/s only. The tilt
   sampler's max joint gap (132.8 ms) already exceeds `deskew_max_gap_s` of
   120 ms, so the withhold rate will rise — measure it, do not assume it.
2. **The forward sector is the OAK-D's field of view.** Before committing to a
   faster mount, check whether the existing `/oak/points` obstacle feed already
   covers +/-30 deg forward with better latency, which would let the lidar sweep
   stay at +/-45 for mapping and drop this constraint entirely. That is a
   cheaper answer than driving the servo harder.

## Status

Prototype and tests only. `virtual_scan.py` is not wired into
`lidar_3d_processor_node`, nothing publishes a virtual `/scan`, the four-arm
interlock is untouched, and continuous sweeping remains **parked-only**.
10 unit tests in `tests/test_virtual_scan.py`.
