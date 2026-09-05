# Parked continuous-sweep validation — 2026-09-05

Resumed after a seven-hour pause, battery charging and relocation from the lab
to the user's apartment room. The robot rebooted; prior /tmp test results were
lost. New scripts and results are stored persistently under
`artifacts/continuous-2026-09-05/` on both machines.

## Preconditions and fixes

The deployed server and processor hashes matched the local files. Calibration
remains direction +1, horizontal zero 2032. No stale test processes were running.
An initial preflight observed a ROS data-plane stall (the live processor also
stopped receiving scans), and aborted before moving the mount. Restarting
x3_server restored delivery; a new preflight passed.

The code changes deployed before the pause are now hardware-tested:

- Read require_settled from the live ROS parameter, rather than caching it once.
- Keep /scan's settled-window requirement even when moving 3D clouds are enabled.
- Use encoder distance to the target to detect continuous endpoints, because
  continuous mode intentionally does not poll the hardware Moving register.
- Never falsify JointState's moving state as a fallback for a failed parameter
  update. Stop the sweep and restore step mode on such a failure.

The previous local and Jetson test runs passed 21 regression tests. No further
production-code changes were needed during this resumed physical validation.
The unrelated octomap_viewer.py working-tree change was left alone.

## Fresh apartment baseline

A 60-second step-and-stare sweep yielded 239 clouds, all transformed, from 519
raw/timed scans across warmup, sweep and homing. There were no nonzero chassis
commands, no odometry translation and no tilted /scan leakage. Floor-candidate
median height was +0.002 m. Lab geometry was not reused as the baseline.

## Continuous sweeps

Each test included approximately 30 seconds of measured continuous motion after
warmup, with motors disabled and all navigation/teleoperation interlocks intact.

| Measurement | 20 deg/s requested | 45 deg/s requested |
|---|---:|---:|
| Measured central-speed median | 20.44 deg/s | 44.97 deg/s |
| Central-speed p10–p90 | 18.46–22.33 deg/s | 42.01–47.81 deg/s |
| Raw scans in measured window | 215 | 218 |
| Deskewed clouds in measured window | 209 | 218 |
| Cloud rate | 6.92 Hz | 7.18 Hz |
| /scan messages during measured sweep | 0 | 0 |
| Nonzero chassis commands | 0 | 0 |
| Odometry translation | 0 m | 0 m |
| Cloud end-of-acquisition latency p50 / p95 | 60 / 109 ms | 60 / 128 ms |
| Maximum observed end latency | 222 ms | 365 ms |
| Processor CPU, percent of one core | 46.94% | 46.58% |
| Whole-system busy (includes collector) | 60.78% | 61.23% |

The diagnostic collector itself used approximately 46.5% of one core. These
system totals therefore include substantial measurement overhead. CPU samples
were taken before artifact transfer. Endpoint reversal necessarily accelerates
and decelerates; the central sweep is approximately constant speed.

## Geometry comparison

`analyze.py` independently fits a floor plane to the fresh parked baseline,
then finds 10 cm XY cells with at least 20 baseline points and over 80% within
5 cm of that plane. Both continuous methods use the SAME return pairs, selected
symmetrically by pair midpoint and the baseline floor cells (0.5–3 m radius).
The initial broad near-floor selection included furniture; the spatial mask
prevents those objects from being reported as floor error.

| Floor deviation | 20 deg/s rigid / deskew | 45 deg/s rigid / deskew |
|---|---:|---:|
| Median | 2.29 / 1.26 cm | 3.47 / 1.31 cm |
| p95 | 4.63 / 4.09 cm | 7.87 / 4.07 cm |
| Fraction within 5 cm | 97.38 / 99.50% | 72.26 / 99.49% |
| Selected paired returns | 106,393 | 110,587 |

This demonstrates reduced motion distortion on selected floor returns. It is
not an independent absolute-accuracy calibration, whole-scene metric, or a
3D localization/registration test. The remaining plane tilt, ranging outliers,
and long delivery tails remain relevant to moving-robot work.

## Final operating policy

Physical tests restore step mode, stop the sweep, home within the configured
horizontal tolerance and leave chassis motors disabled. Continuous sweeping
is validated only while parked. The 120 ms tilt-gap rejection stays enabled;
there is no virtual horizontal scan, no moving-chassis test and no 3D localizer.

## Isolated load suite and remaining work

`sweep_load_bench.py all` completed its five nonexclusive stages. The standalone
servo stage explicitly skipped because it requires taking the serial port from
the live server; no port contention was introduced.

| OctoMap setting | Actual sent / inserted rate | CPU, percent of one core |
|---|---:|---:|
| 8 m, nominal 2.5 Hz | 2.3 / 2.3 Hz | 23.1% |
| 8 m, nominal 7.2 Hz | 6.7 / 6.7 Hz | 58.5% |
| 8 m, nominal 14.4 Hz | 13.5 / 12.4 Hz | 97.5% |
| 4 m, nominal 7.2 Hz (separate run) | 6.7 / 6.7 Hz | 36.9% |

At the measured benchmark throughput, 4 m reduces OctoMap CPU by approximately
37% versus 8 m.

**Resolved 2026-09-05: production range is now 6 m, not 8 m or 4 m.** A finer
cap sweep at the continuous 6.8 Hz cloud rate
(`artifacts/range-caps-2026-09-05/comparison.json`) showed 5 m, 6 m and 8 m all
cost the same CPU (61.3 / 61.1 / 60.3% of one core) and all keep 100% of
occupied endpoints, while 4 m saves only ~4 points of CPU and drops endpoint
retention to 82.8%. The CPU case for a tight cap therefore does not survive
past 4 m, and 4 m is the only setting that loses data. Applied to
`cloud_max_range_m` and to `x3_octomap.launch.py`'s `max_range` default. The benchmark's actual publication rates are below
the requested rates, so these are not exact 7.2/14.4 Hz saturation proofs.

A clean parked baseline measured 56.0% system busy (2.64 of six cores idle).
The baseline stage in `benchmark_all.log` overlapped artifact transfer, so use
`benchmark_baseline_clean.log` for idle headroom. Production processor CPU was
40.3% of one core while parked and 46.6% during the measured 45 deg/s sweep;
these separate samples are not a controlled A/B experiment.

The candidate numpy microbenchmark measured 0.743 ms/scan at 45 deg/s versus
5.843 ms for legacy projectLaser. It excludes the deployed middleware and
buffering costs and must not replace the real processor CPU measurement.
The rate-stage gaps are callback-arrival gaps, not measurement-time gaps.
The benchmark previously mislabeled angular travel across a gap as interpolation
error and a deskew accuracy floor; its explanatory labels are now corrected on
both machines. The archived all-stage log retains the old labels; interpret
those rows only as illustrative angular displacement.

All temporary bench processes exited. Final WebSocket state: mode step,
scanning false, settled_bypass false; /scan flowing and mount +0.70 degrees,
within horizontal tolerance. Motors remain disabled.

Next priorities are the 4 m OctoMap range decision and reducing the timed input's
Point32 deserialization overhead. Moving-while-sweeping remains blocked on a
validated live horizontal obstacle feed; this test does not relax the interlock.
