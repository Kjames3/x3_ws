# Foot-extension diagnostic capture

20.04 seconds; 251 telemetry messages; 83 distinct diagnostic sequences.

| Measurement | Result |
|---|---:|
| Distinct packets with at least one foot | 80/83 (96.4%) |
| Distinct packets with both feet | 79/83 (95.2%) |
| Distinct diagnostic packet rate received | 4.14 Hz |
| Reported capture age when processing started, median / p95 | 299 / 408 ms |
| Reported observation age at telemetry publication, median / p95 | 469 / 673 ms |
| Diagnostic computation wall time, median / p95 / maximum | 34 / 186 / 311 ms |
| Matched RGB/NN-to-depth gap, median / maximum | 13 / 36 ms |
| Shadow suggestion / infeasible / warming-up / stale / no-feet | 65 / 13 / 2 / 2 / 1 |
| Nonzero suggestions (>0.005 m/s) | 2 |
| Distinct foot IDs | 7 left, 7 right |

High packet coverage masks expiry between updates. Repeated snapshots of a packet
can become stale before a newer packet arrives. There are nine recorded no-foot
intervals, 0.059–1.030 s; a last-known-state timing calculation gives 14.5% of the
captured interval without feet. These are telemetry-observed intervals, not precise
camera dropout durations. Telemetry gaps reach 0.923 s and can obscure transitions.
Reported observation age excludes subsequent network/browser delivery delay.

All 13 infeasible packets have at least one foot constraint that alone requires
more than the configured 0.15 m/s speed limit (minimum speed lower bounds range
0.175–1.999 m/s). Thus these are incompatible constraints under the current model,
not merely unexplained optimizer failures. Estimated approach speeds reach
1.17 m/s, and the effective exclusion radius including uncertainty is often
0.6–0.7 m. These are provisional estimates, not ground truth. Raising the robot
speed cap is not justified by this record.

Most feasible results request zero velocity; the two nonzero results peak at
0.075 m/s. A visually present marker therefore does not mean the shadow system
continuously has a usable retreat suggestion. The person root stayed translation-
stationary in odometry, with a 1.62-degree heading range.

## Next diagnostic work

1. Instrument depth extraction versus solver time and reduce compute/queue latency.
2. Inspect foot-region association and ID resets; capture synchronized image/depth
   evidence before judging the position/velocity spikes as true foot motion.
3. Make stale/infeasible episodes persist visibly in the GUI and include durations
   in capture summaries rather than unique-packet coverage alone.
4. Reassess prediction, clearance uncertainty and achievable motion against evidence.
   Keep the system diagnostic-only; do not raise speed limits to conceal infeasibility.

No RGB/depth images or ground-truth foot trajectories were captured, so this log
cannot establish localization accuracy, reaction time to physical motion, or safe
motor-enabled avoidance. Full numeric analysis is in `analysis.json`.

Configuration caveat: this capture did not record camera/depth rate settings.
A separate same-day depth-rate experiment found reduced NN throughput at 15 fps;
do not attribute this capture’s 4.14 Hz to the normal camera configuration without
confirming historical settings. Include those settings in future captures.
