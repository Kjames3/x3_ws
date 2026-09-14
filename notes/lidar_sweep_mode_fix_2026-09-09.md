# Continuous capture mode failure and fix

The 16-station `apartment_mapping_01.npz` contains full approximately ±44°
coverage at stations 1–4, but only negative angles at stations 5–16. The
September 7 service journal is unavailable, so the exact historical trigger
cannot be proven. Current GUI telemetry before deployment showed step mode
and 40 deg/s; this does not establish the configuration during the capture.

A reproducible software failure path explains how a transient failure could
persist: the servo loop started a background `require_settled` parameter
request after starting motion. Failure stopped scanning and reset the global
mode to step. The recorder set continuous mode only once, ignored configuration
messages, and accepted stations with only 20 clouds. A retry could therefore
accept a short, incomplete step sweep and keep doing so for the rest of the run.

Changes:
- Start requests await processor configuration before enabling the sweep,
  without blocking the encoder publisher. Removed the background request.
- Configuration failure stops scanning and restores the settled gate, but
  preserves the requested sweep style for the next attempt.
- Gate writes serialize; start requests recheck motion interlocks, mode, and
  whether a newer start/stop/configuration request superseded them.
- Recorder requests continuous 45 deg/s at every station, checks configuration
  and start acknowledgements, then starts its ten-second timer. Mode/speed
  changes and explicit early stop messages abort the capture.
- Acquisition-time cloud coverage must include both ±35° extremes, at least
  three clouds in each of six pitch bands, at least 40 clouds, cloud gaps <=0.5s,
  and joint gaps <=0.3s with joint samples bracketing cloud timestamps.
  Failure aborts with only prior accepted stations saved. These are conservative
  capture-quality gates, not a claim of localization accuracy.
- Configuration/status messages are saved alongside captures as
  `<label>_sweep_events.json`. Processor failures abort; interlock refusals have
  a three-attempt limit.

Validation: eight regression tests pass on laptop and Jetson. Tests exercise
configuration failure, continuous retry, waiting before enabling motion,
cancellation during configuration, full/partial sweep payloads, cloud/joint
loss, and invalid timestamps. Replaying the saved 16-station capture through
the new quality check accepts 1–4 and rejects 5–16. Source compiles on both.
Deployment backup: robot `artifacts/sweep-mode-fix-2026-09-09/before/`.

Physical continuous-sweep validation remains pending. Start with one parked
station using a new label, inspect its payload, then plan any mapping retakes.
The old capture is preserved; no candidate map was promoted from its partial
stations. The four-arm motion interlock and scan gating remain in place.

After deployment, x3_server restarted successfully. A read-only recorder
preflight received 28 horizontal scans plus live odometry and tilt joints;
no sweep or chassis command was sent by the probe. SHA256 hashes for the
server, recorder, and quality helper match between laptop and robot.
