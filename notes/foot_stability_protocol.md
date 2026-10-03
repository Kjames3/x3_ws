# Foot stability capture protocol

Use the retained 384x512, four-SHAVE, two-thread model. Keep the robot stationary
and one person in view. These captures are diagnostic-only. No restart is
needed. Run each command separately, getting into the initial stance before
starting it. Keep the same distance, floor and lighting across runs.

1. Standing: `python3 scripts/foot_diagnostic_capture.py --host x3 --seconds 20 --label foot-still`
   Face the camera; both feet planted and clearly visible for the entire capture.
   Mark shoe positions with tape if convenient, without moving the camera.
2. Slow motion: same command with `--label foot-slow-extension`.
   Hold still for the first 5 seconds; slowly extend and withdraw the same foot
   for 10 seconds; return to the initial stance and hold for the last 5 seconds.
3. Sideways: same command with `--label foot-sideways`.
   Turn sideways before the countdown and hold the stance throughout. Report
   which foot is farther from the camera and whether it is visually occluded.
4. Brief occlusion: same command with `--label foot-occlusion`.
   Face the camera with both feet visible for 5 seconds. Move one foot behind
   the other briefly, then return it, repeating slowly. Report approximately
   how long each occlusion lasted; the capture has no synchronized video.

Analysis: deduplicate packets by camera session/sequence; evaluate each side
and track ID separately, with counts so fragmentation cannot hide jitter.
For stationary captures, report positional dispersion around each track's
median, apparent speed, track-ID changes and velocity warm-up coverage. Report
no-foot AND single-foot coverage, observation age and telemetry gaps. For
occlusion examine loss/reacquisition and ID churn; this protocol cannot measure
exact recovery latency without a synchronized visibility reference.

Do not interpret real movement during the extension test as localization
noise. Tape and instructions are not metric ground truth; these measurements
assess repeatability and availability, not absolute foot-position accuracy.
Do not add obstacle persistence merely to make gaps disappear before reviewing
the raw loss behavior. The production CBF still does not consume these feet.
