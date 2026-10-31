# Opt-in stationary OAK display trial

Optimization checkpoint committed as286f6c1; prior offline checkpoint94be70f.
Neither pushed. Live integration changes are uncommitted.

## Implementation deployed

Shared geometry in src/oak_color_geometry.py; offline wrappers retain interfaces.
Driver exposes a separate non-draining latest pair; existing recording queue
unchanged. src/oak_surroundings.py runs a bounded3Hz worker, gated by cloud
subscribers; dynamic TF listener exists only while subscribed. Caches RGB
rectification maps/support, applies calibrated correction once, rejects stale
frames (>500ms) and missing/stale TF, and places points in odom using capture-time
optical transform. Separate current base pose accompanies the cloud. Color mask
uses conservative12px boundary exclusion; uncertain valid geometry stays gray.

GUI toggle: Surroundings -> Layers & metrics -> OAK color (experimental), default
off. Up to6000 points (5504 in the sample), separate from Livox/nav/CBF topics.
Renderer uses bounded preallocated point/color buffers and recent robot pose,
hides stale cloud/pose, unsubscribes on hidden view/browser tab/pagehide, and
labels neutral gray as uncertain color. No temporal OAK accumulation or mesh.

Robot backups: ~/x3_ws/artifacts/oak-colored-cloud-2026-10-30/pre-overlay/.
Server restarted twice during authorized stationary deployment. Final shutdown
exception guard is copied to disk for next restart (normal running behavior is
unchanged). To disable: uncheck the overlay; to roll back, restore five backed-up
runtime/web files and restart x3_server. New modules can remain unused.

## Stationary evidence

First steady on/off/on-release test (live/first-run/websocket-check.json):
scan off9.86Hz,on9.99Hz,off9.99Hz; RGBD metadata11.92/12.05/11.92Hz.
OAK2.96Hz,arrival age p95 311ms,payload+Foxglove~261kB/s. Approximate tegrastats
phase averages CPU60.5/61.0/60.2% across cores,RAM1833..1861MB. This is a short
observational trial, not proof of sustained headroom or a15-minute memory gate.

Repeat after listener/visibility fixes: on-window90 clouds/30.04s (2.996Hz),
scan9.986Hz,RGBD metadata12.016Hz,age p95 330ms,~240kB/s. Camera logs29–30FPS.
The repeat initial off-window overlapped startup and the user's browser may also
have subscribed: do not use its low initial rates as a controlled baseline.
A330ms event-loop stall occurred during startup, before the OAK subscription;
no further stall was logged during the measured on-window. This is not a causal
latency guarantee. Perception estimator accuracy/dropouts weren't scored; stream
rates and camera logs are the current evidence.

Live worker p95 elapsed119ms, thread CPU18ms. Thread CPU excludes work on native
library helper threads. End-to-end age meets500ms target in this trial; processing
wall time DOES NOT meet proposed15ms budget. Offline kernel numbers do not include
TF lookup, rectification, serialization, concurrent server scheduling and publish.
Keep opt-in; do not mark all resource gates passed.

Actual binary cloud decoded with the browser's parser:5504 finite XYZ points,
valid RGB channels; fixture verifies red/green order, wrong layout and truncated
input rejection. Python tests:8 passed,1 skipped (ROS unavailable locally for
stale-frame test); earlier combined alignment suite11 passed. JS syntax passes.
User visual placement/responsiveness feedback requested, pending. No browser FPS
measurement, no live movement test, no long-duration memory test, no camera
reconnect/cold-boot test in this stage.

Evidence: artifacts/oak-colored-cloud-2026-10-30/live/ contains websocket counts,
sampled CDR, tegrastats and server metrics. The physical box remains the only
registration target; full perimeter alignment remains unqualified.

## User visual feedback

User confirms the colored cloud looks okay on desks, tables and some chairs.
There is noticeable but modest lag; some objects look like a blurry mix of colors
depending on viewing direction. This supersedes the pending-feedback statement
above: stationary visual validation is PARTIAL, not a clean pass. No FPS numbers
or exact acquisition/render latency were supplied. Depth boundaries, sparse
sampling, surface angle, projection overlap and temporal mismatch are hypotheses,
not identified causes. Next refinement should distinguish viewing a stationary
cloud from actual robot/object movement, and isolate the OAK layer from Livox
before changing calibration or increasing point count. Keep opt-in and no motion
validation claimed.
