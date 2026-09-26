# C1 recording contract — stage 1, ToF provenance

2026-09-23. Active on the Jetson after the user restarted the server. The 30-second
stationary physical ToF capture/replay passed with one visible frame omission;
full C1 synchronization qualification remains pending.
C1 is **in progress**, not complete. Single-versus-dual optical interaction
experiments are deferred until the user can perform them at the apartment.

## What this stage changes

`TeensyToFArrays` remains the sole serial owner. It preserves each complete raw
firmware record inside an `x3.tof.observation.v1` envelope. The server publishes
those envelopes as `std_msgs/String` JSON on `/tof/observations` (reliable,
depth 100). It retains the existing point-cloud topics and their publication
stamps. No camera second owner, firmware reflash, clock-offset guess, or
controller tuning is introduced.

Envelope contract:

| Field | Meaning |
|---|---|
| session_id / connection_id | Host-reader process UUID / serial-open counter; separate discontinuous connections |
| sensor / frame_id | Upper or lower and the corresponding URDF frame |
| payload | Original complete firmware record, including seq, t_ms, read_us, all 64 distances/statuses/target counts |
| receipt_monotonic_ns | Host monotonic time when the serial chunk containing this complete line was read |
| receipt_unix_ns | Host wall-clock sample immediately after that receipt; may jump independently of monotonic time |
| source_read_complete_ms_unwrapped | Per-sensor unwrapped MCU read-completion counter, only within a continuous epoch |
| sensor_epoch | Incremented on ambiguous reset/reordering; not a verified device boot ID |
| continuity | first / continuous / gap / duplicate / reset_or_reorder / non_frame |
| missing_sequence_count | Successful firmware-read sequence numbers missing between accepted progression points |
| accepted | Whether a frame may update the latest cache and existing cloud callback |
| clock | Explicit source semantics and null acquisition/offset/error-bound fields; synchronized=false |
| publication_ros_ns / publication_ros_clock_type | Separate ROS publisher time/domain; never mislabeled acquisition time |

Records read in the same serial chunk share receipt time. Parsing, publishing,
and downstream processing can happen later. Receipt-age freshness uses that
original receipt time, not the time a consumer requests the cache. Duplicate
sequence numbers do not refresh the cache or produce another cloud. Source
clock/sequence reversals start a new epoch; the first ambiguous frame is kept
in evidence but excluded from the live cache. uint32 millis and sequence wraps
are handled modulo 2^32 when forward deltas are less than 2^31. Long outages
or reordered data beyond that limit are intentionally ambiguous.

Reconnect resets continuity tracking, but does not restamp old cached frames.
There is no MCU boot ID, so a reboot during a disconnect cannot be proven from
these records. Reconnection-first records remain explicitly labeled `first`.
Status and stats records carry active flags and firmware cumulative counters.

## Clock limitations and next synchronization decision

`t_ms` is sampled after the I2C read. Subtracting `read_us` estimates the read
start, **not exposure time**. Firmware sequence numbers count successful reads,
not every internal sensor exposure. A USB backlog can contain old observations
with a recent host receipt. This stage exposes that uncertainty; it does not
claim to solve it or use receipt age as an acquisition-age safety bound.

No finite host/device clock-error bound exists yet. An offset inferred from
one-way receipt times would confound clock offset with USB scheduling delay.
Next stage must either add a measured two-way clock exchange/boot ID and
characterize sensor-ready/acquisition timing, or explicitly score ToF only in
stationary windows without claiming synchronized moving-scene observations.
Upper/lower acquisition is not synchronized. Pair by documented time semantics,
not array index or equal sequence number. Never interpolate through unknown
clock epochs to hide missing intervals.

## Capture and deterministic audit

After deploying the reviewed changes into the existing server process, run on
the Jetson with ROS domain 42 and its workspace sourced:

```bash
python3 src/c1_tof_audit.py --output /tmp/c1-tof-trial --seconds 30
python3 src/c1_tof_audit.py --replay /tmp/c1-tof-trial/observations.jsonl
```

Use a new output directory each time. The recorder subscribes to the server;
it does not open `/dev/teensy_tof`. It writes raw envelopes, recorder receipt
times, source/configuration snapshots and hashes, git revision/diff, and an
audit report. The manifest describes the **recorder's checkout**, not a verified
running publisher revision; compare live process source before interpreting it.
Capture with the robot parked and the scene unchanged. User starts any run
requiring walking, target placement, or driving.

Replay uses only stored timestamps, in file order, and never refreshes a frame
with playback time. It reports per-session/connection/sensor/epoch rate,
source gaps, additional recording sequence gaps, duplicates, resets, receipt
intervals >500 ms, read durations, and missing clock capabilities. It reports
`c1_complete:false` deliberately. A zero capture exit status only confirms both
sensor identities delivered valid protocol frames, not C1 qualification.

For a later combined ROS bag, `RECORD_C1=true` adds required `/tof/observations`
and optional per-sensor clouds and RGB topics to `record_bag.sh`. Existing depth,
CameraInfo, odom, scan, tf, tf_static topics remain. Optional RGB topics are
currently not implemented, so their absence is an OPEN gate, not permission to
interpret unpaired depth as RGB-D. The JSONL audit does not replace a multi-sensor
bag or test TF/pose alignment. Bag replay/audit for paired RGB-D remains pending.

## Validation and deployment boundary

Local tests cover payload identity, per-sensor sequence gaps, wrap, reset
quarantine, duplicates not refreshing the real reader's cache, deterministic
replay, and recorder-level sequence loss distinct from serial loss. Existing
ToF geometry and serial-parser tests are included in the regression run.

Live inspection found the Jetson at ceb32f2 with local modifications to
`src/server_x3.py` and `config/tof_floor_baseline.json`. Do not overwrite those
with this checkout. Merge the small instrumentation changes against the live
server and preserve its current service arguments. The patch was checked and applied on the Jetson while preserving its
`--tof-baseline` changes and current floor-baseline files. Backups are under
`artifacts/c1-stage1-2026-09-23/` on the Jetson. Python compilation passed there.
The user subsequently restarted the server and authorized the stationary
payload capture. Evidence: [physical trial report](../artifacts/c1-stage1-2026-09-23/hardware/RESULTS.md).
This work has not restarted the server, homed hardware, or opened a second
serial/camera owner.

Validation result: 41 tests passed, 2 existing tests skipped. A local ROS smoke
test on isolated domain 199 captured 45 frames per simulated sensor; replay
reproduced the per-stream report exactly. This is synthetic evidence only.

Remaining C1 gates:

1. **Completed for ToF:** merged deployment, 30-second physical payload trial
   and deterministic replay. Both ~15.1 fps, zero malformed records, one upper
   sequence omission consistent with a firmware transmit-drop increment.
   No gaps over 500 ms. See the linked trial report.
2. MCU clock/boot/acquisition uncertainty contract and latency characterization.
3. Atomic OAK RGB/depth acquisition metadata and pairing from the existing camera
   owner; eliminate freshly stamped cached detection evidence.
4. Combined RGB/depth/ToF/pose/TF recording with frozen calibration and verified
   timestamp conversions, missing-data behavior, pairing skew and replay coverage.

## Stage 2 — RGB/depth pairing and shared dataset (2026-09-23)

Implemented and staged on the Jetson; activation and physical verification are
pending. Enable the existing server's `--c1-recording` option. It adds a RGB
preview stream to the existing OAK owner and enables its ROS publisher. It
retains the high-rate depth path and existing WebRTC option. No second process
opens the OAK. Cached detections are now published only once per source frame.

Pairs use nearest bracketing device timestamps, a maximum absolute skew of
20 ms, and one-to-one consumption. Buffers are bounded; duplicate, source-gap,
reset/reorder, unpaired-frame and overflow counters remain in evidence. A new
camera connection starts a new session. RGB is original BGR8; depth is original
16UC1 millimetres, preserving invalid zeros and avoiding a float conversion.
The five `/oak/rgbd/` topics contain RGB/depth images, their separate CameraInfo
messages and JSON pair metadata. Each image retains its own timestamp.

DepthAI device and SDK host timestamps are retained separately. The SDK host
estimate is mapped to Python monotonic time by sampling `dai.Clock.now()` in a
measured bracket; publication separately samples ROS-to-monotonic offset.
Sampling spans and estimated source age are recorded. The SDK synchronization
estimate is **not a measured hard error bound**. Pair skew is measured in the
shared camera device clock, independent of that host offset. Default message
timestamps are not relabeled as exposure midpoints. Clock jumps must be examined
before scoring moving-scene results.

The RGB preview is stretched while the existing aligned depth stream uses a
different intrinsic matrix. Preserve both matrices and full RGB distortion
coefficients; `same_pixel_grid_verified=false`. Equal array dimensions are not
proof of pixel registration. Physical image/edge inspection remains required.

Run on the Jetson after activation:

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=42
python3 src/c1_dataset.py record --output artifacts/c1-rgbd-trial --seconds 30
python3 src/c1_dataset.py audit artifacts/c1-rgbd-trial
```

The recorder writes a SQLite ROS bag of paired images, calibration, metadata,
raw ToF envelopes, odom, scan, TF and static TF. Its manifest freezes source and
configuration snapshots and hashes model files when available. Model hashes do
not duplicate large model files, and checkout snapshots alone do not prove the
running publisher revision. Confirm activation before capture.

Audit retains missing topics/payloads and checks image layouts, pair skew,
source continuity, per-frame odom bracketing (100 ms each side), and odom-to-camera
TF at the estimated capture timestamp. Capture boundaries can lack pose support;
those pairs remain flagged, not silently scored. `pair-index.json` identifies
original bag messages and their pixel hashes. `c1_dataset.load_pair(path, index)`
reconstructs the original arrays, checks hashes and returns full provenance.
It never refreshes time or silently resamples/registers the image. Repeating an
audit on the same bag produces the same report.

`RECORD_C1=true` now requires these five paired topics plus `/tof/observations`
in `record_bag.sh`; the dedicated dataset command is preferred for frozen
manifests and automatic audits.

The optional updated Teensy sketch accepts a two-way `s<token>\n` exchange.
The host derives a causal interval for read-completion time without assuming
symmetric USB delay. It expires after 3 seconds and includes millisecond
quantization and an explicitly assumed 1000 ppm relative drift allowance.
The interval is conditional on no intervening MCU reset, not a verified boot
identity or exposure-time bound. Older firmware continues to stream with clock
synchronization marked unavailable. User elected to continue camera work first;
firmware upload remains pending.

Validation: 49 tests pass with ROS Humble sourced, including actual CDR/SQLite
bag serialization, byte-exact RGB/depth replay, deterministic audit, pose/TF
lookup, malformed-pair rejection before either payload is published, bounded
pairing and Teensy clock wrap. The camera graph also constructs with DepthAI
2.32.0.0 on the Jetson without opening the device. Firmware compiles for Teensy
4.1. These are software checks, not a passed physical C1 dataset trial.

Physical stage-2 update: camera activation and Teensy firmware upload are now
complete. The first combined trial found all required topics but failed camera
rate/gap and initial pose-coverage checks. See
[stage-2 trial results](../artifacts/c1-stage2-2026-09-23/RESULTS.md).
A bounded depth-history fix is staged; the next server restart activates it.
The recorder now stores two seconds of lead-in/tail pose history and reports an
explicit scored interval and unscored boundary count. C1 remains incomplete.

Trial03 update: 329 scored pairs/30s, full pose/TF coverage, no >500ms pair gaps.
Camera estimated age still exceeded 500ms for 17 frames. A recording-only 30fps
stereo-rate experiment is staged. Clock jump classification now accounts for
the recorded sampling brackets, while retaining the unadjusted offset changes.
See stage-2 RESULTS for complete metrics; C1 remains open.

Trial05 is the first passing combined recorder trial: 359 scored pairs/30s,
max skew16.585ms, max gap101.315ms, estimated age p95 322.621ms; all pairs have
pose/TF support and all original pixel hashes were verified. Recorder now waits
up to 10s for first payload on all required topics before starting its padded
scored interval. Full C1 qualification remains open for absolute clock errors,
ToF exposure/boot uncertainty and quantitative spatial registration. No C3 work.

For nominal geometric replay, `rgb_on_depth_grid(arrays, row)` returns derived
rectified RGB and a valid-support mask from recorded calibration. Raw arrays
remain unchanged. Do not treat zero-filled unsupported RGB as valid imagery.
Installed versions and additional estimator/model/scaler configuration are now
included in future manifests; trial05's original manifest remains as recorded.

Stationary alignment test started: user target1.0m,width38cm;60 scored pairs.
Top depth edge is ~26.5px below nominally rectified RGB (median); registration
fails. Side invalid bands and floor-contact bottom prevent full coverage.
No calibration correction fitted. Awaiting same target at1.5m for comparison.
See [alignment results](../artifacts/c1-alignment-2026-09-23/RESULTS.md).

At measured1.5m the target top mismatch remains16px median (92.3% coverage),
versus26.5px at1m. Both60-pair captures pass data-integrity checks, neither passes
registration. Next requested pose raises the target15–20cm at fixed1.5m to separate
image-height/scaling effects from a translational range-dependent offset.

Raised1.5m target (150mm lift) moves top mismatch to~40px, outside fixed test
window. Found omitted IMX2141080P sensor crop in RGB preview intrinsics. A
manufacturer-geometry-based correction is staged with sensor/EEPROM guards,
not fitted offsets;56 tests pass. Await restart/retest of unchanged raisedtarget.
See alignment RESULTS; no alignment qualification pass yet.

Crop correction now active and runtime verified(IMX214,native4208x3120). Fresh
raised-target60-pair trial/audit/replay passes data integrity; top offset drops
~40→3px median. Corrected replays of earlier poses give1m:6px,1.5m:1px. Full
registration gate still fails due residual errors, invalid-edge bands and bottom
support occlusion; do not mark C1 complete. No restart pending. See alignment RESULTS.

Rigid wooden-board trial(300x300mm): user distance initially1m from bumper,
clarified approximately1.04m from OAK.60pairs pass capture/replay/hash checks.
Top median2px,p95upperbound5.5px; full edge gate still fails(side invalidbands,
floorcontact bottom). No further calibration shift. Await board at1.5m camera
reference to complete the two-distance comparison. See alignment RESULTS.

Wooden-board two-distance comparison complete:60pairs each atapprox1.04m and
confirmed1.5m,all replay/hash checks pass. Top median2/4px,p95upperbound5.5/6.5px;
full provisional gate still unmet. Matched side samples show width excess rather
than a single horizontalshift; invalid edges remain explicit. Interior patches
read1088/1544mm vs1040/1500mm references, retained separately as range diagnostics.
No further user movement/restart requested; no constant correction fitted.

Subpixel comparison completed at stationary 1.5m board: off/on 119/120 pairs,
clean replay/hash audits. Top median mismatch improves 4→1px, but p95 upper
bound remains6.5px and side coverage fails. Interior median depth worsens
1535→1554mm relative to1500mm reference. Subpixel required reducing stereo
post-processing allocation3→2 SHAVEs/slices; inference-thread reduction alone
failed. No overall accuracy win or C1 pass. Recommend off return capture to
check repeatability. See [subpixel results](../artifacts/c1-subpixel-2026-09-23/RESULTS.md).

A-B-A subpixel test now complete: median interior1535→1554→1537mm at fixed
1500mm reference. Return off has120 hash-verified pairs, identical replay audit,
no audit failures. Top median4→1→1px: improvement is not attributable to subpixel
alone. Original spatial gate remains unmet. Subpixel OFF is restored and active;
no restart pending. Next: reference-controlled multi-distance range validation
and remaining clock/exposure qualification; no constant range correction.
