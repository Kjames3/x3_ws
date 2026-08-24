---
name: feedback_verify_before_long_runs
description: Kamren validates a recording pipeline with a short trial bag and a payload-level check before committing to a long capture session
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 873ed488-d99e-4d1a-bdd5-37d55ee417db
  modified: 2026-08-08T06:25:55.733Z
---

After any change to the recording pipeline, run a **short timed trial** (`RECORD_DURATION=60
./record_bag.sh`, with a countdown before it starts) and verify the resulting bag at the **payload**
level before the user commits to a real capture session.

**Why:** a 30-minute batch was lost to bags that turned out to be bad, and the user now explicitly
asks to confirm a small run first ("I just wanted to confirm before recording more"). Robot time and
room bookings are the scarce resource — a re-record costs a whole session.

**How to apply:** offer the trial run proactively rather than handing back a changed script. Checking
that a bag exists is not enough; the failures here were all payload-level and invisible in
`metadata.yaml`. Check: `zstd -t` integrity, sqlite `integrity_check`, metadata counts vs actual
counts, largest inter-message gap per topic (one missed tick at ~9.8 Hz is 0.42 s — anything larger
is a stall), **and that payloads are actually distinct** rather than a cached buffer re-emitted with
fresh stamps, which is exactly how the frozen `/oak/detections` in
[[project_oak_rosbag_recording]] hid. Also sanity-check odometry path length: three 2026-07-31 bags
had 0.00–0.21 m of motion, which is fine for a shakedown but useless for domain-adaptation
viewpoint diversity.
