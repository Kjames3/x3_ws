---
name: capture
description: Run a data capture on the X3 robot end to end - short trial, payload check, hand the user the command, verify each run, fetch, analyse. Use when the user wants to record runs (C1/C3 RGB-D captures, seg-vs-pose, rosbags, OAK detection logs), asks "am I good to continue" between runs, or asks whether a recording succeeded.
---

# Capturing data on the robot

Robot time, battery and lab access are the scarce resources. A bad batch costs
a whole session, so the order below is not optional.

## 1. Before anything is recorded

- `ssh x3 'systemctl is-active x3_server'` and check the server args the
  capture needs are live: `ssh x3 'cat /etc/systemd/system/x3_server.service.d/*.conf | grep SERVER_ARGS'`.
  RGB-D captures need `--c1-recording`; OAK topics in a bag need `--oak-ros-publish`.
- Battery. Low battery has interrupted captures and copies more than once.
  Read it from the GUI or the server websocket. No bag contains `/voltage`.
- Disk: `ssh x3 'df -h ~ | tail -1'`. RGB-D runs are large (the first 45 or so came to 35 GB).
- Decide run names up front and tell the user the full list. Names drive the
  scoring: `empty-rN`, `stand-<d>m-rN`, `cross-<d>m-rN`, `approach-rN`,
  `startstop-rN`. State how many rounds (usually 3).

## 2. Short trial first

After ANY change to the recording path, or at the start of a session, record
one ~20 s trial (`check00`) and verify it at the payload level before the real
runs. "The file exists" is not a check. Look at:

- `audit.json`: `complete_pairs` at least `seconds * 8`, `failures` near zero,
  no `pair_intervals_over_500ms`.
- For rosbags: message counts against metadata, the largest gap per topic, and
  that payloads actually differ frame to frame (a cached buffer re-stamped
  looks healthy in `metadata.yaml`).

## 3. The user starts the real runs

When the user is part of the experiment (walking a lane, driving, holding a
board), do NOT launch the recording yourself. Give one copy-paste block for
their own SSH terminal, the expected duration, and what a good result line
looks like. Then wait.

```bash
# on the robot
bash ~/x3_ws/scripts/segpose_capture.sh cross-2.4m-r1 25
```

It prints `<name>: N complete RGB/depth pairs, M bad, max gap X ms`. The
recorder's own exit code is unreliable; the audit is what counts. The script
refuses to overwrite an existing run, so a retry needs the next `-rN`.

For plain rosbags use `record_bag.sh` on the robot or `fetch_bag.sh` from the
laptop. Use `ssh -tt`, not `-t`, or Ctrl+C never reaches `ros2 bag record`.

## 4. Between runs

When asked "am I good to continue", actually check the last run rather than
assuming: read its `audit.json`, confirm the scenario matches the name (an
`empty` run with a person in it is the common mistake), and say which run is
next. If someone else walked through the view, re-record under the next `-rN`.

## 5. Fetching and analysing

- Copies are large. Check battery before starting one; rsync with `--partial`
  so it can resume if the robot has to be shut down.
- Long analyses run on the robot under `nohup` with a log in `/tmp`.
  `bash scripts/segpose_progress.sh` on the laptop shows a progress window.
- Say plainly when the robot can be turned off: only once the copy is complete
  and nothing is still running there (`ssh x3 'pgrep -af "python3 src/c"'`).
- Results go in `artifacts/<topic>-<date>/` with a `RESULTS.md`. Raw captures
  stay untracked.

## Traps

- Captures made before `a14944b` (2026-09-28) have the squeezed NN preview:
  box bearings are 2.37x too small. Do not mix them with later ones.
- `c3_score.py` scores with the replay default config unless `config_c` is
  passed. Use `--tracker live` to score what the robot actually runs.
- For crossings use `c3_edge_test.py`, not raw `c3_score` ratios.
