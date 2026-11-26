# server_x3.py CPU hotspots and latency (robot capture 2026-11-26, AGX Orin, Nav2 off)

Sources: `artifacts/overnight-perf-2026-11-26/` (py-spy raw `ovn_server_threads.txt`, `ovn_server_native.txt`,
`gdb_all_threads_bt.txt`, `native_dump_server.txt`, `top_threads_sorted.txt`, `journal_x3_server.txt`, `hz__*.txt`).
Code refs are at the commit the robot runs (ac47d9b + local edits that don't touch these files).

## How the numbers were calibrated (read first)

- The py-spy TIDs match the LWPs in `top`, so **`top` per-thread %CPU is the ground truth** and py-spy only splits it
  across lines. The brief's formula (samples / (40 s × 50 Hz) = samples/2000) **overstates by about 2×**. Examples:
  rclpy spin is 798/2000 = 40% but 20.3% in top; `_capture_loop` is 30% vs 14.3%; oak_floor 23% vs 13%.
  The cause is py-spy counting blocking calls as active: `executors.py:682` is `wait_set.wait()` (393 samples),
  `concurrent/futures/thread.py:81` is `work_queue.get()` (448), plus `time.sleep` in `c3_live.py:292` /
  `foot_diagnostic.py:432` and `qDepth.get()` in `oakd_driver.py:619`.
- In the end, about **1 py-spy sample ≈ 0.025–0.03% of a core** for compute-only threads. All savings below are
  anchored to top.
- Process total in top: **521%**. The 11 OpenBLAS threads add up to **~355%** (27.9–35.2% each). All Python threads
  together come to about 115–160%.
- **A GUI client was connected during the capture.** `websockets.broadcast`, `_encode_frame` and the multi-camera
  `cv2.imencode` jobs exist only because of it, so those savings depend on a client being there. They are not true idle.

---

## Ranked by gain / effort

| # | Fix | CPU saved (est.) | Effort | Latency effect |
|---|-----|------------------|--------|----------------|
| 1 | `OPENBLAS_NUM_THREADS=1` for x3_server | **~3.0–3.5 cores** | 1 line + unit reinstall | Big: removes 11 spinning threads competing with GIL holders |
| 2 | Skip `oled_loop` work when the OLED is absent | ~0 CPU; removes 0.15 s loop stalls | 1 line | Removes the stall every 5 s |
| 3 | Replace the per-second psutil scan + `Thread.start()` | ~3.5% + stalls | small | Removes the 0.22 s stall at `Thread.start` |
| 4 | Arducam capture: `grab()` always, decode only at consumer rate | ~10% | small | none / slightly fresher |
| 5 | Left side cam: same + lazy rotate | ~3–4% | small | none |
| 6 | oak_floor_obstacles: vectorise memory, 3×3 normal equations, 1-D unique | ~6–8% (after #1) | medium | — |
| 7 | TRT side detector: blocking-sync context, person-only postprocess | ~3–5% | small | — |
| 8 | rclpy executor: fewer wakeups/entities (drop `/odom_raw` sub, drop TransformListener) | ~5–10% | medium | Nav2 /cmd_vel→CBF gate shares this thread |
| 9 | Executor job churn: merge per-frame jobs | ~3–8% (client only) | small | — |
| 10 | `os.path.exists(CBF_OFF_FILE)` per command | ~0.7% | trivial | — |

---

## 1. OpenBLAS pool (Task 1)

### Evidence
- `gdb_all_threads_bt.txt`: all 11 extra threads (LWP 71411–71421) are in
  `blas_thread_server()` from **numpy's bundled** `numpy.libs/libopenblas64_p-r0-17488984.3.23.dev.so`
  (build: `NEOVERSEN1 MAX_THREADS=80 USE_OPENMP=0`, per `robot_git_unit.txt`). That's 12 cores, so 11 helper threads.
- Two other OpenBLAS copies are loaded but started no pool threads that gdb can see: opencv's
  `libopenblas-r0…3.3.so` and the system `openblas-pthread 0.3.20` used by scipy (CBF `linprog`, c3
  `linear_sum_assignment`). Setting the env var also covers them.
- In both the gdb snapshot and `native_dump_server.txt` (`wchan=futex_wait_queue_me`) all 11 threads were parked
  in `pthread_cond_wait`, yet each shows ~30% CPU in top. That fits a duty cycle: a threaded BLAS call wakes the pool,
  the workers spin-yield for OpenBLAS's `THREAD_TIMEOUT` waiting for the next job, then park on the condvar. With a
  waker at ~5 Hz the pool spends roughly a third of each period spinning. This report does not pin down the timeout
  value (it is a tick count set by `OPENBLAS_THREAD_TIMEOUT`/build). The observed duty cycle is what counts.

### Who wakes the pool
OpenBLAS threads a call only above a size threshold: gemm runs single-threaded while M·N·K is small, and gemv
threads only when M·N is a few thousand or more. Tiny ops never wake the pool. That covers the 3×3 transforms, the
4×4 Kalman in `c3_person_tracker.py:270-309`, CBF rows in `cbf_filter.py:66-103/370/410`, `traj_predictor` GRU,
`side_person_rays`, and `server_x3.py:1832` (64 ToF points).

- **Confirmed** (native py-spy frames, `ovn_server_native.txt` lines 322/347):
  - `oak_floor_obstacles.py:201` `r = z - A @ coef`, where A is N×3 with N = floor points (thousands):
    `cblas_dgemv64_`. Also the biggest Python-visible line in that thread (106 samples). The thread is
    waiting on the pool handshake for a trivial matvec.
  - `oak_floor_obstacles.py:128` `pts @ R_oc.T` (≤12000×3 @ 3×3, float32 pts upcast to float64): `dgemm_tn`.
- **Candidates by size** (same thread, same rate, LAPACK internally threaded):
  - `oak_floor_obstacles.py:199,205`: `np.linalg.lstsq(A, z)` → dgelsd on N×3, three times per step.
  - Lines 129 and 156: the other N×3 @ 3×3 products.
- All of these run on **one thread at 5 Hz** (`/oak/obstacles` 4.5–4.9 Hz in `hz__oak_obstacles.txt`), about 6
  threaded BLAS/LAPACK calls per step.
- Other possible large callers: `velocity_estimator.py:908,1013` (T×2 @ 2×2, small T). Nothing else in the runtime
  has thousand-row matmuls.
- Only 4 of 1731 native samples landed in BLAS compute. **Nothing in the server benefits from multi-threaded BLAS.**
  The work per call is microseconds, and threading it costs a wake plus a spin.

### Fix
- Robust option: put it in the unit, since systemd applies it before Python starts.
  - `deploy/systemd/x3_server.service`, next to `PYTHONUNBUFFERED`: `Environment=OPENBLAS_NUM_THREADS=1`
  - The robot runs the installed `/etc/systemd/system/x3_server.service` copy (see `robot_git_unit.txt`), so this
    needs `deploy/install_service.sh` or a drop-in, then `daemon-reload` and a restart.
- Belt and braces for manual runs: at the very top of `server_x3.py`, **before line 29 `import numpy as np`**
  (`os` is not imported until line 33, so add `import os` there):
  `os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')`.
  - Precedent: `oak_geometry_worker.py:61` already does this for its subprocess.
  - `threadpoolctl.threadpool_limits(1, 'blas')` after import is an alternative, but the env var is simpler and also
    covers scipy's separate libopenblas.
- Children (`point_to_point_test.py`, `ab_comparison_test.py`) inherit the variable, which is harmless.
- Do **not** add a blanket `OMP_NUM_THREADS=1` without checking first. torch is loaded (velocity MLP) and gdb shows no
  gomp worker threads today, so there is no evidence it is needed.
- Expected result: ~355% drops to ~0. Plan on ~3–3.5 cores saved, keeping some margin for thread-handoff time the
  oak_floor thread will now spend computing inline (microseconds).
- Verify with `top -H -p $(pgrep -f server_x3.py)`: no `blas_thread_server` threads should be busy, and
  `oak_floor_obstacles.py:201` should vanish from py-spy.

---

## 2. Python hotspots (Task 2)

Thread CPU comes from top (`top_threads_sorted.txt`). Line splits come from py-spy with blocking leaves removed.

### 2a. rclpy spin thread: 20.3% (LWP 71266)
- 798 samples. 393 sit at `executors.py:682 wait_set.wait()` (blocking). Of the ~405 that remain:
  - ~230 rebuild the wait set every `spin_once`, in Humble's Python executor: `_wait_for_ready_callbacks` lines
    586–704, including `qos_event.py:90-123` `add_to_wait_set/__enter__/is_ready` (QoS-event waitables).
  - 94 are deserialisation (`_take_subscription`, executors.py:400).
  - About 55 are actual callbacks: `_odom_cb` 17, `_person_candidates_cb` 16, `_people_cloud_tick` 11, `_scan_cb` 6,
    tf listener 3.
- In other words, **~75% of this thread's real CPU is executor overhead per wakeup**, and it scales with
  (#wakeups/s) × (#entities in the wait set).
- Wakeup sources:
  - `/odom` (49 Hz) **and** `/odom_raw`, subscribed at `server_x3.py:1149` for a fallback that is used only when
    `/odom` is silent.
  - `/tf` via the full `TransformListener` (`server_x3.py:1053`). Its only users are `oak_floor_obstacles.py:98`
    and `server_x3.py:1756`.
  - `/scan` 10 Hz, three `/livox/person_candidates*` topics, `/cmd_vel`, and 10 Hz + 2 Hz timers
    (`server_x3.py:1166-1167`).
- Each of the 15 subscriptions and ~14 publishers carries default QoS-event handlers, and these get re-added to the
  wait set on every spin.
- Cheapest fixes, in order:
  1. Drop or lazily create the `/odom_raw` subscription. It is only a fallback, so create it when `/odom` goes stale.
     That removes ~1/3 of odom wakeups.
  2. Replace the `TransformListener` on `/tf`. The bridge already keeps `_static_chains` (`server_x3.py:1126,1625`) and the `/odom`
     pose, which is enough to build odom←camera for oak_floor. That removes every `/tf` wakeup and TFMessage decode.
     Measure `/tf` Hz first: it wasn't captured.
  3. Create publishers/subscriptions with
     `event_callbacks=PublisherEventCallbacks(use_default_callbacks=False)` / `SubscriptionEventCallbacks(...)` so
     QoS-event waitables leave the wait set. Check the Humble signature on the robot before relying on this.
- Estimated saving: 5–10% of a core. The gain also reduces GIL pressure on the Nav2 `/cmd_vel` → CBF gate, which runs
  on this thread (`_gate_cmd_vel_cb`, `server_x3.py:1925`).

### 2b. Arducam right-camera capture: 14.3% (LWP 71281)
- `drivers_x3.py:285` `self._cap.read()`: 590 samples, all of them at this line.
- The camera is opened at **1280×800 MJPG** (`server_x3.py:3034`, `drivers_x3.py:271-276`). `read()` = grab +
  **MJPEG decode to BGR** on every sensor frame, whether or not anyone wants it.
- The P7 gate (`_has_clients`) only skips the lock+copy, and `server_x3.py:5325` forces it True for side-person
  detection anyway.
- Consumers need **5 Hz** (`SIDE_PERSON_HZ`, `server_x3.py:483`), plus GUI preview when a client watches.
- Fix:
  - Loop on `cap.grab()`. Call `cap.retrieve()` only when ≥ 1/consumer_rate has passed since the last decode, or
    decode lazily under the lock inside `get_frame_stamped`.
  - Or `cap.set(cv2.CAP_PROP_FPS, 10)`.
- Estimated saving: ~10% (decode drops from ~30 fps to 5–10 fps).

### 2c. oak_floor_obstacles: 13.0% (LWP 71292), 5 Hz
456 samples. Per step:
- **Plane fit** `floor_relative_height`, `oak_floor_obstacles.py:192-211`: ~150 samples (~4%).
  - Line 201 is the BLAS-pool matvec (§1). Fix #1 already cuts most of this.
  - Then replace `lstsq` ×3 with 3×3 normal equations (`AtA = A.T@A`, `np.linalg.solve`) or a closed form. Use
    `np.partition` instead of `np.median` for the trim threshold.
- **Voxel memory** `FloorObstacleMemory.update`, `oak_floor_obstacles.py:49-52`: ~130 samples (~3.5%).
  - A Python `for x, y, z in pts_odom` loop over every candidate point, with a dict write per point, 5×/s.
  - Fix: run `np.unique` on packed int64 keys first and loop only over the unique cells (or keep the memory as numpy
    arrays plus a key index). Lines 55-64 also rebuild `arr` from a list of lists every step.
- **`np.unique(keys, axis=0)`**, line 151: ~55 samples (~1.5%). `axis=0` goes through a structured-view lexsort.
  Pack the three int keys into one int64 (`(kx<<42)|(ky<<21)|kz` with offsets) and use 1-D `np.unique`, which is
  several times faster.
- `visible()` line 165 (~29) and `publish` (`rclpy.clock.now` + PointCloud2 build) are minor.
- Optional: `np.savez('/tmp/oak_floor_debug.npz')` every 10 s (line 137) is debug I/O and could sit behind a flag.
- Estimated saving: 6–8% of a core on top of #1.

### 2d. Default asyncio executor threads: 42.5% in total (asyncio_0..4, LWPs 71300/71440/71467/71484/75391)
1134 samples:
- **`_side_detect`** (`server_x3.py:5264`, called at 5377): 327 samples.
  - `_preprocess`, `trt_detector.py:179` `cv2.dnn.blobFromImage` on 1280×800 (right) / side frames: ~160.
  - TRT execute/sync, `trt_detector.py:269-274`: ~110.
  - `_postprocess` argmax over [A, C] at `trt_detector.py:215`: ~45.
  - Rate: up to 3 cameras × 5 Hz.
  - Fixes:
    - (a) `cuda.Device(0).make_context(flags=cuda.ctx_flags.SCHED_BLOCKING_SYNC)` at `trt_detector.py:117`. The
      default SCHED_AUTO spin-waits in `stream.synchronize()` on an integrated GPU.
    - (b) For the side path only, `person` (class 0) is wanted, so take `cls_sc[:, 0]` instead of argmax over all
      classes.
    - (c) Let the camera (or the OAK `REAR_DETECT_WH` output already used for rear) deliver the network size, so
      blobFromImage does not resize 1280×800.
  - Estimated saving: 3–5%.
- **`_encode_frame`** (`server_x3.py:5506`, 130) and multi-camera `cv2.imencode` via `asyncio.to_thread`
  (`server_x3.py:5217`, which py-spy shows as a bare `thread.py:58` leaf: 179): **these exist only because a GUI
  client is connected.**
  - Encodes run at up to `_sp()['fps']` per role. The loop polls every 5 ms (`server_x3.py:5231`).
  - The main camera loop also copies a full 1280×800×3 frame every 50 ms with `camera.get_frame`
    (`server_x3.py:5463`) even when `cam_targets` is empty.
  - Fixes: call `_camera_targets()` before `get_frame`, and merge get_frame+resize+encode into one executor job.
- **`thread.py:81` `work_queue.get`** (448 samples) is py-spy counting the idle wait. Top still attributes real CPU to
  these threads beyond the detect/encode work. Many small `to_thread` submissions per second spread over 5 workers
  pay futex wake plus GIL re-acquire each time.
  - Fix: batch the per-role encodes into one job per tick. Optionally give a dedicated 2-thread executor to the
    camera paths.
- Estimated saving: 3–8%, client-dependent.

### 2e. MainThread (event loop): 12.0% (LWP 71081), 247 samples
- `websockets.broadcast` writes: 51 (client-dependent).
- CBF `filter_velocity` (`cbf_filter.py:182/242/339`, `_project:66`): ~64, i.e. ~0.5 ms per call at the 30 Hz
  heartbeat. Fine as it is.
- **`_move_impl` `os.path.exists(CBF_OFF_FILE)`** (`server_x3.py:2067`): 27 samples. That is a stat syscall plus
  Python overhead on every command. Cache it with a 1 s TTL like `_lidar_cbf_on()` (`server_x3.py:504`). Saves ~0.7%.
- `side_person_loop` `get_frame_stamped` (`drivers_x3.py:378`) copies a 3 MB frame every due tick. Fine at 5 Hz.

### 2f. psutil scan `_scan_standalone_tests` (`server_x3.py:5824-5860`): ~3.5% plus event-loop stalls
- 145 samples across short-lived `standalone_scan` threads, with ~260 `/proc` reads via `psutil.process_iter`.
- **This is not a regression to the inline scan.** Commit 2b65bde moved it to a worker thread, ~1 Hz. The design
  that remains still costs something:
  - It constructs psutil objects for every process each second, which holds the GIL in Python.
  - It **starts a brand-new `threading.Thread` from the event loop every second.** Under GIL contention,
    `Thread.start()` blocks in `self._started.wait()`. The journal shows
    `event loop blocked 0.22 s (stall #11)` with exactly that stack
    (`motion_loop` → `_is_standalone_test_running` → `threading.py:940 _started.wait`).
- It is called at 30 Hz from `motion_loop` (`server_x3.py:5924`) and from the Nav2 gate (`server_x3.py:1927`).
- Fixes:
  - Best: the server already knows its own children (`_p2p_proc`, `_ab_test_proc`). For hand-run tools, have them
    touch a `/tmp/x3_standalone_test.<pid>` flag (like `LIDAR_PERSONS_CBF_FILE`) and check for it with `os.scandir`.
  - Minimum: one long-lived daemon thread that sleeps 2 s and reads `/proc/*/cmdline` raw (no psutil), so no thread
    is created from the loop.

### 2g. `oled_loop` (`server_x3.py:6067-6083`)
- The OLED is unplugged on purpose, but `oled` is an `OLEDDisplay` whose `_device is None` (`drivers_x3.py:432-453`).
  So `if oled:` is True, and every 5 s the loop runs `subprocess.run(['iwgetid'])` (`server_x3.py:3393`) **on the
  event loop** to feed a `show()` that returns immediately.
- The journal shows `event loop blocked 0.15 s (stall #13)` inside `_get_ssid` → `subprocess.run` (forking a 1.6 GB
  RSS process).
- Fix: `if oled and oled._device is not None:` (or have `OLEDDisplay.__bool__` reflect the device), and wrap the
  remaining calls in `asyncio.to_thread`.

### 2h. Smaller items, each ≤ 4%
- **Left side camera** `_left_capture` (`server_x3.py:5171` read: 96, `5176` `cv2.rotate`: 28): ~3%.
  - It reads and rotates every frame at sensor rate for a 5 Hz consumer plus the GUI.
  - Fix: same grab/retrieve pattern as 2b, and rotate only the frames that get used.
- `oakd` thread (4%):
  - NN decode `_read_layer` (`oakd_driver.py:1303`, fp16→fp32 astype), and `_decode` amin/amax.
    `_run:619` is the blocking `qDepth.get()`.
  - Fine as it is.
- `velocity_estimator._inference_loop` (4.3%): `_extract_depth_centroids`/`_height_band_mask`
  (`velocity_estimator.py:467,584-647`). Fine.
- `c3-live` and `foot-diagnostic`: about 1% each, mostly `time.sleep` loops at 20 ms. `foot_diagnostic` is a shadow
  evaluator. It could default off (`--no-foot-diagnostic`, `server_x3.py:3226`) when nobody is reviewing it.

---

## 3. Latency: camera → detection → CBF → /cmd_vel_safe (Task 3)

### What actually reaches the CBF
- **Static obstacles**: `/scan` (`_scan_cb`, `server_x3.py:1285-1297` → `_latest_obstacles`) plus ToF obstacles.
- `/oak/obstacles` and `/livox/obstacles` go **only to the Nav2 costmap**, not the CBF.
- **People**: `_people_inputs()` (`server_x3.py:1955-1990`) calls `velocity_estimator.get_estimates()` (OAK path).
  Lidar person tracks are added only while `/tmp/x3_lidar_persons_cbf` is on (`_lidar_cbf_on`, `server_x3.py:504`,
  which reads the file at 1 Hz). Memory says that feed is still off.
- So the camera→CBF path is OAK → oakd thread → VelocityEstimator → person memory → CBF.

### Hops, with the waiting each one adds

| Hop | Where | Mechanism | Added latency |
|-----|-------|-----------|---------------|
| OAK capture → host | device + XLink | pipeline | dominant: median camera delay ~267 ms measured earlier (memory: x3_server thread profile). Not re-measured here |
| oakd thread | `oakd_driver.py:619` `qDepth.get()` | event-driven (good) | NN decode ~ms |
| VelocityEstimator | `velocity_estimator.py:1186-1388`, `INFER_HZ=10` | **polled**: `time.sleep(dt - elapsed)` at lines 1203/1208/1220/1255/1270/1311/1388 | **+0–100 ms, ~50 ms average**. Fix: block on a "new depth frame" event/Condition from the oakd thread instead of sleeping |
| CBF consumer, teleop/heartbeat | `motion_loop`, `server_x3.py:5883-5952`, `asyncio.sleep(0.033)` | polled 30 Hz on the event loop | +0–33 ms nominal. **Measured: `/cmd_vel_safe` 19–21 Hz (not 30) with gaps up to 0.20 s** (`hz__cmd_vel_safe.txt`) |
| CBF consumer, Nav2 | `_gate_cmd_vel_cb`, `server_x3.py:1925`, rclpy spin thread | event-driven per /cmd_vel | executor overhead (2a) plus GIL waits |
| CBF compute | `cbf_filter.filter_twist` | ~0.5 ms | negligible |
| Event-loop stalls | `loop_stall_probe` | GIL starvation / blocking calls | **6 stalls of 0.15–0.79 s in ~75 s of journal** |

The six stalls in detail:
- #11 0.22 s: `Thread.start` in `_is_standalone_test_running`.
- #13 0.15 s: `_get_ssid` subprocess.
- #16 0.21 s: `websockets.broadcast` of a camera frame.
- #12 0.19 s and #14 0.79 s: in `selector.poll` or plain Python lines. Those stacks are captured after the fact, so
  they are only suggestive, but they look like **GIL starvation** (about 20 Python threads plus 11 spinning BLAS
  threads competing for cores).
- `server_x3.py:783` notes that `sys.setswitchinterval` was tried and did not help.

The side-camera path (5 Hz due-timer, `server_x3.py:5356`: +0–200 ms; `side_person_loop` also polls every 20 ms) →
`/side/*/person_rays` → livox tracker process → `/livox/person_candidates_side` reaches the CBF only when the lidar
CBF feed flag is on.

### Latency fixes, ranked
1. Fix #1 (BLAS) plus 2f and 2g: removes the identified loop stalls and most of the CPU competition. This is the
   biggest tail-latency win, because a 0.2–0.8 s stall freezes the heartbeat and the CBF's view of people.
2. VelocityEstimator: wait on a new-frame event instead of a 10 Hz sleep. Saves ~50 ms on average and up to 100 ms.
3. Run the 30 Hz `motion_loop` heartbeat off the event loop (its own thread or a rclpy timer), so GUI and broadcast
   work can't delay `/cmd_vel_safe`. Or at least wait on the motion queue with a 33 ms timeout instead of sleeping, so
   teleop commands go out immediately: `await asyncio.wait_for(motion_queue.get(), 0.033)`.
4. 2a (rclpy wakeups): shorter Nav2 → `/cmd_vel_safe` path under load.

## Caveats
- One 40 s capture, idle robot, GUI client connected, Nav2 off. Driving with Nav2 adds `/cmd_vel` at ~20 Hz through
  the gate on the spin thread.
- `/tf` rate and `/odom_raw` rate were not captured, so the 2a savings are estimates.
- The OpenBLAS spin timeout value was not determined. The fix doesn't depend on it.
