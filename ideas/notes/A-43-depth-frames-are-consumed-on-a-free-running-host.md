---
id: A-43
title: "Depth Frames Are Consumed on a Free-Running Host Clock With No Capture Timestamp — the Feature Δt Is Assumed, Never Measured"
status: Logged
domain: Sensor Fusion
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-43"
session: "4. Sensor Fusion & Hardware Integration"
tags: [idea]
---

# A-43 — Depth Frames Are Consumed on a Free-Running Host Clock With No Capture Timestamp — the Feature Δt Is Assumed, Never Measured

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (One field carried alongside the frame plus a sequence check, and it is the precondition without which June's Idea 33 measures the wrong clock and Idea A-39's smoother is biased rather than merely noisy)
- **Problem:** Every displacement feature the MLP consumes is a bare difference of two positions — `dx = rx - rx_prev`, `dy = ry - ry_prev` (`src/velocity_estimator.py:410–411`) — with no division by time anywhere in the file. The units only close because the model was trained at a fixed cadence: `WINDOW_SIZE = 10  # matches training T=10` and `INFER_HZ = 10  # target inference rate` (lines 33–34). A $0.1\text{ s}$ spacing is baked into the weights, and **nothing in the serving path establishes it.**

  The loop's cadence is a host sleep — `time.sleep(max(0.001, dt - elapsed))` at lines 666–668 — while the samples it differences are *captures*. `_inference_loop` calls `cam_src.get_raw_depth_frame()` (line 430), which returns whatever the OAK worker last stored (`src/oakd_driver.py:593–595`). That worker is handed `inDepth.getFrame()` at line 332 and **discards `inDepth.getTimestamp()`**, the device-side stamp DepthAI attaches to every `ImgFrame`. The only temporal record kept is `self._last_depth_time = time.monotonic()` at line 397 — a host *arrival* time, exposed through `get_depth_frame_age()` (lines 597–600), which the estimator never calls and could not use safely if it did: reading it is a second, separate lock acquisition, so the frame may have been replaced in between.

  The estimator therefore appends exactly one history sample per host cycle regardless of how many captures actually occurred. Three distinct errors follow.
  1. **Rate-dependent scale error.** With an achieved depth rate $f_d$, the mean number of new captures per $0.1\text{ s}$ poll is $f_d / 10$, so the mean true interval between differenced samples is $\max(1/f_d,\, 0.1)$ — and $f_d$ is not the configured rate. `mono_fps=80` is requested (`src/server_x3.py:1074`), but `_run`'s inner loop must also run two `getCvFrame()` copies, the IMU drain, the $85 \times 6300$ NN decode and Idea A-44's $3.6\text{ MB}$ conversion inside a $12.5\text{ ms}$ budget, with `maxSize=1, blocking=False` silently dropping whatever it cannot keep up with. The rate is host- and scene-dependent, and it is invisible to the estimator.
  2. **Duplicate consumption at low rates.** When $f_d < 10$ — the regime the driver's own USB2 warning contemplates ("economy mode ON: mono L/R not streamed so depth + detection keep the bandwidth", `src/oakd_driver.py:297–299`) — some polls read the *same* array twice. That injects an exact $dx = 0$ into the window and a double-size step into the next. At $f_d = 6$, 40% of samples are duplicates, and a pedestrian walking at $1.4\text{ m/s}$ yields a `dx` sequence of roughly $[0,\, 0.28,\, 0,\, 0.14,\, 0.28, \ldots]$ instead of a uniform $0.14$. The induced spread is $\approx 0.12\text{ m}$ — **60% of the entire $0.198$ training spread of the `dx` channel**, and nearly $3\times$ the $0.042\text{ m}$ sensor-noise term Idea A-39 is costed against. Unlike sensor noise it is structured and low-frequency, so no smoother removes it. It is also worse than uninformative: a repeated frame is *no new information*, and the pipeline records it as *evidence of no motion*.
  3. **Phase jitter, always on.** Even at high $f_d$, the consumed frame's age at poll time is uniform on $[0, 1/f_d)$ plus USB and host transport, so consecutive true intervals differ by up to $1/f_d$. At the configured $80\text{ fps}$ that is $\pm 12.5\text{ ms}$ — a $\pm 12.5\%$ multiplicative error on every displacement feature, and a floor, not an estimate, since the achieved rate is lower.

  None of this is covered by what is already logged. June's Idea 33 proposes scaling displacements by `dt_actual` but measures it as host loop wall-time, which is $\approx 0.1\text{ s}$ *by construction even when the frame did not change* — it corrects none of the three. Idea 230's staleness gate fires only above $200\text{ ms}$ and is blind to everything beneath it. And Idea A-39's Savitzky–Golay operator assumes a **uniformly sampled** grid; applied to a window whose true spacing is $[0.1,\, 0,\, 0.2,\, 0.1, \ldots]$ its polynomial fit is not merely noisier but biased.
- **Proposed Solution:** Carry the capture time with the frame and make the window's time axis explicit.
  1. **Stamp at the source.** In `_run` line 332, take `ts = inDepth.getTimestamp().total_seconds()` — the device clock, stamped in hardware at capture and therefore immune to USB and host scheduling — and store it atomically with the frame inside `_process_depth`'s existing lock. Expose one accessor returning both, `get_raw_depth_frame_stamped() -> (frame, ts, seq)`, so there is no read race between a frame and its time. Give `ROS2Bridge` and `AstraCamera` the same accessor (falling back to the ROS header stamp, or `time.monotonic()` at write time where no device stamp exists) so the estimator has a single contract across all three depth sources.
  2. **Skip, don't duplicate.** In `_inference_loop`, if `seq` is unchanged from the previous cycle, do not extract centroids and do not append to any track's `history_global` — but *do* still age the tracker, which is precisely Idea A-40's `self._tracker.update([], [])`, and the reason these two should land together — then re-publish the previous estimates. Without A-40 this fix would create a new stale-track path; with it, the two share one mechanism.
  3. **Normalise on the true interval.** Stamp each `history_global` entry with its capture time (Idea A-40 step 3 already asks for a timestamp on that deque for a different reason — one field serves both) and scale each displacement onto the training grid:
     $$dx_i = \big(r_{x,i} - r_{x,i-1}\big) \cdot \frac{0.1}{t_i - t_{i-1}}$$
     applying the $\pm 0.25\text{ m}$ clamp at line 413 **after** the scaling, so the clamp still bounds the value the model actually sees. Reject a pair outright when $t_i - t_{i-1} > 0.25\text{ s}$ rather than extrapolating across a gap.
  4. **Surface the achieved rate.** `get_depth_fps()` already exists and is already computed on a $1\text{ s}$ window (`src/oakd_driver.py:602–605`, fed at lines 337–339); publish it beside the estimates so a session running at $f_d < 15$ is visible in the readout instead of silently degrading the velocity scale.
- **Expected Benefit:** Makes the $0.1\text{ s}$ the model was trained on an *enforced property* of the served window rather than an assumption nothing checks. Eliminates the duplicate-frame artefact entirely — $\approx 0.12\text{ m}$ of structured, unsmoothable spread on the `dx` channel, 60% of its training range, whenever the depth rate falls below the poll rate, which is the driver's own documented USB2 configuration — and removes a $\pm 12.5\%$ always-on multiplicative error on every displacement feature at the configured $80\text{ fps}$. It is also a precondition for two ideas already logged: June's Idea 33 cannot work while it measures the host clock instead of the capture clock, and Idea A-39's zero-phase smoother is only unbiased on the uniform grid this establishes.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-43`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
