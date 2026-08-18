# What actually blocks the x3_server event loop

**Date:** 2026-08-17 · **Robot:** jetson (Orin Nano) · Follow-up to
`oak_process_isolation_result.md`, which ruled out the OAK driver.

## Answer

Two things, both inside **`motion_loop`** (which is 57.5% of MainThread active time):

| # | Culprit | Cost | Shape |
|---|---|---|---|
| 1 | `_is_standalone_test_running()` — full `psutil.process_iter(['cmdline'])` over ~251 processes, `server_x3.py:2343`, called from `motion_loop:2421` | **~228–261 ms/s**, 34–48% of MainThread ACTIVE time | one burst, exactly 1×/s |
| 2 | `cbf_filter.filter_velocity` (SLSQP) | ~124–179 ms/s, 18–33% of active | spread over the 30 Hz loop |

Culprit 1 is the ~150 ms tail spike. Culprit 2 is the raised p50 floor.
Together they are ~52% of everything the event loop actively does.

## How it was found

`py-spy record --idle --threads --format raw` on the live process. **`--idle` is
essential** — a thread blocked in a syscall has released the GIL and is invisible
to a default py-spy run, so a blocking-I/O stall would not appear at all.

Profiled twice: once idle, once with a WebSocket client connected (matching the
conditions the stalls were originally measured under). Both agree.

## The interesting part: a 37 ms scan costs 260 ms

`scripts/time_proc_scan.py` times the identical scan standalone on the same robot
under the same system load:

    process count: 251
    median 37.3 ms   min 34.5 ms   max 50.5 ms

So the scan is only ~37 ms of actual work, but costs ~230–260 ms of wall time
inside `x3_server`. The control is clean — same machine, same load, same moment;
the only difference is running inside the 9-thread server process.

The mechanism is GIL contention, with the scan as the **victim** rather than the
cause: walking 251 `/proc` entries is ~750 syscalls, and every one releases the
GIL and must re-acquire it against 8 competing threads (oakd, `_inference_loop`,
rclpy `spin`, HTTP `serve_forever`, 4 others). At CPython's default 5 ms switch
interval, re-acquisition dominates. A syscall-dense loop is the worst possible
thing to run on the event-loop thread of a heavily threaded process.

Note this does **not** resurrect the "OAK driver holds the GIL" theory — moving the
OAK driver out of process changed the telemetry rate by nothing, because the
remaining 7 threads supply plenty of contention on their own.

## Suggested fix (not applied)

`_is_standalone_test_running()` gates `drive.move()`, so it must stay correct.
Cheapest safe change: **keep the 1 Hz cache but refresh it from a dedicated
background task via `run_in_executor`**, and have `motion_loop` only read the
cached bool. The loop then pays ~0, and the scan's syscalls happen on a worker
thread where blocking is harmless.

Optionally also make the scan itself ~5× cheaper by reading `/proc/*/cmdline`
directly instead of building psutil `Process` objects (`process_iter` also calls
`create_time()` per process for its identity check).

Rejected: simply lowering the scan frequency — that trades a 1 Hz hiccup for a
rarer but equally large one, and the loop is the wrong place for the work either way.

## Caveat

py-spy's own overhead depressed the telemetry rate during profiling (11.8 → 10.6 Hz),
so the absolute ms/s figures are slightly inflated. The **proportions** between
frames — which is what the diagnosis rests on — are unaffected.
