---
id: A-40
title: "Empty-Frame Fast Path Skips Tracker Aging and Preserves a Stale Window Across a Depth Blackout"
status: Logged
domain: Architecture
roi_tier: High
source: august_improvement_ideas.md
source_id: "A-40"
session: "2. Architecture & Algorithmic Enhancements"
tags: [idea]
---

# A-40 — Empty-Frame Fast Path Skips Tracker Aging and Preserves a Stale Window Across a Depth Blackout

> [!note] Candidate — logged, not decided
> No implementation evidence found in the ROI analysis or in code comments.

- **Date Logged:** 2026-08-01 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Four lines on a path that already exists, bounds an otherwise unbounded ghost track and kills a saturated $2.5\text{ m/s}$ phantom on every blackout recovery)
- **Problem:** The empty-frame fast path at `src/velocity_estimator.py:433–439` clears `self._estimates`, sleeps, and `continue`s — **before** step 4 reaches `tracks = self._tracker.update(...)` at line 472. The broad `except Exception` at lines 662–663 takes the same shortcut. But `ObstacleTracker` ages tracks **only inside `update()`**: the increment-and-expire loop is at lines 63–66 and the `visible_count` decay at lines 98–100. During any cycle that takes the fast path, therefore, **no track ages, none expires, and no confirmation decays**.

  `max_age = 10` (line 49) is documented as bounding a track's life at $1.0\text{ s}$ of non-detection. Across a blackout it bounds nothing at all. A USB stall, a lens fully occluded, or the robot nose-to-wall — the OAK-D's near clip is $0.5\text{ m}$, and passive stereo returns $0.0$ on texture-free surfaces, so `np.any((raw >= 0.5) & (raw <= 4.0))` at line 434 goes false exactly when something is very close — leaves all five tracks alive indefinitely, each holding its pre-blackout `visible_count` (up to 10, hence $\text{conf} = 1.0$) and a `history_global` deque that is now seconds or minutes old.

  The damage lands on recovery. `update()` runs once; any surviving track whose stale global centroid is within `max_dist = 0.8\text{ m}` of a fresh detection matches, and line 93 appends the new sample **directly onto the ancient one**. `_build_window_features` then reads that pair as a $0.1\text{ s}$ step. The $\pm 0.25\text{ m}$ clamp at line 413 turns it into a saturated
  $$\frac{0.25\text{ m}}{0.1\text{ s}} = 2.5\text{ m/s}$$
  which is also the MLP output clip at line 569 — and because the deque holds ten samples, that saturated step stays inside the window for a **full $1.0\text{ s}$**, reported at $\text{conf} = 1.0$ because `visible_count` was never decayed. This is precisely the ghost-track contribution that Idea 201's `visible_count` decay was added to prevent; the fast path routes around the mechanism. Two smaller defects ride along: `self._prev_estimates` is not cleared at lines 435–436, so A-32/J-22's acceleration limiter clamps the post-blackout report against a velocity measured *before* the gap; and June's Idea 261 (time-based expiry) would not help, because it fixes the *rate* at which ages advance, not cycles where they do not advance at all.
- **Proposed Solution:** Make aging unconditional and refuse to bridge a time discontinuity.
  1. **Age on every cycle.** The cheapest correct form needs no new method — call `self._tracker.update([], [])` on the fast path. An empty detection list ages every track, matches none, decays every `visible_count`, and creates nothing, which is exactly the desired semantics. (If the allocation of two empty lists is objectionable, split lines 63–66 and 98–100 into `ObstacleTracker.age_only()` and call that instead.) Do the same in the `except` handler at lines 662–663 so a transient vision error cannot freeze the tracker either.
  2. **Clear the limiter state.** Add `self._prev_estimates = {}` beside `self._estimates = []` at lines 435–436, so no acceleration clamp spans the gap.
  3. **Guard the re-match.** Stamp each `history_global` entry with `time.monotonic()` and reject an append whose gap since the previous sample exceeds $2\Delta t = 0.2\text{ s}$, resetting that track's deque instead. This is the temporal twin of A-35's frame-change discontinuity guard, and it is what makes the fix robust to a blackout shorter than `max_age` (where the track legitimately survives but its window still must not be spliced across the gap).
  4. Composes with, and is independent of, June's Idea 261 and Idea 248: those tune *how long* a track lives; this ensures the clock runs at all.
- **Expected Benefit:** Converts an unbounded stale-track window into the $1.0\text{ s}$ bound `max_age` already promises, and removes a saturated $2.5\text{ m/s}$ phantom velocity — published at full confidence, and persisting for a full second — from every recovery out of a depth blackout, USB stall, or near-clip occlusion. Those are not rare events and they are not neutral ones: the near-clip case triggers precisely when a person has stepped closest to the robot, which is the worst possible moment to hand the TTC scaler a fabricated closing speed. Costs four lines on a path that already exists and nothing at all on the normal path.

---

## Provenance

Generated from `august_improvement_ideas.md` (heading `A-40`) by `scripts/split_ideas.py`. **Do not edit this file** — edit the source log and regenerate, or the two will drift.
