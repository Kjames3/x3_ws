#!/usr/bin/env python3
"""Patch velocity_estimator.py on the robot with a k=4 coherence-persistence gate.

The robot's velocity_estimator.py diverges from this checkout (1052 lines vs the
stale 687 here), so this edits the robot's file in place rather than shipping a
copy over it. Idempotent: re-running detects the marker and exits.

Why the gate. The straightness gate already zeroes a jittering static blob most
of the time, but a random walk transiently *looks* coherent, so ~14% of frames
leak a phantom velocity. Measured on a 300 s parked baseline against a static
object at 3.9 m (logs/ab_v3_20260816_211929.csv):

    k=1 (before)  424 hot frames  14.1%  max 1.04 m/s
    k=3           132             4.4%   max 1.04
    k=4            87             2.9%   max 0.79   <- chosen
    k=8            20             0.7%   max 0.65

The consumer trips on TTC < 3.0 s, which at 3.9 m needs > 1.30 m/s. k=4 puts the
worst phantom at 0.79, a 40% margin instead of 25%.

A real walker is coherent continuously, so persistence costs them only k/10 s
(0.4 s, 0.52 m at 1.3 m/s) -- provided the acceleration clamp does not then make
them ramp up from the gated zeros. See PREV_ESTIMATES note below.
"""
import re
import shutil
import sys
import time
from pathlib import Path

TARGET = Path("/home/jetson/x3_ws/src/velocity_estimator.py")
MARKER = "MIN_COHERENT_FRAMES"

CONST = '''
# --- Coherence persistence --------------------------------------------------------
# The straightness gate above is evaluated per frame, and a jittering centroid will
# occasionally trace a briefly-straight path by chance -- a random walk that looks
# coherent for two or three frames. Measured on a 300 s parked baseline against a
# static object at 3.9 m, that leaked a phantom velocity on 14% of frames, peaking
# at 1.04 m/s against a consumer trigger of 1.30 m/s at that range.
#
# Requiring the track to pass the coherence test on N CONSECUTIVE frames before its
# velocity is believed cuts the leak to 2.9% and the peak to 0.79 m/s. A genuine
# walker is coherent continuously, so this costs them only N/INFER_HZ of latency
# (0.4 s, or 0.52 m at 1.3 m/s) rather than suppressing them.
MIN_COHERENT_FRAMES = 4
'''

STREAK_BLOCK = '''
                # Advance the per-track coherence streak: +1 for every track that
                # passed the straightness test this frame, hard reset to 0 for every
                # other live track. Rebuilt from `tracks` each cycle so dead track ids
                # cannot accumulate.
                _eligible_ids = {t for t, _ in eligible_tracks}
                self._coherent_streak = {
                    tid: (self._coherent_streak.get(tid, 0) + 1 if tid in _eligible_ids else 0)
                    for tid in tracks
                }
                _gated_ids = set()

'''

GATE_BLOCK = '''
                        # Coherence persistence gate. Below the streak threshold we
                        # have no trustworthy velocity for this track yet, so report
                        # zero -- see MIN_COHERENT_FRAMES.
                        if self._coherent_streak.get(tid, 0) < MIN_COHERENT_FRAMES:
                            vx = 0.0
                            vy = 0.0
                            speed = 0.0
                            _gated_ids.add(tid)
'''


def main():
    src = TARGET.read_text()
    if MARKER in src:
        print(f"already patched ({MARKER} present) -- nothing to do")
        return 0

    orig = src

    # 1. constant, right after the straightness tuning block
    anchor = "STRAIGHTNESS_MIN_PATH_M = 0.12\n"
    if anchor not in src:
        print("FAIL: could not find STRAIGHTNESS_MIN_PATH_M anchor")
        return 1
    src = src.replace(anchor, anchor + CONST, 1)

    # 2. streak state in __init__, beside the other per-track dicts
    a2 = "        self._prev_estimates = {}\n"
    if src.count(a2) != 1:
        print(f"FAIL: expected 1 '_prev_estimates = {{}}' init, found {src.count(a2)}")
        return 1
    src = src.replace(
        a2,
        a2 + "        # tid -> consecutive frames passing the coherence test\n"
             "        self._coherent_streak = {}\n",
        1,
    )

    # 3. advance the streak once the per-track loop has classified every track
    a3 = "                if eligible_tracks and self.estimation_enabled and self._model is not None:\n"
    if src.count(a3) != 1:
        print(f"FAIL: expected 1 inference guard, found {src.count(a3)}")
        return 1
    src = src.replace(a3, STREAK_BLOCK + a3, 1)

    # 4. apply the gate to the prediction before it is emitted
    a4 = ("                        speed = float(np.sqrt(vx**2 + vy**2))\n")
    if src.count(a4) != 1:
        print(f"FAIL: expected 1 speed computation, found {src.count(a4)}")
        return 1
    src = src.replace(a4, a4 + GATE_BLOCK, 1)

    # 5. PREV_ESTIMATES: keep gated tracks out of the acceleration-clamp reference.
    #    The clamp limits frame-to-frame change to 3 m/s^2 (0.3 m/s per frame). If a
    #    gated zero were stored as the previous estimate, a walker clearing the gate
    #    would ramp 0 -> 1.3 m/s over four MORE frames, doubling the latency the gate
    #    was budgeted for. A gated frame means "no estimate yet", not "measured zero",
    #    so it must not anchor the clamp.
    a5 = "                self._prev_estimates = {est['id']: est for est in estimates}\n"
    if src.count(a5) != 1:
        print(f"FAIL: expected 1 _prev_estimates rebuild, found {src.count(a5)}")
        return 1
    src = src.replace(
        a5,
        "                self._prev_estimates = {est['id']: est for est in estimates\n"
        "                                       if est['id'] not in _gated_ids}\n",
        1,
    )

    compile(src, str(TARGET), "exec")   # refuse to write syntactically broken code

    bak = TARGET.with_suffix(f".py.bak-persist-{time.strftime('%Y%m%d-%H%M%S')}")
    shutil.copy2(TARGET, bak)
    TARGET.write_text(src)
    print(f"backup  {bak}")
    print(f"patched {TARGET}  ({len(orig.splitlines())} -> {len(src.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
