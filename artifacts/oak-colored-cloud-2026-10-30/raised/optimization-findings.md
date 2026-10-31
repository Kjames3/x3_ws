# Profiling and duplicate-work removal

Checkpoint commit:94be70f (offline work before this optimization). Not pushed.

The earlier repeated-frame~20ms measurement accidentally timed NPZ decompression:
accessing NpzFile[key] reloads/decompresses the array. That number was not the live
cloud kernel cost. cProfile found about half the measured time in numpy zip reads.
Previous findings remain historical; this report corrects their interpretation.

There was also real redundant work: refine called project solely for its mask,
then discarded its projected cloud and repeated depth correction and projection.
Extracted prepare_geometry so both paths share corrected depth and validity/mask;
refine projects only once. No filter threshold, point budget, color semantics,
calibration or sampling selection changed.

Jetson,10 preloaded recorded frames,one warmup sweep and5 measured sweeps;
50 runs per variant, alternating order:
- Original kernel: median6.967ms, p95 7.266ms.
- Optimized kernel: median3.596ms, p95 3.788ms.
- Geometry/confidence mask: median2.571ms, p95 2.725ms.
- Sampling: median0.465ms, p95 0.534ms.
All four output arrays matched exactly across10 frames. Two synthetic tests pass.
Full samples in optimized-profile.json. Reproduction tool:
`oak_color_cloud_profile.py DATASET --before refine-before-optimization.py`.
The baseline module imports the refactored project function, whose algorithm is
unchanged; baseline includes its discarded projection and repeated correction.

Uncached RGB rectification separately measured median5.836ms on10 frames.
Do not add component p95 values and call that measured end-to-end latency.
The cloud kernel meets its proposed15ms target, but full live acceptance still
requires capture/rectification, timestamped placement, serialization, network,
browser performance and matched baseline/on trials. Display quality and perimeter
registration limitations remain. No runtime deployment or camera/service changes.
