# Neutral-boundary refinement

Implemented offline only in oak_color_cloud_refine.py. All valid supported
0.4–3m depths remain eligible for sampling; the conservative12px discontinuity
mask controls RGB eligibility, not geometry retention. Uncertain points are
neutral gray. This does not correct stereo boundary fattening or establish that
neutral geometry is accurate. Spatial grid sampling stays below6000 points;
invalid tile centers fall back to a nearby valid point within that tile.

Source frame result:5504 points,1858 neutral. The geometry no longer has the
large boundary holes caused by the prior deletion filter, and row-major sampling
artifacts are removed. Orange trim still loses its color; printed detail remains
sparse. Neutral points can look like object material: any future UI needs an
explicit legend/toggle for uncertain color. Thin objects can remain undersampled.

Two synthetic tests pass: bounded unique spatial coverage including separated
sparse regions; uncertain geometry retained with neutral RGB, correct BGR order,
and exactly-once depth correction. Tests do not certify real calibration.

An initial sorted sampler evaluated on10 recorded frames was too costly (~96ms
p95) and rejected. Final center-grid sampler measured on one repeated recorded
frame on Jetson: warm kernel p95~19.74ms (10 samples after2 warmup), above the
proposed15ms gate. Excludes rectification, acquisition, ROS serialization,
network and browser cost. Full pipeline acceptance is NOT established; lowering
update rate reduces average work but not per-frame time. Timings reflect existing
robot load and variable clocks. No runtime services/camera configuration changed.

Conclusion: useful offline shape-preservation improvement, not yet a convincing
texture-recognition improvement or a resource-qualified live implementation.
Next work should isolate rectification/mask/projection cost, eliminate duplicate
full-frame work, and test a spatial/region-aware point budget without tailoring
sampling to this known box. Do not increase bandwidth or deploy by default yet.
