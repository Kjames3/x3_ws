# Curved person trajectories in the Reality view (2026-11-27, overnight)

Branch `overnight/curved-person-trajectories-2026-11-27`. Display only: nothing
here feeds the CBF or Nav2.

## Problem

The Reality view draws each moving person's predicted path as a straight
constant-velocity ribbon. It never bends, so a person who is turning still
gets a straight line.

## Selection rule (written before any numbers were looked at)

The candidates are `cv` (today's straight line), `ctrv` (constant turn rate and
velocity, with the turn rate damped) and `gru` (config/traj_gru_deploy.npz). A
candidate becomes the recommended `X3_PERSON_PATH` value only if all three of
these hold on the held-out report runs:

1. On MOVING windows it is no worse than `cv`: FDE at 2 s within +3 % of cv,
   and ADE at 1 s within +3 %.
2. On TURNING windows it has the lowest FDE at 2 s, and the gain over cv is
   larger than the run-to-run spread. Circling runs are reported both
   included and excluded.
3. It is stable enough to show. The median frame-to-frame change of the 2 s
   endpoint on moving tracks is at most 1.5x cv's. A wagging ribbon is worse
   to look at than a straight one.

If nothing passes, the default stays `off` and these notes say why. Turn
damping, clamps and gates are tuned on the TUNE runs only.

Definitions:
- MOVING: ground-truth mean speed over the horizon > 0.3 m/s (the
  trajectory_baselines definition).
- TURNING: MOVING, and the ground-truth heading change over the horizon is
  more than 30 deg. Heading at t0 is the direction of the last 0.3 s of
  history displacement and heading at t0+h is the direction of the last 0.3 s
  of future displacement. Both segments must be faster than 0.3 m/s, so
  standing jitter cannot count as a turn.
