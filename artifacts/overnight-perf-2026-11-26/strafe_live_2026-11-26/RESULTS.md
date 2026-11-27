# Strafe live test, 2026-11-26 ~23:30, apartment (newApartment1), AMCL localized

Goals sent with side_goal.py (NavigateToPose, offset in the robot frame, goal yaw = current map yaw).
All runs went through velocity_smoother and the CBF gate (/cmd_vel_safe). Robot had ~1.5 m clear each side.
Bags on the robot: ~/bags/strafe_2026-11-26/strafe_default_* and strafe_omni_*.
Installed params matched src (sha256), so a stale install was ruled out.

| Run | Params | Goal | Result | Time | max\|vy\| | max\|wz\| | max heading excursion | final heading change |
|---|---|---|---|---:|---:|---:|---:|---:|
| 1 | default (RotationShim 45° + Omni MPPI) | 0.8 m left | SUCCEEDED | 8.2 s | 0.10 | 1.00 | 86° | +12° |
| 2 | omni_direct (plain MPPI) | 0.8 m right | SUCCEEDED | 4.6 s | 0.23 | 0.42 | 43° | -11° |
| 3 | omni_direct | 0.8 m left | SUCCEEDED | 3.9 s | 0.24 | 0.42 | 36° | +8° |
| 4 | omni_direct | 0.8 m right | SUCCEEDED | 4.6 s | 0.26 | 0.44 | 51° | -12° |

Default: 1.4 s pure rotation at 1 rad/s (vx=vy=0), then forward driving, then rotate back. Root cause confirmed.
omni_direct: |vy| > 0.15 within 0.2-0.4 s on /cmd_vel_safe, no rotate-in-place, about half the time.
Open: MPPI still yaws 36-51° mid-strafe with sustained wz ~0.3 (critic weighting; TwirlingCritic or
a heading critic is the next knob). Goal-behind-the-robot regression test NOT run (no confirmed clear space behind).
