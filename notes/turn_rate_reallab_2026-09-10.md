# REAL lab turn-rate audit — 2026-09-10

User manually performed CW/CCW slow/fast approximately full-revolution turns.
User observed slow endpoints within ~10 degrees of start, fast CW approximately
at start, fast CCW ~15 degrees short. Five bags exist (two slow CW runs); all
SQLite integrity checks passed. Bags reside at /home/jetson/bags on x3.

| Bag suffix | Steady command rad/s | Bias-corrected gyro rad/s | Actual / command | Integrated IMU degrees |
| --- | --- | --- | --- | --- |
| turn_cw_slow_20260910_182427 | -0.1300 | -0.7095 | 5.46 | -367.97 |
| turn_cw_slow_20260910_182511 | -0.0900 | -0.6072 | 6.75 | -358.26 |
| turn_ccw_slow_20260910_182545 | +0.1000 | +0.6241 | 6.24 | +368.82 |
| turn_cw_fast_20260910_182657 | -0.5000 | -1.3245 | 2.65 | -353.54 |
| turn_ccw_fast_20260910_182720 | +0.5000 | +1.3103 | 2.62 | +352.06 |

Method: deserialize /cmd_vel and /imu/data_raw using ROS Humble. Bag receipt
timestamps define time. Active command threshold |wz|>0.03 rad/s. Estimate
gyro bias from samples ending 0.5 s before first active command (about 0.001
rad/s for each bag). Integrate trapezoidally from 1 s before first active
command through 2 s after last. Steady samples exclude first 1 s and final
0.5 s, require a command no older than 0.25 s, |command|>=0.05, and command
range <=0.02 rad/s over preceding 0.7 s. Ratios in table use mean actual /
mean command. Angular endpoint measurements are approximate visual references,
not a precision gyro calibration.

CCW slow had a brief translation command, maximum magnitude 0.0222 m/s;
the other bags had zero translation commands. No sensor-scale or motor settings
were changed. The extra earlier CW slow bag independently supports the trend.

Conclusion: previously reported ~2.4x response is still present (~2.6x at
0.5 rad/s), but low-command response is ~6–7x. A single multiplicative command
correction is inappropriate. Driver uses PWM=200*(vx +/- vy +/- omega*0.165)
with wheel gains, then adds +/-28 PWM to each active wheel. For pure rotation,
the pre-gain variable term is only 3.3 PWM at 0.1 rad/s or 16.5 at 0.5 rad/s,
so the fixed offset dominates slow turns. Measurements are consistent with
this explanation; friction, gain and floor load also affect actual response.

Preserve gyro/odom angular scale: physical endpoint observations broadly agree
with integrated gyro, and no evidence supports dividing measured yaw by 2.6.
Next implementation work should address command-to-motor response, separately
validated for pure yaw and combined translation/yaw. Consider IMU feedback with
bounded correction and stale-input handling rather than a blanket scale factor.
The PAA5100JE is not required for yaw-rate measurement. Lateral odom scale -0.5
remains the only calibration change deployed in this session.
