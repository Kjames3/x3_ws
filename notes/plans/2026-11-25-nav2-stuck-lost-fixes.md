# Nav2 "lost / stuck in a loop" on the physical robot — diagnosis, fixes, next methods

Date: 2026-11-25. Branch: `claude/nav2-robot-navigation-issues-660get`.
Status: cheap fixes **applied, not yet drive-tested**. Everything below the
"Applied" section is analysis / proposals, ranked.

## Symptom
On the real robot Nav2 often (a) appears to lose its pose mid-goal, or (b) creeps,
stops, recovers (spin/backup), replans and repeats — sometimes with
`CBF infeasible` warnings, sometimes with nothing logged.

## Root causes (ranked by likelihood)

1. **Ghost obstacles that never clear (range gap).** livox_low, oak_low, keepout,
   tof_* and people are *mark-only* (`clearing: False`); only `/scan` rays erase
   cells. Scan raytraced to the 3.0 m default (global costmap used
   `raytrace_range`, the pre-Humble name, which is ignored) while people mark to
   4 m and oak/livox to 5 m. Marks in the 3–5 m band were permanent → walking
   people left walls → global path flips each replan → "lost"/circling.
   *Remaining height gap:* anything marked above/below the ~0.31 m scan plane can
   still only be cleared by the scan if the scan ray passes over that cell; a
   low mark under a table edge the scan hits short stays. Not fixed cheaply.
2. **MPPI and the CBF disagree, silently.** MPPI sees only the inflated costmap
   (CostCritic weight 3.81, critical 300); the CBF enforces a hard footprint
   margin (`CBF_MARGIN_M` 0.05) plus moving-person rows. MPPI doesn't know its
   command was modified; its warm start assumes it was executed. Result: CBF
   shrinks the command → near-zero progress → progress checker → recovery
   (also CBF-filtered) → same replan → loop. A feasible-but-heavy cut was never
   logged, which is the "something else not reported".
3. **Four uncoordinated people models**: DBaS critic (safe_radius 0.6), the
   `/people/obstacles` costmap cloud, the CBF soft person rows (0.25 m), and
   `people_cap_velocity`. Each pushes in its own direction → dithering.
4. **AMCL overconfident**: all alphas 0.05, recovery alphas 0. Mecanum slip
   during swerves outran the particle cloud with no kidnap recovery.
5. **Smaller**: `motion_model: DiffDrive` (no strafing — the best give-way move
   on a mecanum; blocked by the velocity smoother's default vy=0, see the yaml
   comment); `movement_time_allowance` 10 s fires recovery while yielding.

## Applied (cheap fixes)

All in `src/yahboomcar_nav/params/nav2_params_x3.yaml` unless noted.

| Change | Old → new | Why |
|---|---|---|
| local + global `scan.raytrace_max_range` | 3.0 (default) → 6.0 | clear ≥ every mark-only source's range |
| local + global `scan.obstacle_max_range` | 2.5 default → 5.5 | consistent with raytrace |
| global `scan.raytrace_range` | removed (ignored key) | Humble name is `raytrace_max_range` |
| AMCL `alpha1..4` / `alpha5` | 0.05 → 0.2 / 0.3 | follow roller slip |
| AMCL `recovery_alpha_slow/fast` | 0/0 → 0.001/0.1 | recover from bad pose |
| AMCL `update_min_d/a` | 0.25/0.2 → 0.1/0.1 | more updates at low speed |
| progress checker `movement_time_allowance` | 10 → 20 s | yielding ≠ stuck |
| `server_x3.py` `_log_cbf_cut` | new | one `[cbf-cut]` INFO line per 10 s window (env `X3_CBF_CUT_LOG_PERIOD_S`) when commands were cut >50%, stopped, or had yaw cut |

Tests: `tests/test_cbf_filter.py`, `test_cbf_tof_safety.py`, nav/server tests — 345 passed (laptop, no ROS).

### How to verify on the robot
1. Deploy, rebuild `yahboomcar_nav` (params are installed), restart nav.
2. RViz: `/global_costmap/costmap` + `/local_costmap/costmap`. Have someone walk
   past at 3–4 m, then leave. Cells should clear within a scan or two (before: stayed).
3. `journalctl -u x3_server -f | grep -E "cbf-cut|CBF infeasible"` during a goal.
   Many `cut >50%` lines while MPPI is following a free path ⇒ cause #2 is live;
   do the "Next" item 1.
4. Watch AMCL `/particle_cloud` during a swerve: it should widen, then reconverge.
   If localization gets noisy on straight drives, step alphas back to 0.1–0.15.

## Next (not done — needs a drive test each)
1. **Make MPPI respect more than the CBF**: CostCritic `cost_weight` ~8–10,
   `critical_cost` ~100, or inflation `cost_scaling_factor` 3.0, so the
   planned clearance exceeds the CBF margin and the CBF rarely acts.
2. **One people model.** Make DBaS the single source: feed it predicted
   trajectories, same radius as the CBF person row; drop the people cloud from
   the global costmap (plan around the room, not transient people) or give it a
   decaying layer.
3. **Omni MPPI**: add a `velocity_smoother` block with vy limits, then
   `motion_model: "Omni"`.
4. Height-gap clearing: give livox_low its own clearing (it's a 3D lidar; it
   can raytrace) or use a voxel layer / STVL with decay.

## Alternatives to Nav2 for social navigation

| Approach | Pros | Cons |
|---|---|---|
| **Nav2, consolidated** (MPPI Omni + one predictive people critic, CBF as last-resort) | keeps AMCL, BT recovery, tooling; smallest change | MPPI sampling is jittery; still two layers |
| **Custom MPC controller plugin** (CasADi/acados; people as time-varying constraints, CBF rows inside the optimizer) | planner and safety share one model → no fighting; principled for a capstone write-up | build/tune it; solver-failure handling; Orin compute budget |
| **Social force / ORCA local planner** | simple, fast, legible | local minima, weak in clutter and doorways |
| **Learned policy (CrowdNav-style RL)** | natural crowd behaviour | sim-to-real gap, no guarantees, data hungry |
| **Replace Nav2 entirely** | full control | lose localization integration, recovery, lifecycle — not worth it at this stage |

Recommendation: keep Nav2 for global planning / AMCL / recovery; either fix
MPPI (items above) or drop in an MPC controller plugin; keep the CBF as a thin
filter whose margin is smaller than the planner's clearance.

## Trajectory generation when a pedestrian crosses the robot's path
1. **Score against predicted positions**: for rollout step k, cost distance to
   the person's prediction at t = k·dt (`traj_predictor.py`), with a radius that
   grows with k for uncertainty.
2. **Commit to a passing side** (pass-left / pass-right / yield), keep it for
   1–2 s, and bias the critic toward it — kills left/right dithering.
3. **Strafe**: sideways give-way while holding heading (needs Omni + smoother vy).
4. **Slow along the path instead of steering away** when the time windows of the person's crossing and the robot's arrival overlap — legible, and almost never CBF-infeasible.
5. **CBF γ per person from time-to-collision**, so the filter only acts when
   the planner has failed.
6. **Metrics per crossing scenario**: min person distance, `[cbf-cut]` rate and
   mean cut, path irregularity, time-to-goal.
