# Handoff — Nav2 stuck/lost fixes (2026-11-25)

Branch: `claude/nav2-robot-navigation-issues-660get` (pull it on the laptop).

**Read first:** [notes/plans/2026-11-25-nav2-stuck-lost-fixes.md](notes/plans/2026-11-25-nav2-stuck-lost-fixes.md)
— diagnosis, what was changed, how to verify, next steps, alternatives to Nav2,
and pedestrian-crossing trajectory ideas.

## What changed
- `src/yahboomcar_nav/params/nav2_params_x3.yaml`: scan raytrace/obstacle range
  6.0/5.5 m in both costmaps (fixes never-cleared ghost obstacles), AMCL
  alphas 0.2/0.3 + recovery alphas + update_min 0.1, progress checker 20 s.
- `src/server_x3.py`: `_log_cbf_cut` → `[cbf-cut]` INFO summary every 10 s
  when the CBF cuts commands hard.

## State
- Laptop tests pass (345). **Not deployed, not drive-tested.**

## Next session to-do
1. Deploy (`/deploy`), rebuild `yahboomcar_nav`, restart Nav2.
2. Run the verification steps in the notes file (costmap clearing, `[cbf-cut]`
   log, AMCL particle cloud).
3. Based on `[cbf-cut]` counts, do "Next" item 1 (MPPI clearance > CBF margin),
   then item 2 (single people model).
