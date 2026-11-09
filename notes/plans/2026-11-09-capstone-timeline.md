# Capstone timeline (drafted 2026-11-09)

UCR master's capstone with Prof. Mingyu Cai. The presentation is **late Feb / early Mar 2027**,
so this plan assumes **Fri 2027-02-26** and treats the week of 03-01 as buffer.
Ground truth comes from mocap in the CRIS/CISL labs (not REAL), and those slots must be booked early.

## Thesis claim (working)
A mecanum robot that predicts how people will move (C3/lidar tracks, then the GRU) and keeps
itself safe with a footprint-shaped CBF QP, compared against ORCA, with formal STL monitoring.

## Phases

| Weeks | Dates | Phase | Exit criterion |
|---|---|---|---|
| 1–2 | 11-09 → 11-22 | **Offline consolidation** | GRU sigma calibrated + compared; close-range dropout cause found; OAK surroundings work committed; report skeleton exists |
| 3 | 11-23 → 11-29 | Thanksgiving (light) | Draft writing only: CBF, EKF yaw, velocity-estimator chapters |
| 4–6 | 11-30 → 12-20 | **Integration on robot** | GRU predictions feed the CBF moving-obstacle term (shadow mode first, then live); Nav2 `cmd_vel` routed through the CBF; seated-person lab recordings taken; DBaSCritic fix drive-tested |
| 7–8 | 12-21 → 01-03 | Winter break (buffer) | Offline only: STL monitor specs finalized, Set A/B scored |
| 9 | 01-04 → 01-10 | **Feature freeze** | No new features after 01-10. Only bug fixes |
| 10–12 | 01-11 → 01-31 | **Final evaluation** | Mocap sessions (CRIS/CISL); scenario matrix: approach / cross / start-stop / seated, with CBF vs ORCA, with and without prediction; ≥ 5 runs per cell |
| 13–14 | 02-01 → 02-14 | **Write-up** | Full report draft to Prof. Cai by 02-14 |
| 15 | 02-15 → 02-21 | Revisions + slides | Slides and a demo video (recorded fallback in case the live demo fails) |
| 16 | 02-22 → 02-26 | Rehearse + present | Two dry runs |
| — | 03-01 → 03-05 | Buffer | |

## Addressing the 2026-10-22 professor feedback
- *People lost up close.* Goal: find the 3.76 s dropout cause (dropout log) and fix it, or add
  a confidence threshold. Metric: % of lab time with no estimate (currently 21%), target < 10%.
- *ID churn.* Metric: ID switches per minute in the walking bags, before and after.
- *Nav2 bypasses the CBF.* Must be fixed before the evaluation in weeks 10–12.

## Booking / external dependencies
- [ ] Ask Prof. Cai for CRIS/CISL mocap slots in mid–late January (ask by 11-20).
- [ ] Recruit 2–3 walkers for the evaluation runs.
- [ ] Seated-person lab recordings (person classifier) in weeks 4–6.

## Out of scope unless ahead of schedule
Tilting-lidar revival, Ratatouille, mmWave radar, UWB, surroundings-3D backlog ideas,
deferred person-filter experiments.
