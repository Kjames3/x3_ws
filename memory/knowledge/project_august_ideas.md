---
name: project_august_ideas
description: "august_improvement_ideas.md / august_roi_analysis.md — dual authorship by a scheduled cloud routine and local sessions, and the A-NN numbering hazard"
metadata: 
  node_type: memory
  type: project
  originSessionId: 873ed488-d99e-4d1a-bdd5-37d55ee417db
  modified: 2026-08-08T06:24:31.403Z
---

`august_improvement_ideas.md` and `august_roi_analysis.md` are written by **two independent
writers**: a scheduled cloud routine committing to `origin/main`, and local interactive sessions.
On 2026-08-01 both had written a different set of ideas numbered from **A-05** — same IDs, entirely
different content — so a plain merge would have silently destroyed one lineage. Resolved in commit
`76e4436` by keeping local A-01…A-25 and renumbering the cloud set to **A-26…A-58**, rewriting its
internal cross-references (`A-18` alone appeared 18 times).

**Before adding any idea: `git fetch` and read the highest existing A-NN on `origin/main`, then
continue past it.** Never restart numbering from a local view of the file.

Current state: 58 ideas, matrix rows A-01…A-58 with no gaps, ranks 1–58 contiguous in the ROI doc,
tiers agreeing between both files. §5 of the ROI doc records cross-pass overlaps found because both
writers independently caught the same defects (A-21≡A-27, A-20≡A-33, A-18/A-07/A-25 are pieces of
A-55) plus one genuine conflict: A-15 says *delete* the JIT trace, A-52 says *freeze* it.

These two files are the monoliths flagged in [[project_obsidian_vault]] — decided items are buried
among candidates, and extracting them into atomic notes is the highest-value docs work outstanding.
