---
title: Ideas and Planning
description: Map of content — idea logs, ROI analyses, plan docs, progress
tags: [moc, planning]
---

# Ideas and Planning

Back to [[Home]]. These are the pre-existing docs at the repo root — all of them are notes
in this vault, so `Ctrl+O` reaches them by name.

## Idea logs (chronological)

| Doc | Size | Notes |
|---|---|---|
| [[June_architectural_ideas]] | ~39k words | ideas 1–240, archived |
| [[June_performance_ideas]] | ~8k words | performance & efficiency 1–120 |
| [[June_roi_analysis]] | ~16k words | June ROI tiers |
| [[July_improvement_ideas]] | ~6k words | links back to the June archive |
| [[august_improvement_ideas]] | ~33k words | A-01 … A-58, mostly velocity_MLP |
| [[august_roi_analysis]] | ~5k words | ROI + overlap notes for the August set |

**Numbering caveat:** A-01–A-25 and A-26–A-58 came from two independent review passes that
both started at `A-05`. The second pass was renumbered to A-26–A-58 and its internal
cross-references rewritten. Overlapping pairs are called out in [[august_roi_analysis]].

**Convention:** at the end of each month, completed or historical ideas move into `ideas/`.

## Plan documents

- [[VELOCITY_SELF_TRAINING_PLAN]] — the retrain recipe; §2 data, §3 scoring/ground truth
- [[LIDAR_MOUNT_PLAN]] — riser bracket + URDF update, §1 documents the self-return angles
- [[PERFORMANCE_PLAN]] — the encoding/transport decisions now summarized in [[CLAUDE]]

## Progress

`src/progress.md` (~11k words) is the running engineering log.

## How to use this pile

The idea logs are **candidate** lists, not decisions. When something is actually decided or
tried, write an atomic note in `memory/knowledge/` and link back to the idea ID — otherwise
the conclusion stays buried in a wall of alternatives. That extraction is worth more than any
search tooling; see the reasoning in [[README-vault]].

## Related

- [[Perception]] — where most August ideas land
- [[Home]]
