---
name: project_obsidian_vault
description: "x3_ws repo root doubles as an Obsidian vault with a memory/ layer — why nothing was moved, and why RAG was rejected at this scale"
metadata: 
  node_type: memory
  type: project
  originSessionId: 873ed488-d99e-4d1a-bdd5-37d55ee417db
  modified: 2026-08-08T06:24:22.215Z
---

Built 2026-08-08 on branch `worktree-obsidian-vault`, **draft PR #2, not yet merged to main** —
open `~/x3_ws/.claude/worktrees/obsidian-vault` as a vault to browse it before merging.

**The repo root itself is the vault root**, not a subfolder. That was deliberate: the existing docs
(~130k words) cross-reference each other by relative path and `ideas/` has a documented monthly
archive convention, so a sub-vault would have broken dozens of inline links. Consequence for future
work: **do not move or rename the repo's top-level `.md` files** without fixing referrers. Nothing
was moved to build the vault — only `CLAUDE.md` gained a vault pointer and `.gitignore` gained
Obsidian's per-user state.

`memory/` in the repo is the purely additive organizing layer: `Home.md`, 6 maps of content, 10
knowledge notes imported from Claude's auto-memory, 4 runbooks, 3 templates, a session log. It is a
**copy** of auto-memory, not the same store — the live store this skill curates is still
`~/.claude/projects/-home-kamren-x3-ws/memory/`, and the two drift apart unless re-imported. The
import had to normalize mixed `-`/`_` wikilink slugs, so keep auto-memory `name:` fields
underscore-cased.

**RAG was considered and rejected** for this corpus, and it isn't close: lookups here are for exact
identifiers (`ttyCH341USB0`, `A-30`, `512000 baud`) where embeddings are worse than grep, an index
goes stale on every edit, and chunking cuts the `A-NN` cross-reference threads wikilinks preserve
free. Revisit past ~1000 notes AND when searching by description rather than name — and even then
the cheap move is Obsidian's Omnisearch plugin, not a vector DB. Full argument:
`memory/README-vault.md`. The real bottleneck is that `June_architectural_ideas.md` (39k words) and
`august_improvement_ideas.md` (33k) are monoliths where decided items are buried among candidates.

Obsidian itself is a user-local AppImage install: `~/.local/bin/obsidian` (v1.13.4), with a desktop
launcher and icon; AppImages do not self-update in place.

See [[project_august_ideas]].
