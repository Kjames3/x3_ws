---
title: How this vault works
description: Conventions for the X3 project memory vault, and why there is no RAG
tags: [meta]
---

# How this vault works

## Where the vault root is

The vault root is the **repository root** (`~/x3_ws`), not this folder. That was deliberate:
the existing docs (`august_improvement_ideas.md`, `ideas/June_*.md`, `LIDAR_MOUNT_PLAN.md`, …)
cross-reference each other with relative markdown links, and the idea logs follow a documented
monthly-archiving convention into `ideas/`. Moving them into a sub-vault would have broken
dozens of inline references. With the root as the vault, every one of those files is already
a note, every existing link still resolves, and `memory/` just adds the organizing layer.

Build noise (`build/`, `install/`, `log/`, `models/`, `src/yolo_models/`, `.claude/`) is
excluded from search, graph, and link suggestions via `.obsidian/app.json` → `userIgnoreFilters`.
Those folders are gitignored anyway; they still appear in the file explorer, which is harmless.

## Folder layout

```
memory/
  Home.md          entry point — start here
  README-vault.md  this file
  maps/            maps of content: one per subsystem, links outward
  knowledge/       atomic durable facts, one per note
  runbooks/        "how do I X" procedures
  log/             dated session entries, YYYY-MM-DD.md
  templates/       note templates (Templates plugin points here)
```

## Conventions

**One fact per knowledge note.** If a note needs "and also", it's two notes. Small notes are
what make links meaningful and search results precise.

**Link generously.** A `[[wikilink]]` to a note that doesn't exist yet is not an error — it's a
marker that the note is worth writing. Backlinks (`Ctrl+Shift+B`) are the main navigation tool.

**Date everything.** Convert "last week" to an absolute date when you write it. A note that
says "recently" is worthless in four months.

**Record the why, not the what.** The code already says what it does; git already says what
changed. The vault is for what neither records: why a parameter is that value, what you tried
that failed, which quirk cost you an afternoon.

**Never paste code that lives in the repo.** Link to it as `src/server_x3.py:412` instead —
that stays correct when the code moves, and a paste silently rots.

## Relationship to Claude's auto-memory

`~/.claude/projects/-home-kamren-x3-ws/memory/` is the **capture buffer**: Claude writes there
automatically mid-session. This vault is the **durable store**. The notes in `knowledge/` were
imported from that buffer on 2026-08-07 (wikilink slugs normalized from mixed `-`/`_` forms so
they actually resolve).

The two can drift. Treat the vault as authoritative and re-import periodically rather than
editing both. This is a curation step, not a sync — the buffer is noisy by design.

## Why there is no RAG here (and when to reconsider)

You asked. My honest read: **RAG would be a net negative at this scale.**

The whole corpus is ~130k words — call it ~170k tokens. That is small enough that:

1. **Exact search beats semantic search on this content.** What you actually look up is
   `ttyCH341USB0`, `A-30`, `nav2_params_x3.yaml`, `512000 baud`. Those are identifiers.
   Embeddings are *worse* than grep at identifiers — they blur the exact token you need into
   a neighbourhood of similar-sounding ones. Obsidian's search and `grep` return the line.
2. **The index would be the thing that breaks.** A vector store needs rebuilding on every edit.
   A stale index that confidently returns last month's answer is a worse failure mode than a
   search that returns nothing, because you can't see that it's stale.
3. **Chunking destroys the structure you already have.** The idea logs are numbered and
   cross-referenced (`A-30` refers to `A-26`). Chunking cuts those threads; wikilinks preserve
   them, and backlinks make them bidirectional for free.
4. **Claude can just read the files.** With grep to narrow and Read to load, an agent gets the
   exact text with no retrieval layer to misfire. The bottleneck here has never been retrieval.

Where the real leverage is instead: the 39k-word `June_architectural_ideas.md` and 33k-word
`august_improvement_ideas.md` are monoliths. Splitting the *decided* items out into atomic
knowledge notes buys more than any retrieval system would — because the problem isn't finding
the text, it's that the conclusions are buried in a wall of candidates.

**Reconsider RAG when** the vault passes roughly 1,000 notes or ~1M words, *and* you find
yourself searching by description rather than by name ("that thing about the wheel slipping on
tile") often enough to be annoying. At that point the cheap move is still not a vector DB —
it's Obsidian's Omnisearch plugin, which does fuzzy local search with no external service.
