---
name: project_dream_skill
description: "The /dream memory-curation skill — where it is installed from, where its report and backups live, how apply is guarded, and why overnight runs are not happening"
metadata:
  node_type: memory
  type: project
  originSessionId: 345f863e-947e-4ef2-948c-dc797e88c2aa
  modified: 2026-08-08T06:24:11.027Z
---

Source of truth is `github.com/Kjames3/LLM-Skills`, `Skills/dream/` (consolidated there
2026-08-08, commit f9bee25). Install with `cd LLM-Skills/Skills/dream && ./install.sh`
(`--project` for a project-local install, `--schedule` to wire the 3am timer). An older
incompatible variant that used to sit at `Skills/` root — python extractor, `~/.claude/memory/`
paths — was deleted; the original prompt is kept as `ORIGIN-PROMPT.md`.

- Local install: `~/.claude/skills/dream/` — `SKILL.md` + `extract_turns.sh`, `dream_hash.sh`,
  `dream_backup.sh`, `run-dream.sh`. SKILL.md resolves its own directory into `$DREAM`, so call
  helpers as `"$DREAM"/dream_hash.sh`, never by a hardcoded path.
- State lives OUTSIDE `~/.claude/` on purpose (protected config dir; an early overnight run failed
  writing there): report `~/claude-dream/dream-report.md`, backups
  `~/claude-dream/backups/<date>/<run-id>/<project-slug>/`, log `~/claude-dream/dream.log`.
- `/dream` proposes and waits; `/dream apply 1,3|1-3|all` applies. Only typo/index/link repairs are
  auto-applied. Every non-CREATE proposal carries a sha256 `Target hash`, re-checked at apply time
  so approved text never lands on a file that changed since review; validate a finished report with
  `"$DREAM"/dream_hash.sh --check` (exit non-zero = not applyable).
- `extract_turns.sh N` selects transcripts by **file mtime**, so a small N can miss long-running
  sessions — on 2026-08-07 a 24h window held only trivial sessions and 7–9d was needed.
- Helpers take a project slug (e.g. `-home-kamren-x3-ws`) plus a **bare filename**, never a path,
  and an optional third argument `memory|notes` selecting the store.
- **No overnight schedule is installed.** As of 2026-08-07 `systemctl --user list-timers` lists
  zero timers and `~/.config/systemd/user/` is empty — an earlier hand-install attempt did not
  take. `/dream` only runs when invoked. To enable: `./install.sh --schedule`, then
  `loginctl enable-linger` so it fires while logged out.

See [[project_robot_deploy]].
