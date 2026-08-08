#!/usr/bin/env python3
"""Import Claude's auto-memory capture buffer into the vault's knowledge folder.

The buffer at ~/.claude/projects/-home-kamren-x3-ws/memory/ is where Claude writes
mid-session. `memory/knowledge/` is the durable copy that ships with the repo. The
two drift apart, so this makes the re-import a one-liner instead of a hand merge.

    python3 scripts/import_auto_memory.py            # import, rewrite Home.md index
    python3 scripts/import_auto_memory.py --check    # report drift, change nothing (exit 1 if any)

Notes are copied near-verbatim: the buffer's frontmatter (name/description/metadata)
is already the vault's knowledge-note format. The only rewrite is wikilink slug
normalization -- the buffer has historically mixed [[some-note]] and [[some_note]],
and only the underscore form resolves against these filenames.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BUFFER = Path.home() / ".claude" / "projects" / "-home-kamren-x3-ws" / "memory"
KNOWLEDGE = REPO_ROOT / "memory" / "knowledge"
HOME_NOTE = REPO_ROOT / "memory" / "Home.md"

# MEMORY.md is the buffer's own index; Home.md is the vault's. Don't import it.
SKIP = {"MEMORY.md"}

INDEX_START = "<!-- knowledge-index:start -->"
INDEX_END = "<!-- knowledge-index:end -->"

WIKILINK = re.compile(r"\[\[([^\]|#]+)((?:[|#][^\]]*)?)\]\]")
FRONTMATTER_FIELD = re.compile(r"^(name|description):\s*(.*)$", re.MULTILINE)


def normalize_links(text: str, known: set[str]) -> str:
    """Rewrite [[some-note]] -> [[some_note]] when only the latter is a real note."""

    def sub(match: re.Match[str]) -> str:
        target, suffix = match.group(1), match.group(2)
        stripped = target.strip()
        if stripped not in known:
            underscored = stripped.replace("-", "_")
            if underscored in known:
                return f"[[{underscored}{suffix}]]"
        return match.group(0)

    return WIKILINK.sub(sub, text)


def read_field(text: str, field: str) -> str:
    """Pull a top-level frontmatter scalar, tolerating quotes."""
    for match in FRONTMATTER_FIELD.finditer(text):
        if match.group(1) == field:
            return match.group(2).strip().strip('"').strip("'")
    return ""


def build_index(notes: dict[str, str]) -> str:
    """One bullet per note: [[slug]] -- description, sorted by slug."""
    lines = []
    for slug in sorted(notes):
        description = read_field(notes[slug], "description") or "(no description)"
        lines.append(f"- [[{slug}]] — {description}")
    return "\n".join(lines)


def splice_index(home_text: str, index: str) -> str:
    """Replace the region between the index markers, inserting them if absent."""
    if INDEX_START in home_text and INDEX_END in home_text:
        pattern = re.compile(
            re.escape(INDEX_START) + r".*?" + re.escape(INDEX_END), re.DOTALL
        )
        return pattern.sub(f"{INDEX_START}\n{index}\n{INDEX_END}", home_text)
    raise SystemExit(
        f"{HOME_NOTE} is missing the {INDEX_START} / {INDEX_END} markers; "
        "add them around the knowledge list so this script can maintain it."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="report drift without writing; exit 1 if the vault is out of date",
    )
    parser.add_argument(
        "--buffer",
        type=Path,
        default=BUFFER,
        help=f"auto-memory buffer to import from (default: {BUFFER})",
    )
    args = parser.parse_args()

    if not args.buffer.is_dir():
        print(f"auto-memory buffer not found: {args.buffer}", file=sys.stderr)
        return 1

    sources = {p.stem: p for p in sorted(args.buffer.glob("*.md")) if p.name not in SKIP}
    if not sources:
        print(f"no notes found in {args.buffer}", file=sys.stderr)
        return 1

    known = set(sources)
    imported = {slug: normalize_links(p.read_text(), known) for slug, p in sources.items()}

    added, updated, unchanged = [], [], []
    for slug, text in imported.items():
        dest = KNOWLEDGE / f"{slug}.md"
        if not dest.exists():
            added.append(slug)
        elif dest.read_text() != text:
            updated.append(slug)
        else:
            unchanged.append(slug)

    # Notes that exist only in the vault: curated by hand, or dropped from the
    # buffer. Never delete them automatically -- just say so.
    orphans = sorted(
        p.stem for p in KNOWLEDGE.glob("*.md") if p.stem not in imported
    )

    home_text = HOME_NOTE.read_text()
    new_home = splice_index(home_text, build_index(imported))
    home_stale = new_home != home_text

    if args.check:
        for slug in added:
            print(f"MISSING  {slug}")
        for slug in updated:
            print(f"DRIFTED  {slug}")
        for slug in orphans:
            print(f"ORPHAN   {slug} (in vault, not in buffer)")
        if home_stale:
            print("STALE    memory/Home.md knowledge index")
        if added or updated or home_stale:
            print(f"\n{len(added)} missing, {len(updated)} drifted — run without --check")
            return 1
        print(f"vault is up to date ({len(unchanged)} notes)")
        return 0

    KNOWLEDGE.mkdir(parents=True, exist_ok=True)
    for slug in added + updated:
        (KNOWLEDGE / f"{slug}.md").write_text(imported[slug])
    if home_stale:
        HOME_NOTE.write_text(new_home)

    print(f"added {len(added)}, updated {len(updated)}, unchanged {len(unchanged)}")
    for slug in added:
        print(f"  + {slug}")
    for slug in updated:
        print(f"  ~ {slug}")
    for slug in orphans:
        print(f"  ! {slug} (vault-only, left alone)")
    if home_stale:
        print("  ~ memory/Home.md knowledge index")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
