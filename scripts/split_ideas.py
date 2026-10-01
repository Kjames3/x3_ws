#!/usr/bin/env python3
"""Generate one atomic note per idea from the monolithic idea logs.

The idea logs are 90k words across four files. Asking "was A-30 decided, or is it
still a candidate?" means grepping a 225 KB file where accepted and candidate items
sit interleaved -- and the answer often isn't in the file at all, because the only
record that an idea shipped is a code comment referencing its number.

This derives a browsable per-idea view from those logs:

    python3 scripts/split_ideas.py           # regenerate ideas/notes/ + ideas/INDEX.md
    python3 scripts/split_ideas.py --check   # report staleness, change nothing (exit 1)

The monoliths stay the single source of truth -- notes are a *generated view*, never
hand-edited, so the two cannot drift. Same contract as scripts/import_auto_memory.py.

Two hazards this fixes, both real and both previously undocumented:

1. **ID collision.** `June_architectural_ideas.md` numbers ideas 1..340 and
   `June_performance_ideas.md` numbers a completely different set 1..120. Every bare
   "Idea N" for N <= 120 is ambiguous. Notes carry a namespaced id (JA-/JP-/J-/A-)
   that is unique across the corpus.

2. **Lost decisions.** No idea in any log is marked implemented. The evidence lives
   in `June_roi_analysis.md`'s "Already Implemented Ideas" list and in `Idea N`
   comments in the source. Both are folded into each note's `status`.
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
NOTES_DIR = REPO_ROOT / "ideas" / "notes"
INDEX_FILE = REPO_ROOT / "ideas" / "INDEX.md"
ROI_FILE = REPO_ROOT / "ideas" / "June_roi_analysis.md"

# source doc -> namespace prefix. The prefix is what makes an id unique corpus-wide.
SOURCES = {
    "ideas/June_architectural_ideas.md": "JA",
    "ideas/June_performance_ideas.md": "JP",
    "July_improvement_ideas.md": "J",
    "august_improvement_ideas.md": "A",
}

# Matches both heading dialects the logs use: "### 12. Title" and "### Idea A-30: Title".
HEADING = re.compile(r'^###\s+(?:Idea\s+)?([A-Z]-?\d+|\d+)[.:]\s+(.*?)\s*$', re.M)
SECTION = re.compile(r'^##\s+(.*?)\s*$', re.M)
CODE_GLOBS = ("src/**/*.py", "src/**/*.js", "src/**/*.html")
CODE_SKIP = ("build/", "install/", "log/", "yolo_models/", "__pycache__")


def slugify(title: str, limit: int = 48) -> str:
    s = re.sub(r'`|\$\$|\\', '', title).lower()
    s = re.sub(r'[^a-z0-9]+', '-', s).strip('-')
    return s[:limit].rstrip('-') or "untitled"


def canonical(ns: str, raw: str) -> str:
    """JA + '7' -> 'JA-007'; A + 'A-01' -> 'A-01'. Zero-padded so files sort naturally."""
    digits = re.sub(r'^[A-Z]-?', '', raw)
    width = 3 if ns in ("JA", "JP") else 2
    return f"{ns}-{int(digits):0{width}d}"


def parse_source(path: Path, ns: str) -> list[dict]:
    """Split one monolith into its ideas, keeping each section's body verbatim."""
    text = path.read_text()
    sections = [(m.start(), m.group(1)) for m in SECTION.finditer(text)]
    heads = list(HEADING.finditer(text))
    out = []
    for i, m in enumerate(heads):
        end = heads[i + 1].start() if i + 1 < len(heads) else len(text)
        body = text[m.end():end].strip()
        # The nearest preceding "## ..." is the review session this idea came from.
        session = ""
        for pos, name in sections:
            if pos < m.start():
                session = name
            else:
                break
        out.append({
            "ns": ns,
            "raw_id": m.group(1),
            "id": canonical(ns, m.group(1)),
            "title": m.group(2).strip(),
            "body": body,
            "session": session,
            "source": str(path.relative_to(REPO_ROOT)),
        })
    return out


def parse_summary_matrix(path: Path) -> dict[str, dict]:
    """Pull Domain / ROI Tier / Status out of a doc's prioritization table, if it has one."""
    out = {}
    for line in path.read_text().splitlines():
        m = re.match(r'^\|\s*\*\*([A-Z]-?\d+)\*\*\s*\|(.+)\|\s*$', line)
        if not m:
            continue
        cols = [c.strip() for c in m.group(2).split('|')]
        if len(cols) >= 5:
            out[m.group(1)] = {
                "domain": cols[1],
                "roi_tier": cols[3].strip('*'),
                "logged_status": cols[4].strip('*'),
            }
    return out


def parse_implemented(path: Path) -> dict[int, str]:
    """The 'Already Implemented Ideas' list in the ROI analysis — bare numeric ids."""
    if not path.exists():
        return {}
    text = path.read_text()
    if "Already Implemented Ideas" not in text:
        return {}
    section = text.split("Already Implemented Ideas", 1)[1].split("\n## ", 1)[0]
    return {int(n): t.strip() for n, t in re.findall(r'\*\*Idea (\d+)\*\*:\s*([^\n]+)', section)}


def scan_code_refs() -> dict[int, list[str]]:
    """Find `Idea N` mentions in source. This is the only record that an idea shipped."""
    refs: dict[int, set[str]] = collections.defaultdict(set)
    for glob in CODE_GLOBS:
        for p in REPO_ROOT.glob(glob):
            rel = str(p.relative_to(REPO_ROOT))
            if any(s in rel for s in CODE_SKIP):
                continue
            try:
                text = p.read_text(errors="ignore")
            except OSError:
                continue
            for m in re.finditer(r'\b(?:Idea|IDEA)[ #]*(\d+)\b', text):
                refs[int(m.group(1))].add(rel)
    return {k: sorted(v) for k, v in refs.items()}


def build(ideas: list[dict], implemented: dict[int, str], code_refs: dict[int, list[str]],
          matrices: dict[str, dict]) -> list[dict]:
    for idea in ideas:
        meta = matrices.get(idea["source"], {}).get(idea["raw_id"], {})
        idea["domain"] = meta.get("domain", "")
        idea["roi_tier"] = meta.get("roi_tier", "")
        idea["logged_status"] = meta.get("logged_status", "")

        # Bare "Idea N" evidence is only meaningful in the June architectural
        # namespace -- that is the numbering the ROI analysis and the code comments
        # both use (293/307 ROI references resolve there, 0 to the performance log).
        num = int(re.sub(r'^[A-Z]-?', '', idea["raw_id"]))
        evidence, refs = [], []
        if idea["ns"] == "JA":
            if num in implemented:
                evidence.append("listed in June_roi_analysis.md → Already Implemented Ideas")
            refs = code_refs.get(num, [])
            if refs:
                evidence.append(f"referenced in {', '.join(refs)}")
        idea["code_refs"] = refs
        idea["evidence"] = evidence
        idea["status"] = "Implemented" if evidence else "Logged"
    return ideas


def render_note(idea: dict) -> str:
    fm = [
        "---",
        f'id: {idea["id"]}',
        f'title: "{idea["title"].replace(chr(92), "").replace(chr(34), chr(39))}"',
        f'status: {idea["status"]}',
    ]
    if idea["domain"]:
        fm.append(f'domain: {idea["domain"]}')
    if idea["roi_tier"]:
        fm.append(f'roi_tier: {idea["roi_tier"]}')
    fm += [
        f'source: {idea["source"]}',
        f'source_id: "{idea["raw_id"]}"',
    ]
    if idea["session"]:
        fm.append(f'session: "{idea["session"]}"')
    if idea["code_refs"]:
        fm.append(f'code_refs: [{", ".join(idea["code_refs"])}]')
    fm += ["tags: [idea]", "---", ""]

    head = [f'# {idea["id"]} — {idea["title"]}', ""]
    if idea["status"] == "Implemented":
        head.append("> [!success] Implemented")
        for e in idea["evidence"]:
            head.append(f"> - {e}")
    else:
        head.append("> [!note] Candidate — logged, not decided")
        head.append("> No implementation evidence found in the ROI analysis or in code comments.")
    head.append("")

    foot = [
        "",
        "---",
        "",
        "## Provenance",
        "",
        f'Generated from `{idea["source"]}` (heading `{idea["raw_id"]}`) by '
        "`scripts/split_ideas.py`. **Do not edit this file** — edit the source log and "
        "regenerate, or the two will drift.",
    ]
    if idea["ns"] in ("JA", "JP"):
        other = "June_performance_ideas.md" if idea["ns"] == "JA" else "June_architectural_ideas.md"
        foot.append("")
        foot.append(
            f'> [!warning] Ambiguous legacy id\n'
            f'> This idea is `{idea["raw_id"]}` in its own log, but `{other}` also numbers a '
            f'different idea `{idea["raw_id"]}`. Always use the namespaced id `{idea["id"]}`.'
        )
    # Blank lines around the body are load-bearing: without one before the closing
    # rule, "text\n---" is a setext H2 and swallows the last line of the idea.
    return "\n".join(fm + head) + "\n" + idea["body"] + "\n" + "\n".join(foot) + "\n"


def render_index(ideas: list[dict]) -> str:
    by_ns = collections.defaultdict(list)
    for i in ideas:
        by_ns[i["ns"]].append(i)
    total = len(ideas)
    done = sum(1 for i in ideas if i["status"] == "Implemented")
    names = {"JA": "June — Architectural", "JP": "June — Performance",
             "J": "July — Improvements", "A": "August — Improvements"}

    out = [
        "---", "title: Idea Index", "tags: [moc, idea]", "---", "",
        "# Idea Index", "",
        "**Generated — do not edit.** Run `python3 scripts/split_ideas.py` to refresh, ",
        "`--check` to verify. One note per idea lives in `ideas/notes/`; the monolithic ",
        "logs remain the source of truth.", "",
        f"{total} ideas across {len(by_ns)} logs — **{done} with implementation evidence**, "
        f"{total - done} still candidates.", "",
        "> [!warning] Legacy ids collide",
        "> `June_architectural_ideas.md` numbers ideas 1–340 and `June_performance_ideas.md`",
        "> numbers a *different* set 1–120. A bare \"Idea 81\" is ambiguous. Use the namespaced",
        "> ids below (`JA-081` vs `JP-081`); they are unique across the whole corpus.", "",
        "> [!note] How status is decided",
        "> No idea log marks anything as done. `Implemented` here means the idea is listed in",
        "> `June_roi_analysis.md` → *Already Implemented Ideas*, or its number appears in a",
        "> source comment. Both apply only to the `JA` namespace, which is the numbering those",
        "> two records use — so a `JP`/`J`/`A` idea showing `Logged` means *no evidence was",
        "> looked for*, not that it was rejected.", "",
    ]
    for ns in ("JA", "JP", "J", "A"):
        items = sorted(by_ns.get(ns, []), key=lambda i: i["id"])
        if not items:
            continue
        d = sum(1 for i in items if i["status"] == "Implemented")
        out += [
            f"## {names[ns]} (`{ns}-`)", "",
            f"{len(items)} ideas, {d} implemented — source: `{items[0]['source']}`", "",
            "| ID | Title | Status | ROI | Domain |",
            "| :-- | :-- | :-- | :-- | :-- |",
        ]
        for i in items:
            title = i["title"].replace("|", "\\|")
            mark = "✅ Implemented" if i["status"] == "Implemented" else "candidate"
            out.append(
                f'| [[{i["id"]}-{slugify(i["title"])}\\|{i["id"]}]] | {title} | {mark} '
                f'| {i["roi_tier"] or "—"} | {i["domain"] or "—"} |'
            )
        out.append("")
    return "\n".join(out) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="report drift without writing; exit 1 if regeneration is needed")
    args = ap.parse_args()

    ideas, matrices = [], {}
    for rel, ns in SOURCES.items():
        path = REPO_ROOT / rel
        if not path.exists():
            print(f"missing source: {rel}", file=sys.stderr)
            return 1
        matrices[rel] = parse_summary_matrix(path)
        ideas += parse_source(path, ns)

    build(ideas, parse_implemented(ROI_FILE), scan_code_refs(), matrices)

    dupes = [k for k, v in collections.Counter(i["id"] for i in ideas).items() if v > 1]
    if dupes:
        print(f"namespaced ids are not unique: {dupes[:10]}", file=sys.stderr)
        return 1

    wanted = {f'{i["id"]}-{slugify(i["title"])}.md': render_note(i) for i in ideas}
    wanted["../INDEX.md"] = render_index(ideas)

    changed, created = [], []
    for name, text in wanted.items():
        dest = (NOTES_DIR / name).resolve()
        if not dest.exists():
            created.append(name)
        elif dest.read_text() != text:
            changed.append(name)
    stale = sorted(p.name for p in NOTES_DIR.glob("*.md")) if NOTES_DIR.exists() else []
    stale = [n for n in stale if n not in wanted]

    if args.check:
        for n in created:
            print(f"MISSING  {n}")
        for n in changed:
            print(f"DRIFTED  {n}")
        for n in stale:
            print(f"STALE    {n} (no longer in any source log)")
        if created or changed or stale:
            print(f"\n{len(created)} missing, {len(changed)} drifted, {len(stale)} stale "
                  f"— run without --check")
            return 1
        print(f"ideas/notes is up to date ({len(ideas)} ideas)")
        return 0

    NOTES_DIR.mkdir(parents=True, exist_ok=True)
    for name, text in wanted.items():
        (NOTES_DIR / name).resolve().write_text(text)
    for name in stale:
        (NOTES_DIR / name).unlink()

    done = sum(1 for i in ideas if i["status"] == "Implemented")
    print(f"{len(ideas)} ideas -> {NOTES_DIR.relative_to(REPO_ROOT)}  "
          f"({len(created)} new, {len(changed)} updated, {len(stale)} removed)")
    print(f"  implemented: {done}   candidates: {len(ideas) - done}")
    for ns in ("JA", "JP", "J", "A"):
        n = sum(1 for i in ideas if i["ns"] == ns)
        if n:
            print(f"  {ns:<3} {n:4d}")
    print(f"  index: {INDEX_FILE.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
