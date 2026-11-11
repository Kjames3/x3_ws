"""Every repo path named in the agent rules must exist.

The rules (CLAUDE.md, the repo skills, the repo agents) name files in
backticks.  A rule that points at a file that is gone sends the reader
nowhere, and nothing else notices.  Only literal paths under tracked
top-level folders are checked; placeholders and globs are skipped.
"""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
TOP = ("src/", "scripts/", "notes/", "config/", "deploy/", "tests/", ".claude/")
PLACEHOLDER = set("<>*{}$") | {"..."}


def _rule_files():
    files = [ROOT / "CLAUDE.md"]
    files += sorted((ROOT / ".claude" / "skills").glob("*/SKILL.md"))
    files += sorted((ROOT / ".claude" / "agents").glob("*.md"))
    return [f for f in files if f.is_file()]


def _named_paths():
    found = []
    for f in _rule_files():
        text = f.read_text()
        tokens = re.findall(r"`([^`\n]+)`", text) + re.findall(r"\]\(([^)\s]+)\)", text)
        for tok in tokens:
            tok = tok.split(":")[0].strip()
            if " " in tok or not tok.startswith(TOP):
                continue
            if any(p in tok for p in PLACEHOLDER):
                continue
            found.append((f.relative_to(ROOT).as_posix(), tok))
    return sorted(set(found))


def test_rules_name_some_paths():
    assert len(_named_paths()) > 20


@pytest.mark.parametrize("rule_file,path", _named_paths())
def test_named_path_exists(rule_file, path):
    assert (ROOT / path).exists(), f"{rule_file} names `{path}`, which does not exist"
