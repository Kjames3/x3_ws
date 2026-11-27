"""Furniture look V2 (src/web/reality-furniture.js): runs the node check."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _node_major():
    node = shutil.which('node')
    if node is None:
        return 0
    out = subprocess.run([node, '--version'], capture_output=True, text=True).stdout
    m = re.match(r'v(\d+)', out.strip())
    return int(m.group(1)) if m else 0


# reality-structure.js (required by the check) uses optional chaining.
@pytest.mark.skipif(_node_major() < 16, reason='needs node >= 16')
def test_reality_furniture_look_check():
    subprocess.run(['node', 'tests/web/reality_furniture_look_check.cjs'], cwd=ROOT, check=True)
