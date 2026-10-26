"""Put src/ and every src/tools/<group>/ on sys.path.

Runtime modules live at the top of src/; offline and bench tools live in
src/tools/<group>/ and import each other by bare module name.
"""
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[1] / 'src'
for _d in [_SRC, *sorted(p for p in (_SRC / 'tools').iterdir() if p.is_dir())]:
    if str(_d) not in sys.path:
        sys.path.insert(0, str(_d))
