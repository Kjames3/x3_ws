"""Every element id the GUI scripts look up must exist in GUI.html.

A renamed or deleted id does not throw: getElementById returns null and the
control silently stops working.  Only literal ids are checked; ids built at
runtime (`gp-pill-${i}`, 'tab-' + name) are out of reach.
"""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'src' / 'web'
HTML_IDS = re.findall(r'\bid="([^"]+)"', (WEB / 'GUI.html').read_text())

# Looked up by main.js but no longer in GUI.html.  Every use is null-guarded,
# so these are dead code rather than broken controls.  Do not add to this list
# to make a failure go away -- fix the id.
KNOWN_ABSENT = {'auto-drive-wrapper', 'fps-camera-inline', 'fps-yolo-inline'}


def _lookups(script):
    text = (WEB / script).read_text()
    found = set(re.findall(r"getElementById\(\s*['\"]([^'\"]+)['\"]\s*\)", text))
    if re.search(r'const \$ = \(id\) => document\.getElementById\(id\)', text):
        found |= set(re.findall(r"\$\(\s*['\"]([^'\"]+)['\"]\s*\)", text))
    return found


def test_html_ids_are_unique():
    dupes = sorted({i for i in HTML_IDS if HTML_IDS.count(i) > 1})
    assert not dupes, f'duplicate ids in GUI.html: {dupes}'


def test_script_lookups_exist_in_html():
    missing = {}
    for script in sorted(p.name for p in WEB.glob('*.js')):
        gone = _lookups(script) - set(HTML_IDS) - KNOWN_ABSENT
        if gone:
            missing[script] = sorted(gone)
    assert not missing, f'ids looked up but not in GUI.html: {missing}'


def test_known_absent_list_is_current():
    stale = KNOWN_ABSENT & set(HTML_IDS)
    assert not stale, f'now present in GUI.html, drop from KNOWN_ABSENT: {sorted(stale)}'
    used = set().union(*(_lookups(p.name) for p in WEB.glob('*.js')))
    unused = KNOWN_ABSENT - used
    assert not unused, f'no longer looked up, drop from KNOWN_ABSENT: {sorted(unused)}'


def test_local_scripts_referenced_by_html_exist():
    html = (WEB / 'GUI.html').read_text()
    srcs = re.findall(r'<script src="([^"]+)"', html)
    local = [s.split('?')[0] for s in srcs if not s.startswith('http')]
    assert local
    for s in local:
        assert (WEB / s).is_file(), f'GUI.html loads {s}, which does not exist'
    # lidar-worker.js is loaded with new Worker(), not a <script> tag.
    assert (WEB / 'lidar-worker.js').is_file()
