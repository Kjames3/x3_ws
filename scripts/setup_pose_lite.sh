#!/usr/bin/env bash
set -euo pipefail
POSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
POSE_ENV="$POSE_ROOT/.cache/pose-lite/venv"
if command -v uv >/dev/null 2>&1; then
    if [ ! -x "$POSE_ENV/bin/python" ]; then uv venv "$POSE_ENV" --python 3.12; fi
    uv pip install --python "$POSE_ENV/bin/python" -r "$POSE_ROOT/scripts/requirements_pose_lite.txt"
else
    if [ ! -x "$POSE_ENV/bin/python" ]; then python3.12 -m venv "$POSE_ENV"; fi
    "$POSE_ENV/bin/python" -m pip install -r "$POSE_ROOT/scripts/requirements_pose_lite.txt"
fi
"$POSE_ENV/bin/python" - "$POSE_ROOT" <<'PY'
import hashlib, pathlib, sys, urllib.request
p = pathlib.Path(sys.argv[1]) / '.cache/pose-lite/pose_landmarker_lite.task'
expected = '59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a'
if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
    data = urllib.request.urlopen('https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task', timeout=60).read()
    if hashlib.sha256(data).hexdigest() != expected:
        raise SystemExit('Model checksum mismatch')
    tmp = p.with_suffix('.download')
    tmp.write_bytes(data)
    tmp.replace(p)
print('Pose Lite model verified:', p)
PY
