#!/usr/bin/env bash
set -euo pipefail
POSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
POSE_PYTHON="$POSE_ROOT/.cache/pose-lite/venv/bin/python"
if [ ! -x "$POSE_PYTHON" ]; then
    echo 'Run bash scripts/setup_pose_lite.sh first.' >&2
    exit 1
fi
exec "$POSE_PYTHON" "$POSE_ROOT/src/pose_lite_prototype.py" "$@"
