#!/usr/bin/env bash
# =============================================================================
# check_large_files.sh — guard against committing bags and oversized binaries
# =============================================================================
# This repo legitimately carries large-ish binary assets (STL meshes up to
# ~18 MB, YOLO .pt/.onnx weights), so the size limit is set above those. What it
# is really here to stop is a rosbag getting committed by accident: recordings
# are hundreds of MB to multiple GB, and record_bag.sh has already been run with
# its output landing inside the workspace (see the "Rosbag Data Collection"
# section of README.md). A bag in git history is effectively permanent.
#
# Usage:
#   scripts/ci/check_large_files.sh          # check tracked files
#   MAX_MB=50 scripts/ci/check_large_files.sh
#
# Exit 0 = clean, 1 = violation found.
# =============================================================================

set -euo pipefail

# Largest tracked file today is ~17.9 MB (imu_link.STL). 25 MB leaves headroom
# for a comparable asset while still catching anything bag-sized.
MAX_MB="${MAX_MB:-25}"
MAX_BYTES=$((MAX_MB * 1024 * 1024))

# Rosbag / recording artefacts are never welcome regardless of size — a freshly
# started bag can be small for the first few seconds.
BLOCKED_PATTERNS=(
    '*.db3'
    '*.db3.zstd'
    '*.bag'
    '*.mcap'
)

cd "$(git rev-parse --show-toplevel)"

violations=0

# ── Blocked file types ─────────────────────────────────────────────────────────
for pattern in "${BLOCKED_PATTERNS[@]}"; do
    while IFS= read -r file; do
        [[ -z "$file" ]] && continue
        echo "  [BLOCKED TYPE] $file — rosbag/recording artefact must not be committed"
        violations=$((violations + 1))
    done < <(git ls-files -- "$pattern")
done

# ── Oversized files ────────────────────────────────────────────────────────────
while IFS= read -r -d '' file; do
    [[ -f "$file" ]] || continue        # skip submodule entries / deleted paths
    size=$(stat -c%s "$file" 2>/dev/null || echo 0)
    if (( size > MAX_BYTES )); then
        printf '  [TOO LARGE] %s — %.1f MB exceeds the %s MB limit\n' \
            "$file" "$(echo "$size" | awk '{print $1/1048576}')" "$MAX_MB"
        violations=$((violations + 1))
    fi
done < <(git ls-files -z)

# ── Result ─────────────────────────────────────────────────────────────────────
if (( violations > 0 )); then
    echo ""
    echo "======================================================================="
    echo "  FAILED — $violations file(s) should not be in the repository."
    echo ""
    echo "  Rosbags belong outside the workspace, e.g. ~/bags/ (the default"
    echo "  output of record_bag.sh). If a bag was committed, remove it from"
    echo "  the index before pushing:"
    echo "      git rm --cached <file> && echo '<file>' >> .gitignore"
    echo ""
    echo "  If a large asset is genuinely required, raise the limit:"
    echo "      MAX_MB=<n> scripts/ci/check_large_files.sh"
    echo "======================================================================="
    exit 1
fi

echo "  OK — no blocked types, no file over ${MAX_MB} MB ($(git ls-files | wc -l) tracked files)."
