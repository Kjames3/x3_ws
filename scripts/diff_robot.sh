#!/usr/bin/env bash
# Report where the robot's checkout and this laptop's checkout diverge.
#
# The two machines drift constantly: the laptop carries the git history (the
# robot is dozens of commits behind on main) while the robot accumulates
# hand-edits that were never committed anywhere.  Neither side is uniformly
# "newer", so this script only REPORTS -- it never copies.  Read the output,
# then move individual files in whichever direction is actually correct.
#
#   bash scripts/diff_robot.sh            # summary
#   bash scripts/diff_robot.sh <path>     # unified diff of one file (robot -> laptop)
set -euo pipefail

HOST="${X3_HOST:-x3}"
RWS="${X3_WS:-x3_ws}"
LWS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ $# -ge 1 ]; then
    ssh "$HOST" "cat ~/$RWS/$1" | diff -u --label "robot/$1" - --label "laptop/$1" "$LWS/$1"
    exit 0
fi

# Source-ish files only.  Build output, logs, model blobs and *.bak snapshots
# are expected to differ and would drown the signal.
inventory() {
    cat <<'INNER'
cd ~/WS_PLACEHOLDER 2>/dev/null || cd LWS_PLACEHOLDER
find src scripts config *.sh *.md -type f \
  \( -name '*.py' -o -name '*.sh' -o -name '*.js' -o -name '*.html' -o -name '*.css' \
     -o -name '*.yaml' -o -name '*.yml' -o -name '*.xml' -o -name '*.service' \
     -o -name '*.rules' -o -name '*.md' -o -name '*.rviz' -o -name '*.urdf' \
     -o -name '*.xacro' -o -name '*.cpp' -o -name '*.hpp' \) 2>/dev/null \
  | grep -vE '(^|/)(build|install|log|logs|blobs|checkpoints|fixtures)/|\.bak|~$' \
  | grep -v ' ' \
  | sort | xargs -r md5sum
INNER
}

tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
inventory | sed "s|WS_PLACEHOLDER|$RWS|; s|LWS_PLACEHOLDER|$LWS|" > "$tmp/inv.sh"

bash "$tmp/inv.sh" 2>/dev/null | awk '{h=$1;$1="";sub(/^ +/,"");print h"\t"$0}' | sort -t$'\t' -k2 > "$tmp/L"
ssh "$HOST" 'bash -s' < "$tmp/inv.sh" 2>/dev/null | awk '{h=$1;$1="";sub(/^ +/,"");print h"\t"$0}' | sort -t$'\t' -k2 > "$tmp/R"
cut -f2 "$tmp/L" > "$tmp/Lp"; cut -f2 "$tmp/R" > "$tmp/Rp"

echo "=== git ==="
echo "  laptop HEAD: $(git -C "$LWS" log --oneline -1)"
echo "  robot  HEAD: $(ssh "$HOST" "git -C ~/$RWS log --oneline -1")"

echo
echo "=== on the robot only ==="
comm -13 "$tmp/Lp" "$tmp/Rp" | sed 's/^/  /'
echo
echo "=== on the laptop only ==="
comm -23 "$tmp/Lp" "$tmp/Rp" | sed 's/^/  /'
echo
echo "=== present on both, contents differ ==="
join -t$'\t' -1 2 -2 2 -o 0,1.1,2.1 "$tmp/L" "$tmp/R" \
  | awk -F'\t' '$2!=$3{print "  "$1}'
echo
echo "Inspect one with:  bash scripts/diff_robot.sh <path>"
