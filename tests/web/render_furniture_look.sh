#!/usr/bin/env bash
# Render before/after screenshots of the Reality-view furniture stand-ins
# with headless Chrome (software GL). Usage: tests/web/render_furniture_look.sh [OUT_DIR]
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=${1:-artifacts/surroundings-2026-11-27/furniture-look}
CHROME=${CHROME:-$(command -v google-chrome || command -v chromium || command -v chromium-browser)}
mkdir -p "$OUT"
PAGE="file://$PWD/tests/web/furniture_look_harness.html"
shot() {  # name query
    local prof; prof=$(mktemp -d)
    "$CHROME" --headless=new --no-sandbox --user-data-dir="$prof" --use-gl=angle --use-angle=swiftshader \
        --enable-unsafe-swiftshader --allow-file-access-from-files --hide-scrollbars \
        --window-size=1280,760 --virtual-time-budget=2500 --run-all-compositor-stages-before-draw \
        --screenshot="$OUT/$1.png" "$PAGE?$2" >/dev/null 2>&1
    "$CHROME" --headless=new --no-sandbox --user-data-dir="$prof" --use-gl=angle --use-angle=swiftshader \
        --enable-unsafe-swiftshader --allow-file-access-from-files --virtual-time-budget=2500 \
        --dump-dom "$PAGE?$2" 2>/dev/null | grep -o '<pre id="stats">[^<]*' | sed "s/<pre id=\"stats\">/$1: /" >> "$OUT/stats.txt"
    rm -rf "$prof"
}
: > "$OUT/stats.txt"
for view in overview close top lineup; do
    shot "before_$view" "furnitureLookV2=0&view=$view"
    shot "after_$view" "furnitureLookV2=1&view=$view"
done
shot "after_fade_250ms" "furnitureLookV2=1&view=overview&freeze=250"
cat "$OUT/stats.txt"
