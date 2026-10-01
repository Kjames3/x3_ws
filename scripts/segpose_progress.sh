#!/usr/bin/env bash
# Laptop popup showing rough progress of the robot's seg-vs-pose scorer.
#   bash scripts/segpose_progress.sh [host] [steps-per-run]   # default x3, 6
# Use 2 steps per run for a --front-ends seg-merged rescore (merge + score),
# 5 for c3_depth_cap_test.py (far reference + 4 scoring passes).
# Counts log milestones: 4 detection passes per run ("person boxes"), then
# 2 scoring passes per run ("<run> done"). Detection dominates, so the bar
# runs fast at the end; the percentage is rough.
host="${1:-x3}"
log=/tmp/segpose-score.log
runs=$(ssh "$host" 'ls -d ~/x3_ws/artifacts/c3-captures-segpose/*/ | wc -l')
total=$((runs * ${2:-6}))
(
while :; do
    read -r boxes done alive last < <(ssh -o ConnectTimeout=5 "$host" \
        "b=\$(grep -cE 'person boxes|: merged |: far reference' $log); d=\$(grep -c ' done\$' $log); \
         a=\$(pgrep -fc 'python3 src/c3_(seg_vs_pose|depth_cap_test|calibration|edge_test)' || true); \
         l=\$(grep -E 'detect |done\$|: merged |: far reference' $log | tail -1 | cut -c1-60 | tr ' ' '_'); \
         echo \$b \$d \$a \$l" 2>/dev/null) || { echo "# robot unreachable, retrying"; sleep 15; continue; }
    n=$((boxes + done)); pct=$((100 * n / total)); [ "$pct" -gt 99 ] && pct=99
    if [ "${alive:-0}" = 0 ]; then
        # Both scorers print their results table last.
        if ssh "$host" "tail -1 $log | grep -q '^|'"; then
            echo 100; echo "# Done: results table written"
        else
            echo "# Scorer stopped without results -- check $log on the robot"
        fi
        break
    fi
    echo "$pct"
    echo "# ${n}/${total} steps  (${last//_/ })"
    sleep 20
done
) | zenity --progress --title="Seg vs pose scoring on $host" --width=480 \
      --text="Starting..." --percentage=0 --auto-kill 2>/dev/null
notify-send "Seg vs pose scoring" "Finished (or stopped) on $host" 2>/dev/null || true
