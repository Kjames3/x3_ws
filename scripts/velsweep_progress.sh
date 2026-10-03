#!/bin/bash
# Live progress of the velocity-MLP sweep on REAL-1. Refreshes every 30 s.
#   bash scripts/velsweep_progress.sh [interval_s]
HOST="${VELSWEEP_HOST:-kinova@10.12.140.145}"
DIR="${VELSWEEP_DIR:-x3_velsweep4}"
INTERVAL="${1:-30}"

while true; do
    OUT=$(ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST" "
        cd ~/$DIR || exit 1
        pgrep -f sweep_velocity_prow.py >/dev/null && echo STATE running || echo STATE stopped
        echo TOTAL \$(~/miniconda3/envs/graspnet/bin/python -c 'import sweep_velocity_prow as s; print(len(s.CONFIGS))' 2>/dev/null)
        nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader | sed 's/^/GPU /'
        grep -E '^=====|^  sic |^    epoch|early stop|Traceback|sweep complete' out/sweep.log
    " 2>&1)
    clear
    if ! grep -q '^STATE' <<<"$OUT"; then
        echo "REAL-1 unreachable ($(date +%H:%M:%S)):"; echo "$OUT"; sleep "$INTERVAL"; continue
    fi
    awk -v now="$(date +%H:%M:%S)" '
        /^STATE/ {state=$2; next}
        /^TOTAL/ {total=$2; next}
        /^GPU/   {sub(/^GPU /,""); gpu=$0; next}
        /^=====/ {cur=$2; epoch="starting"; next}
        /^    epoch/ {epoch="epoch " $2 "  val " $6; next}
        /early stop/ {epoch="scoring"; next}
        /^  sic / {
            done++; secs=$NF; gsub(/[^0-9]/,"",secs); sum+=secs
            rows[done]=sprintf("  %-28s sic %s  near %s  far %s  fast %s  jitter %s  hot %s",
                               cur, $2, $4, $6, $8, $11, $13)
            cur=""; next
        }
        /Traceback/ {errs++; next}
        /sweep complete/ {complete=1}
        END {
            if (total=="") total=23
            pct = total ? int(100*done/total) : 0
            bar=""; for (i=0;i<40;i++) bar = bar (i < pct*40/100 ? "#" : "-")
            printf "Velocity MLP sweep #4 on REAL-1          %s\n\n", now
            printf "  [%s] %d/%d configs (%d%%)\n", bar, done, total, pct
            if (complete)            printf "  Status: COMPLETE\n"
            else if (state=="running") {
                printf "  Status: running   now: %s (%s)\n", cur, epoch
                if (done) printf "  ETA: about %d min (avg %d s per config)\n", (total-done)*sum/done/60, sum/done
            } else                   printf "  Status: STOPPED before finishing\n"
            if (errs) printf "  Errors: %d config(s) failed, see out/sweep.log\n", errs
            printf "  GPU: %s\n\n", gpu
            printf "  RMSE in m/s (lower is better); jitter = mean phantom speed, hot = share > 0.30 m/s\n"
            for (i=1;i<=done;i++) print rows[i]
        }' <<<"$OUT"
    grep -q 'sweep complete' <<<"$OUT" && break
    sleep "$INTERVAL"
done
