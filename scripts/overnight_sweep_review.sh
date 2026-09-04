#!/usr/bin/env bash
# Bounded, read-only overnight review of the offline sweep benchmark.
# It writes reports only; it never executes the benchmark or edits source files.
set -euo pipefail

runner="${OLLAMA_BIN:-/home/kamren/.local/bin/ollama}"
model="${LOCAL_CODE_MODEL:-qwen2.5-coder:7b}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target="$repo_root/src/sweep_load_bench.py"
report_dir="$repo_root/local_llm_runs/sweep_load_bench_$(date +%Y%m%d-%H%M%S)"

mkdir -p "$report_dir"

for _ in $(seq 1 180); do
  if "$runner" list | awk 'NR > 1 { print $1 }' | grep -qx "$model"; then
    break
  fi
  sleep 20
done

if ! "$runner" list | awk 'NR > 1 { print $1 }' | grep -qx "$model"; then
  echo "Model was not available after one hour: $model" >&2
  exit 1
fi

review() {
  local label="$1"
  local start="$2"
  local end="$3"
  local focus="$4"

  {
    printf '%s\n\n' \
      'You are conducting a READ-ONLY review of an offline Python benchmark for a mobile robot.' \
      'Do not propose executing hardware actions, accessing a robot, deploying, or modifying source files.' \
      "Focus: $focus" \
      'Identify only concrete defects that could make an offline measurement misleading, unsafe to interpret, or leave private child processes behind.' \
      'For each finding, cite the function and exact evidence. Then provide at most two minimal unified-diff proposals and safe local validation commands.' \
      "This is segment $label (lines $start-$end) of sweep_load_bench.py:" \
      ''
    sed -n "${start},${end}p" "$target"
  } | "$runner" run "$model" >"$report_dir/${label}.md"
}

review '01_process_lifecycle' 1 250 \
  'Process discovery, CPU accounting, process groups, cleanup, and failure handling.'
review '02_deskew_and_tf' 251 520 \
  'Synthetic geometry, timing, deskew calculations, TF handling, correctness of the comparison baseline.'
review '03_ros_and_summary' 521 797 \
  'Private ROS namespaces, octomap test fidelity, cleanup paths, result reporting, and misleading conclusions.'

printf 'Read-only review complete: %s\n' "$report_dir"
