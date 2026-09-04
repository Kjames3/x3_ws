#!/usr/bin/env bash
# Ask the local coding model to review the offline sweep-load benchmark.
# Read-only by design: it neither runs the benchmark nor edits the repository.
set -euo pipefail

runner="${OLLAMA_BIN:-/home/kamren/.local/bin/ollama}"
model="${LOCAL_CODE_MODEL:-qwen2.5-coder:7b}"
target="${1:-src/sweep_load_bench.py}"

if [[ ! -x "$runner" ]]; then
  echo "Ollama is not installed at $runner" >&2
  exit 1
fi
if [[ ! -f "$target" ]]; then
  echo "Target does not exist: $target" >&2
  exit 1
fi

{
  cat <<'PROMPT'
You are reviewing an offline Python benchmark used for a mobile robot project.
This is a READ-ONLY analysis. Do not suggest executing hardware actions or changing
motor, servo, CBF, deployment, or ROS live-stack code. Concentrate on whether the
benchmark can produce misleading performance measurements or fail to clean up its
own private child processes.

Return exactly these sections:
1. Findings — only concrete, evidence-based issues, with function names.
2. Proposed minimal patches — unified diffs only; no more than three.
3. Validation plan — commands safe to run locally/offline.
4. Risks and assumptions.

Here is the file to inspect:
PROMPT
  sed -n '1,2600p' "$target"
} | "$runner" run "$model"
