#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
ARTIFACT_ROOT_DEFAULT="$REPO_ROOT/tools/perf/artifacts"

# Stable default workloads for 4096-body profiling.
DYNAMIC_BVH_ARGS=(
    --bodies=4096
    --queries=2800
    --iterations=60
    --warmup=10
    --samples=5
    --seed=1337
    --world=8
    --body-radius=0.5
    --query-radius=0.8
)
CONTACT_SOLVER_ARGS=(
    --bodies=4096
    --manifolds=12000
    --iterations=60
    --warmup=10
    --samples=5
    --seed=1337
    --adaptive-epsilon=0.001
)
SLEEP_AWARE_ARGS=(
    --bodies=4096
    --awake-ratio=0.25
    --iterations=80
    --warmup=20
    --samples=5
    --seed=1337
)
ISLAND_SLEEP_ARGS=(
    --bodies=4096
    --ticks=20000
    --warmup=4000
    --samples=5
    --seed=1337
)

usage() {
    cat <<'EOF'
Run or compare repeatable performance captures.

Usage:
  tools/perf/run_suite.sh capture --label LABEL [--preset PRESET] [--artifact-root DIR] [--trace TRACE_FILE]
  tools/perf/run_suite.sh compare --before DIR_OR_LABEL --after DIR_OR_LABEL [--artifact-root DIR] [--max-median-drift-pct PCT]
  tools/perf/run_suite.sh pair [--before-label LABEL] [--after-label LABEL] [--preset PRESET] [--artifact-root DIR]
                           [--trace-before TRACE_FILE] [--trace-after TRACE_FILE] [--max-median-drift-pct PCT]

Notes:
  - 'capture' writes benchmark JSON artifacts (and optional tracy CSV exports) to tools/perf/artifacts/<label>.
  - 'compare' prints before/after benchmark metric tables and tracy zone deltas.
  - 'pair' captures before and after in one command, then compares and applies the median drift gate.
EOF
}

resolve_artifact_dir() {
    local input="$1"
    local artifact_root="$2"
    if [[ -d "$input" ]]; then
        (cd "$input" && pwd)
        return 0
    fi
    if [[ -d "$artifact_root/$input" ]]; then
        (cd "$artifact_root/$input" && pwd)
        return 0
    fi
    return 1
}

find_bench_binary() {
    local preset="$1"
    local bench_name="$2"
    local target="javelin_bench_${bench_name}"
    local path_a="$REPO_ROOT/build/$preset/bench/$target"
    local path_b="$REPO_ROOT/build/$preset/$target"
    if [[ -x "$path_a" ]]; then
        printf '%s\n' "$path_a"
        return 0
    fi
    if [[ -x "$path_b" ]]; then
        printf '%s\n' "$path_b"
        return 0
    fi
    return 1
}

ensure_tracy_csvexport() {
    if [[ -n "${TRACY_CSVEXPORT:-}" && -x "$TRACY_CSVEXPORT" ]]; then
        printf '%s\n' "$TRACY_CSVEXPORT"
        return 0
    fi

    local existing_candidates=(
        "$REPO_ROOT/build/tracy-csvexport/tracy-csvexport"
        "$REPO_ROOT/build/dev/tracy-csvexport/tracy-csvexport"
    )
    for candidate in "${existing_candidates[@]}"; do
        if [[ -x "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done

    local source_candidates=(
        "$REPO_ROOT/build/dev/_deps/tracy-src/csvexport"
        "$REPO_ROOT/build/_deps/tracy-src/csvexport"
    )
    local source_dir=""
    for candidate in "${source_candidates[@]}"; do
        if [[ -d "$candidate" ]]; then
            source_dir="$candidate"
            break
        fi
    done

    if [[ -z "$source_dir" ]]; then
        return 1
    fi

    local build_dir="$REPO_ROOT/build/tracy-csvexport"
    cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$build_dir" -j"$(nproc)" >/dev/null
    if [[ -x "$build_dir/tracy-csvexport" ]]; then
        printf '%s\n' "$build_dir/tracy-csvexport"
        return 0
    fi
    return 1
}

write_manifest() {
    local output_file="$1"
    local label="$2"
    local preset="$3"
    local trace_file="$4"

    local git_commit
    git_commit="$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || printf 'unknown')"

    OUTPUT_FILE="$output_file" \
    LABEL="$label" \
    PRESET="$preset" \
    TRACE_FILE="$trace_file" \
    GIT_COMMIT="$git_commit" \
    REPO_ROOT="$REPO_ROOT" \
    python3 - <<'PY'
import json
import os
import platform
from datetime import datetime, timezone

manifest = {
    "label": os.environ["LABEL"],
    "preset": os.environ["PRESET"],
    "git_commit": os.environ["GIT_COMMIT"],
    "repo_root": os.environ["REPO_ROOT"],
    "captured_at_utc": datetime.now(timezone.utc).isoformat(),
    "hostname": platform.node(),
    "platform": platform.platform(),
    "trace_file": os.environ["TRACE_FILE"] or None,
    "benchmarks": {
        "dynamic_bvh": [
            "--bodies=4096",
            "--queries=2800",
            "--iterations=60",
            "--warmup=10",
            "--samples=5",
            "--seed=1337",
            "--world=8",
            "--body-radius=0.5",
            "--query-radius=0.8",
        ],
        "contact_solver_kernel": [
            "--bodies=4096",
            "--manifolds=12000",
            "--iterations=60",
            "--warmup=10",
            "--samples=5",
            "--seed=1337",
            "--adaptive-epsilon=0.001",
        ],
        "sleep_aware_collision": [
            "--bodies=4096",
            "--awake-ratio=0.25",
            "--iterations=80",
            "--warmup=20",
            "--samples=5",
            "--seed=1337",
        ],
        "island_sleep_wake": [
            "--bodies=4096",
            "--ticks=20000",
            "--warmup=4000",
            "--samples=5",
            "--seed=1337",
        ],
    },
}

with open(os.environ["OUTPUT_FILE"], "w", encoding="utf-8") as f:
    json.dump(manifest, f, indent=2)
    f.write("\n")
PY
}

run_bench_capture() {
    local output_dir="$1"
    local preset="$2"
    local bench_name="$3"
    shift 3

    local bench_bin
    if ! bench_bin="$(find_bench_binary "$preset" "$bench_name")"; then
        echo "error: benchmark binary not found for '$bench_name' in preset '$preset'" >&2
        return 1
    fi

    local json_out="$output_dir/bench/${bench_name}.json"
    local log_out="$output_dir/logs/${bench_name}.log"

    echo "[perf] ${bench_name}"
    "$bench_bin" "$@" "--json-out=${json_out}" | tee "$log_out"
}

capture_suite() {
    local label="$1"
    local preset="$2"
    local artifact_root="$3"
    local trace_file="$4"

    local output_dir="$artifact_root/$label"
    if [[ -e "$output_dir" ]]; then
        echo "error: output directory already exists: $output_dir" >&2
        return 1
    fi

    mkdir -p "$output_dir/bench" "$output_dir/logs" "$output_dir/tracy"

    echo "[perf] building preset '$preset'"
    (cd "$REPO_ROOT" && just build "$preset")

    run_bench_capture "$output_dir" "$preset" dynamic_bvh "${DYNAMIC_BVH_ARGS[@]}"
    run_bench_capture "$output_dir" "$preset" contact_solver_kernel "${CONTACT_SOLVER_ARGS[@]}"
    run_bench_capture "$output_dir" "$preset" sleep_aware_collision "${SLEEP_AWARE_ARGS[@]}"
    run_bench_capture "$output_dir" "$preset" island_sleep_wake "${ISLAND_SLEEP_ARGS[@]}"

    if [[ -n "$trace_file" ]]; then
        if [[ ! -f "$trace_file" ]]; then
            echo "error: trace file not found: $trace_file" >&2
            return 1
        fi

        local tracy_csvexport
        if ! tracy_csvexport="$(ensure_tracy_csvexport)"; then
            echo "error: could not find or build tracy-csvexport" >&2
            return 1
        fi

        echo "[perf] tracy export: $trace_file"
        "$tracy_csvexport" "$trace_file" >"$output_dir/tracy/inclusive.csv"
        "$tracy_csvexport" -e "$trace_file" >"$output_dir/tracy/self.csv"
        printf '%s\n' "$trace_file" >"$output_dir/tracy/source_trace.txt"
    fi

    write_manifest "$output_dir/manifest.json" "$label" "$preset" "$trace_file"
    echo "[perf] capture complete: $output_dir"
}

compare_suite() {
    local before_input="$1"
    local after_input="$2"
    local artifact_root="$3"
    local max_median_drift_pct="$4"

    local before_dir
    if ! before_dir="$(resolve_artifact_dir "$before_input" "$artifact_root")"; then
        echo "error: unable to resolve before artifact directory: $before_input" >&2
        return 1
    fi
    local after_dir
    if ! after_dir="$(resolve_artifact_dir "$after_input" "$artifact_root")"; then
        echo "error: unable to resolve after artifact directory: $after_input" >&2
        return 1
    fi

    python3 "$SCRIPT_DIR/compare.py" \
        --before "$before_dir" \
        --after "$after_dir" \
        --max-median-drift-pct "$max_median_drift_pct"
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

command="$1"
shift

case "$command" in
capture)
    label=""
    preset="dev"
    artifact_root="$ARTIFACT_ROOT_DEFAULT"
    trace_file=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
        --label)
            label="$2"
            shift 2
            ;;
        --preset)
            preset="$2"
            shift 2
            ;;
        --artifact-root)
            artifact_root="$2"
            shift 2
            ;;
        --trace)
            trace_file="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option for capture: $1" >&2
            exit 1
            ;;
        esac
    done

    if [[ -z "$label" ]]; then
        echo "error: --label is required for capture" >&2
        exit 1
    fi

    mkdir -p "$artifact_root"
    capture_suite "$label" "$preset" "$artifact_root" "$trace_file"
    ;;
compare)
    before_input=""
    after_input=""
    artifact_root="$ARTIFACT_ROOT_DEFAULT"
    max_median_drift_pct="3.0"

    while [[ $# -gt 0 ]]; do
        case "$1" in
        --before)
            before_input="$2"
            shift 2
            ;;
        --after)
            after_input="$2"
            shift 2
            ;;
        --artifact-root)
            artifact_root="$2"
            shift 2
            ;;
        --max-median-drift-pct)
            max_median_drift_pct="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option for compare: $1" >&2
            exit 1
            ;;
        esac
    done

    if [[ -z "$before_input" || -z "$after_input" ]]; then
        echo "error: --before and --after are required for compare" >&2
        exit 1
    fi

    compare_suite "$before_input" "$after_input" "$artifact_root" "$max_median_drift_pct"
    ;;
pair)
    before_label="before_$(date -u +%Y%m%dT%H%M%SZ)"
    after_label="after_$(date -u +%Y%m%dT%H%M%SZ)"
    preset="dev"
    artifact_root="$ARTIFACT_ROOT_DEFAULT"
    trace_before=""
    trace_after=""
    max_median_drift_pct="3.0"

    while [[ $# -gt 0 ]]; do
        case "$1" in
        --before-label)
            before_label="$2"
            shift 2
            ;;
        --after-label)
            after_label="$2"
            shift 2
            ;;
        --preset)
            preset="$2"
            shift 2
            ;;
        --artifact-root)
            artifact_root="$2"
            shift 2
            ;;
        --trace-before)
            trace_before="$2"
            shift 2
            ;;
        --trace-after)
            trace_after="$2"
            shift 2
            ;;
        --max-median-drift-pct)
            max_median_drift_pct="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option for pair: $1" >&2
            exit 1
            ;;
        esac
    done

    mkdir -p "$artifact_root"
    capture_suite "$before_label" "$preset" "$artifact_root" "$trace_before"
    capture_suite "$after_label" "$preset" "$artifact_root" "$trace_after"
    compare_suite "$before_label" "$after_label" "$artifact_root" "$max_median_drift_pct"
    ;;
-h | --help)
    usage
    ;;
*)
    echo "error: unknown command '$command'" >&2
    usage
    exit 1
    ;;
esac
