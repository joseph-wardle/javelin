#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

DEFAULT_TRACY_ZONES: tuple[str, ...] = (
    "Physics tick",
    "Physics broad phase parallel",
    "Physics broad phase query",
    "Physics broad phase update",
    "Physics solve",
    "Physics solve positions",
    "Physics narrow phase",
    "Physics refresh manifold persistence",
    "Physics normalize candidate pairs",
    "Physics sort manifolds",
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Compare two perf harness artifact directories.")
    ap.add_argument("--before", required=True, help="Before artifact directory (path).")
    ap.add_argument("--after", required=True, help="After artifact directory (path).")
    ap.add_argument(
        "--max-median-drift-pct",
        type=float,
        default=3.0,
        help="Fail if any benchmark median metric drifts above this absolute percentage (default: 3.0).",
    )
    ap.add_argument(
        "--tracy-zones",
        default=",".join(DEFAULT_TRACY_ZONES),
        help="Comma-separated zone names to compare from tracy CSV exports.",
    )
    return ap.parse_args()


def fail(msg: str) -> int:
    print(f"error: {msg}")
    return 1


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_bench_metrics(run_dir: Path) -> dict[str, dict[str, float]]:
    bench_dir = run_dir / "bench"
    out: dict[str, dict[str, float]] = {}
    if not bench_dir.is_dir():
        return out

    for json_file in sorted(bench_dir.glob("*.json")):
        data = load_json(json_file)
        bench = str(data.get("bench", json_file.stem))
        compare_metrics = data.get("compare_metrics", {})
        if not isinstance(compare_metrics, dict):
            continue

        numeric_metrics: dict[str, float] = {}
        for key, value in compare_metrics.items():
            if isinstance(value, (int, float)):
                numeric_metrics[str(key)] = float(value)

        if numeric_metrics:
            out[bench] = numeric_metrics

    return out


def load_tracy_stats(csv_path: Path) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    if not csv_path.is_file():
        return out

    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row.get("name")
            if not name:
                continue
            try:
                mean_ns = float(row["mean_ns"])
                total_ns = float(row["total_ns"])
                counts = float(row["counts"])
            except (KeyError, ValueError):
                continue
            out[name] = {
                "mean_ms": mean_ns / 1e6,
                "total_s": total_ns / 1e9,
                "count": counts,
            }
    return out


def pct_delta(before: float, after: float) -> float:
    if math.isclose(before, 0.0, abs_tol=1e-15):
        if math.isclose(after, 0.0, abs_tol=1e-15):
            return 0.0
        return math.nan
    return ((after - before) / before) * 100.0


def format_float(value: float, decimals: int = 3) -> str:
    if math.isnan(value):
        return "n/a"
    return f"{value:.{decimals}f}"


def print_table(headers: list[str], rows: Iterable[list[str]]) -> None:
    row_list = list(rows)
    if not row_list:
        print("(no data)")
        return

    widths = [len(h) for h in headers]
    for row in row_list:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))

    def render(cells: list[str]) -> str:
        return "  ".join(cell.ljust(widths[i]) for i, cell in enumerate(cells))

    print(render(headers))
    print(render(["-" * w for w in widths]))
    for row in row_list:
        print(render(row))


def compare_bench(before_dir: Path, after_dir: Path) -> tuple[list[list[str]], list[float]]:
    before = load_bench_metrics(before_dir)
    after = load_bench_metrics(after_dir)

    rows: list[list[str]] = []
    median_drifts: list[float] = []

    common_benches = sorted(set(before) & set(after))
    for bench in common_benches:
        common_metrics = sorted(set(before[bench]) & set(after[bench]))
        for metric in common_metrics:
            before_value = before[bench][metric]
            after_value = after[bench][metric]
            delta = pct_delta(before_value, after_value)
            rows.append(
                [
                    bench,
                    metric,
                    format_float(before_value),
                    format_float(after_value),
                    format_float(delta),
                ]
            )
            if "median" in metric.lower() and not math.isnan(delta):
                median_drifts.append(abs(delta))

    return rows, median_drifts


def compare_tracy(before_dir: Path, after_dir: Path, zones: list[str]) -> tuple[list[list[str]], list[list[str]]]:
    before_inclusive = load_tracy_stats(before_dir / "tracy" / "inclusive.csv")
    after_inclusive = load_tracy_stats(after_dir / "tracy" / "inclusive.csv")
    before_self = load_tracy_stats(before_dir / "tracy" / "self.csv")
    after_self = load_tracy_stats(after_dir / "tracy" / "self.csv")

    def build_rows(lhs: dict[str, dict[str, float]], rhs: dict[str, dict[str, float]]) -> list[list[str]]:
        rows: list[list[str]] = []
        for zone in zones:
            if zone not in lhs or zone not in rhs:
                continue
            before_mean = lhs[zone]["mean_ms"]
            after_mean = rhs[zone]["mean_ms"]
            delta = pct_delta(before_mean, after_mean)
            rows.append(
                [
                    zone,
                    format_float(before_mean),
                    format_float(after_mean),
                    format_float(delta),
                ]
            )
        return rows

    return build_rows(before_inclusive, after_inclusive), build_rows(before_self, after_self)


def main() -> int:
    args = parse_args()
    before_dir = Path(args.before)
    after_dir = Path(args.after)

    if not before_dir.is_dir():
        return fail(f"before directory does not exist: {before_dir}")
    if not after_dir.is_dir():
        return fail(f"after directory does not exist: {after_dir}")

    zones = [z.strip() for z in str(args.tracy_zones).split(",") if z.strip()]
    if not zones:
        zones = list(DEFAULT_TRACY_ZONES)

    print(f"before: {before_dir}")
    print(f"after : {after_dir}")
    print("")

    bench_rows, median_drifts = compare_bench(before_dir, after_dir)
    print("Bench Metrics (before vs after)")
    print_table(
        ["bench", "metric", "before", "after", "delta_pct"],
        bench_rows,
    )
    print("")

    tracy_inclusive_rows, tracy_self_rows = compare_tracy(before_dir, after_dir, zones)

    print("Tracy Zone Deltas (inclusive mean_ms)")
    print_table(["zone", "before", "after", "delta_pct"], tracy_inclusive_rows)
    print("")

    print("Tracy Zone Deltas (self mean_ms)")
    print_table(["zone", "before", "after", "delta_pct"], tracy_self_rows)
    print("")

    threshold = args.max_median_drift_pct
    if median_drifts:
        max_drift = max(median_drifts)
    else:
        max_drift = math.nan

    print("Median Drift Gate")
    if math.isnan(max_drift):
        print("  no comparable median metrics were found")
        return 1

    print(f"  threshold: {threshold:.3f}%")
    print(f"  observed max absolute median drift: {max_drift:.3f}%")
    if max_drift > threshold:
        print("  result: FAIL")
        return 2

    print("  result: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
