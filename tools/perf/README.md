# Performance Harness

This harness provides repeatable before/after comparisons for benchmark medians/p95 and Tracy zone deltas.

## Commands

Capture one artifact set:

```bash
tools/perf/run_suite.sh capture --label my_run --preset dev --trace /path/to/capture.tracy
```

Compare two existing artifact sets:

```bash
tools/perf/run_suite.sh compare --before run_a --after run_b --max-median-drift-pct 3
```

Capture both sides and compare in one command:

```bash
tools/perf/run_suite.sh pair \
  --before-label run_a \
  --after-label run_b \
  --trace-before /path/to/before.tracy \
  --trace-after /path/to/after.tracy \
  --max-median-drift-pct 3
```

## Artifact Layout

Each run is stored under `tools/perf/artifacts/<label>/`:

- `manifest.json`: environment + benchmark arguments used
- `bench/*.json`: machine-readable benchmark outputs
- `logs/*.log`: raw benchmark stdout
- `tracy/inclusive.csv`: Tracy zone summary (inclusive)
- `tracy/self.csv`: Tracy zone summary (self time)

## Drift Gate

`compare.py` enforces a max absolute drift threshold across benchmark median metrics.

- default threshold: `3%`
- exit code `0`: pass
- exit code `2`: fail
