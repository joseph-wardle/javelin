#!/usr/bin/env python3
from __future__ import annotations

import argparse
import dataclasses
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Mapping, Sequence


@dataclasses.dataclass(frozen=True)
class NinjaLogEntry:
    start_ms: int
    end_ms: int
    output: str

    @property
    def dur_ms(self) -> int:
        return max(0, self.end_ms - self.start_ms)


@dataclasses.dataclass(frozen=True)
class BuildConfig:
    name: str
    cmake_build_type: str


_BUILD_CONFIGS: Mapping[str, BuildConfig] = {
    "debug": BuildConfig(name="debug", cmake_build_type="Debug"),
    "dev": BuildConfig(name="dev", cmake_build_type="RelWithDebInfo"),
    "release": BuildConfig(name="release", cmake_build_type="Release"),
}


def _require_project_root(root: Path) -> None:
    cmakelists = root / "CMakeLists.txt"
    if not cmakelists.is_file():
        raise RuntimeError(
            f"Expected to run from project root (missing {cmakelists}). "
            "cd to the repo root and try again."
        )


def _run(cmd: Sequence[str], *, cwd: Path | None = None) -> None:
    # Stream output directly; failures raise with a clean message.
    try:
        subprocess.run(cmd, cwd=cwd, check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Command failed (exit {e.returncode}): {' '.join(cmd)}") from e


def _cmake_configure(
        *,
        project_root: Path,
        build_dir: Path,
        cfg: BuildConfig,
        cmake: str,
        generator: str,
        c_compiler: str,
        cxx_compiler: str,
        jobs: int,
        build_examples: bool,
        enable_tracy: bool,
        extra_cxx_flags: str | None,
) -> float:
    cmd: list[str] = [
        cmake,
        "-S",
        str(project_root),
        "-B",
        str(build_dir),
        "-G",
        generator,
        f"-DCMAKE_C_COMPILER={c_compiler}",
        f"-DCMAKE_CXX_COMPILER={cxx_compiler}",
        f"-DCMAKE_BUILD_TYPE={cfg.cmake_build_type}",
        f"-DJAVELIN_BUILD_EXAMPLES={'ON' if build_examples else 'OFF'}",
        f"-DJAVELIN_ENABLE_TRACY={'ON' if enable_tracy else 'OFF'}",
    ]
    if extra_cxx_flags:
        cmd.append(f"-DCMAKE_CXX_FLAGS={extra_cxx_flags}")

    t0 = time.perf_counter()
    _run(cmd)
    return time.perf_counter() - t0


def _cmake_clean(*, build_dir: Path, cmake: str) -> float:
    t0 = time.perf_counter()
    # "clean" exists for Ninja/Makefile generators; ignore failure just in case.
    try:
        _run([cmake, "--build", str(build_dir), "--target", "clean"])
    except RuntimeError:
        pass
    return time.perf_counter() - t0


def _cmake_build(*, build_dir: Path, cmake: str, jobs: int) -> float:
    t0 = time.perf_counter()
    _run([cmake, "--build", str(build_dir), f"-j{jobs}"])
    return time.perf_counter() - t0


def _read_ninja_log(path: Path) -> list[NinjaLogEntry]:
    if not path.is_file():
        raise RuntimeError(
            f"{path} not found. This script requires the Ninja generator."
        )

    entries: list[NinjaLogEntry] = []

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 4:
                continue
            # Format: start\tend\trestat\toutput\tcommandhash
            try:
                start_ms = int(parts[0])
                end_ms = int(parts[1])
            except ValueError:
                continue
            output = parts[3]
            entries.append(NinjaLogEntry(start_ms=start_ms, end_ms=end_ms, output=output))

    if not entries:
        raise RuntimeError(f"{path} contained no entries (did the build do any work?).")
    return entries


def _wall_time_from_log(entries: Sequence[NinjaLogEntry]) -> float:
    min_start = min(e.start_ms for e in entries)
    max_end = max(e.end_ms for e in entries)
    return max(0.0, (max_end - min_start) / 1000.0)


def _ext_of_output(output: str) -> str:
    p = Path(output)
    ext = p.suffix.lower()
    return ext if ext else "<none>"


def _summarize_by_ext(entries: Sequence[NinjaLogEntry]) -> list[tuple[str, int, float, float]]:
    buckets: dict[str, list[int]] = defaultdict(list)
    for e in entries:
        buckets[_ext_of_output(e.output)].append(e.dur_ms)

    rows: list[tuple[str, int, float, float]] = []
    for ext, durs in buckets.items():
        total_s = sum(durs) / 1000.0
        avg_s = (total_s / len(durs)) if durs else 0.0
        rows.append((ext, len(durs), total_s, avg_s))

    rows.sort(key=lambda r: r[2], reverse=True)  # by total_s desc
    return rows


_QUERY_INPUT_RE = re.compile(r"^\s+input:\s+(?P<path>.+)$", re.IGNORECASE)


def _first_interesting_input_from_ninja_query(query_text: str) -> str | None:
    # Ninja's `-t query` output is human text. We try to grab the first "input:" line
    # that looks like a source-ish file.
    candidates: list[str] = []
    for line in query_text.splitlines():
        m = _QUERY_INPUT_RE.match(line)
        if not m:
            continue
        p = m.group("path").strip()
        candidates.append(p)

    if not candidates:
        return None

    def score(path: str) -> int:
        s = path.lower()
        # Prefer real sources over generated artifacts.
        if s.endswith((".cppm", ".cpp", ".cxx", ".cc", ".c")):
            return 0
        if s.endswith((".hpp", ".h", ".hh")):
            return 1
        if s.endswith((".pcm", ".ifc")):
            return 2
        return 3

    candidates.sort(key=score)
    return candidates[0]


def _ninja_query_input(*, ninja: str, build_dir: Path, output: str) -> str | None:
    try:
        proc = subprocess.run(
            [ninja, "-C", str(build_dir), "-t", "query", output],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except subprocess.CalledProcessError:
        return None
    return _first_interesting_input_from_ninja_query(proc.stdout)


def _print_table(rows: Sequence[Sequence[str]]) -> None:
    widths = [0] * max((len(r) for r in rows), default=0)
    for r in rows:
        for i, c in enumerate(r):
            widths[i] = max(widths[i], len(c))
    for r in rows:
        line = "  ".join(c.ljust(widths[i]) for i, c in enumerate(r))
        print(line)


def _write_trace_json(entries: Sequence[NinjaLogEntry], out_path: Path) -> None:
    # Chrome/Perfetto "traceEvents" format with complete events.
    # All events are on a single thread; it’s still useful for “what was slow”.
    events: list[dict[str, object]] = [
        {"ph": "M", "pid": 1, "tid": 1, "name": "process_name", "args": {"name": "ninja"}},
        {"ph": "M", "pid": 1, "tid": 1, "name": "thread_name", "args": {"name": "build"}},
    ]

    for e in entries:
        events.append(
            {
                "name": e.output,
                "cat": "ninja",
                "ph": "X",
                "ts": e.start_ms * 1000,  # us
                "dur": e.dur_ms * 1000,   # us
                "pid": 1,
                "tid": 1,
            }
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    import json  # local import to keep module load minimal

    out_path.write_text(json.dumps({"traceEvents": events}), encoding="utf-8")


def main(argv: Sequence[str]) -> int:
    ap = argparse.ArgumentParser(description="Clean-build and profile compile times using Ninja logs.")
    ap.add_argument(
        "--config",
        choices=sorted(_BUILD_CONFIGS.keys()),
        default="dev",
        help="Build configuration (default: dev = RelWithDebInfo).",
    )
    ap.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Build directory. Default: build/profile/<config>",
    )
    ap.add_argument("--jobs", type=int, default=(os.cpu_count() or 8), help="Parallel build jobs.")
    ap.add_argument("--top", type=int, default=20, help="Show top N slowest build steps.")
    ap.add_argument("--no-examples", action="store_true", help="Configure with -DJAVELIN_BUILD_EXAMPLES=OFF.")
    ap.add_argument("--no-tracy", action="store_true", help="Configure with -DJAVELIN_ENABLE_TRACY=OFF.")
    ap.add_argument("--ftime-trace", action="store_true", help="Add -ftime-trace to C++ flags (clang).")
    ap.add_argument("--cmake", default="cmake", help="cmake executable (default: cmake).")
    ap.add_argument("--ninja", default="ninja", help="ninja executable (default: ninja).")
    ap.add_argument("--generator", default="Ninja", help="CMake generator (default: Ninja).")
    ap.add_argument("--cc", default="clang", help="C compiler (default: clang).")
    ap.add_argument("--cxx", default="clang++", help="C++ compiler (default: clang++).")
    ap.add_argument(
        "--trace-out",
        type=Path,
        default=None,
        help="Write Perfetto/Chrome trace JSON here. Default: <build-dir>/ninja_trace.json",
    )
    ap.add_argument("--no-trace", action="store_true", help="Do not write trace JSON.")
    ap.add_argument("--no-query", action="store_true", help="Do not call 'ninja -t query' for top steps.")
    args = ap.parse_args(list(argv))

    project_root = Path.cwd()
    _require_project_root(project_root)

    cfg = _BUILD_CONFIGS[args.config]
    build_dir = args.build_dir or (project_root / "build" / "profile" / cfg.name)
    build_dir = build_dir.resolve()

    build_examples = not args.no_examples
    enable_tracy = not args.no_tracy
    extra_cxx_flags = "-ftime-trace" if args.ftime_trace else None

    trace_out = args.trace_out or (build_dir / "ninja_trace.json")

    print(f"== javelin build profile ==")
    print(f"config       : {cfg.name} ({cfg.cmake_build_type})")
    print(f"build dir    : {build_dir}")
    print(f"generator    : {args.generator}")
    print(f"compilers    : cc={args.cc}, cxx={args.cxx}")
    print(f"jobs         : {args.jobs}")
    print(f"examples     : {'ON' if build_examples else 'OFF'}")
    print(f"tracy        : {'ON' if enable_tracy else 'OFF'}")
    print(f"ftime-trace  : {'ON' if args.ftime_trace else 'OFF'}")
    print()

    build_dir.mkdir(parents=True, exist_ok=True)

    configure_s = _cmake_configure(
        project_root=project_root,
        build_dir=build_dir,
        cfg=cfg,
        cmake=args.cmake,
        generator=args.generator,
        c_compiler=args.cc,
        cxx_compiler=args.cxx,
        jobs=args.jobs,
        build_examples=build_examples,
        enable_tracy=enable_tracy,
        extra_cxx_flags=extra_cxx_flags,
    )

    # Ensure the next build is a clean rebuild and the log contains only this run.
    ninja_log = build_dir / ".ninja_log"
    if ninja_log.exists():
        ninja_log.unlink()

    clean_s = _cmake_clean(build_dir=build_dir, cmake=args.cmake)
    if ninja_log.exists():
        ninja_log.unlink()

    build_s = _cmake_build(build_dir=build_dir, cmake=args.cmake, jobs=args.jobs)

    entries = _read_ninja_log(ninja_log)
    wall_s = _wall_time_from_log(entries)

    print()
    print("=== Timing ===")
    print(f"configure wall time : {configure_s:.3f} s")
    print(f"clean wall time     : {clean_s:.3f} s")
    print(f"build wall time     : {build_s:.3f} s")
    print(f"ninja log wall time : {wall_s:.3f} s")
    print(f"logged steps        : {len(entries)}")

    print()
    print("=== Time by output extension (sorted by total time) ===")
    ext_rows = _summarize_by_ext(entries)
    table: list[list[str]] = [["ext", "count", "total(s)", "avg(s)"]]
    for ext, count, total_s, avg_s in ext_rows[:15]:
        table.append([ext, str(count), f"{total_s:.3f}", f"{avg_s:.3f}"])
    _print_table(table)

    print()
    print(f"=== Top {args.top} slowest build steps ===")
    top = sorted(entries, key=lambda e: e.dur_ms, reverse=True)[: args.top]

    rows: list[list[str]] = [["time(s)", "output", "input (best-effort)"]]
    for e in top:
        inp = ""
        if not args.no_query:
            maybe = _ninja_query_input(ninja=args.ninja, build_dir=build_dir, output=e.output)
            inp = maybe or ""
        rows.append([f"{e.dur_ms / 1000.0:.3f}", e.output, inp])
    _print_table(rows)

    if not args.no_trace:
        _write_trace_json(entries, trace_out)
        print()
        print(f"trace JSON written : {trace_out}")
        print("open with          : https://ui.perfetto.dev  (or chrome://tracing)")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        raise SystemExit(1)
