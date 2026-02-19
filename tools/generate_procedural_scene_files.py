#!/usr/bin/env python3
"""
Generate deterministic .jvscene files that match the old procedural spawn model.

Requested outputs:
- Powers of two from 16 to 4096.
- Every 500 from 5000 to 15000.
"""

from __future__ import annotations

import math
from pathlib import Path

RADIUS_MIN = 0.25
RADIUS_MAX = 0.6
BOX_HALF_MIN = 0.2
BOX_HALF_MAX = 0.6
HEIGHT_MIN = 6.0
HEIGHT_MAX = 300.0
PILE_RADIUS = 2.5
TWO_PI = 2.0 * math.pi


def u32(value: int) -> int:
    return value & 0xFFFFFFFF


def hash_u32(value: int) -> int:
    x = u32(value)
    x ^= x >> 16
    x = u32(x * 0x7FEB352D)
    x ^= x >> 15
    x = u32(x * 0x846CA68B)
    x ^= x >> 16
    return u32(x)


def hash_to_unit(value: int) -> float:
    h = hash_u32(value)
    return float(h & 0x00FFFFFF) / float(0x01000000)


def spawn_seed(idx: int) -> int:
    return u32(idx * 747796405 + 2891336453)


def spawn_shape_kind(idx: int) -> str:
    seed = spawn_seed(idx)
    pick = hash_to_unit(seed ^ 0xC2B2AE35)
    return "sphere" if pick < 0.5 else "box"


def spawn_radius(idx: int) -> float:
    seed = spawn_seed(idx)
    rand_radius = hash_to_unit(seed)
    return RADIUS_MIN + rand_radius * (RADIUS_MAX - RADIUS_MIN)


def spawn_cube_half_extent(idx: int) -> float:
    seed = spawn_seed(idx)
    rand_half = hash_to_unit(seed ^ 0x1F123BB5)
    return BOX_HALF_MIN + rand_half * (BOX_HALF_MAX - BOX_HALF_MIN)


def spawn_position(idx: int) -> tuple[float, float, float]:
    seed = spawn_seed(idx)
    rand_height = hash_to_unit(seed ^ 0x9E3779B9)
    rand_r = hash_to_unit(seed ^ 0x85EBCA6B)
    rand_angle = hash_to_unit(seed ^ 0xC2B2AE35)

    height = HEIGHT_MIN + rand_height * (HEIGHT_MAX - HEIGHT_MIN)
    radius = math.sqrt(rand_r) * PILE_RADIUS
    angle = rand_angle * TWO_PI
    px = math.cos(angle) * radius
    pz = math.sin(angle) * radius
    return px, height, pz


def fmt_f32(value: float) -> str:
    if value == 0.0:
        value = 0.0
    return format(value, ".9g")


def scene_counts() -> list[int]:
    powers_of_two = [2**p for p in range(4, 13)]  # 16..4096
    large_counts = list(range(5000, 15001, 500))  # 5000..15000
    return powers_of_two + large_counts


def write_scene_file(path: Path, count: int) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("# javelin scene file (.jvscene)\n")
        f.write("# schema=v1 units=m one-record-per-line key=value\n")
        f.write(f"# source=legacy_procedural_spawn count={count}\n")
        f.write("scene version=1 units=m\n")
        f.write("\n")
        f.write("# shapes\n")

        for idx in range(count):
            shape_id = f"shape_{idx:05d}"
            kind = spawn_shape_kind(idx)
            if kind == "sphere":
                radius = spawn_radius(idx)
                f.write(f"shape id={shape_id} kind=sphere r={fmt_f32(radius)}\n")
            else:
                half = spawn_cube_half_extent(idx)
                half_text = fmt_f32(half)
                f.write(f"shape id={shape_id} kind=box hx={half_text} hy={half_text} hz={half_text}\n")

        f.write("\n")
        f.write("# bodies\n")
        for idx in range(count):
            body_id = f"body_{idx:05d}"
            shape_id = f"shape_{idx:05d}"
            px, py, pz = spawn_position(idx)
            f.write(
                "body "
                f"id={body_id} shape={shape_id} motion=dynamic material=0 mesh=0 "
                f"px={fmt_f32(px)} py={fmt_f32(py)} pz={fmt_f32(pz)} "
                "ox=0 oy=0 oz=0 ow=1 "
                "vx=0 vy=0 vz=0 "
                "wx=0 wy=0 wz=0\n"
            )


def main() -> None:
    out_dir = Path("assets/scenes/procedural")
    out_dir.mkdir(parents=True, exist_ok=True)

    counts = scene_counts()
    for count in counts:
        out_path = out_dir / f"pile_{count:05d}.jvscene"
        write_scene_file(out_path, count)
        print(f"wrote {out_path}")

    print(f"generated {len(counts)} files")


if __name__ == "__main__":
    main()
