#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Literal, NamedTuple, cast

import numpy as np
import PyOpenColorIO as OCIO


class DisplayView(NamedTuple):
    display: str
    view: str


@dataclass(frozen=True)
class TextureInfo2D:
    texture_name: str
    sampler_name: str
    width: int
    height: int
    channel: str
    dimensions: str
    interpolation: str
    npy_file: str
    suggested_gl_target: Literal["GL_TEXTURE_1D", "GL_TEXTURE_2D"]


@dataclass(frozen=True)
class TextureInfo3D:
    texture_name: str
    sampler_name: str
    edge_len: int
    interpolation: str
    npy_file: str
    suggested_gl_target: Literal["GL_TEXTURE_3D"] = "GL_TEXTURE_3D"


@dataclass(frozen=True)
class UniformInfo:
    name: str
    type: str
    buffer_offset: int
    value: Any  # JSON-serializable (float/bool/list)


@dataclass(frozen=True)
class Manifest:
    ocio_version: str
    ocio_config: str
    src: str
    display: str
    view: str
    glsl_language: str
    shader_function: str
    resource_prefix: str
    uniform_buffer_size: int
    uniforms: list[UniformInfo]
    textures_2d: list[TextureInfo2D]
    textures_3d: list[TextureInfo3D]


def _fail(msg: str) -> None:
    raise SystemExit(f"error: {msg}")


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def _load_config(config_spec: str) -> OCIO.Config:
    """
    config_spec may be:
      - an OCIO URI (e.g. ocio://cg-config-latest, ocio://studio-config-latest)
      - a filesystem path to a .ocio / .ocioz
    """
    try:
        # CreateFromFile supports OCIO URIs for built-ins too. :contentReference[oaicite:4]{index=4}
        return OCIO.Config.CreateFromFile(config_spec)
    except Exception as e:
        _fail(f"could not load OCIO config '{config_spec}': {e}")


def _iter_displays(config: OCIO.Config) -> list[str]:
    # OCIO exposes display iteration; method names vary a bit across bindings.
    # We'll support both patterns.
    if hasattr(config, "getNumDisplays") and hasattr(config, "getDisplay"):
        n = int(getattr(config, "getNumDisplays")())
        return [str(getattr(config, "getDisplay")(i)) for i in range(n)]
    if hasattr(config, "getDisplays"):
        return [str(d) for d in getattr(config, "getDisplays")()]
    # Last resort: default only.
    return [str(config.getDefaultDisplay())]


def _iter_views(config: OCIO.Config, display: str) -> list[str]:
    if hasattr(config, "getNumViews") and hasattr(config, "getView"):
        n = int(getattr(config, "getNumViews")(display))
        return [str(getattr(config, "getView")(display, i)) for i in range(n)]
    if hasattr(config, "getViews"):
        return [str(v) for v in getattr(config, "getViews")(display)]
    return [str(config.getDefaultView(display))]


def _pick_display_view(
    config: OCIO.Config,
    display_opt: str | None,
    view_opt: str | None,
) -> DisplayView:
    displays = _iter_displays(config)
    if not displays:
        _fail("OCIO config has no displays")

    def norm(s: str) -> str:
        return s.casefold()

    if display_opt is not None:
        if display_opt not in displays:
            _fail(f"display '{display_opt}' not found. Available: {displays}")
        display = display_opt
    else:
        # Prefer an sRGB-ish display if present, else default. (For SDR window output.)
        srgb_like = [d for d in displays if "srgb" in norm(d)]
        display = srgb_like[0] if srgb_like else str(config.getDefaultDisplay())

    views = _iter_views(config, display)
    if not views:
        _fail(f"display '{display}' has no views")

    if view_opt is not None:
        if view_opt not in views:
            _fail(f"view '{view_opt}' not found for display '{display}'. Available: {views}")
        view = view_opt
    else:
        # Prefer an SDR-ish view if present, else display default.
        sdr_like = [v for v in views if "sdr" in norm(v) or "srgb" in norm(v)]
        view = sdr_like[0] if sdr_like else str(config.getDefaultView(display))

    return DisplayView(display=display, view=view)


def _make_gpu_shader_desc(
    gpu: OCIO.GPUProcessor,
    function_name: str,
    resource_prefix: str,
) -> OCIO.GpuShaderDesc:
    # OCIO GPU extraction example in docs uses GLSL 4.0 language. :contentReference[oaicite:5]{index=5}
    shader_desc = OCIO.GpuShaderDesc.CreateShaderDesc(language=OCIO.GPU_LANGUAGE_GLSL_4_0)
    if hasattr(shader_desc, "setFunctionName"):
        shader_desc.setFunctionName(function_name)
    if hasattr(shader_desc, "setResourcePrefix"):
        shader_desc.setResourcePrefix(resource_prefix)

    gpu.extractGpuShaderInfo(shader_desc)  # :contentReference[oaicite:6]{index=6}
    return shader_desc


def _serialize_uniforms(shader_desc: OCIO.GpuShaderDesc) -> list[UniformInfo]:
    out: list[UniformInfo] = []
    try:
        uniforms = shader_desc.getUniforms()  # :contentReference[oaicite:7]{index=7}
    except Exception:
        return out

    # UniformIterator yields tuples: (name, UniformData). :contentReference[oaicite:8]{index=8}
    for item in uniforms:
        name, udata = cast(tuple[Any, Any], item)
        name_s = str(name)
        t = str(getattr(udata, "type"))
        offset = int(getattr(udata, "bufferOffset"))

        # Export a JSON-friendly value. We do NOT attempt to pack a UBO here because
        # std140 packing/strides depend on the generated shader declarations; offsets
        # are still exported so you can build a UBO later if desired.
        value: Any
        if hasattr(udata, "getBool") and "BOOL" in t:
            value = bool(udata.getBool())
        elif hasattr(udata, "getDouble") and "DOUBLE" in t:
            value = float(udata.getDouble())
        elif hasattr(udata, "getFloat3") and "FLOAT3" in t:
            value = [float(x) for x in udata.getFloat3()]
        elif hasattr(udata, "getVectorFloat") and "VECTOR_FLOAT" in t:
            arr = np.asarray(udata.getVectorFloat(), dtype=np.float32)
            value = [float(x) for x in arr.tolist()]
        elif hasattr(udata, "getVectorInt") and "VECTOR_INT" in t:
            arr_i = np.asarray(udata.getVectorInt(), dtype=np.int32)
            value = [int(x) for x in arr_i.tolist()]
        else:
            # Fallback: best-effort string
            value = str(udata)

        out.append(UniformInfo(name=name_s, type=t, buffer_offset=offset, value=value))
    return out


def _infer_gl_target_for_2d(tex: OCIO.GpuShaderDesc.Texture) -> Literal["GL_TEXTURE_1D", "GL_TEXTURE_2D"]:
    # OCIO differentiates 1D vs 2D via TextureDimensions. :contentReference[oaicite:9]{index=9}
    dims = str(getattr(tex, "dimensions"))
    return "GL_TEXTURE_1D" if "TEXTURE_1D" in dims else "GL_TEXTURE_2D"


def _export_textures(
    shader_desc: OCIO.GpuShaderDesc,
    out_dir: Path,
) -> tuple[list[TextureInfo2D], list[TextureInfo3D]]:
    tex2d_infos: list[TextureInfo2D] = []
    tex3d_infos: list[TextureInfo3D] = []

    # 1D/2D textures
    for i, tex in enumerate(shader_desc.getTextures()):  # :contentReference[oaicite:10]{index=10}
        tex_name = str(tex.textureName)
        samp_name = str(tex.samplerName)
        w = int(tex.width)
        h = int(tex.height)
        channel = str(tex.channel)
        dims = str(tex.dimensions)
        interp = str(tex.interpolation)

        values = np.asarray(tex.getValues(), dtype=np.float32)  # :contentReference[oaicite:11]{index=11}
        # OCIO returns LUT data "as-is" for GPU upload. :contentReference[oaicite:12]{index=12}
        npy_file = f"tex2d_{i}_{_safe_name(tex_name)}.npy"
        np.save(out_dir / npy_file, values)

        tex2d_infos.append(
            TextureInfo2D(
                texture_name=tex_name,
                sampler_name=samp_name,
                width=w,
                height=h,
                channel=channel,
                dimensions=dims,
                interpolation=interp,
                npy_file=npy_file,
                suggested_gl_target=_infer_gl_target_for_2d(tex),
            )
        )

    # 3D textures
    get_3d = getattr(shader_desc, "get3DTextures", None)
    if callable(get_3d):
        for i, tex3 in enumerate(get_3d()):  # :contentReference[oaicite:13]{index=13}
            tex_name = str(tex3.textureName)
            samp_name = str(tex3.samplerName)
            edge = int(tex3.edgeLen)
            interp = str(tex3.interpolation)

            values = np.asarray(tex3.getValues(), dtype=np.float32)  # :contentReference[oaicite:14]{index=14}
            npy_file = f"tex3d_{i}_{_safe_name(tex_name)}.npy"
            np.save(out_dir / npy_file, values)

            tex3d_infos.append(
                TextureInfo3D(
                    texture_name=tex_name,
                    sampler_name=samp_name,
                    edge_len=edge,
                    interpolation=interp,
                    npy_file=npy_file,
                )
            )

    return tex2d_infos, tex3d_infos


def _safe_name(s: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]+", "_", s).strip("_") or "unnamed"


def _write_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def _wrap_fullscreen_fragment(shader_text: str, function_name: str) -> str:
    """
    Generates a minimal GLSL fragment shader that:
      - samples an input HDR/scene-linear texture
      - runs the OCIO function to get display-referred sRGB
    You will likely splice the OCIO block into your own shader instead.
    """
    return f"""#version 460 core

// ---- OCIO generated block (verbatim) ----
{shader_text}
// ---- end OCIO block ----

layout(location = 0) out vec4 outColor;

// Your scene-linear render target (ACEScg) bound by your app:
layout(binding = 0) uniform sampler2D uSceneColor;
in vec2 vUv;

void main()
{{
    vec4 scene = texture(uSceneColor, vUv);   // scene.rgb is ACEScg (scene-linear)
    outColor = {function_name}(scene);
}}
"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate OCIO GLSL + LUT textures for ACEScg -> sRGB display.")
    ap.add_argument("--config", default="ocio://cg-config-latest", help="OCIO config URI or path. Default: ocio://cg-config-latest")
    ap.add_argument("--display", default=None, help="Display name (optional).")
    ap.add_argument("--view", default=None, help="View name (optional).")
    ap.add_argument("--src", default=OCIO.ROLE_SCENE_LINEAR, help="Source colorspace/role. Default: ROLE_SCENE_LINEAR")
    ap.add_argument("--out-dir", default="ocio_out", help="Output directory.")
    ap.add_argument("--function-name", default="OCIODisplay", help="Name for the generated OCIO GLSL function.")
    ap.add_argument("--resource-prefix", default="ocio_", help="Prefix for generated resources to avoid collisions.")
    args = ap.parse_args(argv)

    out_dir = Path(args.out_dir)
    _ensure_dir(out_dir)

    config = _load_config(str(args.config))
    dv = _pick_display_view(config, args.display, args.view)

    # Build a processor for src -> (display, view). :contentReference[oaicite:15]{index=15}
    try:
        processor = config.getProcessor(str(args.src), dv.display, dv.view, OCIO.TRANSFORM_DIR_FORWARD)
    except Exception:
        # Fallback: DisplayViewTransform route. :contentReference[oaicite:16]{index=16}
        tr = OCIO.DisplayViewTransform()
        tr.setSrc(str(args.src))
        tr.setDisplay(dv.display)
        tr.setView(dv.view)
        processor = config.getProcessor(tr)

    gpu = processor.getDefaultGPUProcessor()  # :contentReference[oaicite:17]{index=17}
    shader_desc = _make_gpu_shader_desc(
        gpu=gpu,
        function_name=str(args.function_name),
        resource_prefix=str(args.resource_prefix),
    )

    shader_text = shader_desc.getShaderText()  # :contentReference[oaicite:18]{index=18}
    _write_text(out_dir / "ocio_shader.glsl", shader_text)
    _write_text(out_dir / "example_fullscreen.frag", _wrap_fullscreen_fragment(shader_text, str(args.function_name)))

    tex2d_infos, tex3d_infos = _export_textures(shader_desc, out_dir)

    uniforms = _serialize_uniforms(shader_desc)
    ubo_size = int(shader_desc.getUniformBufferSize())  # :contentReference[oaicite:19]{index=19}

    manifest = Manifest(
        ocio_version=str(getattr(OCIO, "__version__", "unknown")),
        ocio_config=str(args.config),
        src=str(args.src),
        display=dv.display,
        view=dv.view,
        glsl_language="GPU_LANGUAGE_GLSL_4_0",
        shader_function=str(args.function_name),
        resource_prefix=str(args.resource_prefix),
        uniform_buffer_size=ubo_size,
        uniforms=uniforms,
        textures_2d=tex2d_infos,
        textures_3d=tex3d_infos,
    )

    (out_dir / "manifest.json").write_text(
        json.dumps(asdict(manifest), indent=2, sort_keys=True),
        encoding="utf-8",
    )

    print(f"Wrote: {out_dir/'ocio_shader.glsl'}")
    print(f"Wrote: {out_dir/'example_fullscreen.frag'}")
    print(f"Wrote: {out_dir/'manifest.json'}")
    print(f"Textures: {len(tex2d_infos)} (1D/2D), {len(tex3d_infos)} (3D)")
    print(f"Selected display/view: {dv.display!r} / {dv.view!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

