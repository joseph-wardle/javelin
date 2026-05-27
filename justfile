set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

build_root := "build"
default_preset := "dev"
sandbox_target := "javelin_sandbox"
scene_tool_target := "javelin_scene_tool"
capture_seconds := "10"
capture_fps := "60"
capture_width := "1280"
capture_height := "720"
fmt_globs := "-g '*.{c,cc,cpp,cxx,cppm,ixx,h,hpp,hxx}'"
tidy_globs := "-g '*.{c,cc,cpp,cxx,cppm,ixx}'"
fmt_style := "{BasedOnStyle: LLVM, IndentWidth: 4, TabWidth: 4, UseTab: Never, ColumnLimit: 120}"

_check-build-tools:
    @command -v cmake >/dev/null
    @command -v ninja >/dev/null
    @command -v clang >/dev/null
    @command -v clang++ >/dev/null
    @command -v mold >/dev/null

_check-format-tools:
    @command -v clang-format >/dev/null

_check-tidy-tools:
    @command -v clang-tidy >/dev/null

_check-capture-tools:
    @command -v ffmpeg >/dev/null

configure preset=default_preset: _check-build-tools
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{ preset }}" in
        debug) build_type="Debug" ;;
        dev) build_type="RelWithDebInfo" ;;
        release) build_type="Release" ;;
        *) echo "Unknown preset '{{ preset }}' (expected debug|dev|release)" >&2; exit 1 ;;
    esac
    cmake -S . -B "{{ build_root }}/{{ preset }}" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE="$build_type" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DCMAKE_C_COMPILER=clang \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_CXX_FLAGS="-stdlib=libc++" \
        -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libc++ -fuse-ld=mold" \
        -DCMAKE_SHARED_LINKER_FLAGS="-stdlib=libc++ -fuse-ld=mold" \
        -DCMAKE_MODULE_LINKER_FLAGS="-stdlib=libc++ -fuse-ld=mold"

build preset=default_preset: (configure preset)
    cmake --build "{{ build_root }}/{{ preset }}"

run preset=default_preset *args: (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail
    cmake --build "{{ build_root }}/{{ preset }}" --target "{{ sandbox_target }}"
    bin_a="{{ build_root }}/{{ preset }}/examples/{{ sandbox_target }}"
    bin_b="{{ build_root }}/{{ preset }}/{{ sandbox_target }}"
    if [[ -x "$bin_a" ]]; then
        "$bin_a" {{ args }}
    elif [[ -x "$bin_b" ]]; then
        "$bin_b" {{ args }}
    else
        echo "Sandbox binary not found after building target '{{ sandbox_target }}'." >&2
        exit 1
    fi

offline-render preset=default_preset *args: _check-capture-tools (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail
    scene_arg="assets/scenes/samples/stack.jvscene"
    frames_arg="600"
    output_arg="capture/offline"
    width_arg="1920"
    height_arg="1080"
    positional=()
    for arg in {{ args }}; do
        case "$arg" in
            scene=*) scene_arg="${arg#scene=}" ;;
            frames=*) frames_arg="${arg#frames=}" ;;
            output=*) output_arg="${arg#output=}" ;;
            width=*) width_arg="${arg#width=}" ;;
            height=*) height_arg="${arg#height=}" ;;
            *) positional+=("$arg") ;;
        esac
    done
    if (( ${#positional[@]} > 0 )); then scene_arg="${positional[0]}"; fi
    if (( ${#positional[@]} > 1 )); then frames_arg="${positional[1]}"; fi
    if (( ${#positional[@]} > 2 )); then output_arg="${positional[2]}"; fi
    if (( ${#positional[@]} > 3 )); then width_arg="${positional[3]}"; fi
    if (( ${#positional[@]} > 4 )); then height_arg="${positional[4]}"; fi
    frame_dir="${output_arg}.frames"
    video_path="${output_arg}.mp4"
    cmake --build "{{ build_root }}/{{ preset }}" --target "{{ sandbox_target }}"
    bin_a="{{ build_root }}/{{ preset }}/examples/{{ sandbox_target }}"
    bin_b="{{ build_root }}/{{ preset }}/{{ sandbox_target }}"
    if [[ -x "$bin_a" ]]; then
        "$bin_a" offline-render "$scene_arg" --frames="$frames_arg" --output-dir="$frame_dir" --width="$width_arg" --height="$height_arg"
    elif [[ -x "$bin_b" ]]; then
        "$bin_b" offline-render "$scene_arg" --frames="$frames_arg" --output-dir="$frame_dir" --width="$width_arg" --height="$height_arg"
    else
        echo "Sandbox binary not found after building target '{{ sandbox_target }}'." >&2
        exit 1
    fi
    ffmpeg -y -framerate 60 -i "$frame_dir/frame_%06d.ppm" -c:v libx264 -pix_fmt yuv420p "$video_path"
    rm -f "$frame_dir"/frame_*.ppm "$frame_dir"/ffmpeg_command.txt
    rmdir "$frame_dir" 2>/dev/null || true

# Capture a 10-second clip for each sample scene, one scene per sandbox process.
capture-sample-scenes preset=default_preset: _check-capture-tools (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail

    frames_arg=$(( {{ capture_seconds }} * {{ capture_fps }} ))
    output_root="capture/scenes"

    sample_scenes=(
        assets/scenes/samples/stack.jvscene
        assets/scenes/samples/grid3d.jvscene
        assets/scenes/samples/grid2d_roll.jvscene
        assets/scenes/samples/pyr3d.jvscene
        assets/scenes/samples/cube_fall.jvscene
        assets/scenes/samples/sphere_fall.jvscene
        assets/scenes/samples/pyramid.jvscene
        assets/scenes/samples/domino.jvscene
        assets/scenes/samples/cradle.jvscene
        assets/scenes/samples/slope.jvscene
        assets/scenes/samples/stadium.jvscene
        assets/scenes/samples/pendulum_wall.jvscene
    )

    cmake --build "{{ build_root }}/{{ preset }}" --target "{{ sandbox_target }}"
    bin_a="{{ build_root }}/{{ preset }}/examples/{{ sandbox_target }}"
    bin_b="{{ build_root }}/{{ preset }}/{{ sandbox_target }}"
    if [[ -x "$bin_a" ]]; then
        sandbox_bin="$bin_a"
    elif [[ -x "$bin_b" ]]; then
        sandbox_bin="$bin_b"
    else
        echo "Sandbox binary not found after building target '{{ sandbox_target }}'." >&2
        exit 1
    fi

    capture_scene() {
        local scene_path="$1"
        local scene_name="${scene_path##*/}"
        scene_name="${scene_name%.jvscene}"

        local frame_dir="${output_root}/${scene_name}.frames"
        local video_path="${output_root}/${scene_name}.mp4"

        rm -rf "$frame_dir"
        mkdir -p "$frame_dir"

        "$sandbox_bin" offline-render "$scene_path" \
            --frames="$frames_arg" \
            --output-dir="$frame_dir" \
            --width="{{ capture_width }}" \
            --height="{{ capture_height }}"

        ffmpeg -y -framerate "{{ capture_fps }}" -i "$frame_dir/frame_%06d.ppm" \
            -c:v libx264 -pix_fmt yuv420p "$video_path"
        rm -f "$frame_dir"/frame_*.ppm "$frame_dir"/ffmpeg_command.txt
        rmdir "$frame_dir" 2>/dev/null || true

        echo "Wrote $video_path"
    }

    for scene in "${sample_scenes[@]}"; do
        capture_scene "$scene"
    done

scene-tool preset=default_preset *args: (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail
    cmake --build "{{ build_root }}/{{ preset }}" --target "{{ scene_tool_target }}"
    bin_a="{{ build_root }}/{{ preset }}/examples/{{ scene_tool_target }}"
    bin_b="{{ build_root }}/{{ preset }}/{{ scene_tool_target }}"
    if [[ -x "$bin_a" ]]; then
        "$bin_a" {{ args }}
    elif [[ -x "$bin_b" ]]; then
        "$bin_b" {{ args }}
    else
        echo "Scene tool binary not found after building target '{{ scene_tool_target }}'." >&2
        exit 1
    fi

bench name preset=default_preset *args: (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail
    target="javelin_bench_{{ name }}"
    cmake --build "{{ build_root }}/{{ preset }}" --target "$target"
    bin_a="{{ build_root }}/{{ preset }}/bench/$target"
    bin_b="{{ build_root }}/{{ preset }}/$target"
    if [[ -x "$bin_a" ]]; then
        "$bin_a" {{ args }}
    elif [[ -x "$bin_b" ]]; then
        "$bin_b" {{ args }}
    else
        echo "Benchmark binary not found after building target '$target'." >&2
        exit 1
    fi

clean preset=default_preset:
    rm -rf "{{ build_root }}/{{ preset }}"

fmt mode="": _check-format-tools
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ "{{ mode }}" == "check" || "{{ mode }}" == "--check" ]]; then
        rg --files -0 {{ fmt_globs }} -g '!build/**' | \
            xargs -0 clang-format --dry-run --Werror --style='{{ fmt_style }}'
    else
        rg --files -0 {{ fmt_globs }} -g '!build/**' | \
            xargs -0 clang-format -i --style='{{ fmt_style }}'
    fi

tidy preset=default_preset: _check-tidy-tools (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail
    rg --files -0 {{ tidy_globs }} -g '!build/**' -g '!third_party/**' | \
        xargs -0 clang-tidy -p "{{ build_root }}/{{ preset }}"

scenes-generate-procedural counts="":
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ -n "{{ counts }}" ]]; then
        python3 tools/generate_procedural_scene_files.py --counts "{{ counts }}"
    else
        python3 tools/generate_procedural_scene_files.py
    fi

scenes-verify-roundtrip preset=default_preset: (configure preset)
    #!/usr/bin/env bash
    set -euo pipefail
    cmake --build "{{ build_root }}/{{ preset }}" --target "{{ scene_tool_target }}"
    bin_a="{{ build_root }}/{{ preset }}/examples/{{ scene_tool_target }}"
    bin_b="{{ build_root }}/{{ preset }}/{{ scene_tool_target }}"
    if [[ -x "$bin_a" ]]; then
        tool_bin="$bin_a"
    elif [[ -x "$bin_b" ]]; then
        tool_bin="$bin_b"
    else
        echo "Scene tool binary not found. Build target '{{ scene_tool_target }}' first." >&2
        exit 1
    fi

    sample_scenes=(
        assets/scenes/samples/stack.jvscene
        assets/scenes/samples/grid3d.jvscene
        assets/scenes/samples/grid2d_roll.jvscene
        assets/scenes/samples/pyr3d.jvscene
        assets/scenes/samples/cube_fall.jvscene
        assets/scenes/samples/sphere_fall.jvscene
    )
    for scene in "${sample_scenes[@]}"; do
        "$tool_bin" verify-roundtrip "$scene"
    done

    large_scene="assets/scenes/procedural/pile_15000.jvscene"
    if [[ ! -f "$large_scene" ]]; then
        echo "Generating '$large_scene' for large-scene verification"
        python3 tools/generate_procedural_scene_files.py --counts "15000"
    fi
    "$tool_bin" verify-roundtrip "$large_scene"
