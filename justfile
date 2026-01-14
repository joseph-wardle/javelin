set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

build_root := "build"
default_preset := "dev"
sandbox_target := "javelin_sandbox"
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

run preset=default_preset *args: (build preset)
    #!/usr/bin/env bash
    set -euo pipefail
    bin_a="{{ build_root }}/{{ preset }}/examples/{{ sandbox_target }}"
    bin_b="{{ build_root }}/{{ preset }}/{{ sandbox_target }}"
    if [[ -x "$bin_a" ]]; then
        "$bin_a" {{ args }}
    elif [[ -x "$bin_b" ]]; then
        "$bin_b" {{ args }}
    else
        echo "Sandbox binary not found. Build target '{{ sandbox_target }}' first." >&2
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
    rg --files -0 {{ tidy_globs }} -g '!build/**' | \
        xargs -0 clang-tidy -p "{{ build_root }}/{{ preset }}"
