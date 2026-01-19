module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.display_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;

namespace javelin::detail {
constexpr std::string_view kDisplayVertexShader = R"glsl(
#version 460 core
out vec2 v_uv;
void main() {
    const vec2 verts[3] = vec2[](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    const vec2 pos = verts[gl_VertexID];
    v_uv = pos * 0.5 + 0.5;
    gl_Position = vec4(pos, 0.0, 1.0);
}
)glsl";

constexpr std::string_view kDisplayFragmentPrelude = R"glsl(
#version 460 core
in vec2 v_uv;
layout(location = 0) out vec4 frag_color;

uniform sampler2D u_scene_color;

// ---- OCIO generated block (verbatim) ----
)glsl";

constexpr std::string_view kDisplayFragmentSuffix = R"glsl(
// ---- end OCIO block ----

void main() {
    vec4 scene = texture(u_scene_color, v_uv);
    frag_color = OCIODisplay(scene);
}
)glsl";

// Paths and LUT sizes match assets/ocio/acescg_to_srgb/manifest.json.
constexpr std::string_view kOcioShaderPath = "assets/ocio/acescg_to_srgb/ocio_shader.glsl";
constexpr std::string_view kReachMPath = "assets/ocio/acescg_to_srgb/lut/tex2d_0_ocio_reach_m_table_0.bin";
constexpr std::string_view kGamutCuspPath = "assets/ocio/acescg_to_srgb/lut/tex2d_1_ocio_gamut_cusp_table_0.bin";
constexpr i32 kReachMSize = 362;
constexpr i32 kGamutCuspSize = 362;

std::string read_text_file(std::string_view path) noexcept {
    std::ifstream file{std::string{path}, std::ios::binary};
    if (!file) {
        log::error(render, "Display shader read failed: {}", path);
        return {};
    }

    std::string text{std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};
    if (text.empty()) {
        log::error(render, "Display shader is empty: {}", path);
    }
    return text;
}

std::vector<f32> read_f32_file(std::string_view path, const usize expected_count) noexcept {
    std::error_code ec;
    const auto byte_size = std::filesystem::file_size(std::string{path}, ec);
    if (ec) {
        log::error(render, "LUT size query failed: {}", path);
        return {};
    }
    if (byte_size == 0 || (byte_size % sizeof(f32)) != 0) {
        log::error(render, "LUT size mismatch: {}", path);
        return {};
    }

    const usize count = static_cast<usize>(byte_size / sizeof(f32));
    if (expected_count != 0 && count != expected_count) {
        log::warn(render, "LUT {} expected {} floats, got {}", path, expected_count, count);
    }

    std::vector<f32> data(count);
    std::ifstream file{std::string{path}, std::ios::binary};
    if (!file) {
        log::error(render, "LUT read failed: {}", path);
        return {};
    }
    file.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(byte_size));
    if (!file) {
        log::error(render, "LUT read incomplete: {}", path);
        return {};
    }
    return data;
}

u32 compile_shader(const GLenum type, const std::string_view source) noexcept {
    const GLuint shader = glCreateShader(type);
    const char *src = source.data();
    const GLint len = static_cast<GLint>(source.size());
    glShaderSource(shader, 1, &src, &len);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == GL_TRUE) {
        return shader;
    }

    std::array<char, 1024> info{};
    glGetShaderInfoLog(shader, static_cast<GLsizei>(info.size()), nullptr, info.data());
    log::error(render, "Display shader compile failed: {}", info.data());
    glDeleteShader(shader);
    return 0;
}

u32 link_program(const u32 vs, const u32 fs) noexcept {
    const GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == GL_TRUE) {
        glDetachShader(program, vs);
        glDetachShader(program, fs);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return program;
    }

    std::array<char, 1024> info{};
    glGetProgramInfoLog(program, static_cast<GLsizei>(info.size()), nullptr, info.data());
    log::error(render, "Display shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}
} // namespace javelin::detail

export namespace javelin {

struct DisplayPass final {
    template <class Device> void init(Device &) {
        log::info(render, "Display pass initialized");
        create_shader_();
        create_luts_();
        create_vao_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Display pass shutdown");
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_color == 0) {
            return;
        }

        ZoneScopedN("DisplayPass");
        TracyGpuZone("DisplayPass");
        if (!ctx.debug.apply_color_transform || program_ == 0 || vao_ == 0 || reach_m_tex_ == 0 ||
            gamut_cusp_tex_ == 0) {
            if (ctx.targets.scene_fbo == 0) {
                return;
            }
            glBindFramebuffer(GL_READ_FRAMEBUFFER, ctx.targets.scene_fbo);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
            glViewport(0, 0, ctx.extent.width, ctx.extent.height);
            glBlitFramebuffer(0, 0, ctx.extent.width, ctx.extent.height, 0, 0, ctx.extent.width, ctx.extent.height,
                              GL_COLOR_BUFFER_BIT, GL_NEAREST);
            return;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glDisable(GL_BLEND);
        glDisable(GL_FRAMEBUFFER_SRGB);

        glUseProgram(program_);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, ctx.targets.scene_color);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_1D, reach_m_tex_);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_1D, gamut_cusp_tex_);

        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_1D, 0);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_1D, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);

        glUseProgram(0);
        glDepthMask(GL_TRUE);
    }

  private:
    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const std::string ocio_block = detail::read_text_file(detail::kOcioShaderPath);
        if (ocio_block.empty()) {
            return;
        }

        std::string fragment_source;
        fragment_source.reserve(detail::kDisplayFragmentPrelude.size() + ocio_block.size() +
                                detail::kDisplayFragmentSuffix.size());
        fragment_source.append(detail::kDisplayFragmentPrelude);
        fragment_source.append(ocio_block);
        fragment_source.append(detail::kDisplayFragmentSuffix);

        const u32 vs = detail::compile_shader(GL_VERTEX_SHADER, detail::kDisplayVertexShader);
        const u32 fs = detail::compile_shader(GL_FRAGMENT_SHADER, fragment_source);
        if (vs == 0 || fs == 0) {
            if (vs != 0) {
                glDeleteShader(vs);
            }
            if (fs != 0) {
                glDeleteShader(fs);
            }
            return;
        }

        program_ = detail::link_program(vs, fs);
        if (program_ == 0) {
            return;
        }

        u_scene_color_ = glGetUniformLocation(program_, "u_scene_color");
        u_reach_m_ = glGetUniformLocation(program_, "ocio_reach_m_table_0Sampler");
        u_gamut_cusp_ = glGetUniformLocation(program_, "ocio_gamut_cusp_table_0Sampler");

        glUseProgram(program_);
        if (u_scene_color_ >= 0) {
            glUniform1i(u_scene_color_, 0);
        }
        if (u_reach_m_ >= 0) {
            glUniform1i(u_reach_m_, 1);
        }
        if (u_gamut_cusp_ >= 0) {
            glUniform1i(u_gamut_cusp_, 2);
        }
        glUseProgram(0);
    }

    void create_luts_() {
        if (reach_m_tex_ != 0 || gamut_cusp_tex_ != 0) {
            return;
        }

        const auto reach_m = detail::read_f32_file(detail::kReachMPath, static_cast<usize>(detail::kReachMSize));
        const auto gamut_cusp =
            detail::read_f32_file(detail::kGamutCuspPath, static_cast<usize>(detail::kGamutCuspSize) * 3);
        if (reach_m.empty() || gamut_cusp.empty()) {
            return;
        }

        glGenTextures(1, &reach_m_tex_);
        glBindTexture(GL_TEXTURE_1D, reach_m_tex_);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexImage1D(GL_TEXTURE_1D, 0, GL_R32F, detail::kReachMSize, 0, GL_RED, GL_FLOAT, reach_m.data());

        glGenTextures(1, &gamut_cusp_tex_);
        glBindTexture(GL_TEXTURE_1D, gamut_cusp_tex_);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB32F, detail::kGamutCuspSize, 0, GL_RGB, GL_FLOAT, gamut_cusp.data());

        glBindTexture(GL_TEXTURE_1D, 0);
    }

    void create_vao_() {
        if (vao_ != 0) {
            return;
        }

        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);
        glBindVertexArray(0);
    }

    void release_() {
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }
        if (reach_m_tex_ != 0) {
            glDeleteTextures(1, &reach_m_tex_);
            reach_m_tex_ = 0;
        }
        if (gamut_cusp_tex_ != 0) {
            glDeleteTextures(1, &gamut_cusp_tex_);
            gamut_cusp_tex_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
        u_scene_color_ = -1;
        u_reach_m_ = -1;
        u_gamut_cusp_ = -1;
    }

  private:
    u32 program_{};
    u32 vao_{};
    u32 reach_m_tex_{};
    u32 gamut_cusp_tex_{};
    i32 u_scene_color_{-1};
    i32 u_reach_m_{-1};
    i32 u_gamut_cusp_{-1};
};

} // namespace javelin
