module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.world_grid_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.mat4;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;

namespace javelin::detail {
constexpr std::string_view kGridVertexShader = R"glsl(
#version 460 core
out vec2 v_ndc;
void main() {
    const vec2 verts[3] = vec2[](
        vec2(-1.0, -1.0),
        vec2(3.0, -1.0),
        vec2(-1.0, 3.0)
    );
    v_ndc = verts[gl_VertexID];
    gl_Position = vec4(v_ndc, 0.0, 1.0);
}
)glsl";

constexpr std::string_view kGridFragmentShader = R"glsl(
#version 460 core
in vec2 v_ndc;
uniform mat4 u_view_proj;
uniform mat4 u_inv_view_proj;
uniform vec3 u_color;
uniform vec3 u_camera_pos;
uniform float u_minor_cell;
uniform float u_major_cell;
uniform float u_minor_width_px;
uniform float u_major_width_px;
uniform float u_fade_start;
uniform float u_fade_end;
out vec4 frag_color;

float saturate(float x) { return clamp(x, 0.0, 1.0); }

float grid_coverage(vec2 world_xz, float cell_size, float width_px) {
    vec2 uv = world_xz / cell_size;
    vec2 duv = fwidth(uv);
    vec2 cell = abs(fract(uv) - 0.5);
    vec2 line_half = duv * width_px;
    vec2 aa = duv * 1.5;
    vec2 cov = smoothstep(line_half + aa, line_half - aa, 0.5 - cell);
    return max(cov.x, cov.y);
}

void main() {
    vec4 near_h = u_inv_view_proj * vec4(v_ndc, -1.0, 1.0);
    vec4 far_h = u_inv_view_proj * vec4(v_ndc, 1.0, 1.0);
    vec3 near = near_h.xyz / near_h.w;
    vec3 far = far_h.xyz / far_h.w;
    vec3 dir = normalize(far - near);
    if (abs(dir.y) < 1e-5) {
        discard;
    }

    float t = -near.y / dir.y;
    if (t <= 0.0) {
        discard;
    }

    vec3 world = near + dir * t;
    vec2 xz = world.xz;

    float dist = length(xz - u_camera_pos.xz);
    float fade = 1.0 - saturate((dist - u_fade_start) / max(0.0001, (u_fade_end - u_fade_start)));

    float minor = grid_coverage(xz, u_minor_cell, u_minor_width_px) * fade;
    float major = grid_coverage(xz, u_major_cell, u_major_width_px) * fade;

    float coverage = max(minor * 0.35, major * 0.9);
    if (coverage <= 1e-5) {
        discard;
    }
    frag_color = vec4(u_color, coverage);

    vec4 clip = u_view_proj * vec4(world, 1.0);
    float depth = clip.z / clip.w * 0.5 + 0.5;
    gl_FragDepth = clamp(depth, 0.0, 1.0);
}
)glsl";

// Precomputed via inverse OCIO display transform to preserve the pre-ACES look.
constexpr Vec3 kClearColor = Vec3{0.032089f, 0.031913f, 0.037314f};
constexpr Vec3 kGridColor = Vec3{0.132375f, 0.139544f, 0.169055f};

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
    log::error(render, "World grid shader compile failed: {}", info.data());
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
    log::error(render, "World grid shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}
} // namespace javelin::detail

export namespace javelin {

struct WorldGridPass final {
    struct Settings final {
        f32 minor_cell{1.0f};
        f32 major_cell{5.0f};
        f32 minor_width_px{1.0f};
        f32 major_width_px{1.6f};
        f32 fade_start{20.0f};
        f32 fade_end{50.0f};
        // ACEScg value precomputed to match the old sRGB display look.
        Vec3 color{detail::kGridColor};
    };

    Settings settings{};

    template <class Device> void init(Device &) {
        log::info(render, "Initializing World grid pass");
        create_shader_();
        create_grid_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down World grid pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0) {
            return;
        }

        ZoneScopedN("WorldGridPass");
        TracyGpuZone("WorldGridPass");
        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        const Vec3 clear_aces = detail::kClearColor;
        glClearColor(clear_aces.x, clear_aces.y, clear_aces.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (!ctx.debug.draw_grid || program_ == 0 || vao_ == 0) {
            return;
        }

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform3f(u_color_, settings.color.x, settings.color.y, settings.color.z);
        const Mat4 inv_view = inverse_or_identity(ctx.camera.view);
        const Vec3 camera_pos = transform_point_affine(inv_view, Vec3{0.0f, 0.0f, 0.0f});
        glUniform3f(u_camera_pos_, camera_pos.x, camera_pos.y, camera_pos.z);
        const Mat4 inv_view_proj = inverse_or_identity(ctx.camera.view_proj);
        glUniformMatrix4fv(u_inv_view_proj_, 1, GL_FALSE, inv_view_proj.data());
        glUniform1f(u_minor_cell_, settings.minor_cell);
        glUniform1f(u_major_cell_, settings.major_cell);
        glUniform1f(u_minor_width_px_, settings.minor_width_px);
        glUniform1f(u_major_width_px_, settings.major_width_px);
        glUniform1f(u_fade_start_, settings.fade_start);
        glUniform1f(u_fade_end_, settings.fade_end);

        glBindVertexArray(vao_);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertex_count_));
        glBindVertexArray(0);
        glUseProgram(0);
    }

  private:
    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const u32 vs = detail::compile_shader(GL_VERTEX_SHADER, detail::kGridVertexShader);
        const u32 fs = detail::compile_shader(GL_FRAGMENT_SHADER, detail::kGridFragmentShader);
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

        u_view_proj_ = glGetUniformLocation(program_, "u_view_proj");
        u_inv_view_proj_ = glGetUniformLocation(program_, "u_inv_view_proj");
        u_color_ = glGetUniformLocation(program_, "u_color");
        u_camera_pos_ = glGetUniformLocation(program_, "u_camera_pos");
        u_minor_cell_ = glGetUniformLocation(program_, "u_minor_cell");
        u_major_cell_ = glGetUniformLocation(program_, "u_major_cell");
        u_minor_width_px_ = glGetUniformLocation(program_, "u_minor_width_px");
        u_major_width_px_ = glGetUniformLocation(program_, "u_major_width_px");
        u_fade_start_ = glGetUniformLocation(program_, "u_fade_start");
        u_fade_end_ = glGetUniformLocation(program_, "u_fade_end");
    }

    void create_grid_() {
        if (vao_ != 0) {
            return;
        }

        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);
        glBindVertexArray(0);
        vertex_count_ = 3;
    }

    void release_() {
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
        vertex_count_ = 0;
        u_view_proj_ = -1;
        u_inv_view_proj_ = -1;
        u_color_ = -1;
        u_camera_pos_ = -1;
        u_minor_cell_ = -1;
        u_major_cell_ = -1;
        u_minor_width_px_ = -1;
        u_major_width_px_ = -1;
        u_fade_start_ = -1;
        u_fade_end_ = -1;
    }

  private:
    u32 program_{};
    u32 vao_{};
    i32 vertex_count_{};
    i32 u_view_proj_{-1};
    i32 u_inv_view_proj_{-1};
    i32 u_color_{-1};
    i32 u_camera_pos_{-1};
    i32 u_minor_cell_{-1};
    i32 u_major_cell_{-1};
    i32 u_minor_width_px_{-1};
    i32 u_major_width_px_{-1};
    i32 u_fade_start_{-1};
    i32 u_fade_end_{-1};
};

} // namespace javelin
