module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.contact_debug_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.contact_debug;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;

namespace javelin::detail {
constexpr std::string_view kContactDebugVertexShader = R"glsl(
#version 460 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

uniform mat4 u_view_proj;
uniform float u_point_size;

out vec3 v_color;

void main() {
    gl_Position = u_view_proj * vec4(a_position, 1.0);
    gl_PointSize = u_point_size;
    v_color = a_color;
}
)glsl";

constexpr std::string_view kContactDebugFragmentShader = R"glsl(
#version 460 core
in vec3 v_color;
layout(location = 0) out vec4 frag_color;

uniform float u_alpha;

void main() {
    frag_color = vec4(v_color, u_alpha);
}
)glsl";

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
    log::error(render, "Contact debug shader compile failed: {}", info.data());
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
    log::error(render, "Contact debug shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}
} // namespace javelin::detail

export namespace javelin {

struct ContactDebugPass final {
    struct Settings final {
        // World-space length of visualized contact normals.
        f32 normal_length{0.25f};
        // Raster point size in pixels.
        f32 point_size_px{7.0f};
        // Line/point alpha blended over the scene.
        f32 alpha{0.95f};
    };

    Settings settings{};

    template <class Device> void init(Device &) {
        log::info(render, "Initializing contact debug pass");
        create_shader_();
        create_buffers_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down contact debug pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0 || !ctx.debug.draw_contacts ||
            ctx.contacts.count == 0u) {
            return;
        }
        if (program_ == 0 || vao_ == 0 || vbo_ == 0) {
            return;
        }

        ZoneScopedN("ContactDebugPass");
        TracyGpuZone("ContactDebugPass");

        build_debug_vertices_(ctx.contacts);
        if (point_vertex_count_ == 0 || line_vertex_count_ == 0) {
            return;
        }
        upload_vertices_();

        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        // Draw contacts as an x-ray overlay so they remain visible through bodies.
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform1f(u_point_size_, settings.point_size_px);
        glUniform1f(u_alpha_, settings.alpha);

        glBindVertexArray(vao_);
        glDrawArrays(GL_POINTS, 0, point_vertex_count_);
        glLineWidth(kContactNormalLineWidthPx);
        glDrawArrays(GL_LINES, point_vertex_count_, line_vertex_count_);
        glLineWidth(1.0f);
        glBindVertexArray(0);
        glUseProgram(0);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

  private:
    struct Vertex final {
        Vec3 position{};
        Vec3 color{};
    };

    static constexpr Vec3 kContactPointColorNew = linear_srgb_to_acescg(Vec3{1.0f, 0.20f, 0.10f});
    static constexpr Vec3 kContactPointColorPersisted = linear_srgb_to_acescg(Vec3{0.30f, 1.0f, 0.30f});
    static constexpr Vec3 kContactNormalColor = linear_srgb_to_acescg(Vec3{0.20f, 0.65f, 1.0f});
    static constexpr f32 kContactNormalLineWidthPx = 3.0f;

    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const u32 vs = detail::compile_shader(GL_VERTEX_SHADER, detail::kContactDebugVertexShader);
        const u32 fs = detail::compile_shader(GL_FRAGMENT_SHADER, detail::kContactDebugFragmentShader);
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
        u_point_size_ = glGetUniformLocation(program_, "u_point_size");
        u_alpha_ = glGetUniformLocation(program_, "u_alpha");
    }

    void create_buffers_() {
        if (vao_ != 0 || vbo_ != 0) {
            return;
        }

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(offsetof(Vertex, color)));

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void build_debug_vertices_(const ContactDebugSnapshot &contacts) {
        const usize contact_count = static_cast<usize>(contacts.count);
        const usize point_vertex_count = contact_count;
        const usize line_vertex_count = contact_count * 2u;
        const usize total_vertex_count = point_vertex_count + line_vertex_count;

        vertices_.resize(total_vertex_count);
        point_vertex_count_ = static_cast<i32>(point_vertex_count);
        line_vertex_count_ = static_cast<i32>(line_vertex_count);

        // Layout: [all point vertices][all line segment vertices].
        usize line_write = point_vertex_count;
        for (usize i = 0; i < contact_count; ++i) {
            const Vec3 position = contacts.points[i];
            Vec3 normal = contacts.normals[i];
            if (!normal.try_normalize()) {
                normal = Vec3::unit_y();
            }

            const bool persisted = contacts.persisted[i] != 0u;
            const Vec3 point_color = persisted ? kContactPointColorPersisted : kContactPointColorNew;
            const Vec3 normal_tip = position + normal * settings.normal_length;

            vertices_[i] = Vertex{
                .position = position,
                .color = point_color,
            };
            vertices_[line_write + 0u] = Vertex{
                .position = position,
                .color = kContactNormalColor,
            };
            vertices_[line_write + 1u] = Vertex{
                .position = normal_tip,
                .color = kContactNormalColor,
            };
            line_write += 2u;
        }
    }

    void upload_vertices_() {
        const usize needed_vertex_count = vertices_.size();
        if (needed_vertex_count == 0u) {
            return;
        }

        if (needed_vertex_count > vertex_capacity_) {
            vertex_capacity_ = std::max(needed_vertex_count, vertex_capacity_ + vertex_capacity_ / 2u + 1u);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_capacity_ * sizeof(Vertex)), nullptr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        }

        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(needed_vertex_count * sizeof(Vertex)),
                        vertices_.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void release_() {
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }
        if (vbo_ != 0) {
            glDeleteBuffers(1, &vbo_);
            vbo_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }

        point_vertex_count_ = 0;
        line_vertex_count_ = 0;
        vertex_capacity_ = 0u;
        vertices_.clear();
        u_view_proj_ = -1;
        u_point_size_ = -1;
        u_alpha_ = -1;
    }

  private:
    u32 program_{};
    u32 vao_{};
    u32 vbo_{};
    i32 point_vertex_count_{};
    i32 line_vertex_count_{};
    usize vertex_capacity_{};
    std::vector<Vertex> vertices_{};
    i32 u_view_proj_{-1};
    i32 u_point_size_{-1};
    i32 u_alpha_{-1};
};

} // namespace javelin
