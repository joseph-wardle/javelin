module;

#include <glad/gl.h>

export module javelin.render.passes.world_grid_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.vec3;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;

namespace javelin::detail {
constexpr std::string_view kGridVertexShader = R"glsl(
#version 460 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_view_proj;
void main() {
    gl_Position = u_view_proj * vec4(a_pos, 1.0);
}
)glsl";

constexpr std::string_view kGridFragmentShader = R"glsl(
#version 460 core
uniform vec3 u_color;
out vec4 frag_color;
void main() {
    frag_color = vec4(u_color, 1.0);
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
    log::error("Grid shader compile failed: {}", info.data());
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
    log::error("Grid shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}
} // namespace javelin::detail

export namespace javelin {

struct WorldGridPass final {
    struct Settings final {
        f32 half_extent{20.0f};
        f32 step{1.0f};
        Vec3 color{0.28f, 0.30f, 0.34f};
    };

    Settings settings{};

    template <class Device> void init(Device &) {
        create_shader_();
        create_grid_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) { release_(); }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0) {
            return;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (!ctx.debug.draw_grid || program_ == 0 || vao_ == 0) {
            return;
        }

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform3f(u_color_, settings.color.x, settings.color.y, settings.color.z);

        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertex_count_));
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
        u_color_ = glGetUniformLocation(program_, "u_color");
    }

    void create_grid_() {
        if (vao_ != 0 || vbo_ != 0) {
            return;
        }

        if (settings.step <= 0.0f || settings.half_extent <= 0.0f) {
            return;
        }

        const i32 line_count = static_cast<i32>(settings.half_extent / settings.step);
        const f32 extent = settings.step * static_cast<f32>(line_count);
        const i32 line_vertices = (line_count * 2 + 1) * 4;

        std::vector<Vec3> vertices;
        vertices.reserve(static_cast<usize>(line_vertices));

        for (i32 i = -line_count; i <= line_count; ++i) {
            const f32 t = static_cast<f32>(i) * settings.step;
            vertices.push_back(Vec3{-extent, 0.0f, t});
            vertices.push_back(Vec3{extent, 0.0f, t});
            vertices.push_back(Vec3{t, 0.0f, -extent});
            vertices.push_back(Vec3{t, 0.0f, extent});
        }

        vertex_count_ = static_cast<i32>(vertices.size());

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vec3)), vertices.data(),
                     GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glBindVertexArray(0);
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
        vertex_count_ = 0;
        u_view_proj_ = -1;
        u_color_ = -1;
    }

  private:
    u32 program_{};
    u32 vao_{};
    u32 vbo_{};
    i32 vertex_count_{};
    i32 u_view_proj_{-1};
    i32 u_color_{-1};
};

} // namespace javelin
