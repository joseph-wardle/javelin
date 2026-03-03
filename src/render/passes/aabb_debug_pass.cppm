module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.aabb_debug_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;
import javelin.physics.aabb_debug;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;

namespace javelin::detail {
u32 compile_aabb_shader(const GLenum type, const std::string_view source) noexcept {
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
    log::error(render, "AABB debug shader compile failed: {}", info.data());
    glDeleteShader(shader);
    return 0;
}

u32 link_aabb_program(const u32 vs, const u32 fs) noexcept {
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
    log::error(render, "AABB debug shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}

constexpr std::string_view kAabbDebugVertexShader = R"glsl(
#version 460 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

uniform mat4 u_view_proj;

out vec3 v_color;

void main() {
    gl_Position = u_view_proj * vec4(a_position, 1.0);
    v_color = a_color;
}
)glsl";

constexpr std::string_view kAabbDebugFragmentShader = R"glsl(
#version 460 core
in vec3 v_color;
layout(location = 0) out vec4 frag_color;

uniform float u_alpha;

void main() {
    frag_color = vec4(v_color, u_alpha);
}
)glsl";
} // namespace javelin::detail

export namespace javelin {

struct AabbDebugPass final {
    struct Settings final {
        // Alpha of wireframe lines blended over the scene.
        f32 alpha{0.75f};
    };

    Settings settings{};

    template <class Device> void init(Device &) {
        log::info(render, "Initializing AABB debug pass");
        create_shader_();
        create_buffers_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down AABB debug pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0 || !ctx.debug.draw_aabbs || ctx.aabbs.count == 0u) {
            return;
        }
        if (program_ == 0 || vao_ == 0 || vbo_ == 0) {
            return;
        }

        ZoneScopedN("AabbDebugPass");
        TracyGpuZone("AabbDebugPass");

        build_aabb_vertices_(ctx.aabbs);
        upload_vertices_();

        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        // X-ray overlay: drawn without depth test so physics bounds remain visible
        // through opaque geometry, making shape/bound mismatches easy to spot.
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform1f(u_alpha_, settings.alpha);

        glBindVertexArray(vao_);
        glDrawArrays(GL_LINES, 0, line_vertex_count_);
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

    // Amber wireframe — distinct from contact debug colors (red/green points, blue normals).
    static constexpr Vec3 kAabbWireframeColor = linear_srgb_to_acescg(Vec3{0.9f, 0.7f, 0.1f});

    // 12 edges × 2 endpoints = 24 vertices per AABB wireframe box.
    static constexpr u32 kVerticesPerAabb = 24u;

    // Each row is one edge: indices into the 8-corner local array below.
    // Corners are numbered:
    //   0:(min,min,min)  1:(max,min,min)  2:(max,max,min)  3:(min,max,min)  (near face, z=min)
    //   4:(min,min,max)  5:(max,min,max)  6:(max,max,max)  7:(min,max,max)  (far  face, z=max)
    static constexpr std::array<std::array<u8, 2>, 12> kEdges = {{
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 0}, // near face bottom/top/left/right
        {4, 5},
        {5, 6},
        {6, 7},
        {7, 4}, // far  face bottom/top/left/right
        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7}, // lateral edges connecting near to far
    }};

    void build_aabb_vertices_(const AabbDebugSnapshot &aabbs) {
        const usize aabb_count = static_cast<usize>(aabbs.count);
        vertices_.resize(aabb_count * kVerticesPerAabb);
        line_vertex_count_ = static_cast<GLsizei>(vertices_.size());

        usize write = 0;
        for (usize i = 0; i < aabb_count; ++i) {
            const Aabb &box = aabbs.aabbs[i];
            const Vec3 corners[8] = {
                {box.min.x, box.min.y, box.min.z}, {box.max.x, box.min.y, box.min.z}, {box.max.x, box.max.y, box.min.z},
                {box.min.x, box.max.y, box.min.z}, {box.min.x, box.min.y, box.max.z}, {box.max.x, box.min.y, box.max.z},
                {box.max.x, box.max.y, box.max.z}, {box.min.x, box.max.y, box.max.z},
            };
            for (const std::array<u8, 2> &edge : kEdges) {
                vertices_[write++] = Vertex{.position = corners[edge[0]], .color = kAabbWireframeColor};
                vertices_[write++] = Vertex{.position = corners[edge[1]], .color = kAabbWireframeColor};
            }
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

    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const u32 vs = detail::compile_aabb_shader(GL_VERTEX_SHADER, detail::kAabbDebugVertexShader);
        const u32 fs = detail::compile_aabb_shader(GL_FRAGMENT_SHADER, detail::kAabbDebugFragmentShader);
        if (vs == 0 || fs == 0) {
            if (vs != 0) {
                glDeleteShader(vs);
            }
            if (fs != 0) {
                glDeleteShader(fs);
            }
            return;
        }

        program_ = detail::link_aabb_program(vs, fs);
        if (program_ == 0) {
            return;
        }

        u_view_proj_ = glGetUniformLocation(program_, "u_view_proj");
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

        line_vertex_count_ = 0;
        vertex_capacity_ = 0u;
        vertices_.clear();
        u_view_proj_ = -1;
        u_alpha_ = -1;
    }

  private:
    u32 program_{};
    u32 vao_{};
    u32 vbo_{};
    GLsizei line_vertex_count_{};
    usize vertex_capacity_{};
    std::vector<Vertex> vertices_{};
    i32 u_view_proj_{-1};
    i32 u_alpha_{-1};
};

} // namespace javelin
