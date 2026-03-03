module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.constraint_debug_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.constraint_types;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;
import javelin.scene.pose_channel;

namespace javelin::detail {

constexpr std::string_view kConstraintDebugVertexShader = R"glsl(
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

constexpr std::string_view kConstraintDebugFragmentShader = R"glsl(
#version 460 core
in vec3 v_color;
layout(location = 0) out vec4 frag_color;

uniform float u_alpha;

void main() {
    frag_color = vec4(v_color, u_alpha);
}
)glsl";

u32 compile_constraint_shader(const GLenum type, const std::string_view source) noexcept {
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
    log::error(render, "Constraint debug shader compile failed: {}", info.data());
    glDeleteShader(shader);
    return 0;
}

u32 link_constraint_program(const u32 vs, const u32 fs) noexcept {
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
    log::error(render, "Constraint debug shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}
} // namespace javelin::detail

export namespace javelin {

// Renders a line between the world-space anchor points of each distance constraint.
//
// Pipeline position: after WorldGridPass, before x-ray debug overlays.
// Depth mode: x-ray (no depth test) — constraint wires remain visible through geometry
//   so the full constraint topology can be inspected regardless of body occlusion.
//
// Rigid constraints (compliance = 0): icy blue-white — stiff rod.
// Soft constraints  (compliance > 0): warm orange     — elastic spring.
struct ConstraintDebugPass final {
    struct Settings final {
        // Alpha of constraint lines blended over the scene.
        f32 alpha{0.85f};
    };

    Settings settings{};

    template <class Device> void init(Device &) {
        log::info(render, "Initializing constraint debug pass");
        create_shader_();
        create_buffers_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down constraint debug pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0 || !ctx.debug.draw_constraints ||
            ctx.view.constraints.empty()) {
            return;
        }
        if (program_ == 0 || vao_ == 0 || vbo_ == 0) {
            return;
        }

        ZoneScopedN("ConstraintDebugPass");
        TracyGpuZone("ConstraintDebugPass");

        build_vertices_(ctx);
        if (line_vertex_count_ == 0) {
            return;
        }
        upload_vertices_();

        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        // X-ray overlay: constraint wires stay visible through geometry so the
        // full pendulum/chain topology is always readable.
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform1f(u_alpha_, settings.alpha);

        glBindVertexArray(vao_);
        glLineWidth(kConstraintLineWidthPx);
        glDrawArrays(GL_LINES, 0, line_vertex_count_);
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

    // Rigid rod — icy blue-white; evokes a stiff wire or steel cable.
    static constexpr Vec3 kRigidConstraintColor = linear_srgb_to_acescg(Vec3{0.75f, 0.90f, 1.00f});
    // Elastic spring — warm orange; high contrast against the rigid color and the
    // sleep/contact debug overlays.
    static constexpr Vec3 kSoftConstraintColor = linear_srgb_to_acescg(Vec3{1.00f, 0.55f, 0.10f});
    // Line width in pixels; slightly thicker than the default so wires are readable
    // at a distance without being visually overwhelming.
    static constexpr f32 kConstraintLineWidthPx = 2.0f;

    void build_vertices_(const RenderContext &ctx) {
        const usize constraint_count = ctx.view.constraints.size();
        const u32 body_count = ctx.poses.count;
        vertices_.clear();
        vertices_.reserve(constraint_count * 2u);

        for (const DistanceConstraint &c : ctx.view.constraints) {
            if (c.body_a >= body_count || c.body_b >= body_count) {
                continue;
            }
            const PoseSample pose_a = sample_pose(ctx.poses, c.body_a, ctx.pose_alpha);
            const PoseSample pose_b = sample_pose(ctx.poses, c.body_b, ctx.pose_alpha);

            const Vec3 world_a = pose_a.position + rotate(pose_a.orientation, c.anchor_a);
            const Vec3 world_b = pose_b.position + rotate(pose_b.orientation, c.anchor_b);

            const Vec3 color = (c.compliance == 0.0f) ? kRigidConstraintColor : kSoftConstraintColor;
            vertices_.push_back(Vertex{.position = world_a, .color = color});
            vertices_.push_back(Vertex{.position = world_b, .color = color});
        }

        line_vertex_count_ = static_cast<GLsizei>(vertices_.size());
    }

    void upload_vertices_() {
        const usize needed = vertices_.size();
        if (needed == 0u) {
            return;
        }

        if (needed > vertex_capacity_) {
            vertex_capacity_ = std::max(needed, vertex_capacity_ + vertex_capacity_ / 2u + 1u);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_capacity_ * sizeof(Vertex)), nullptr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        }

        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(needed * sizeof(Vertex)), vertices_.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const u32 vs = detail::compile_constraint_shader(GL_VERTEX_SHADER, detail::kConstraintDebugVertexShader);
        const u32 fs = detail::compile_constraint_shader(GL_FRAGMENT_SHADER, detail::kConstraintDebugFragmentShader);
        if (vs == 0 || fs == 0) {
            if (vs != 0) {
                glDeleteShader(vs);
            }
            if (fs != 0) {
                glDeleteShader(fs);
            }
            return;
        }

        program_ = detail::link_constraint_program(vs, fs);
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
