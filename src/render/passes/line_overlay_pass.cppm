module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.line_overlay_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.shader_utils;
import javelin.render.types;

namespace javelin::detail {

// Shared shader used by every line-overlay debug pass.
//
// `u_point_size` and `gl_PointSize` are present even on line-only passes — the uniform
// is simply left at its default sentinel location (-1) for builders that never draw
// points, so the glUniform1f call is skipped at runtime.  Setting gl_PointSize has no
// effect on GL_LINES draws.
constexpr std::string_view kLineOverlayVertexShader = R"glsl(
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

constexpr std::string_view kLineOverlayFragmentShader = R"glsl(
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

// Vertex layout common to all line-overlay passes.
struct LineOverlayVertex final {
    Vec3 position{};
    Vec3 color{};
};

// Counts returned by a Builder::fill() call.  Contact debug draws both points and lines;
// the other debug passes set point_count = 0.  Layout in the vertex buffer is always
// `[point vertices][line vertices]`.
struct LineOverlayCounts final {
    i32 point_count{0};
    i32 line_count{0};
};

// Template-driven debug pass that draws a vertex stream as x-ray-overlay lines (and
// optionally points) over the rendered scene.  The `Builder` concept supplies:
//
//   using Settings = ...;                             // pass-specific settings struct
//   static constexpr std::string_view kLabel;         // shader/log label
//   static constexpr f32 kLineWidthPx;                // glLineWidth before the lines draw
//   static bool enabled(const RenderContext &);       // run-this-frame gate
//   static f32 alpha(const Settings &);               // u_alpha value
//   static LineOverlayCounts fill(const RenderContext &, const Settings &,
//                                 std::vector<LineOverlayVertex> &);
//
// And, only for builders that draw points (i.e. kDrawsPoints == true):
//
//   static constexpr bool kDrawsPoints = true;
//   static f32 point_size_px(const Settings &);       // u_point_size value
//
// Each pass file becomes a small Builder + a `using PassName = LineOverlayPass<Builder>;`.
template <class Builder> struct LineOverlayPass final {
    using Settings = typename Builder::Settings;

    Settings settings{};

    template <class Device> void init(Device &) {
        log::info(render, "Initializing {} pass", Builder::kLabel);
        create_shader_();
        create_buffers_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down {} pass", Builder::kLabel);
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.targets.ready() || !Builder::enabled(ctx)) {
            return;
        }
        if (program_ == 0 || vao_ == 0 || vbo_ == 0) {
            return;
        }

        vertices_.clear();
        const LineOverlayCounts counts = Builder::fill(ctx, settings, vertices_);
        if (counts.point_count == 0 && counts.line_count == 0) {
            return;
        }
        upload_();

        ctx.targets.bind_for_drawing();
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        // X-ray overlay: drawn without depth test so debug primitives remain visible
        // through scene geometry, making issues easy to spot regardless of occlusion.
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform1f(u_alpha_, Builder::alpha(settings));
        if constexpr (point_drawing_enabled_()) {
            if (u_point_size_ >= 0) {
                glUniform1f(u_point_size_, Builder::point_size_px(settings));
            }
        }

        glBindVertexArray(vao_);
        if constexpr (point_drawing_enabled_()) {
            if (counts.point_count > 0) {
                glEnable(GL_PROGRAM_POINT_SIZE);
                glDrawArrays(GL_POINTS, 0, counts.point_count);
                glDisable(GL_PROGRAM_POINT_SIZE);
            }
        }
        if (counts.line_count > 0) {
            glLineWidth(Builder::kLineWidthPx);
            glDrawArrays(GL_LINES, counts.point_count, counts.line_count);
            glLineWidth(1.0f);
        }
        glBindVertexArray(0);
        glUseProgram(0);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }

  private:
    static constexpr bool point_drawing_enabled_() noexcept {
        if constexpr (requires { Builder::kDrawsPoints; }) {
            return Builder::kDrawsPoints;
        } else {
            return false;
        }
    }

    void create_shader_() {
        if (program_ != 0) {
            return;
        }
        program_ = shader_utils::build_program(detail::kLineOverlayVertexShader, detail::kLineOverlayFragmentShader,
                                               Builder::kLabel);
        if (program_ == 0) {
            return;
        }
        u_view_proj_ = glGetUniformLocation(program_, "u_view_proj");
        u_alpha_ = glGetUniformLocation(program_, "u_alpha");
        u_point_size_ = glGetUniformLocation(program_, "u_point_size");
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
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineOverlayVertex), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineOverlayVertex),
                              reinterpret_cast<void *>(offsetof(LineOverlayVertex, color)));

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void upload_() {
        const usize needed = vertices_.size();
        if (needed == 0u) {
            return;
        }
        if (needed > vertex_capacity_) {
            vertex_capacity_ = std::max(needed, vertex_capacity_ + vertex_capacity_ / 2u + 1u);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_capacity_ * sizeof(LineOverlayVertex)), nullptr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        }
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(needed * sizeof(LineOverlayVertex)),
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
        vertex_capacity_ = 0u;
        vertices_.clear();
        u_view_proj_ = -1;
        u_alpha_ = -1;
        u_point_size_ = -1;
    }

    u32 program_{};
    u32 vao_{};
    u32 vbo_{};
    usize vertex_capacity_{};
    std::vector<LineOverlayVertex> vertices_{};
    i32 u_view_proj_{-1};
    i32 u_alpha_{-1};
    i32 u_point_size_{-1};
};

} // namespace javelin
