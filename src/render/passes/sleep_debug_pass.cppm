module;

#include <glad/gl.h>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.sleep_debug_pass;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.render_targets;
import javelin.render.types;
import javelin.scene.pose_channel;
import javelin.scene.shapes;

namespace javelin::detail {

// --- Shader sources ---

constexpr std::string_view kSleepDebugVertexShader = R"glsl(
#version 460 core
layout(location = 0) in vec3 a_position;

layout(location = 1) in vec3 a_instance_pos;
layout(location = 2) in vec3 a_instance_scale;
layout(location = 3) in vec4 a_instance_rot;
layout(location = 4) in vec3 a_instance_color;

uniform mat4 u_view_proj;

out vec3 v_color;

vec3 quat_rotate(vec4 q, vec3 v) {
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

void main() {
    vec3 world = a_instance_pos + quat_rotate(a_instance_rot, a_position * a_instance_scale);
    v_color = a_instance_color;
    gl_Position = u_view_proj * vec4(world, 1.0);
}
)glsl";

constexpr std::string_view kSleepDebugFragmentShader = R"glsl(
#version 460 core
in vec3 v_color;
layout(location = 0) out vec4 frag_color;

uniform float u_alpha;

void main() {
    frag_color = vec4(v_color, u_alpha);
}
)glsl";

// --- Shader helpers ---

u32 compile_sleep_shader(const GLenum type, const std::string_view source) noexcept {
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
    log::error(render, "Sleep debug shader compile failed: {}", info.data());
    glDeleteShader(shader);
    return 0;
}

u32 link_sleep_program(const u32 vs, const u32 fs) noexcept {
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
    log::error(render, "Sleep debug shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}

// --- Mesh generation ---

struct PositionMesh final {
    std::vector<Vec3> positions;
    std::vector<u32> indices;
};

u32 sleep_midpoint_index(const u32 a, const u32 b, std::vector<Vec3> &positions, std::unordered_map<u64, u32> &cache) {
    const u64 key = (static_cast<u64>(std::min(a, b)) << 32) | static_cast<u64>(std::max(a, b));
    if (const auto it = cache.find(key); it != cache.end()) {
        return it->second;
    }
    Vec3 mid = (positions[a] + positions[b]) * 0.5f;
    mid.try_normalize();
    const u32 idx = static_cast<u32>(positions.size());
    positions.push_back(mid);
    cache.emplace(key, idx);
    return idx;
}

// Icosphere at subdivision level 2 — matches GeometryPass sphere precision.
// Positions lie on the unit sphere; scale is applied per-instance in the shader.
[[nodiscard]] PositionMesh make_sleep_icosphere() {
    constexpr i32 kSubdivisions = 2;
    const f32 t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    const std::array<Vec3, 12> base_positions = {
        Vec3{-1.0f, t, 0.0f}, Vec3{1.0f, t, 0.0f}, Vec3{-1.0f, -t, 0.0f}, Vec3{1.0f, -t, 0.0f},
        Vec3{0.0f, -1.0f, t}, Vec3{0.0f, 1.0f, t}, Vec3{0.0f, -1.0f, -t}, Vec3{0.0f, 1.0f, -t},
        Vec3{t, 0.0f, -1.0f}, Vec3{t, 0.0f, 1.0f}, Vec3{-t, 0.0f, -1.0f}, Vec3{-t, 0.0f, 1.0f},
    };
    const std::array<u32, 60> base_indices = {
        0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4,  11, 10, 2,  10, 7, 6, 7, 1, 8,
        3, 9,  4, 3, 4, 2, 3, 2, 6, 3, 6, 8,  3, 8,  9,  4, 9, 5, 2, 4,  11, 6,  2,  10, 8,  6, 7, 9, 8, 1,
    };

    PositionMesh mesh{};
    mesh.positions.assign(base_positions.begin(), base_positions.end());
    for (auto &p : mesh.positions) {
        p.try_normalize();
    }
    mesh.indices.assign(base_indices.begin(), base_indices.end());

    for (i32 level = 0; level < kSubdivisions; ++level) {
        std::unordered_map<u64, u32> midpoint_cache;
        midpoint_cache.reserve(mesh.indices.size());
        std::vector<u32> next_indices;
        next_indices.reserve(mesh.indices.size() * 4);

        for (usize i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const u32 a = mesh.indices[i + 0];
            const u32 b = mesh.indices[i + 1];
            const u32 c = mesh.indices[i + 2];
            const u32 ab = sleep_midpoint_index(a, b, mesh.positions, midpoint_cache);
            const u32 bc = sleep_midpoint_index(b, c, mesh.positions, midpoint_cache);
            const u32 ca = sleep_midpoint_index(c, a, mesh.positions, midpoint_cache);
            next_indices.push_back(a);
            next_indices.push_back(ab);
            next_indices.push_back(ca);
            next_indices.push_back(b);
            next_indices.push_back(bc);
            next_indices.push_back(ab);
            next_indices.push_back(c);
            next_indices.push_back(ca);
            next_indices.push_back(bc);
            next_indices.push_back(ab);
            next_indices.push_back(bc);
            next_indices.push_back(ca);
        }
        mesh.indices = std::move(next_indices);
    }
    return mesh;
}

// Box mesh matching GeometryPass winding — positions only, 24 vertices (4 per face).
// Scale is applied per-instance; half-extents of 1.0 cover [-1, 1] on each axis.
[[nodiscard]] PositionMesh make_sleep_cube() {
    PositionMesh mesh{};
    mesh.positions = {
        // +X              -X              +Y              -Y              +Z              -Z
        {1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, 1},   {-1, 1, -1}, {-1, -1, -1},
        {-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}, {1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}, {1, -1, -1}, {1, -1, 1},
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}, {1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}, {1, 1, -1},
    };
    mesh.indices = {
        0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
        12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23,
    };
    return mesh;
}

} // namespace javelin::detail

export namespace javelin {

// Renders a semi-transparent color overlay on sleeping and recently-woken bodies.
//
// Pipeline position: after WorldGridPass, before x-ray debug overlays.
// Depth mode: GL_LEQUAL with polygon offset — overlay sits on visible surfaces only
//   (sleeping bodies fully occluded by other geometry are not shown).
//
// Sleep tint: desaturated gray — body is dormant, physics is skipping it.
// Wake flash:  warm yellow — body just received a contact and re-entered simulation;
//              decays over kWakeFlashFrames render frames.
struct SleepDebugPass final {
    struct Settings final {
        // Alpha of the colored overlay blended over the geometry pass output.
        f32 alpha{0.50f};
    };

    Settings settings{};

    template <class Device> void init(Device &) {
        log::info(render, "Initializing sleep debug pass");
        create_shader_();
        create_sphere_();
        create_cube_();
        create_instance_buffer_(sphere_vao_, sphere_instance_vbo_);
        create_instance_buffer_(cube_vao_, cube_instance_vbo_);
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down sleep debug pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0 || !ctx.debug.draw_sleep_state) {
            return;
        }
        if (ctx.poses.sleep_flags.empty()) {
            // No sleep data yet (first frame or scene not loaded).
            // Clear stale per-body state so a subsequent enable starts clean.
            prev_sleep_flags_.clear();
            wake_timers_.clear();
            return;
        }
        if (program_ == 0 || sphere_vao_ == 0 || cube_vao_ == 0) {
            return;
        }

        ZoneScopedN("SleepDebugPass");
        TracyGpuZone("SleepDebugPass");

        update_wake_timers_(ctx.poses.sleep_flags);
        build_instances_(ctx);

        if (sphere_instance_count_ <= 0 && cube_instance_count_ <= 0) {
            return;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        // Surface overlay: only shown on visible body surfaces, not x-ray.
        // GL_LEQUAL lets the overlay draw exactly where geometry was written.
        // Polygon offset pushes it fractionally toward the camera to prevent
        // z-fighting with the geometry pass depth values.
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_FALSE);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());
        glUniform1f(u_alpha_, settings.alpha);

        if (sphere_instance_count_ > 0) {
            glBindVertexArray(sphere_vao_);
            glDrawElementsInstanced(GL_TRIANGLES, sphere_index_count_, GL_UNSIGNED_INT, nullptr,
                                    sphere_instance_count_);
            glBindVertexArray(0);
        }
        if (cube_instance_count_ > 0) {
            glBindVertexArray(cube_vao_);
            glDrawElementsInstanced(GL_TRIANGLES, cube_index_count_, GL_UNSIGNED_INT, nullptr, cube_instance_count_);
            glBindVertexArray(0);
        }

        glUseProgram(0);
        glPolygonOffset(0.0f, 0.0f);
        glDisable(GL_POLYGON_OFFSET_FILL);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
    }

  private:
    struct InstanceData final {
        Vec3 position{};
        Vec3 scale{};
        Quat orientation{};
        Vec3 color{};
    };

    // Sleep tint: blue-gray.  Desaturation signals the body is frozen in place.
    static constexpr Vec3 kSleepColor = linear_srgb_to_acescg(Vec3{0.38f, 0.38f, 0.45f});
    // Wake flash: warm yellow.  High contrast against the sleep gray and normal colors.
    static constexpr Vec3 kWakeColor = linear_srgb_to_acescg(Vec3{1.00f, 0.85f, 0.10f});
    // Number of render frames the wake flash remains visible after a body wakes.
    // 30 frames ≈ 0.5 s at 60 Hz — noticeable without being distracting.
    static constexpr u32 kWakeFlashFrames = 30u;

    // Advances wake flash timers for this frame.
    //
    // On a sleeping → awake transition the timer is set to kWakeFlashFrames and
    // counts down by 1 each subsequent frame.  While > 0 the body renders yellow.
    // Resize resets all timers when the body count changes (e.g. scene reload).
    void update_wake_timers_(const std::span<const u8> sleep_flags) {
        const u32 count = static_cast<u32>(sleep_flags.size());
        if (prev_sleep_flags_.size() != count) {
            prev_sleep_flags_.assign(count, 0u);
            wake_timers_.assign(count, 0u);
        }
        for (u32 i = 0; i < count; ++i) {
            if (prev_sleep_flags_[i] != 0u && sleep_flags[i] == 0u) {
                // Body just woke: start the flash countdown.
                wake_timers_[i] = kWakeFlashFrames;
            } else if (wake_timers_[i] > 0u) {
                --wake_timers_[i];
            }
            prev_sleep_flags_[i] = sleep_flags[i];
        }
    }

    void build_instances_(const RenderContext &ctx) {
        const usize count = std::min({ctx.view.shape_kind.size(), ctx.view.shape_index.size(),
                                      ctx.poses.curr_positions.size(), ctx.poses.sleep_flags.size()});
        sphere_instances_.clear();
        cube_instances_.clear();

        for (usize i = 0; i < count; ++i) {
            const bool sleeping = ctx.poses.sleep_flags[i] != 0u;
            const bool waking = (i < wake_timers_.size()) && (wake_timers_[i] > 0u) && !sleeping;
            if (!sleeping && !waking) {
                continue;
            }

            const Vec3 color = sleeping ? kSleepColor : kWakeColor;
            const PoseSample pose = sample_pose(ctx.poses, static_cast<u32>(i), ctx.pose_alpha);

            const u32 shape_id = ctx.view.shape_index[i];
#ifndef NDEBUG
            if (shape_id >= ctx.view.shapes.size()) {
                continue;
            }
#endif
            const ShapeData &shape = ctx.view.shapes[shape_id];

            switch (ctx.view.shape_kind[i]) {
            case ShapeKind::sphere: {
                const SphereShape &sphere = shape_sphere(shape);
                sphere_instances_.push_back(InstanceData{
                    .position = pose.position,
                    .scale = Vec3{sphere.radius},
                    .orientation = pose.orientation,
                    .color = color,
                });
            } break;
            case ShapeKind::box: {
                const BoxShape &box = shape_box(shape);
                cube_instances_.push_back(InstanceData{
                    .position = pose.position,
                    .scale = box.half_extents,
                    .orientation = pose.orientation,
                    .color = color,
                });
            } break;
            }
        }

        upload_instances_(sphere_instance_vbo_, sphere_instances_, sphere_instance_capacity_, sphere_instance_count_);
        upload_instances_(cube_instance_vbo_, cube_instances_, cube_instance_capacity_, cube_instance_count_);
    }

    static void upload_instances_(const u32 instance_vbo, const std::vector<InstanceData> &instances, usize &capacity,
                                  i32 &count) {
        count = static_cast<i32>(instances.size());
        if (count <= 0 || instance_vbo == 0) {
            count = 0;
            return;
        }

        const usize needed = instances.size();
        if (needed > capacity) {
            capacity = std::max(needed, capacity + capacity / 2u + 1u);
            glBindBuffer(GL_ARRAY_BUFFER, instance_vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(capacity * sizeof(InstanceData)), nullptr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBindBuffer(GL_ARRAY_BUFFER, instance_vbo);
        }
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(needed * sizeof(InstanceData)), instances.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const u32 vs = detail::compile_sleep_shader(GL_VERTEX_SHADER, detail::kSleepDebugVertexShader);
        const u32 fs = detail::compile_sleep_shader(GL_FRAGMENT_SHADER, detail::kSleepDebugFragmentShader);
        if (vs == 0 || fs == 0) {
            if (vs != 0) {
                glDeleteShader(vs);
            }
            if (fs != 0) {
                glDeleteShader(fs);
            }
            return;
        }
        program_ = detail::link_sleep_program(vs, fs);
        if (program_ == 0) {
            return;
        }
        u_view_proj_ = glGetUniformLocation(program_, "u_view_proj");
        u_alpha_ = glGetUniformLocation(program_, "u_alpha");
    }

    void create_sphere_() {
        if (sphere_vao_ != 0) {
            return;
        }

        const detail::PositionMesh mesh = detail::make_sleep_icosphere();
        sphere_index_count_ = static_cast<i32>(mesh.indices.size());

        glGenVertexArrays(1, &sphere_vao_);
        glGenBuffers(1, &sphere_vbo_);
        glGenBuffers(1, &sphere_ibo_);

        glBindVertexArray(sphere_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, sphere_vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.positions.size() * sizeof(Vec3)),
                     mesh.positions.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphere_ibo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(u32)),
                     mesh.indices.data(), GL_STATIC_DRAW);

        // location 0: vertex position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        log::info(render, "Sleep debug sphere ready (verts={}, indices={})", mesh.positions.size(),
                  mesh.indices.size());
    }

    void create_cube_() {
        if (cube_vao_ != 0) {
            return;
        }

        const detail::PositionMesh mesh = detail::make_sleep_cube();
        cube_index_count_ = static_cast<i32>(mesh.indices.size());

        glGenVertexArrays(1, &cube_vao_);
        glGenBuffers(1, &cube_vbo_);
        glGenBuffers(1, &cube_ibo_);

        glBindVertexArray(cube_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, cube_vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.positions.size() * sizeof(Vec3)),
                     mesh.positions.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ibo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(u32)),
                     mesh.indices.data(), GL_STATIC_DRAW);

        // location 0: vertex position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        log::info(render, "Sleep debug cube ready (verts={}, indices={})", mesh.positions.size(), mesh.indices.size());
    }

    // Binds the instance VBO to an existing VAO and configures per-instance attributes.
    //
    // Attribute layout (all per-instance, divisor=1):
    //   location 1 — position    (vec3, offset  0)
    //   location 2 — scale       (vec3, offset 12)
    //   location 3 — orientation (vec4, offset 24)  — quaternion as xyzw
    //   location 4 — color       (vec3, offset 40)
    void create_instance_buffer_(const u32 vao, u32 &instance_vbo) {
        if (vao == 0 || instance_vbo != 0) {
            return;
        }

        glGenBuffers(1, &instance_vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, instance_vbo);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), nullptr);
        glVertexAttribDivisor(1, 1);

        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              reinterpret_cast<void *>(offsetof(InstanceData, scale)));
        glVertexAttribDivisor(2, 1);

        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              reinterpret_cast<void *>(offsetof(InstanceData, orientation)));
        glVertexAttribDivisor(3, 1);

        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              reinterpret_cast<void *>(offsetof(InstanceData, color)));
        glVertexAttribDivisor(4, 1);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void release_() {
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }

        if (sphere_instance_vbo_ != 0) {
            glDeleteBuffers(1, &sphere_instance_vbo_);
            sphere_instance_vbo_ = 0;
        }
        if (sphere_ibo_ != 0) {
            glDeleteBuffers(1, &sphere_ibo_);
            sphere_ibo_ = 0;
        }
        if (sphere_vbo_ != 0) {
            glDeleteBuffers(1, &sphere_vbo_);
            sphere_vbo_ = 0;
        }
        if (sphere_vao_ != 0) {
            glDeleteVertexArrays(1, &sphere_vao_);
            sphere_vao_ = 0;
        }

        if (cube_instance_vbo_ != 0) {
            glDeleteBuffers(1, &cube_instance_vbo_);
            cube_instance_vbo_ = 0;
        }
        if (cube_ibo_ != 0) {
            glDeleteBuffers(1, &cube_ibo_);
            cube_ibo_ = 0;
        }
        if (cube_vbo_ != 0) {
            glDeleteBuffers(1, &cube_vbo_);
            cube_vbo_ = 0;
        }
        if (cube_vao_ != 0) {
            glDeleteVertexArrays(1, &cube_vao_);
            cube_vao_ = 0;
        }

        sphere_index_count_ = 0;
        cube_index_count_ = 0;
        sphere_instance_count_ = 0;
        cube_instance_count_ = 0;
        sphere_instance_capacity_ = 0u;
        cube_instance_capacity_ = 0u;
        sphere_instances_.clear();
        cube_instances_.clear();
        prev_sleep_flags_.clear();
        wake_timers_.clear();
        u_view_proj_ = -1;
        u_alpha_ = -1;
    }

  private:
    u32 program_{};

    u32 sphere_vao_{};
    u32 sphere_vbo_{};
    u32 sphere_ibo_{};
    u32 sphere_instance_vbo_{};
    i32 sphere_index_count_{};
    i32 sphere_instance_count_{};
    usize sphere_instance_capacity_{};
    std::vector<InstanceData> sphere_instances_{};

    u32 cube_vao_{};
    u32 cube_vbo_{};
    u32 cube_ibo_{};
    u32 cube_instance_vbo_{};
    i32 cube_index_count_{};
    i32 cube_instance_count_{};
    usize cube_instance_capacity_{};
    std::vector<InstanceData> cube_instances_{};

    // Per-body state retained across frames for wake flash detection.
    std::vector<u8> prev_sleep_flags_{};
    std::vector<u32> wake_timers_{};

    i32 u_view_proj_{-1};
    i32 u_alpha_{-1};
};

} // namespace javelin
