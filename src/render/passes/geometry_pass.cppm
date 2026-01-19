module;

#include <glad/gl.h>
#include <tracy/Tracy.hpp>

export module javelin.render.passes.geometry_pass;

import std;

import javelin.core.types;
import javelin.core.logging;
import javelin.math.vec3;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.types;
import javelin.scene.entity;
import javelin.scene.shapes;

namespace javelin::detail {
constexpr i32 kMaterialCount = 4;

constexpr std::string_view kGeometryVertexShader = R"glsl(
#version 460 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_instance_pos;
layout(location = 3) in float a_instance_radius;
layout(location = 4) in uint a_instance_material;

uniform mat4 u_view_proj;

flat out uint v_material_id;
out vec3 v_normal;

void main() {
    vec3 world = a_instance_pos + a_position * a_instance_radius;
    v_material_id = a_instance_material;
    v_normal = a_normal;
    gl_Position = u_view_proj * vec4(world, 1.0);
}
)glsl";

constexpr std::string_view kGeometryFragmentShader = R"glsl(
#version 460 core
flat in uint v_material_id;
in vec3 v_normal;

layout(location = 0) out vec4 frag_color;

const int kMaterialCount = 4;
uniform vec3 u_material_colors[kMaterialCount];
uniform vec3 u_light_dir;
uniform vec3 u_light_color;
uniform vec3 u_ambient_color;

void main() {
    int idx = int(v_material_id);
    idx = clamp(idx, 0, kMaterialCount - 1);
    vec3 n = normalize(v_normal);
    vec3 l = normalize(-u_light_dir);
    float ndotl = max(dot(n, l), 0.0);

    vec3 base = u_material_colors[idx];
    vec3 lit = base * (u_ambient_color + u_light_color * ndotl);
    frag_color = vec4(lit, 1.0);
}
)glsl";

struct IcosphereMesh final {
    std::vector<Vec3> positions;
    std::vector<u32> indices;
};

[[nodiscard]] u64 edge_key(const u32 a, const u32 b) noexcept {
    const u32 lo = std::min(a, b);
    const u32 hi = std::max(a, b);
    return (static_cast<u64>(lo) << 32) | static_cast<u64>(hi);
}

u32 midpoint_index(const u32 a, const u32 b, std::vector<Vec3> &positions,
                   std::unordered_map<u64, u32> &cache) {
    const u64 key = edge_key(a, b);
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

[[nodiscard]] IcosphereMesh make_icosphere(const i32 subdivisions) {
    const f32 t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    const std::array<Vec3, 12> base_positions = {
        Vec3{-1.0f, t, 0.0f}, Vec3{1.0f, t, 0.0f},  Vec3{-1.0f, -t, 0.0f}, Vec3{1.0f, -t, 0.0f},
        Vec3{0.0f, -1.0f, t}, Vec3{0.0f, 1.0f, t},  Vec3{0.0f, -1.0f, -t}, Vec3{0.0f, 1.0f, -t},
        Vec3{t, 0.0f, -1.0f}, Vec3{t, 0.0f, 1.0f},  Vec3{-t, 0.0f, -1.0f}, Vec3{-t, 0.0f, 1.0f},
    };

    const std::array<u32, 60> base_indices = {
        0, 11, 5,  0, 5,  1,  0, 1,  7,  0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4,
        11, 10, 2, 10, 7, 6,  7, 1,  8,  3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8,
        3, 8,  9, 4, 9, 5,  2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1,
    };

    IcosphereMesh mesh{};
    mesh.positions.assign(base_positions.begin(), base_positions.end());
    for (auto &p : mesh.positions) {
        p.try_normalize();
    }
    mesh.indices.assign(base_indices.begin(), base_indices.end());

    const i32 steps = std::max(0, subdivisions);
    for (i32 level = 0; level < steps; ++level) {
        std::unordered_map<u64, u32> midpoint_cache;
        midpoint_cache.reserve(mesh.indices.size());

        std::vector<u32> next_indices;
        next_indices.reserve(mesh.indices.size() * 4);

        for (usize i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const u32 a = mesh.indices[i + 0];
            const u32 b = mesh.indices[i + 1];
            const u32 c = mesh.indices[i + 2];

            const u32 ab = midpoint_index(a, b, mesh.positions, midpoint_cache);
            const u32 bc = midpoint_index(b, c, mesh.positions, midpoint_cache);
            const u32 ca = midpoint_index(c, a, mesh.positions, midpoint_cache);

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
    log::error(render, "Geometry shader compile failed: {}", info.data());
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
    log::error(render, "Geometry shader link failed: {}", info.data());
    glDeleteProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return 0;
}
} // namespace javelin::detail

export namespace javelin {

struct GeometryPass final {
    template <class Device> void init(Device &) {
        log::info(render, "Initializing geometry pass");
        create_shader_();
        create_icosphere_();
        create_instance_buffer_();
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down geometry pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        ZoneScopedN("GeometryPass");
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0) {
            return;
        }
        if (program_ == 0 || vao_ == 0 || instance_vbo_ == 0 || index_count_ <= 0) {
            return;
        }
        update_instances_(ctx);
        if (instance_count_ <= 0) {
            return;
        }

        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());

        glBindVertexArray(vao_);
        glDrawElementsInstanced(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr, instance_count_);
        glBindVertexArray(0);

        glUseProgram(0);
    }

  private:
    struct Vertex final {
        Vec3 position{};
        Vec3 normal{};
    };

    struct InstanceData final {
        Vec3 position{};
        f32 radius{};
        u32 material_id{};
    };

    void create_icosphere_() {
        if (vao_ != 0 || vbo_ != 0 || ibo_ != 0) {
            return;
        }

        constexpr i32 kSubdivisions = 2;
        const detail::IcosphereMesh mesh = detail::make_icosphere(kSubdivisions);

        std::vector<Vertex> vertices;
        vertices.reserve(mesh.positions.size());
        for (const Vec3 &p : mesh.positions) {
            vertices.push_back(Vertex{.position = p, .normal = p.normalized_or_zero()});
        }

        index_count_ = static_cast<i32>(mesh.indices.size());

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &ibo_);

        glBindVertexArray(vao_);

        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data(),
                     GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(u32)),
                     mesh.indices.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(offsetof(Vertex, normal)));

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void create_shader_() {
        if (program_ != 0) {
            return;
        }

        const u32 vs = detail::compile_shader(GL_VERTEX_SHADER, detail::kGeometryVertexShader);
        const u32 fs = detail::compile_shader(GL_FRAGMENT_SHADER, detail::kGeometryFragmentShader);
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
        u_material_colors_ = glGetUniformLocation(program_, "u_material_colors");
        u_light_dir_ = glGetUniformLocation(program_, "u_light_dir");
        u_light_color_ = glGetUniformLocation(program_, "u_light_color");
        u_ambient_color_ = glGetUniformLocation(program_, "u_ambient_color");

        constexpr std::array<Vec3, detail::kMaterialCount> kLinearSrgb = {
            Vec3{0.65f, 0.65f, 0.68f},
            Vec3{0.85f, 0.25f, 0.20f},
            Vec3{0.20f, 0.70f, 0.25f},
            Vec3{0.20f, 0.35f, 0.80f},
        };

        std::array<Vec3, detail::kMaterialCount> acescg{};
        for (usize i = 0; i < acescg.size(); ++i) {
            acescg[i] = linear_srgb_to_acescg(kLinearSrgb[i]);
        }

        std::array<f32, detail::kMaterialCount * 3> packed{};
        for (usize i = 0; i < acescg.size(); ++i) {
            packed[i * 3 + 0] = acescg[i].x;
            packed[i * 3 + 1] = acescg[i].y;
            packed[i * 3 + 2] = acescg[i].z;
        }

        glUseProgram(program_);
        if (u_material_colors_ >= 0) {
            glUniform3fv(u_material_colors_, detail::kMaterialCount, packed.data());
        }
        if (u_light_dir_ >= 0) {
            const Vec3 dir = Vec3{0.6f, -1.0f, -0.4f}.normalized_or_zero();
            glUniform3f(u_light_dir_, dir.x, dir.y, dir.z);
        }
        if (u_light_color_ >= 0) {
            const Vec3 light_aces = linear_srgb_to_acescg(Vec3{1.0f, 1.0f, 1.0f});
            glUniform3f(u_light_color_, light_aces.x, light_aces.y, light_aces.z);
        }
        if (u_ambient_color_ >= 0) {
            const Vec3 ambient_aces = linear_srgb_to_acescg(Vec3{0.08f, 0.08f, 0.10f});
            glUniform3f(u_ambient_color_, ambient_aces.x, ambient_aces.y, ambient_aces.z);
        }
        glUseProgram(0);
    }

    void create_instance_buffer_() {
        if (instance_vbo_ != 0 || vao_ == 0) {
            return;
        }

        glGenBuffers(1, &instance_vbo_);
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), nullptr);
        glVertexAttribDivisor(2, 1);

        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              reinterpret_cast<void *>(offsetof(InstanceData, radius)));
        glVertexAttribDivisor(3, 1);

        glEnableVertexAttribArray(4);
        glVertexAttribIPointer(4, 1, GL_UNSIGNED_INT, sizeof(InstanceData),
                               reinterpret_cast<void *>(offsetof(InstanceData, material_id)));
        glVertexAttribDivisor(4, 1);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void update_instances_(const RenderContext &ctx) {
        const usize count = std::min({ctx.view.shape_kind.size(), ctx.view.sphere.size(), ctx.view.material.size(),
                                      ctx.poses.curr_positions.size()});
        instance_data_.clear();
        instance_data_.reserve(count);

        for (usize i = 0; i < count; ++i) {
            if (ctx.view.shape_kind[i] != ShapeKind::sphere) {
                continue;
            }

            instance_data_.push_back(InstanceData{
                .position = ctx.poses.curr_positions[i],
                .radius = ctx.view.sphere[i].radius,
                .material_id = ctx.view.material[i].value,
            });
        }

        instance_count_ = static_cast<i32>(instance_data_.size());
        if (instance_count_ == 0) {
            return;
        }

        const usize needed = instance_data_.size();
        if (needed > instance_capacity_) {
            instance_capacity_ = std::max(needed, instance_capacity_ + instance_capacity_ / 2 + 1);
            glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(instance_capacity_ * sizeof(InstanceData)), nullptr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
        }

        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(needed * sizeof(InstanceData)),
                        instance_data_.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void release_() {
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }
        if (instance_vbo_ != 0) {
            glDeleteBuffers(1, &instance_vbo_);
            instance_vbo_ = 0;
        }
        if (ibo_ != 0) {
            glDeleteBuffers(1, &ibo_);
            ibo_ = 0;
        }
        if (vbo_ != 0) {
            glDeleteBuffers(1, &vbo_);
            vbo_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
        index_count_ = 0;
        instance_count_ = 0;
        instance_capacity_ = 0;
        instance_data_.clear();
        u_view_proj_ = -1;
        u_material_colors_ = -1;
        u_light_dir_ = -1;
        u_light_color_ = -1;
        u_ambient_color_ = -1;
    }

  private:
    u32 program_{};
    u32 vao_{};
    u32 vbo_{};
    u32 ibo_{};
    u32 instance_vbo_{};
    i32 index_count_{};
    i32 instance_count_{};
    usize instance_capacity_{};
    std::vector<InstanceData> instance_data_{};
    i32 u_view_proj_{-1};
    i32 u_material_colors_{-1};
    i32 u_light_dir_{-1};
    i32 u_light_color_{-1};
    i32 u_ambient_color_{-1};
};

} // namespace javelin
