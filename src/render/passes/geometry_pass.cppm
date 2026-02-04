module;

#include <glad/gl.h>
#include <tracy/Tracy.hpp>
#include <tracy/TracyOpenGL.hpp>

export module javelin.render.passes.geometry_pass;

import std;

import javelin.core.types;
import javelin.core.logging;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.types;
import javelin.scene.entity;
import javelin.scene.pose_channel;
import javelin.scene.shapes;

namespace javelin::detail {
constexpr i32 kMaterialCount = 4;
// Precomputed via inverse OCIO display transform to preserve the pre-ACES look.
constexpr Vec3 kClearColor = Vec3{0.032089f, 0.031913f, 0.037314f};

constexpr std::string_view kGeometryVertexShader = R"glsl(
#version 460 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec3 a_instance_pos;
layout(location = 3) in vec3 a_instance_scale;
layout(location = 4) in uint a_instance_material;
layout(location = 5) in vec4 a_instance_rot;

uniform mat4 u_view_proj;

flat out uint v_material_id;
out vec3 v_world_normal;
out vec3 v_local_normal;

vec3 quat_rotate(vec4 q, vec3 v) {
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

void main() {
    vec3 local_pos = a_position * a_instance_scale;
    vec3 world = a_instance_pos + quat_rotate(a_instance_rot, local_pos);
    v_material_id = a_instance_material;
    v_world_normal = normalize(quat_rotate(a_instance_rot, a_normal));
    v_local_normal = a_normal;
    gl_Position = u_view_proj * vec4(world, 1.0);
}
)glsl";

constexpr std::string_view kGeometryFragmentShader = R"glsl(
#version 460 core
flat in uint v_material_id;
in vec3 v_world_normal;
in vec3 v_local_normal;

layout(location = 0) out vec4 frag_color;

const int kMaterialCount = 4;
uniform vec3 u_material_colors[kMaterialCount];
uniform vec3 u_axis_colors[3];
uniform vec3 u_light_dir;
uniform vec3 u_light_color;
uniform vec3 u_ambient_color;
uniform int u_axis_mode;

void main() {
    int idx = int(v_material_id);
    idx = clamp(idx, 0, kMaterialCount - 1);
    vec3 n = normalize(v_world_normal);
    vec3 local_n = normalize(v_local_normal);
    vec3 l = normalize(-u_light_dir);
    float ndotl = max(dot(n, l), 0.0);

    vec3 base = u_material_colors[idx];
    vec3 lighting = u_ambient_color + u_light_color * ndotl;
    vec3 lit = base * lighting;

    if (u_axis_mode != 0) {
        frag_color = vec4(lit, 1.0);
        return;
    }

    float band_width = 0.18;
    float band_feather = 0.04;
    float bx = 1.0 - smoothstep(band_width, band_width + band_feather, abs(local_n.x));
    float by = 1.0 - smoothstep(band_width, band_width + band_feather, abs(local_n.y));
    float bz = 1.0 - smoothstep(band_width, band_width + band_feather, abs(local_n.z));
    float band_mask = clamp(bx + by + bz, 0.0, 1.0);
    vec3 band_color = bx * u_axis_colors[0] + by * u_axis_colors[1] + bz * u_axis_colors[2];
    vec3 shaded = mix(lit, band_color * lighting, band_mask);

    frag_color = vec4(shaded, 1.0);
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

u32 midpoint_index(const u32 a, const u32 b, std::vector<Vec3> &positions, std::unordered_map<u64, u32> &cache) {
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
        Vec3{-1.0f, t, 0.0f}, Vec3{1.0f, t, 0.0f}, Vec3{-1.0f, -t, 0.0f}, Vec3{1.0f, -t, 0.0f},
        Vec3{0.0f, -1.0f, t}, Vec3{0.0f, 1.0f, t}, Vec3{0.0f, -1.0f, -t}, Vec3{0.0f, 1.0f, -t},
        Vec3{t, 0.0f, -1.0f}, Vec3{t, 0.0f, 1.0f}, Vec3{-t, 0.0f, -1.0f}, Vec3{-t, 0.0f, 1.0f},
    };

    const std::array<u32, 60> base_indices = {
        0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4,  11, 10, 2,  10, 7, 6, 7, 1, 8,
        3, 9,  4, 3, 4, 2, 3, 2, 6, 3, 6, 8,  3, 8,  9,  4, 9, 5, 2, 4,  11, 6,  2,  10, 8,  6, 7, 9, 8, 1,
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
        create_cube_mesh_();
        create_instance_buffer_(sphere_vao_, sphere_instance_vbo_);
        create_instance_buffer_(cube_vao_, cube_instance_vbo_);
    }

    template <class Device> void resize(Device &, Extent2D) {}

    template <class Device> void shutdown(Device &) {
        log::info(render, "Shutting down geometry pass");
        release_();
    }

    void execute(RenderContext &ctx) {
        ZoneScopedN("GeometryPass");
        TracyGpuZone("GeometryPass");
        if (!ctx.extent.is_valid() || ctx.targets.scene_fbo == 0) {
            return;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, ctx.targets.scene_fbo);
        glViewport(0, 0, ctx.extent.width, ctx.extent.height);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glClearColor(detail::kClearColor.x, detail::kClearColor.y, detail::kClearColor.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        update_instances_(ctx);
        if (sphere_instance_count_ <= 0 && cube_instance_count_ <= 0) {
            return;
        }

        if (program_ == 0) {
            return;
        }

        glUseProgram(program_);
        glUniformMatrix4fv(u_view_proj_, 1, GL_FALSE, ctx.camera.view_proj.data());

        if (sphere_instance_count_ > 0 && sphere_vao_ != 0 && sphere_index_count_ > 0 &&
            sphere_instance_vbo_ != 0) {
            if (u_axis_mode_ >= 0) {
                glUniform1i(u_axis_mode_, 0);
            }
            glBindVertexArray(sphere_vao_);
            glDrawElementsInstanced(GL_TRIANGLES, sphere_index_count_, GL_UNSIGNED_INT, nullptr,
                                    sphere_instance_count_);
            glBindVertexArray(0);
        }
        if (cube_instance_count_ > 0 && cube_vao_ != 0 && cube_index_count_ > 0 && cube_instance_vbo_ != 0) {
            if (u_axis_mode_ >= 0) {
                glUniform1i(u_axis_mode_, 1);
            }
            glBindVertexArray(cube_vao_);
            glDrawElementsInstanced(GL_TRIANGLES, cube_index_count_, GL_UNSIGNED_INT, nullptr,
                                    cube_instance_count_);
            glBindVertexArray(0);
        }

        glUseProgram(0);
    }

  private:
    struct Vertex final {
        Vec3 position{};
        Vec3 normal{};
    };

    struct InstanceData final {
        Vec3 position{};
        Vec3 scale{};
        Quat orientation{};
        u32 material_id{};
    };

    void create_icosphere_() {
        if (sphere_vao_ != 0 || sphere_vbo_ != 0 || sphere_ibo_ != 0) {
            return;
        }

        constexpr i32 kSubdivisions = 2;
        const detail::IcosphereMesh mesh = detail::make_icosphere(kSubdivisions);

        std::vector<Vertex> vertices;
        vertices.reserve(mesh.positions.size());
        for (const Vec3 &p : mesh.positions) {
            vertices.push_back(Vertex{.position = p, .normal = p.normalized_or_zero()});
        }

        sphere_index_count_ = static_cast<i32>(mesh.indices.size());

        glGenVertexArrays(1, &sphere_vao_);
        glGenBuffers(1, &sphere_vbo_);
        glGenBuffers(1, &sphere_ibo_);

        glBindVertexArray(sphere_vao_);

        glBindBuffer(GL_ARRAY_BUFFER, sphere_vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data(),
                     GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphere_ibo_);
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

        log::info(render, "Icosphere mesh ready (verts={}, indices={})", vertices.size(), mesh.indices.size());
    }

    void create_cube_mesh_() {
        if (cube_vao_ != 0 || cube_vbo_ != 0 || cube_ibo_ != 0) {
            return;
        }

        std::array<Vertex, 24> vertices = {
            // +X
            Vertex{.position = Vec3{1.0f, -1.0f, -1.0f}, .normal = Vec3{1.0f, 0.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, 1.0f, -1.0f}, .normal = Vec3{1.0f, 0.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, 1.0f, 1.0f}, .normal = Vec3{1.0f, 0.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, -1.0f, 1.0f}, .normal = Vec3{1.0f, 0.0f, 0.0f}},
            // -X
            Vertex{.position = Vec3{-1.0f, -1.0f, 1.0f}, .normal = Vec3{-1.0f, 0.0f, 0.0f}},
            Vertex{.position = Vec3{-1.0f, 1.0f, 1.0f}, .normal = Vec3{-1.0f, 0.0f, 0.0f}},
            Vertex{.position = Vec3{-1.0f, 1.0f, -1.0f}, .normal = Vec3{-1.0f, 0.0f, 0.0f}},
            Vertex{.position = Vec3{-1.0f, -1.0f, -1.0f}, .normal = Vec3{-1.0f, 0.0f, 0.0f}},
            // +Y
            Vertex{.position = Vec3{-1.0f, 1.0f, -1.0f}, .normal = Vec3{0.0f, 1.0f, 0.0f}},
            Vertex{.position = Vec3{-1.0f, 1.0f, 1.0f}, .normal = Vec3{0.0f, 1.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, 1.0f, 1.0f}, .normal = Vec3{0.0f, 1.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, 1.0f, -1.0f}, .normal = Vec3{0.0f, 1.0f, 0.0f}},
            // -Y
            Vertex{.position = Vec3{-1.0f, -1.0f, 1.0f}, .normal = Vec3{0.0f, -1.0f, 0.0f}},
            Vertex{.position = Vec3{-1.0f, -1.0f, -1.0f}, .normal = Vec3{0.0f, -1.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, -1.0f, -1.0f}, .normal = Vec3{0.0f, -1.0f, 0.0f}},
            Vertex{.position = Vec3{1.0f, -1.0f, 1.0f}, .normal = Vec3{0.0f, -1.0f, 0.0f}},
            // +Z
            Vertex{.position = Vec3{-1.0f, -1.0f, 1.0f}, .normal = Vec3{0.0f, 0.0f, 1.0f}},
            Vertex{.position = Vec3{1.0f, -1.0f, 1.0f}, .normal = Vec3{0.0f, 0.0f, 1.0f}},
            Vertex{.position = Vec3{1.0f, 1.0f, 1.0f}, .normal = Vec3{0.0f, 0.0f, 1.0f}},
            Vertex{.position = Vec3{-1.0f, 1.0f, 1.0f}, .normal = Vec3{0.0f, 0.0f, 1.0f}},
            // -Z
            Vertex{.position = Vec3{1.0f, -1.0f, -1.0f}, .normal = Vec3{0.0f, 0.0f, -1.0f}},
            Vertex{.position = Vec3{-1.0f, -1.0f, -1.0f}, .normal = Vec3{0.0f, 0.0f, -1.0f}},
            Vertex{.position = Vec3{-1.0f, 1.0f, -1.0f}, .normal = Vec3{0.0f, 0.0f, -1.0f}},
            Vertex{.position = Vec3{1.0f, 1.0f, -1.0f}, .normal = Vec3{0.0f, 0.0f, -1.0f}},
        };

        std::array<u32, 36> indices = {
            0, 1, 2, 0, 2, 3,       4, 5, 6, 4, 6, 7,       8, 9, 10, 8, 10, 11,
            12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23,
        };

        cube_index_count_ = static_cast<i32>(indices.size());

        glGenVertexArrays(1, &cube_vao_);
        glGenBuffers(1, &cube_vbo_);
        glGenBuffers(1, &cube_ibo_);

        glBindVertexArray(cube_vao_);

        glBindBuffer(GL_ARRAY_BUFFER, cube_vbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data(),
                     GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ibo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(u32)), indices.data(),
                     GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(offsetof(Vertex, normal)));

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        log::info(render, "Cube mesh ready (verts={}, indices={})", vertices.size(), indices.size());
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
        u_axis_colors_ = glGetUniformLocation(program_, "u_axis_colors");
        u_light_dir_ = glGetUniformLocation(program_, "u_light_dir");
        u_light_color_ = glGetUniformLocation(program_, "u_light_color");
        u_ambient_color_ = glGetUniformLocation(program_, "u_ambient_color");
        u_axis_mode_ = glGetUniformLocation(program_, "u_axis_mode");

        // TEMP: hardcoded material palette for the test scene.
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

        constexpr std::array<Vec3, 3> kAxisLinearSrgb = {
            Vec3{1.0f, 0.15f, 0.10f},
            Vec3{0.10f, 1.0f, 0.20f},
            Vec3{0.10f, 0.45f, 1.0f},
        };
        std::array<f32, 3 * 3> axis_packed{};
        for (usize i = 0; i < kAxisLinearSrgb.size(); ++i) {
            const Vec3 aces = linear_srgb_to_acescg(kAxisLinearSrgb[i]);
            axis_packed[i * 3 + 0] = aces.x;
            axis_packed[i * 3 + 1] = aces.y;
            axis_packed[i * 3 + 2] = aces.z;
        }

        glUseProgram(program_);
        if (u_material_colors_ >= 0) {
            glUniform3fv(u_material_colors_, detail::kMaterialCount, packed.data());
        }
        if (u_axis_colors_ >= 0) {
            glUniform3fv(u_axis_colors_, 3, axis_packed.data());
        }
        if (u_axis_mode_ >= 0) {
            glUniform1i(u_axis_mode_, 0);
        }
        // TEMP: basic directional + ambient lighting for the test scene.
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

    void create_instance_buffer_(const u32 vao, u32 &instance_vbo) {
        if (instance_vbo != 0 || vao == 0) {
            return;
        }

        glGenBuffers(1, &instance_vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, instance_vbo);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData), nullptr);
        glVertexAttribDivisor(2, 1);

        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              reinterpret_cast<void *>(offsetof(InstanceData, scale)));
        glVertexAttribDivisor(3, 1);

        glEnableVertexAttribArray(4);
        glVertexAttribIPointer(4, 1, GL_UNSIGNED_INT, sizeof(InstanceData),
                               reinterpret_cast<void *>(offsetof(InstanceData, material_id)));
        glVertexAttribDivisor(4, 1);

        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                              reinterpret_cast<void *>(offsetof(InstanceData, orientation)));
        glVertexAttribDivisor(5, 1);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void update_instances_(const RenderContext &ctx) {
        const usize count = std::min({ctx.view.shape_kind.size(), ctx.view.shape_index.size(), ctx.view.material.size(),
                                      ctx.poses.curr_positions.size(), ctx.poses.curr_orientations.size()});
        sphere_instances_.clear();
        cube_instances_.clear();
        sphere_instances_.reserve(count);
        cube_instances_.reserve(count);

        for (usize i = 0; i < count; ++i) {
            const u32 shape_id = ctx.view.shape_index[i];
#ifndef NDEBUG
            if (shape_id >= ctx.view.shapes.size()) {
                log::error(render, "Render shape index out of range (id={} shape_id={})", i, shape_id);
                std::terminate();
            }
#endif
            const ShapeData &shape = ctx.view.shapes[shape_id];
            const PoseSample pose = sample_pose(ctx.poses, static_cast<u32>(i), ctx.pose_alpha);
            switch (ctx.view.shape_kind[i]) {
            case ShapeKind::sphere: {
#ifndef NDEBUG
                if (shape.kind != ShapeKind::sphere) {
                    log::error(render, "Render expects sphere shape (id={})", i);
                    std::terminate();
                }
#endif
                const SphereShape &sphere = shape_sphere(shape);
                sphere_instances_.push_back(InstanceData{
                    .position = pose.position,
                    .scale = Vec3{sphere.radius},
                    .orientation = pose.orientation,
                    .material_id = ctx.view.material[i].value,
                });
            } break;
            case ShapeKind::box: {
#ifndef NDEBUG
                if (shape.kind != ShapeKind::box) {
                    log::error(render, "Render expects box shape (id={})", i);
                    std::terminate();
                }
#endif
                const BoxShape &box = shape_box(shape);
                cube_instances_.push_back(InstanceData{
                    .position = pose.position,
                    .scale = box.half_extents,
                    .orientation = pose.orientation,
                    .material_id = ctx.view.material[i].value,
                });
            } break;
            }
        }

        upload_instances_(sphere_instance_vbo_, sphere_instances_, sphere_instance_capacity_, sphere_instance_count_);
        upload_instances_(cube_instance_vbo_, cube_instances_, cube_instance_capacity_, cube_instance_count_);
    }

    static void upload_instances_(const u32 instance_vbo, const std::vector<InstanceData> &instances,
                                  usize &capacity, i32 &count) {
        count = static_cast<i32>(instances.size());
        if (count <= 0) {
            return;
        }
        if (instance_vbo == 0) {
            count = 0;
            return;
        }

        const usize needed = instances.size();
        if (needed > capacity) {
            capacity = std::max(needed, capacity + capacity / 2 + 1);
            glBindBuffer(GL_ARRAY_BUFFER, instance_vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(capacity * sizeof(InstanceData)), nullptr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBindBuffer(GL_ARRAY_BUFFER, instance_vbo);
        }

        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(needed * sizeof(InstanceData)),
                        instances.data());
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
        if (cube_instance_vbo_ != 0) {
            glDeleteBuffers(1, &cube_instance_vbo_);
            cube_instance_vbo_ = 0;
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
        sphere_instance_capacity_ = 0;
        cube_instance_capacity_ = 0;
        sphere_instances_.clear();
        cube_instances_.clear();
        u_view_proj_ = -1;
        u_material_colors_ = -1;
        u_axis_colors_ = -1;
        u_light_dir_ = -1;
        u_light_color_ = -1;
        u_ambient_color_ = -1;
        u_axis_mode_ = -1;
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
    i32 u_view_proj_{-1};
    i32 u_material_colors_{-1};
    i32 u_axis_colors_{-1};
    i32 u_light_dir_{-1};
    i32 u_light_color_{-1};
    i32 u_ambient_color_{-1};
    i32 u_axis_mode_{-1};
};

} // namespace javelin
