module;

#include <tracy/Tracy.hpp>

module javelin.scene;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.physics.constraint_types;
import javelin.scene.entity;
import javelin.scene.physics_materials;
import javelin.scene.scene_file;
import javelin.scene.shapes;

namespace javelin {
namespace {

// Derived physical properties (inverse mass + inverse principal inertia)
// computed at load time from the authored shape, density, and motion intent.
// Static bodies are encoded by inv_mass == 0, which propagates to inv_inertia
// to short-circuit the solver's mass terms.

[[nodiscard]] f32 dynamic_inv_mass_for_sphere(const f32 radius, const f32 density) noexcept {
    // mass = density * r^3 (r^3 is proportional to the true sphere volume 4/3*pi*r^3;
    // the constant factor is absorbed into the unit system and kept consistent across shapes).
    const f32 mass = density * radius * radius * radius;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}

[[nodiscard]] f32 dynamic_inv_mass_for_box(const Vec3 half_extents, const f32 density) noexcept {
    // mass = density * full box volume = density * 8 * hx * hy * hz.
    const f32 mass = density * 8.0f * half_extents.x * half_extents.y * half_extents.z;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}

[[nodiscard]] Vec3 sphere_inv_inertia(const f32 radius, const f32 inv_mass) noexcept {
    if (inv_mass <= 0.0f) {
        return Vec3{};
    }
    const f32 r2 = radius * radius;
    if (r2 <= 1e-6f) {
        return Vec3{};
    }
    const f32 inv_inertia = 2.5f * inv_mass / r2;
    return Vec3{inv_inertia};
}

[[nodiscard]] Vec3 box_inv_inertia(const Vec3 half_extents, const f32 inv_mass) noexcept {
    if (inv_mass <= 0.0f) {
        return Vec3{};
    }
    const f32 hx2 = half_extents.x * half_extents.x;
    const f32 hy2 = half_extents.y * half_extents.y;
    const f32 hz2 = half_extents.z * half_extents.z;
    const f32 denom_x = hy2 + hz2;
    const f32 denom_y = hx2 + hz2;
    const f32 denom_z = hx2 + hy2;
    const f32 inv_ix = (denom_x > 1e-6f) ? (3.0f * inv_mass / denom_x) : 0.0f;
    const f32 inv_iy = (denom_y > 1e-6f) ? (3.0f * inv_mass / denom_y) : 0.0f;
    const f32 inv_iz = (denom_z > 1e-6f) ? (3.0f * inv_mass / denom_z) : 0.0f;
    return Vec3{inv_ix, inv_iy, inv_iz};
}

[[nodiscard]] Vec3 shape_inv_inertia(const ShapeKind kind, const ShapeData &shape, const f32 inv_mass) noexcept {
#ifndef NDEBUG
    if (kind != shape.kind) {
        log::error(scene, "Shape kind mismatch during inertia compute");
        std::terminate();
    }
#endif
    switch (kind) {
    case ShapeKind::sphere:
        return sphere_inv_inertia(shape_sphere(shape).radius, inv_mass);
    case ShapeKind::box:
        return box_inv_inertia(shape_box(shape).half_extents, inv_mass);
    }
    return Vec3{};
}

[[nodiscard]] f32 shape_inv_mass(const SceneFileBodyMotion motion, const ShapeData &shape,
                                 const f32 density) noexcept {
    if (motion == SceneFileBodyMotion::static_body) {
        return 0.0f;
    }
    switch (shape.kind) {
    case ShapeKind::sphere:
        return dynamic_inv_mass_for_sphere(shape_sphere(shape).radius, density);
    case ShapeKind::box:
        return dynamic_inv_mass_for_box(shape_box(shape).half_extents, density);
    }
    return 0.0f;
}

} // namespace

Scene Scene::load_scene_from_disk(std::filesystem::path scene_path) {
    ZoneScopedN("Scene load from disk");
    log::info(scene, "Loading scene file: {}", scene_path.string());

    const auto file = SceneFile::load(scene_path);
    if (!file) {
        log::error(scene, "{}", format_scene_file_error(file.error()));
        std::terminate();
    }

    const SceneFile &in = *file;

    Scene out{};
    const u32 body_count = static_cast<u32>(in.bodies.size());

    out.reserve(body_count);
    out.count_ = body_count;
    out.shapes_.clear();
    out.shapes_.reserve(in.shapes.size());
    u32 sphere_count = 0;
    u32 box_count = 0;
    u32 static_count = 0;
    u32 dynamic_count = 0;

    std::unordered_map<std::string_view, u32> shape_lookup{};
    shape_lookup.reserve(in.shapes.size() * 2u + 1u);
    out.shape_ids_.clear();
    out.shape_ids_.reserve(in.shapes.size());
    for (const SceneFileShape &shape : in.shapes) {
        const u32 shape_index = static_cast<u32>(out.shapes_.size());
        out.shapes_.push_back(shape.shape);
        out.shape_ids_.push_back(shape.id);
        shape_lookup.emplace(shape.id, shape_index);
    }

    // Density lookup keyed by MaterialId.value, used during the body loop.
    // Most scenes have few materials so a small lambda with linear search is sufficient.
    // Unlisted materials default to kDefaultMaterialDensity (1.0).
    auto material_density = [&](const u32 mat_id) -> f32 {
        for (const SceneFilePhysicsMaterial &mat : in.physics_materials) {
            if (mat.id == mat_id) {
                return mat.density;
            }
        }
        return kDefaultMaterialDensity;
    };

    for (u32 idx = 0; idx < out.count_; ++idx) {
        const SceneFileBody &body = in.bodies[idx];
        const auto shape_it = shape_lookup.find(body.shape_id);
        if (shape_it == shape_lookup.end()) {
            log::error(scene, "Body references unknown shape (body='{}' shape='{}')", body.id, body.shape_id);
            std::terminate();
        }
        const u32 shape_index = shape_it->second;
        const ShapeData &shape = out.shapes_[shape_index];
        const ShapeKind kind = shape.kind;

        out.alive_[idx] = 1u;
        out.generation_[idx] = 1;
        out.shape_kind_[idx] = kind;
        out.shape_index_[idx] = shape_index;

        const f32 density = material_density(body.material.value);
        const f32 inv_mass = shape_inv_mass(body.motion, shape, density);
        out.inv_mass_[idx] = inv_mass;
        out.inv_inertia_[idx] = shape_inv_inertia(kind, shape, inv_mass);

        out.body_ids_[idx] = body.id;
        out.body_motion_[idx] = body.motion;
        out.material_[idx] = body.material;
        out.mesh_[idx] = body.mesh;
        out.position_[idx] = body.position;
        out.velocity_[idx] = body.velocity;
        out.orientation_[idx] = body.orientation;
        out.angular_velocity_[idx] = body.angular_velocity;

        switch (kind) {
        case ShapeKind::sphere:
            ++sphere_count;
            break;
        case ShapeKind::box:
            ++box_count;
            break;
        }
        if (body.motion == SceneFileBodyMotion::static_body) {
            ++static_count;
        } else {
            ++dynamic_count;
        }
    }

    // Size the pool to cover every MaterialId used: both body references and
    // explicit physics_material records. All entries default to kDefaultPhysicsMaterial;
    // authored records then overwrite their specific indices.
    u32 max_material_value = 0u;
    for (u32 idx = 0; idx < out.count_; ++idx) {
        max_material_value = std::max(max_material_value, out.material_[idx].value);
    }
    for (const SceneFilePhysicsMaterial &authored : in.physics_materials) {
        max_material_value = std::max(max_material_value, authored.id);
    }
    out.physics_material_restitution_.assign(max_material_value + 1u, kDefaultPhysicsMaterial.restitution);
    out.physics_material_friction_.assign(max_material_value + 1u, kDefaultPhysicsMaterial.friction);
    out.physics_material_density_.assign(max_material_value + 1u, kDefaultMaterialDensity);
    for (const SceneFilePhysicsMaterial &authored : in.physics_materials) {
        out.physics_material_restitution_[authored.id] = authored.material.restitution;
        out.physics_material_friction_[authored.id] = authored.material.friction;
        out.physics_material_density_[authored.id] = authored.density;
    }

    // Resolve constraint body string ids to scene body indices.
    out.constraints_.clear();
    out.constraint_ids_.clear();
    if (!in.constraints.empty()) {
        // Build a body-id → scene index map.  The body order in in.bodies matches
        // the scene SoA order established by the loop above.
        std::unordered_map<std::string_view, u32> body_lookup{};
        body_lookup.reserve(out.count_ * 2u + 1u);
        for (u32 idx = 0; idx < out.count_; ++idx) {
            body_lookup.emplace(out.body_ids_[idx], idx);
        }

        out.constraints_.reserve(in.constraints.size());
        out.constraint_ids_.reserve(in.constraints.size());
        for (const SceneFileConstraint &c : in.constraints) {
            const auto a_it = body_lookup.find(c.body_a_id);
            const auto b_it = body_lookup.find(c.body_b_id);
            // Body refs were already cross-validated by SceneFile::load(); this is a safety net.
            if (a_it == body_lookup.end() || b_it == body_lookup.end()) {
                log::error(scene, "Constraint '{}' references unknown body (body_a='{}' body_b='{}')", c.id,
                           c.body_a_id, c.body_b_id);
                std::terminate();
            }
            out.constraints_.push_back(DistanceConstraint{
                .body_a = a_it->second,
                .body_b = b_it->second,
                .anchor_a = c.anchor_a,
                .anchor_b = c.anchor_b,
                .rest_length = c.rest_length,
                .compliance = c.compliance,
            });
            out.constraint_ids_.push_back(c.id);
        }
    }

    out.snapshot_initial_state_from_sim_();
    out.publish_poses_from_sim();
    log::info(scene,
              "Loaded scene file '{}' version={} units={} shapes={} bodies={} "
              "(dynamic={}, static={}, spheres={}, boxes={}) constraints={}",
              scene_path.string(), in.version, in.units, in.shapes.size(), out.count_, dynamic_count, static_count,
              sphere_count, box_count, out.constraints_.size());
    return out;
}

std::expected<void, SceneFileError>
Scene::save_scene_to_disk(std::filesystem::path scene_path, const SceneFileSaveOptions &save_options) const {
    ZoneScopedN("Scene save to disk");
    log::info(scene, "Saving scene file: {}", scene_path.string());

    auto error = [&scene_path](std::string message) -> std::expected<void, SceneFileError> {
        return std::unexpected(SceneFileError{
            .path = scene_path,
            .line = 0u,
            .message = std::move(message),
        });
    };

    SceneFile out{};
    out.clear();

    // Export any physics_material pool entries that differ from the default.
    // Material id=0 equal to kDefaultPhysicsMaterial with default density is implicit and skipped.
    const usize pool_size = physics_material_restitution_.size();
    auto is_default_material = [&](const u32 i) {
        const f32 density =
            (i < physics_material_density_.size()) ? physics_material_density_[i] : kDefaultMaterialDensity;
        return physics_material_restitution_[i] == kDefaultPhysicsMaterial.restitution &&
               physics_material_friction_[i] == kDefaultPhysicsMaterial.friction &&
               density == kDefaultMaterialDensity;
    };
    u32 authored_material_count = 0u;
    for (u32 i = 0; i < pool_size; ++i) {
        if (!is_default_material(i)) {
            ++authored_material_count;
        }
    }
    out.reserve(static_cast<u32>(shapes_.size()), count_, authored_material_count,
                static_cast<u32>(constraints_.size()));
    for (u32 i = 0; i < pool_size; ++i) {
        if (is_default_material(i)) {
            continue;
        }
        const f32 density =
            (i < physics_material_density_.size()) ? physics_material_density_[i] : kDefaultMaterialDensity;
        out.physics_materials.push_back(SceneFilePhysicsMaterial{
            .id = i,
            .material =
                PhysicsMaterial{
                    .restitution = physics_material_restitution_[i],
                    .friction = physics_material_friction_[i],
                },
            .density = density,
        });
    }

    std::vector<std::string> shape_ids{};
    shape_ids.reserve(shapes_.size());
    for (u32 i = 0; i < shapes_.size(); ++i) {
        std::string id{};
        if (i < shape_ids_.size() && !shape_ids_[i].empty()) {
            id = shape_ids_[i];
        } else {
            id = std::format("shape_{:05}", i);
        }
        shape_ids.push_back(id);
        out.shapes.push_back(SceneFileShape{
            .id = std::move(id),
            .shape = shapes_[i],
        });
    }

    // Collect effective body string ids in scene-index order; constraints reference them by name.
    std::vector<std::string> out_body_ids{};
    out_body_ids.reserve(count_);
    for (u32 i = 0; i < count_; ++i) {
        if (shape_index_[i] >= shape_ids.size()) {
            return error(std::format("Cannot export body {}: shape_index={} is out "
                                     "of range (shape_count={})",
                                     i, shape_index_[i], shape_ids.size()));
        }

        std::string body_id{};
        if (i < body_ids_.size() && !body_ids_[i].empty()) {
            body_id = body_ids_[i];
        } else {
            body_id = std::format("body_{:05}", i);
        }

        out_body_ids.push_back(body_id);
        out.bodies.push_back(SceneFileBody{
            .id = std::move(body_id),
            .shape_id = shape_ids[shape_index_[i]],
            .motion = body_motion_[i],
            .material = material_[i],
            .mesh = mesh_[i],
            .position = initial_position_[i],
            .orientation = initial_orientation_[i],
            .velocity = initial_velocity_[i],
            .angular_velocity = initial_angular_velocity_[i],
        });
    }

    // Export distance constraints: map numeric body indices back to string ids.
    for (usize i = 0; i < constraints_.size(); ++i) {
        const DistanceConstraint &c = constraints_[i];
        if (c.body_a >= out_body_ids.size() || c.body_b >= out_body_ids.size()) {
            return error(std::format("Cannot export constraint {}: body index out of range "
                                     "(body_a={} body_b={} body_count={})",
                                     i, c.body_a, c.body_b, out_body_ids.size()));
        }
        std::string constraint_id{};
        if (i < constraint_ids_.size() && !constraint_ids_[i].empty()) {
            constraint_id = constraint_ids_[i];
        } else {
            constraint_id = std::format("constraint_{:05}", i);
        }
        out.constraints.push_back(SceneFileConstraint{
            .id = std::move(constraint_id),
            .body_a_id = out_body_ids[c.body_a],
            .body_b_id = out_body_ids[c.body_b],
            .anchor_a = c.anchor_a,
            .anchor_b = c.anchor_b,
            .rest_length = c.rest_length,
            .compliance = c.compliance,
        });
    }

    auto save_result = out.save(scene_path, save_options);
    if (!save_result) {
        return std::unexpected(save_result.error());
    }

    log::info(scene, "Saved scene file '{}' (shapes={}, bodies={}, constraints={})", scene_path.string(),
              out.shapes.size(), out.bodies.size(), out.constraints.size());
    return {};
}

} // namespace javelin
