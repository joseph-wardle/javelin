module;

#include <tracy/Tracy.hpp>

export module javelin.scene;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.constraint_types;
import javelin.scene.entity;
import javelin.scene.physics_materials;
import javelin.scene.physics_view;
import javelin.scene.pose_channel;
import javelin.scene.render_view;
import javelin.scene.scene_file;
import javelin.scene.shapes;

export namespace javelin {
namespace detail {
[[nodiscard]] inline f32 dynamic_inv_mass_for_sphere(const f32 radius, const f32 density) noexcept {
    // mass = density * r^3 (r^3 is proportional to the true sphere volume 4/3*pi*r^3;
    // the constant factor is absorbed into the unit system and kept consistent across shapes).
    const f32 mass = density * radius * radius * radius;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}

[[nodiscard]] inline f32 dynamic_inv_mass_for_box(const Vec3 half_extents, const f32 density) noexcept {
    // mass = density * full box volume = density * 8 * hx * hy * hz.
    const f32 mass = density * 8.0f * half_extents.x * half_extents.y * half_extents.z;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}

[[nodiscard]] inline Vec3 sphere_inv_inertia(const f32 radius, const f32 inv_mass) noexcept {
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

[[nodiscard]] inline Vec3 box_inv_inertia(const Vec3 half_extents, const f32 inv_mass) noexcept {
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

[[nodiscard]] inline Vec3 shape_inv_inertia(const ShapeKind kind, const ShapeData &shape, const f32 inv_mass) noexcept {
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

[[nodiscard]] inline f32 shape_inv_mass(const SceneFileBodyMotion motion, const ShapeData &shape,
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
} // namespace detail

struct Scene final {
    /*
    Scene data contract (v1)

    Authored + serialized:
    - shape definitions (kind + shape parameters).
    - per-body authored state: shape reference, material, mesh, initial
    position/orientation, initial linear velocity, initial angular velocity, and
    motion intent (dynamic/static).

    Derived at load/runtime (not serialized):
    - identity/liveness bookkeeping: generation_, alive_.
    - runtime shape indirection/cache: shape_kind_, shape_index_, shapes_.
    - physics derived properties: inv_mass_, inv_inertia_.
    - physics material pool: physics_material_restitution_, physics_material_friction_;
      sized to cover max MaterialId.value across both body references and authored
      physics_material records; all entries default to kDefaultPhysicsMaterial, then
      authored records overwrite their specific indices.
    - sleep state: sleep_timer_ counts consecutive ticks a body has been below the
      sleep velocity threshold; asleep_ (u8, 0=awake/1=asleep) is set once the timer
      reaches the threshold. Both reset to 0 on restore_simulation_from_initial_().
    - constraint definitions: constraints_ holds authored DistanceConstraints (body
      pairs, local anchors, rest length, compliance). constraint_ids_ preserves the
      authored string ids for round-trip scene-file export. Constraints reference
      bodies by scene index and carry no simulation state of their own.
    - transient runtime state: capacity_, count_, pose channel buffers/timestamps.

    Notes:
    - body_motion_ is authored state and is preserved exactly on scene
    save/export.
    - inv_mass_ and inv_inertia_ are always recomputed from authored shape +
    body_motion_.
    - Render/physics consume the SoA runtime arrays; scene-file data is an
    authored source, not the in-memory layout contract.
    */
    void reserve(u32 capacity) {
        capacity_ = capacity;

        // identity
        generation_.resize(capacity_);
        alive_.resize(capacity_, 0u);

        // authored/static
        shape_kind_.resize(capacity_, ShapeKind::sphere);
        shape_index_.resize(capacity_);
        shapes_.reserve(capacity_);
        shape_ids_.clear();
        shape_ids_.reserve(capacity_);
        body_ids_.resize(capacity_);
        body_motion_.resize(capacity_, SceneFileBodyMotion::dynamic_body);
        material_.resize(capacity_);
        // Physics material pool always starts with the default at index 0.
        // load_scene_from_disk expands it to cover all authored material indices.
        physics_material_restitution_.assign(1u, kDefaultPhysicsMaterial.restitution);
        physics_material_friction_.assign(1u, kDefaultPhysicsMaterial.friction);
        mesh_.resize(capacity_);
        inv_mass_.resize(capacity_, 1.0f);
        inv_inertia_.resize(capacity_);

        // simulation
        position_.resize(capacity_);
        velocity_.resize(capacity_);
        orientation_.resize(capacity_);
        angular_velocity_.resize(capacity_);
        // sleep state: zeroed so all bodies start awake on first load.
        sleep_timer_.resize(capacity_, 0u);
        asleep_.resize(capacity_, 0u);
        initial_position_.resize(capacity_);
        initial_velocity_.resize(capacity_);
        initial_orientation_.resize(capacity_);
        initial_angular_velocity_.resize(capacity_);

        poses_.reserve(capacity_);
    }

    [[nodiscard]] PhysicsView physics_view() noexcept {
        return PhysicsView{
            .count = count_,
            .generation = std::span<const u32>{generation_.data(), count_},
            .alive = std::span<const u8>{alive_.data(), count_},
            .shape_kind = std::span<const ShapeKind>{shape_kind_.data(), count_},
            .shape_index = std::span<const u32>{shape_index_.data(), count_},
            .shapes = std::span<const ShapeData>{shapes_.data(), shapes_.size()},
            .material = std::span<const MaterialId>{material_.data(), count_},
            .material_restitution =
                std::span<const f32>{physics_material_restitution_.data(), physics_material_restitution_.size()},
            .material_friction =
                std::span<const f32>{physics_material_friction_.data(), physics_material_friction_.size()},
            .mesh = std::span<const MeshId>{mesh_.data(), count_},
            .inv_mass = std::span<const f32>{inv_mass_.data(), count_},
            .inv_inertia = std::span<const Vec3>{inv_inertia_.data(), count_},
            .position = std::span<Vec3>{position_.data(), count_},
            .velocity = std::span<Vec3>{velocity_.data(), count_},
            .orientation = std::span<Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<Vec3>{angular_velocity_.data(), count_},
            .sleep_timer = std::span<u32>{sleep_timer_.data(), count_},
            .asleep = std::span<u8>{asleep_.data(), count_},
            .constraints = std::span<const DistanceConstraint>{constraints_.data(), constraints_.size()},
            .poses = poses_,
        };
    }

    [[nodiscard]] RenderView render_view() const noexcept {
        return RenderView{
            .generation = std::span<const u32>{generation_.data(), count_},
            .alive = std::span<const u8>{alive_.data(), count_},
            .shape_kind = std::span<const ShapeKind>{shape_kind_.data(), count_},
            .shape_index = std::span<const u32>{shape_index_.data(), count_},
            .shapes = std::span<const ShapeData>{shapes_.data(), shapes_.size()},
            .material = std::span<const MaterialId>{material_.data(), count_},
            .mesh = std::span<const MeshId>{mesh_.data(), count_},
            .inv_mass = std::span<const f32>{inv_mass_.data(), count_},
            .inv_inertia = std::span<const Vec3>{inv_inertia_.data(), count_},
            .constraints = std::span<const DistanceConstraint>{constraints_.data(), constraints_.size()},
            .position = std::span<const Vec3>{position_.data(), count_},
            .velocity = std::span<const Vec3>{velocity_.data(), count_},
            .orientation = std::span<const Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<const Vec3>{angular_velocity_.data(), count_},
            .poses = poses_,
        };
    }

    // Render reads this each frame for interpolation.
    [[nodiscard]] PoseSnapshot pose_snapshot() const noexcept { return poses_.snapshot(); }

    // Physics calls this after stepping to publish.
    void publish_poses_from_sim() noexcept {
        auto out = poses_.write_pose(count_);
        for (u32 i = 0; i < count_; ++i) {
            out.positions[i] = position_[i];
            out.orientations[i] = orientation_[i];
            out.sleep_flags[i] = 0u; // bodies are always awake at load/reset
        }
        poses_.publish();
    }

    void reset_simulation() noexcept {
        ZoneScopedN("Scene reset simulation");
        restore_simulation_from_initial_();
        publish_poses_from_sim();
    }

    [[nodiscard]] std::expected<void, SceneFileError>
    save_scene_to_disk(std::filesystem::path scene_path, const SceneFileSaveOptions &save_options = {}) const {
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

  private:
    void snapshot_initial_state_from_sim_() noexcept {
        for (u32 i = 0; i < count_; ++i) {
            initial_position_[i] = position_[i];
            initial_velocity_[i] = velocity_[i];
            initial_orientation_[i] = orientation_[i];
            initial_angular_velocity_[i] = angular_velocity_[i];
        }
    }

    void restore_simulation_from_initial_() noexcept {
        for (u32 i = 0; i < count_; ++i) {
            position_[i] = initial_position_[i];
            velocity_[i] = initial_velocity_[i];
            orientation_[i] = initial_orientation_[i];
            angular_velocity_[i] = initial_angular_velocity_[i];
            // All bodies wake on reset: any accumulated sleep state is stale.
            sleep_timer_[i] = 0u;
            asleep_[i] = 0u;
        }
    }

  public:
    static Scene load_scene_from_disk(std::filesystem::path scene_path) {
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

        // Build a density lookup keyed by MaterialId.value before the body loop.
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
            const f32 inv_mass = detail::shape_inv_mass(body.motion, shape, density);
            out.inv_mass_[idx] = inv_mass;
            out.inv_inertia_[idx] = detail::shape_inv_inertia(kind, shape, inv_mass);

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

  private:
    // Runtime bookkeeping (derived/transient).
    u32 capacity_{0};
    u32 count_{0};

    // Runtime identity/liveness (derived, not serialized).
    std::vector<u32> generation_{};
    std::vector<u8> alive_{};

    // Runtime authored caches + indirection (derived from authored scene-file
    // records).
    std::vector<ShapeKind> shape_kind_{};
    std::vector<u32> shape_index_{};
    std::vector<ShapeData> shapes_{};

    // Authored logical ids (serialized values, preserved for round-trip export).
    std::vector<std::string> shape_ids_{};
    std::vector<std::string> body_ids_{};

    // Authored constraints (serialized values, preserved for round-trip export).
    // constraints_:     distance constraint definitions (body pairs, anchors, rest length, compliance).
    // constraint_ids_:  per-constraint string ids, parallel to constraints_.
    std::vector<DistanceConstraint> constraints_{};
    std::vector<std::string> constraint_ids_{};

    // Authored motion intent (serialized values, preserved for round-trip export).
    std::vector<SceneFileBodyMotion> body_motion_{};

    // Authored render/material references (serialized values).
    std::vector<MaterialId> material_{}; // per-body index into physics material pool
    std::vector<MeshId> mesh_{};

    // Physics material pool: indexed by MaterialId.value.
    // Always contains kDefaultPhysicsMaterial at index 0; sized to cover the
    // maximum MaterialId.value used by any body in the scene.
    // density is load-time only; it is stored here solely for the save round-trip.
    std::vector<f32> physics_material_restitution_{};
    std::vector<f32> physics_material_friction_{};
    std::vector<f32> physics_material_density_{};

    // Physics derived values (recomputed during load/build).
    std::vector<f32> inv_mass_{};
    std::vector<Vec3> inv_inertia_{};

    // Simulation state (seeded from authored initial state; then physics-owned).
    std::vector<Vec3> position_{};
    std::vector<Vec3> velocity_{};
    std::vector<Quat> orientation_{};
    std::vector<Vec3> angular_velocity_{};

    // Sleep state (physics-owned; reset to 0 on restore_simulation_from_initial_).
    // sleep_timer_: consecutive ticks the body's speed has stayed below the sleep threshold.
    // asleep_:      0 = awake, 1 = asleep; set once sleep_timer_ reaches the threshold.
    std::vector<u32> sleep_timer_{};
    std::vector<u8> asleep_{};

    std::vector<Vec3> initial_position_{};
    std::vector<Vec3> initial_velocity_{};
    std::vector<Quat> initial_orientation_{};
    std::vector<Vec3> initial_angular_velocity_{};

    // Presentation channel (runtime only: physics publishes, render reads).
    PoseChannel poses_{};
};
} // namespace javelin
