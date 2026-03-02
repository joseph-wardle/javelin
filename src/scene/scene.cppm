module;

#include <tracy/Tracy.hpp>

export module javelin.scene;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.scene.entity;
import javelin.scene.physics_materials;
import javelin.scene.physics_view;
import javelin.scene.pose_channel;
import javelin.scene.render_view;
import javelin.scene.scene_file;
import javelin.scene.shapes;

export namespace javelin {
namespace detail {
[[nodiscard]] inline f32 dynamic_inv_mass_for_sphere(const f32 radius) noexcept {
    // Scene-file contract (v1): dynamic sphere mass uses unit density and r^3
    // proportional volume.
    const f32 mass = radius * radius * radius;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}

[[nodiscard]] inline f32 dynamic_inv_mass_for_box(const Vec3 half_extents) noexcept {
    // Scene-file contract (v1): dynamic box mass uses unit density and full box
    // volume.
    const f32 mass = 8.0f * half_extents.x * half_extents.y * half_extents.z;
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

[[nodiscard]] inline f32 shape_inv_mass(const SceneFileBodyMotion motion, const ShapeData &shape) noexcept {
    if (motion == SceneFileBodyMotion::static_body) {
        return 0.0f;
    }
    switch (shape.kind) {
    case ShapeKind::sphere:
        return dynamic_inv_mass_for_sphere(shape_sphere(shape).radius);
    case ShapeKind::box:
        return dynamic_inv_mass_for_box(shape_box(shape).half_extents);
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
      Material 0 may be overridden at runtime via set_physics_material(0u, ...) so that
      global ImGui sliders (restitution/friction) tune the fallback material live.
    - sleep state: sleep_timer_ counts consecutive ticks a body has been below the
      sleep velocity threshold; asleep_ (u8, 0=awake/1=asleep) is set once the timer
      reaches the threshold. Both reset to 0 on restore_simulation_from_initial_().
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
            .material_restitution = std::span<const f32>{physics_material_restitution_.data(), physics_material_restitution_.size()},
            .material_friction = std::span<const f32>{physics_material_friction_.data(), physics_material_friction_.size()},
            .mesh = std::span<const MeshId>{mesh_.data(), count_},
            .inv_mass = std::span<const f32>{inv_mass_.data(), count_},
            .inv_inertia = std::span<const Vec3>{inv_inertia_.data(), count_},
            .position = std::span<Vec3>{position_.data(), count_},
            .velocity = std::span<Vec3>{velocity_.data(), count_},
            .orientation = std::span<Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<Vec3>{angular_velocity_.data(), count_},
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
            .position = std::span<const Vec3>{position_.data(), count_},
            .velocity = std::span<const Vec3>{velocity_.data(), count_},
            .orientation = std::span<const Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<const Vec3>{angular_velocity_.data(), count_},
            .poses = poses_,
        };
    }

    // Updates a physics material in the runtime pool by id.
    // id=0 (the implicit default) is always present; ids beyond the pool size are ignored.
    // Intended for live overrides: the global ImGui sliders call this each tick to push
    // their current values into material 0, which any body with no explicit material uses.
    void set_physics_material(const u32 id, const PhysicsMaterial material) noexcept {
        if (id >= physics_material_restitution_.size()) {
            return;
        }
        physics_material_restitution_[id] = material.restitution;
        physics_material_friction_[id] = material.friction;
    }

    // Render reads this each frame for interpolation.
    [[nodiscard]] PoseSnapshot pose_snapshot() const noexcept { return poses_.snapshot(); }

    // Physics calls this after stepping to publish.
    void publish_poses_from_sim() noexcept {
        auto out = poses_.write_pose(count_);
        for (u32 i = 0; i < count_; ++i) {
            out.positions[i] = position_[i];
            out.orientations[i] = orientation_[i];
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
        // Material id=0 equal to kDefaultPhysicsMaterial is implicit and skipped.
        u32 authored_material_count = 0u;
        for (u32 i = 0; i < physics_material_restitution_.size(); ++i) {
            if (physics_material_restitution_[i] != kDefaultPhysicsMaterial.restitution ||
                physics_material_friction_[i] != kDefaultPhysicsMaterial.friction) {
                ++authored_material_count;
            }
        }
        out.reserve(static_cast<u32>(shapes_.size()), count_, authored_material_count);
        for (u32 i = 0; i < physics_material_restitution_.size(); ++i) {
            if (physics_material_restitution_[i] == kDefaultPhysicsMaterial.restitution &&
                physics_material_friction_[i] == kDefaultPhysicsMaterial.friction) {
                continue;
            }
            out.physics_materials.push_back(SceneFilePhysicsMaterial{
                .id = i,
                .material = PhysicsMaterial{
                    .restitution = physics_material_restitution_[i],
                    .friction    = physics_material_friction_[i],
                },
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

        auto save_result = out.save(scene_path, save_options);
        if (!save_result) {
            return std::unexpected(save_result.error());
        }

        log::info(scene, "Saved scene file '{}' (shapes={}, bodies={})", scene_path.string(), out.shapes.size(),
                  out.bodies.size());
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

            const f32 inv_mass = detail::shape_inv_mass(body.motion, shape);
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
        for (const SceneFilePhysicsMaterial &authored : in.physics_materials) {
            out.physics_material_restitution_[authored.id] = authored.material.restitution;
            out.physics_material_friction_[authored.id] = authored.material.friction;
        }

        out.snapshot_initial_state_from_sim_();
        out.publish_poses_from_sim();
        log::info(scene,
                  "Loaded scene file '{}' version={} units={} shapes={} bodies={} "
                  "(dynamic={}, static={}, spheres={}, "
                  "boxes={})",
                  scene_path.string(), in.version, in.units, in.shapes.size(), out.count_, dynamic_count, static_count,
                  sphere_count, box_count);
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

    // Authored motion intent (serialized values, preserved for round-trip export).
    std::vector<SceneFileBodyMotion> body_motion_{};

    // Authored render/material references (serialized values).
    std::vector<MaterialId> material_{};   // per-body index into physics material pool
    std::vector<MeshId> mesh_{};

    // Physics material pool: indexed by MaterialId.value.
    // Always contains kDefaultPhysicsMaterial at index 0; sized to cover the
    // maximum MaterialId.value used by any body in the scene.
    std::vector<f32> physics_material_restitution_{};
    std::vector<f32> physics_material_friction_{};

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
    std::vector<u8>  asleep_{};

    std::vector<Vec3> initial_position_{};
    std::vector<Vec3> initial_velocity_{};
    std::vector<Quat> initial_orientation_{};
    std::vector<Vec3> initial_angular_velocity_{};

    // Presentation channel (runtime only: physics publishes, render reads).
    PoseChannel poses_{};
};
} // namespace javelin
