module;

#include <tracy/Tracy.hpp>

export module javelin.scene;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.constraint_types;
import javelin.scene.bodies;
import javelin.scene.entity;
import javelin.scene.physics_materials;
import javelin.scene.pose_channel;
import javelin.scene.scene_file;
import javelin.scene.shapes;

export namespace javelin {

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

    [[nodiscard]] Bodies bodies() noexcept {
        return Bodies{
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

    [[nodiscard]] BodiesRead bodies() const noexcept {
        return BodiesRead{
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
            .position = std::span<const Vec3>{position_.data(), count_},
            .velocity = std::span<const Vec3>{velocity_.data(), count_},
            .orientation = std::span<const Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<const Vec3>{angular_velocity_.data(), count_},
            .sleep_timer = std::span<const u32>{sleep_timer_.data(), count_},
            .asleep = std::span<const u8>{asleep_.data(), count_},
            .constraints = std::span<const DistanceConstraint>{constraints_.data(), constraints_.size()},
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

    // Serialise the authored portion of the scene back to disk. Body simulation state
    // is exported from the initial-state mirrors, so saving a stepped scene still
    // round-trips the authored configuration.  Defined in scene_builder.cppm.
    [[nodiscard]] std::expected<void, SceneFileError>
    save_scene_to_disk(std::filesystem::path scene_path,
                       const SceneFileSaveOptions &save_options = {}) const;

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
    // Construct a Scene from an authored scene file. Computes derived properties
    // (per-body inv_mass / inv_inertia from shape + material density + motion intent),
    // resolves shape/body/constraint string ids, sizes the material pool, and
    // publishes the initial pose snapshot.  Defined in scene_builder.cppm.
    [[nodiscard]] static Scene load_scene_from_disk(std::filesystem::path scene_path);

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
