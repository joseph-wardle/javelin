export module javelin.scene.physics_view;

import std;

import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.scene.entity;
import javelin.scene.shapes;
import javelin.scene.pose_channel;

export namespace javelin {
struct PhysicsView final {
    /// TODO: add spawn/despawn APIs here)
    u32 &count;

    // identity (read)
    std::span<const u32> generation;
    std::span<const u8> alive;

    // authored/static (read)
    std::span<const ShapeKind> shape_kind;
    std::span<const u32> shape_index;        // per-body index into shape pool
    std::span<const ShapeData> shapes;       // shape pool, indexed by shape_index[body]
    std::span<const MaterialId> material;    // per-body index into physics material pool
    std::span<const f32> material_restitution; // physics material pool (see PhysicsMaterial), indexed by MaterialId.value
    std::span<const f32> material_friction;    // physics material pool (see PhysicsMaterial), indexed by MaterialId.value
    std::span<const MeshId> mesh;
    std::span<const f32> inv_mass;
    std::span<const Vec3> inv_inertia;

    // sim state (read/write)
    std::span<Vec3> position;
    std::span<Vec3> velocity;
    std::span<Quat> orientation;
    std::span<Vec3> angular_velocity;

    // sleep state (read/write)
    // sleep_timer: consecutive ticks this body's speed has been below the sleep threshold.
    // asleep:      0 = awake, 1 = asleep; set once sleep_timer reaches the threshold.
    std::span<u32> sleep_timer;
    std::span<u8>  asleep;

    // presentation write access
    PoseChannel &poses;
};
} // namespace javelin
