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
    std::span<const u32> shape_index;  // per-body index into shape pool
    std::span<const ShapeData> shapes; // shape pool
    std::span<const MaterialId> material;
    std::span<const MeshId> mesh;
    std::span<const f32> inv_mass;
    std::span<const Vec3> inv_inertia;

    // sim state (read/write)
    std::span<Vec3> position;
    std::span<Vec3> velocity;
    std::span<Quat> orientation;
    std::span<Vec3> angular_velocity;

    // presentation write access
    PoseChannel &poses;
};
} // namespace javelin
