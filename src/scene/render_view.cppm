export module javelin.scene.render_view;

import std;

import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.scene.shapes;
import javelin.scene.entity;
import javelin.scene.pose_channel;

export namespace javelin {

struct RenderView final {
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

    // sim state (read)
    std::span<const Vec3> position;
    std::span<const Vec3> velocity;
    std::span<const Quat> orientation;
    std::span<const Vec3> angular_velocity;

    // presentation read access
    const PoseChannel &poses;
};

} // namespace javelin
