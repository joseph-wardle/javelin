export module javelin.scene.render_view;

import std;

import javelin.core.types;
import javelin.scene.shapes;
import javelin.scene.entity;
import javelin.scene.pose_channel;

export namespace javelin {

struct RenderView final {
    // authored/static (read)
    std::span<const ShapeKind> shape_kind;
    std::span<const SphereShape> sphere;
    std::span<const MaterialId> material;
    std::span<const MeshId> mesh;

    // presentation read access
    const PoseChannel &poses;
};

} // namespace javelin
