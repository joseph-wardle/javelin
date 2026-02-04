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
    std::span<const ShapeData> shapes;     // shape pool
    std::span<const u32> shape_index;      // per-body index into shape pool
    std::span<const MaterialId> material;
    std::span<const MeshId> mesh;

    // presentation read access
    const PoseChannel &poses;
};

} // namespace javelin
