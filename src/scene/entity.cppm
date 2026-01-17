export module javelin.scene.entity;

import javelin.core.types;

export namespace javelin {

struct EntityId final {
    u32 index{};
    u32 generation{};
};

struct MaterialId final {
    u32 value{};
};
struct MeshId final {
    u32 value{};
};

} // namespace javelin
