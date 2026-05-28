module;

#include <tracy/Tracy.hpp>

export module javelin.physics.bounds_builder;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.physics.aabb;
import javelin.scene.bodies;
import javelin.scene.shapes;

export namespace javelin {

// Per-body world-space AABB cache, rebuilt once per tick before the broad phase.
//
// Bounds are computed from pre-integration transforms: the broad and narrow
// phases see the same positions/orientations the solver will later mutate, so
// the cache stays consistent for the rest of the tick.  Box AABBs are
// conservative (axis-aligned support over each box axis).
struct BoundsBuilder final {
    void reserve(const u32 count) { cache_.reserve(count); }

    void update(const Bodies &view) {
        ZoneScopedN("Physics build bounds cache");
        const u32 count = view.count;
        cache_.resize(count);
        for (u32 i = 0; i < count; ++i) {
#ifndef NDEBUG
            if (view.shape_index[i] >= view.shapes.size()) {
                log::error(physics, "Shape index out of range (id={} shape_id={})", i, view.shape_index[i]);
                std::terminate();
            }
#endif
            const ShapeData &shape = view.shapes[view.shape_index[i]];
            switch (view.shape_kind[i]) {
            case ShapeKind::sphere: {
#ifndef NDEBUG
                if (shape.kind != ShapeKind::sphere) {
                    log::error(physics, "Shape kind mismatch (id={})", i);
                    std::terminate();
                }
#endif
                const SphereShape &sphere = shape_sphere(shape);
                cache_[i] = Aabb::from_sphere(view.position[i], sphere.radius);
            } break;
            case ShapeKind::box: {
#ifndef NDEBUG
                if (shape.kind != ShapeKind::box) {
                    log::error(physics, "Shape kind mismatch (id={})", i);
                    std::terminate();
                }
#endif
                const BoxShape &box = shape_box(shape);
                const Mat3 rot = to_mat3(view.orientation[i]);
                const Vec3 c0 = rot.col(0);
                const Vec3 c1 = rot.col(1);
                const Vec3 c2 = rot.col(2);
                const Vec3 abs0{std::fabs(c0.x), std::fabs(c0.y), std::fabs(c0.z)};
                const Vec3 abs1{std::fabs(c1.x), std::fabs(c1.y), std::fabs(c1.z)};
                const Vec3 abs2{std::fabs(c2.x), std::fabs(c2.y), std::fabs(c2.z)};
                const Vec3 extents =
                    abs0 * box.half_extents.x + abs1 * box.half_extents.y + abs2 * box.half_extents.z;
                const Vec3 center = view.position[i];
                cache_[i] = Aabb{center - extents, center + extents};
            } break;
            }
        }
    }

    [[nodiscard]] std::span<const Aabb> bounds() const noexcept { return std::span<const Aabb>{cache_}; }

    // The dynamic BVH update path mutates a body's leaf using its current AABB.
    // That path lives in the body-partition stage, which already knows the body
    // index it is updating.  Indexing remains by global body id.
    [[nodiscard]] Aabb operator[](const u32 id) const noexcept { return cache_[id]; }

  private:
    std::vector<Aabb> cache_{};
};

} // namespace javelin
