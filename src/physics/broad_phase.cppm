module;

#include <tracy/Tracy.hpp>

export module javelin.physics.broad_phase;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.types;

export namespace javelin {

void broad_phase_sphere_pairs(std::span<const Vec3> position, std::span<const Vec3> velocity,
                              std::span<const f32> inv_mass, const f32 dt,
                              DynamicBvh &dynamic_bvh, const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
                              std::vector<BodyPair> &pairs, std::vector<u32> &query_hits) {
    ZoneScopedN("Physics broad phase");
    pairs.clear();
    const u32 count = static_cast<u32>(position.size());
    if (count == 0) {
        return;
    }

    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] > 0.0f) {
            dynamic_bvh.update(i, bounds_cache[i]);
        } else {
            dynamic_bvh.remove(i);
        }
    }

    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }

        const Aabb swept = Aabb::sweep(bounds_cache[i], velocity[i], dt);

        query_hits.clear();
        dynamic_bvh.query(swept, query_hits);
        for (const u32 j : query_hits) {
            if (j <= i) {
                continue;
            }
            pairs.push_back(BodyPair{.a = i, .b = j});
        }

        if (!static_bvh.empty()) {
            query_hits.clear();
            static_bvh.query(swept, query_hits);
            for (const u32 j : query_hits) {
                if (j == i) {
                    continue;
                }
                pairs.push_back(BodyPair{.a = i, .b = j});
            }
        }
    }
}

} // namespace javelin
