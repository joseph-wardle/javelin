module;

#include <tracy/Tracy.hpp>

export module javelin.physics.broad_phase;

import std;
import javelin.core.types;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.types;

export namespace javelin {

void broad_phase_sphere_pairs(std::span<const u32> dynamic_ids, DynamicBvh &dynamic_bvh, const StaticBvh &static_bvh,
                              std::span<const Aabb> bounds_cache, std::vector<BodyPair> &pairs,
                              std::vector<u32> &query_hits, std::vector<u32> &query_stack) {
    ZoneScopedN("Physics broad phase");
    pairs.clear();
    if (dynamic_ids.empty()) {
        return;
    }

    // dynamic_ids are built in ascending order so j <= id filters duplicates/self.
    for (const u32 id : dynamic_ids) {
        dynamic_bvh.update(id, bounds_cache[id]);
    }

    const bool has_static = !static_bvh.empty();
    for (const u32 id : dynamic_ids) {
        const Aabb query_bounds = bounds_cache[id];

        query_hits.clear();
        dynamic_bvh.query(query_bounds, query_hits, query_stack);
        for (const u32 j : query_hits) {
            if (j <= id) {
                continue;
            }
            pairs.push_back(BodyPair{.a = id, .b = j});
        }

        if (has_static) {
            query_hits.clear();
            static_bvh.query(query_bounds, query_hits, query_stack);
            for (const u32 j : query_hits) {
                if (j == id) {
                    continue;
                }
                pairs.push_back(BodyPair{.a = id, .b = j});
            }
        }
    }
}

} // namespace javelin
