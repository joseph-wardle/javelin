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

// Broad phase contract:
// - input bounds are world-space AABBs for the current frame.
// - output pairs are potential overlaps only; narrow phase confirms contacts.
// - ordering inside each chunk follows dynamic id order; global canonicalization
//   and de-duplication are performed by PhysicsSystem.
struct BroadPhaseScratch final {
    std::vector<u32> query_hits{};
    std::vector<u32> query_stack{};

    void reserve(const u32 count, const u32 query_stack_factor) {
        query_hits.reserve(count);
        query_stack.reserve(static_cast<usize>(count) * query_stack_factor);
    }
};

void broad_phase_update_dynamic_bvh(std::span<const u32> dynamic_ids, DynamicBvh &dynamic_bvh,
                                    std::span<const Aabb> bounds_cache) {
    ZoneScopedN("Physics broad phase update");
    if (dynamic_ids.empty()) {
        return;
    }
    for (const u32 id : dynamic_ids) {
        dynamic_bvh.update(id, bounds_cache[id]);
    }
}

// Chunked query helper: operates on a contiguous span with per-thread scratch/output.
void broad_phase_chunk(std::span<const u32> dynamic_ids, const DynamicBvh &dynamic_bvh, const StaticBvh &static_bvh,
                       std::span<const Aabb> bounds_cache, std::vector<BodyPair> &pairs, BroadPhaseScratch &scratch) {
    ZoneScopedN("Physics broad phase query");
    pairs.clear();
    if (dynamic_ids.empty()) {
        return;
    }

    const bool has_static = !static_bvh.empty();
    // dynamic_ids are built in ascending order so j <= id filters duplicates/self.
    for (const u32 id : dynamic_ids) {
        const Aabb query_bounds = bounds_cache[id];

        scratch.query_hits.clear();
        dynamic_bvh.query(query_bounds, scratch.query_hits, scratch.query_stack);
        for (const u32 j : scratch.query_hits) {
            if (j <= id) {
                continue;
            }
            pairs.push_back(BodyPair{.a = id, .b = j});
        }

        if (has_static) {
            scratch.query_hits.clear();
            static_bvh.query(query_bounds, scratch.query_hits, scratch.query_stack);
            for (const u32 j : scratch.query_hits) {
                if (j == id) {
                    continue;
                }
                pairs.push_back(BodyPair{.a = id, .b = j});
            }
        }
    }
}

// Main entrypoint for broad phase pair generation.
void broad_phase_generate_pairs(std::span<const u32> dynamic_ids, const DynamicBvh &dynamic_bvh,
                                const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
                                std::vector<BodyPair> &pairs, BroadPhaseScratch &scratch) {
    broad_phase_chunk(dynamic_ids, dynamic_bvh, static_bvh, bounds_cache, pairs, scratch);
}

} // namespace javelin
