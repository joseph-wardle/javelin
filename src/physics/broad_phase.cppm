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
    std::vector<u32> query_stack_overflow{};

    void reserve(const u32 count, const u32 query_stack_factor) {
        query_stack_overflow.reserve(static_cast<usize>(count) * query_stack_factor);
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
void broad_phase_chunk(std::span<const u32> query_dynamic_ids, const DynamicBvh &dynamic_bvh,
                       const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
                       std::span<const u8> query_dynamic_mask, std::vector<BodyPair> &pairs,
                       BroadPhaseScratch &scratch) {
    ZoneScopedN("Physics broad phase query");
    pairs.clear();
    if (query_dynamic_ids.empty()) {
        return;
    }

    const bool has_static = !static_bvh.empty();
    // query_dynamic_ids are sorted ascending.
    // Duplicate rule:
    // - query-vs-query pairs are emitted once by id order.
    // - query-vs-nonquery pairs are always emitted (nonquery body never queries back).
    for (const u32 id : query_dynamic_ids) {
        const Aabb query_bounds = bounds_cache[id];

        dynamic_bvh.query_each_overlap(
            query_bounds,
            [&](const u32 j) {
                if (j == id) {
                    return;
                }
                const bool j_is_query_body = (j < query_dynamic_mask.size()) && query_dynamic_mask[j] != 0u;
                if (j_is_query_body && j < id) {
                    return;
                }
                pairs.push_back(BodyPair{.a = id, .b = j});
            },
            scratch.query_stack_overflow);

        if (!has_static) {
            continue;
        }

        static_bvh.query_each_overlap(
            query_bounds,
            [&](const u32 j) {
                if (j == id) {
                    return;
                }
                pairs.push_back(BodyPair{.a = id, .b = j});
            },
            scratch.query_stack_overflow);
    }
}

// Main entrypoint for broad phase pair generation.
void broad_phase_generate_pairs(std::span<const u32> query_dynamic_ids, const DynamicBvh &dynamic_bvh,
                                const StaticBvh &static_bvh, std::span<const Aabb> bounds_cache,
                                std::span<const u8> query_dynamic_mask, std::vector<BodyPair> &pairs,
                                BroadPhaseScratch &scratch) {
    broad_phase_chunk(query_dynamic_ids, dynamic_bvh, static_bvh, bounds_cache, query_dynamic_mask, pairs, scratch);
}

} // namespace javelin
