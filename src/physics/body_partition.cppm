module;

#include <tracy/Tracy.hpp>

export module javelin.physics.body_partition;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.scene.bodies;

export namespace javelin {

// Partitions bodies into static / dynamic sets and tracks which dynamic
// bodies are awake (and which of those moved this tick).  Owns both BVHs.
//
// Lifecycle:
// - mark_dirty() forces a rebuild of the static/dynamic split and the static
//   BVH on the next refresh().  It is set on body-count changes and resets.
// - refresh() rebuilds body sets if dirty, then builds the awake-dynamic
//   mask and id list.  Call it once per tick before broad phase.
// - update_dynamic_bvh() folds new bounds into the dynamic BVH and records
//   the subset of awake bodies that actually moved this tick.
//
// Invariants:
// - inv_mass[i] == 0 ⇔ static.
// - awake_dynamic_mask_[i] != 0 ⇒ i is dynamic and not asleep.
// - moved_awake_dynamic_ids_ ⊆ awake_dynamic_ids_.
struct BodyPartition final {
    void mark_dirty() noexcept { dirty_ = true; }

    void reserve(const u32 count) {
        ZoneScopedN("BodyPartition reserve");
        dynamic_bvh_.reserve(count);
        static_bvh_.reserve(count);
        static_ids_.reserve(count);
        dynamic_ids_.reserve(count);
        awake_dynamic_ids_.reserve(count);
        awake_dynamic_mask_.reserve(count);
        moved_awake_dynamic_ids_.reserve(count);
        moved_awake_mask_.reserve(count);
    }

    // Rebuilds the static/dynamic split if dirty, then builds the awake set.
    // bounds is used only when the static BVH must be rebuilt.
    // Returns true if the static/dynamic split was rebuilt this call — the
    // caller uses this to decide between full-awake and moved-only queries.
    [[nodiscard]] bool refresh(const Bodies &view, std::span<const Aabb> bounds) {
        bool rebuilt = false;
        if (dirty_) {
            rebuild_static_dynamic_split_(view, bounds);
            last_count_ = view.count;
            dirty_ = false;
            rebuilt = true;
        }
        build_awake_dynamic_set_(view.count, view.asleep);
        return rebuilt;
    }

    // Updates the dynamic BVH for the given body ids using current bounds, and
    // records the subset of awake bodies that moved (for incremental broad
    // phase).  bvh_update_ids should be the full dynamic_ids on the tick after
    // a rebuild (to refresh sleeping leaves), and awake_dynamic_ids otherwise.
    void update_dynamic_bvh(std::span<const u32> bvh_update_ids, const u32 body_count,
                            std::span<const Aabb> bounds) {
        ZoneScopedN("BodyPartition update dynamic BVH");
        if (moved_awake_mask_.size() < body_count) {
            moved_awake_mask_.resize(body_count);
        }
        std::fill_n(moved_awake_mask_.begin(), body_count, static_cast<u8>(0u));

        moved_awake_dynamic_ids_.clear();
        moved_awake_dynamic_ids_.reserve(awake_dynamic_ids_.size());
        for (const u32 id : bvh_update_ids) {
#ifndef NDEBUG
            if (id >= body_count) {
                log::error(physics, "BVH update id out of range while collecting moved set (id={} count={})", id,
                           body_count);
                std::terminate();
            }
#endif
            const bool moved = dynamic_bvh_.update(id, bounds[id]);
            if (!moved || awake_dynamic_mask_[id] == 0u) {
                continue;
            }
            moved_awake_mask_[id] = 1u;
            moved_awake_dynamic_ids_.push_back(id);
        }
    }

    // Accessors.  Spans are valid until the next refresh()/update call.
    [[nodiscard]] std::span<const u32> dynamic_ids() const noexcept { return std::span<const u32>{dynamic_ids_}; }
    [[nodiscard]] std::span<const u32> awake_dynamic_ids() const noexcept {
        return std::span<const u32>{awake_dynamic_ids_};
    }
    [[nodiscard]] std::span<const u8> awake_dynamic_mask(const u32 body_count) const noexcept {
        return std::span<const u8>{awake_dynamic_mask_.data(), body_count};
    }
    [[nodiscard]] std::span<const u32> moved_awake_dynamic_ids() const noexcept {
        return std::span<const u32>{moved_awake_dynamic_ids_};
    }
    [[nodiscard]] std::span<const u8> moved_awake_mask(const u32 body_count) const noexcept {
        return std::span<const u8>{moved_awake_mask_.data(), body_count};
    }

    [[nodiscard]] const DynamicBvh &dynamic_bvh() const noexcept { return dynamic_bvh_; }
    [[nodiscard]] DynamicBvh &dynamic_bvh() noexcept { return dynamic_bvh_; }
    [[nodiscard]] const StaticBvh &static_bvh() const noexcept { return static_bvh_; }

  private:
    void rebuild_static_dynamic_split_(const Bodies &view, std::span<const Aabb> bounds) {
        ZoneScopedN("BodyPartition rebuild body sets");
        if (view.count != last_count_) {
            // Count changed: clear to avoid stale nodes for removed ids.
            dynamic_bvh_.clear();
        }
        static_ids_.clear();
        dynamic_ids_.clear();
        for (u32 i = 0; i < view.count; ++i) {
            if (view.inv_mass[i] == 0.0f) {
                static_ids_.push_back(i);
            } else {
                dynamic_ids_.push_back(i);
            }
        }

        for (const u32 id : static_ids_) {
            dynamic_bvh_.remove(id);
        }

        if (!static_ids_.empty()) {
            static_bvh_.build(static_ids_, bounds);
        } else {
            static_bvh_.clear();
        }
    }

    void build_awake_dynamic_set_(const u32 body_count, std::span<const u8> asleep) {
        ZoneScopedN("BodyPartition build awake dynamic ids");
        if (awake_dynamic_mask_.size() < body_count) {
            awake_dynamic_mask_.resize(body_count);
        }
        std::fill_n(awake_dynamic_mask_.begin(), body_count, static_cast<u8>(0u));

        awake_dynamic_ids_.clear();
        awake_dynamic_ids_.reserve(dynamic_ids_.size());
        for (const u32 id : dynamic_ids_) {
#ifndef NDEBUG
            if (id >= body_count) {
                log::error(physics, "Dynamic id out of range while building awake set (id={} count={})", id,
                           body_count);
                std::terminate();
            }
#endif
            if (asleep[id] != 0u) {
                continue;
            }
            awake_dynamic_mask_[id] = 1u;
            awake_dynamic_ids_.push_back(id);
        }
    }

    DynamicBvh dynamic_bvh_{};
    StaticBvh static_bvh_{};
    std::vector<u32> static_ids_{};
    std::vector<u32> dynamic_ids_{};
    std::vector<u32> awake_dynamic_ids_{};
    std::vector<u8> awake_dynamic_mask_{};
    std::vector<u32> moved_awake_dynamic_ids_{};
    std::vector<u8> moved_awake_mask_{};
    bool dirty_{true};
    u32 last_count_{0};
};

} // namespace javelin
