module;

#include <tracy/Tracy.hpp>

export module javelin.physics.debug_publisher;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.physics.aabb;
import javelin.physics.aabb_debug;
import javelin.physics.contact_debug;
import javelin.physics.types;

export namespace javelin {

// Bundles the two physics->render debug channels (contact points + per-body
// AABBs) plus their enable state.
//
// Threading model:
// - enable flags are atomic; readers (render thread) and writers (UI thread)
//   may both touch them.  The physics thread reads them at most once per tick.
// - publish_*_snapshot helpers run only on the physics thread.
// - snapshot accessors run on the render thread and use the channels' built-in
//   triple-buffered handoff.
//
// Edge-on-disable behavior: when an enable flag transitions from true to
// false, the next tick publishes an empty snapshot exactly once so the
// renderer drops the stale visualization without waiting for a re-enable.
struct DebugPublisher final {
    void reserve(const u32 body_count) {
        // Contact debug payload budget: manifold reserve heuristic is
        // 4 * body_count, each manifold contributes up to 4 points; floor at
        // 64 to avoid tiny startup allocations.
        const u32 estimated_point_capacity = std::max<u32>(body_count * 16u, 64u);
        contact_channel_.reserve(estimated_point_capacity);
        contact_channel_.publish_empty(0u);
        aabb_channel_.reserve(body_count);
        aabb_channel_.publish_empty(0u);
    }

    // Called on reset: drop any pending visualization and force re-publish
    // gating on the next tick.
    void publish_empty_on_reset(const u64 step_id) noexcept {
        contact_channel_.publish_empty(step_id);
        aabb_channel_.publish_empty(step_id);
        contact_enabled_last_tick_ = false;
        aabb_enabled_last_tick_ = false;
    }

    // Toggle helpers (callable from any thread).
    void set_contact_enabled(const bool enabled) noexcept {
        contact_enabled_.store(enabled, std::memory_order_release);
    }
    void set_aabb_enabled(const bool enabled) noexcept {
        aabb_enabled_.store(enabled, std::memory_order_release);
    }

    // Atomic getters (callable from any thread).
    [[nodiscard]] bool contact_enabled() const noexcept { return contact_enabled_.load(std::memory_order_acquire); }
    [[nodiscard]] bool aabb_enabled() const noexcept { return aabb_enabled_.load(std::memory_order_acquire); }

    [[nodiscard]] ContactDebugSnapshot contact_snapshot() const noexcept { return contact_channel_.snapshot(); }
    [[nodiscard]] AabbDebugSnapshot aabb_snapshot() const noexcept { return aabb_channel_.snapshot(); }

    // Physics-thread helpers.  Each call returns true if a payload was
    // published this tick.  Enable-edge handling is encapsulated: the caller
    // does not need to track last-tick state.
    bool maybe_publish_contacts(std::span<const Vec3> position, std::span<const Quat> orientation,
                                std::span<const ContactManifold> manifolds, const u64 step_id) {
        const bool enabled = contact_enabled();
        if (enabled) {
            publish_contacts_(position, orientation, manifolds, step_id);
        } else if (contact_enabled_last_tick_) {
            contact_channel_.publish_empty(step_id);
        }
        contact_enabled_last_tick_ = enabled;
        return enabled;
    }

    bool maybe_publish_aabbs(std::span<const Aabb> bounds, const u32 count, const u64 step_id) {
        const bool enabled = aabb_enabled();
        if (enabled) {
            publish_aabbs_(bounds, count, step_id);
        } else if (aabb_enabled_last_tick_) {
            aabb_channel_.publish_empty(step_id);
        }
        aabb_enabled_last_tick_ = enabled;
        return enabled;
    }

  private:
    void publish_contacts_(std::span<const Vec3> position, std::span<const Quat> orientation,
                           std::span<const ContactManifold> manifolds, const u64 step_id) {
        ZoneScopedN("Physics publish contact debug");
        u32 point_count = 0u;
        for (const ContactManifold &manifold : manifolds) {
            point_count += manifold.point_count;
        }
        ContactDebugWrite out = contact_channel_.write_contacts(point_count);

        u32 out_index = 0u;
        for (const ContactManifold &manifold : manifolds) {
            if (manifold.point_count == 0u) {
                continue;
            }

            Vec3 normal = manifold.normal;
            if (!normal.try_normalize()) {
                normal = Vec3::unit_y();
            }
            const u32 a = manifold.a;
            const bool has_body_b = manifold.b != kInvalidBody;
            const u32 b = has_body_b ? manifold.b : kInvalidBody;

            for (u32 point_index = 0u; point_index < manifold.point_count; ++point_index) {
                const ContactPoint &point = manifold.points[point_index];
                const Vec3 world_a = position[a] + rotate(orientation[a], point.local_anchor_a);

                Vec3 point_world = world_a;
                if (has_body_b) {
                    const Vec3 world_b = position[b] + rotate(orientation[b], point.local_anchor_b);
                    point_world = (world_a + world_b) * 0.5f;
                }

                out.points[out_index] = point_world;
                out.normals[out_index] = normal;
                out.separations[out_index] = point.separation;
                out.normal_impulses[out_index] = point.normal_impulse;
                out.persisted[out_index] = point.persisted ? static_cast<u8>(1u) : static_cast<u8>(0u);
                ++out_index;
            }
        }

#ifndef NDEBUG
        if (out_index != point_count) {
            log::error(physics, "Contact debug packing mismatch expected={} actual={}", point_count, out_index);
            std::terminate();
        }
#endif
        contact_channel_.publish(out_index, step_id);
    }

    void publish_aabbs_(std::span<const Aabb> bounds, const u32 count, const u64 step_id) {
        ZoneScopedN("Physics publish AABB debug");
        AabbDebugWrite out = aabb_channel_.write_aabbs(count);
        std::copy_n(bounds.data(), count, out.aabbs.data());
        aabb_channel_.publish(count, step_id);
    }

    ContactDebugChannel contact_channel_{};
    AabbDebugChannel aabb_channel_{};
    std::atomic<bool> contact_enabled_{false};
    std::atomic<bool> aabb_enabled_{false};
    // Physics-thread-only state: tracks enable->disable transitions so we can
    // publish one empty snapshot to clear stale data without paying per-tick
    // writes while disabled.
    bool contact_enabled_last_tick_{false};
    bool aabb_enabled_last_tick_{false};
};

} // namespace javelin
