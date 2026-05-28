module;

#include <tracy/Tracy.hpp>

export module javelin.physics.island_manager;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.physics.constraint_types;
import javelin.physics.types;

export namespace javelin {

// Owns the connected-component view of contact + constraint edges across
// dynamic bodies, plus the persistent sleep-island bookkeeping that decides
// when settled groups go to sleep and when active edges wake them back up.
//
// Per-tick lifecycle:
//   1. build_activity_masks(view, manifolds) — fills the contact + constraint
//      activity bitmaps.  Resting-clamp and sleep timers consume the contact
//      mask so constraint-only motion is not mistaken for resting contact.
//   2. build_dynamic_islands(view, manifolds) — runs union-find over the
//      contact and constraint edges to group dynamic bodies into islands.
//      Returns the largest island size, which the contact solver uses to
//      decide adaptive iteration mode.
//   3. wake_with_active_edges(view, manifolds, sleep_timer, asleep) — for any
//      edge that connects an awake body to a sleeping one, wakes the sleeping
//      island.
//   4. update_sleep_timers(view, sleep_timer, asleep) — increments per-body
//      timers for resting bodies, resets them for moving bodies, and decays
//      slowly when the body briefly leaves contact (hysteresis).
//   5. put_settled_to_sleep(view, sleep_timer, asleep) — marks every body in
//      an island asleep when the entire island has reached the threshold,
//      and records the island in the persistent sleep-island ledger.
//
// `clamp_rest_counter` is the per-body consecutive-tick counter consumed by
// the velocity clamp.  It lives here because it shares the contact-activity
// signal and the same per-body lifetime.
struct IslandManager final {
    struct WakeStats final {
        u32 woken_island_count{};
        u32 woken_body_count{};
    };

    struct SleepStats final {
        u32 slept_island_count{};
        u32 slept_body_count{};
    };

    // Sleep thresholds.  See the comments at the original definitions for the
    // rationale behind each value (60 ticks = 1 s settling, 5 cm/s linear /
    // 0.10 rad/s angular as the rest cutoff).
    static constexpr u32 kSleepTickThreshold = 60u;
    static constexpr f32 kSleepLinearSpeedThreshold = 0.05f;
    static constexpr f32 kSleepAngularSpeedThreshold = 0.10f;
    static constexpr f32 kSleepLinearSpeedThresholdSq =
        kSleepLinearSpeedThreshold * kSleepLinearSpeedThreshold;
    static constexpr f32 kSleepAngularSpeedThresholdSq =
        kSleepAngularSpeedThreshold * kSleepAngularSpeedThreshold;
    // Sleep-timer hysteresis: moving bodies reset immediately, still-but-not-in-contact
    // bodies decay one tick at a time so a one-frame contact flicker does not restart
    // the full sleep countdown.
    static constexpr u32 kSleepTimerDecayPerMiss = 1u;

    static constexpr u32 kInvalidIsland = std::numeric_limits<u32>::max();

    void reserve(const u32 capacity) {
        island_parent_.reserve(capacity);
        island_rank_.reserve(capacity);
        island_member_head_.reserve(capacity);
        island_member_next_.reserve(capacity);
        island_member_count_.reserve(capacity);
        island_roots_.reserve(capacity);
        sleep_island_of_body_.reserve(capacity);
        sleep_island_next_body_.reserve(capacity);
        sleep_island_head_.reserve(capacity);
        sleep_island_size_.reserve(capacity);
        sleep_island_free_ids_.reserve(capacity);
        contact_activity_mask_.reserve(capacity);
        constraint_activity_mask_.reserve(capacity);
        clamp_rest_counter_.reserve(capacity);
        if (sleep_island_of_body_.size() < capacity) {
            sleep_island_of_body_.resize(capacity, kInvalidIsland);
        }
        if (sleep_island_next_body_.size() < capacity) {
            sleep_island_next_body_.resize(capacity, kInvalidBody);
        }
        if (contact_activity_mask_.size() < capacity) {
            contact_activity_mask_.resize(capacity, 0u);
        }
        if (constraint_activity_mask_.size() < capacity) {
            constraint_activity_mask_.resize(capacity, 0u);
        }
        if (clamp_rest_counter_.size() < capacity) {
            clamp_rest_counter_.resize(capacity, 0u);
        }
    }

    // Drop transient island scratch and reset the persistent sleep ledger to
    // the body capacity.  Called on body-count changes and on reset.
    void clear(const u32 capacity) {
        island_roots_.clear();
        sleep_island_head_.clear();
        sleep_island_size_.clear();
        sleep_island_free_ids_.clear();
        sleep_island_of_body_.assign(capacity, kInvalidIsland);
        sleep_island_next_body_.assign(capacity, kInvalidBody);
        contact_activity_mask_.assign(capacity, 0u);
        constraint_activity_mask_.assign(capacity, 0u);
        clamp_rest_counter_.assign(capacity, 0u);
    }

    void build_activity_masks(const u32 body_count, std::span<const ContactManifold> manifolds,
                              std::span<const DistanceConstraint> constraints, std::span<const f32> inv_mass) {
        ZoneScopedN("Physics build activity masks");
        if (contact_activity_mask_.size() < body_count) {
            contact_activity_mask_.resize(body_count);
        }
        if (constraint_activity_mask_.size() < body_count) {
            constraint_activity_mask_.resize(body_count);
        }
        std::fill_n(contact_activity_mask_.begin(), body_count, static_cast<u8>(0u));
        std::fill_n(constraint_activity_mask_.begin(), body_count, static_cast<u8>(0u));

        for (const ContactManifold &manifold : manifolds) {
            if (manifold.point_count == 0u) {
                continue;
            }
#ifndef NDEBUG
            if (manifold.a >= body_count || (manifold.b != kInvalidBody && manifold.b >= body_count)) {
                log::error(physics, "Contact mask manifold id out of range (a={} b={} count={})", manifold.a,
                           manifold.b, body_count);
                std::terminate();
            }
#endif
            if (inv_mass[manifold.a] > 0.0f) {
                contact_activity_mask_[manifold.a] = 1u;
            }
            if (manifold.b != kInvalidBody && inv_mass[manifold.b] > 0.0f) {
                contact_activity_mask_[manifold.b] = 1u;
            }
        }

        for (const DistanceConstraint &constraint : constraints) {
#ifndef NDEBUG
            if (constraint.body_a >= body_count || constraint.body_b >= body_count) {
                log::error(physics, "Constraint mask id out of range (a={} b={} count={})", constraint.body_a,
                           constraint.body_b, body_count);
                std::terminate();
            }
#endif
            if (inv_mass[constraint.body_a] > 0.0f) {
                constraint_activity_mask_[constraint.body_a] = 1u;
            }
            if (inv_mass[constraint.body_b] > 0.0f) {
                constraint_activity_mask_[constraint.body_b] = 1u;
            }
        }
    }

    [[nodiscard]] u32 build_dynamic_islands(const u32 body_count, std::span<const f32> inv_mass,
                                            std::span<const ContactManifold> manifolds,
                                            std::span<const DistanceConstraint> constraints,
                                            std::span<const u32> dynamic_ids) {
        ZoneScopedN("Physics build islands");
        if (island_parent_.size() < body_count) {
            island_parent_.resize(body_count, kInvalidBody);
            island_rank_.resize(body_count, 0u);
            island_member_head_.resize(body_count, kInvalidBody);
            island_member_next_.resize(body_count, kInvalidBody);
            island_member_count_.resize(body_count, 0u);
        }

        std::fill_n(island_parent_.begin(), body_count, kInvalidBody);
        std::fill_n(island_rank_.begin(), body_count, static_cast<u8>(0u));
        std::fill_n(island_member_head_.begin(), body_count, kInvalidBody);
        std::fill_n(island_member_next_.begin(), body_count, kInvalidBody);
        std::fill_n(island_member_count_.begin(), body_count, 0u);
        island_roots_.clear();

        for (const u32 body : dynamic_ids) {
#ifndef NDEBUG
            if (body >= body_count) {
                log::error(physics, "Dynamic id out of range while building islands (id={} count={})", body,
                           body_count);
                std::terminate();
            }
#endif
            island_parent_[body] = body;
        }

        for (const ContactManifold &manifold : manifolds) {
            if (manifold.point_count == 0u || manifold.b == kInvalidBody) {
                continue;
            }
            const u32 a = manifold.a;
            const u32 b = manifold.b;
#ifndef NDEBUG
            if (a >= body_count || b >= body_count) {
                log::error(physics, "Island contact manifold id out of range (a={} b={} count={})", a, b, body_count);
                std::terminate();
            }
#endif
            if (inv_mass[a] == 0.0f || inv_mass[b] == 0.0f) {
                continue;
            }
            union_(a, b);
        }

        for (const DistanceConstraint &constraint : constraints) {
            const u32 a = constraint.body_a;
            const u32 b = constraint.body_b;
#ifndef NDEBUG
            if (a >= body_count || b >= body_count) {
                log::error(physics, "Island constraint id out of range (a={} b={} count={})", a, b, body_count);
                std::terminate();
            }
#endif
            if (inv_mass[a] == 0.0f || inv_mass[b] == 0.0f) {
                continue;
            }
            union_(a, b);
        }

        u32 max_island_size = 0u;
        for (const u32 body : dynamic_ids) {
            const u32 root = find_root_(body);
            if (island_member_head_[root] == kInvalidBody) {
                island_roots_.push_back(root);
            }
            island_member_next_[body] = island_member_head_[root];
            island_member_head_[root] = body;
            ++island_member_count_[root];
            max_island_size = std::max(max_island_size, island_member_count_[root]);
        }
        TracyPlot("physics_island_count", static_cast<i64>(island_roots_.size()));
        TracyPlot("physics_max_island_size", static_cast<i64>(max_island_size));
        return max_island_size;
    }

    [[nodiscard]] WakeStats wake_with_active_edges(std::span<const ContactManifold> manifolds,
                                                   std::span<const DistanceConstraint> constraints,
                                                   std::span<const f32> inv_mass, std::span<u8> asleep,
                                                   std::span<u32> sleep_timer) {
        ZoneScopedN("Physics wake sleeping islands");
        WakeStats stats{};

        auto wake_if_sleeping = [&](const u32 body) {
            if (asleep[body] == 0u) {
                return;
            }
            const u32 island_id = sleep_island_of_body_[body];
            if (island_id == kInvalidIsland) {
                asleep[body] = 0u;
                sleep_timer[body] = 0u;
                ++stats.woken_body_count;
                return;
            }
            const u32 woke = wake_sleep_island_(island_id, asleep, sleep_timer);
            if (woke > 0u) {
                ++stats.woken_island_count;
                stats.woken_body_count += woke;
            }
        };

        for (const ContactManifold &manifold : manifolds) {
            if (manifold.point_count == 0u || manifold.b == kInvalidBody) {
                continue;
            }
            const u32 a = manifold.a;
            const u32 b = manifold.b;
            if (inv_mass[a] == 0.0f || inv_mass[b] == 0.0f) {
                continue;
            }
            const bool a_awake = asleep[a] == 0u;
            const bool b_awake = asleep[b] == 0u;
            if (a_awake == b_awake) {
                continue;
            }
            wake_if_sleeping(a_awake ? b : a);
        }

        for (const DistanceConstraint &constraint : constraints) {
            const u32 a = constraint.body_a;
            const u32 b = constraint.body_b;
            // Constraint wake propagation is dynamic-dynamic only.
            // Static-anchored constraints do not wake sleeping bodies directly;
            // sleep eligibility is contact-driven, so this avoids perpetual
            // wake/sleep thrash on anchored chains.
            if (inv_mass[a] == 0.0f || inv_mass[b] == 0.0f) {
                continue;
            }
            const bool a_awake = asleep[a] == 0u;
            const bool b_awake = asleep[b] == 0u;
            if (a_awake == b_awake) {
                continue;
            }
            wake_if_sleeping(a_awake ? b : a);
        }

        TracyPlot("physics_islands_woken", static_cast<i64>(stats.woken_island_count));
        TracyPlot("physics_bodies_woken", static_cast<i64>(stats.woken_body_count));
        return stats;
    }

    // Update per-body sleep timers using contact activity and final velocities.
    // Timer policy:
    // - in contact + under threshold: increment.
    // - over threshold: reset immediately.
    // - under threshold but no contact: decay (hysteresis), avoiding full resets
    //   from one-frame contact flicker.
    // Sleeping and static bodies are skipped.
    void update_sleep_timers(const u32 body_count, std::span<const Vec3> velocity,
                             std::span<const Vec3> angular_velocity, std::span<const f32> inv_mass,
                             std::span<u32> sleep_timer, std::span<const u8> asleep) noexcept {
        ZoneScopedN("Physics update sleep timers");
        const std::span<const u8> in_contact = contact_activity_mask(body_count);
        for (u32 i = 0; i < body_count; ++i) {
            if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
                continue;
            }
            const bool at_rest = velocity[i].length_sq() <= kSleepLinearSpeedThresholdSq &&
                                 angular_velocity[i].length_sq() <= kSleepAngularSpeedThresholdSq;
            if (in_contact[i] != 0u && at_rest) {
                if (sleep_timer[i] < std::numeric_limits<u32>::max()) {
                    ++sleep_timer[i];
                }
            } else if (!at_rest) {
                sleep_timer[i] = 0u;
            } else {
                sleep_timer[i] =
                    (sleep_timer[i] > kSleepTimerDecayPerMiss) ? (sleep_timer[i] - kSleepTimerDecayPerMiss) : 0u;
            }
        }
    }

    [[nodiscard]] SleepStats put_settled_to_sleep(std::span<u32> sleep_timer, std::span<u8> asleep) {
        ZoneScopedN("Physics sleep islands");
        SleepStats stats{};
        for (const u32 root : island_roots_) {
            const u32 component_head = island_member_head_[root];
            if (component_head == kInvalidBody) {
                continue;
            }

            bool has_awake_member = false;
            bool all_awake_members_ready = true;
            u32 body = component_head;
            while (body != kInvalidBody) {
                if (asleep[body] == 0u) {
                    has_awake_member = true;
                    if (sleep_timer[body] < kSleepTickThreshold) {
                        all_awake_members_ready = false;
                    }
                }
                body = island_member_next_[body];
            }

            if (!has_awake_member || !all_awake_members_ready) {
                continue;
            }

            u32 member_count = 0u;
            body = component_head;
            while (body != kInvalidBody) {
                asleep[body] = 1u;
                sleep_timer[body] = std::max(sleep_timer[body], kSleepTickThreshold);
                ++member_count;
                body = island_member_next_[body];
            }
            register_sleep_island_for_component_(component_head);
            ++stats.slept_island_count;
            stats.slept_body_count += member_count;
        }
        TracyPlot("physics_islands_slept", static_cast<i64>(stats.slept_island_count));
        TracyPlot("physics_bodies_slept", static_cast<i64>(stats.slept_body_count));
        return stats;
    }

    [[nodiscard]] std::span<const u8> contact_activity_mask(const u32 body_count) const noexcept {
        return std::span<const u8>{contact_activity_mask_.data(), body_count};
    }

    [[nodiscard]] std::span<u8> clamp_rest_counter(const u32 body_count) noexcept {
        return std::span<u8>{clamp_rest_counter_.data(), body_count};
    }

  private:
    [[nodiscard]] u32 find_root_(const u32 body) {
        u32 root = body;
        while (island_parent_[root] != root) {
            root = island_parent_[root];
        }
        u32 current = body;
        while (island_parent_[current] != current) {
            const u32 parent = island_parent_[current];
            island_parent_[current] = root;
            current = parent;
        }
        return root;
    }

    void union_(const u32 lhs, const u32 rhs) {
        u32 root_lhs = find_root_(lhs);
        u32 root_rhs = find_root_(rhs);
        if (root_lhs == root_rhs) {
            return;
        }
        const u8 rank_lhs = island_rank_[root_lhs];
        const u8 rank_rhs = island_rank_[root_rhs];
        if (rank_lhs < rank_rhs) {
            std::swap(root_lhs, root_rhs);
        }
        island_parent_[root_rhs] = root_lhs;
        if (rank_lhs == rank_rhs) {
            ++island_rank_[root_lhs];
        }
    }

    [[nodiscard]] u32 allocate_sleep_island_id_() {
        if (!sleep_island_free_ids_.empty()) {
            const u32 id = sleep_island_free_ids_.back();
            sleep_island_free_ids_.pop_back();
            return id;
        }
        const u32 id = static_cast<u32>(sleep_island_head_.size());
        sleep_island_head_.push_back(kInvalidBody);
        sleep_island_size_.push_back(0u);
        return id;
    }

    void register_sleep_island_for_component_(const u32 component_head) {
        if (component_head == kInvalidBody) {
            return;
        }
        const u32 island_id = allocate_sleep_island_id_();
        u32 member_count = 0u;
        u32 body = component_head;
        while (body != kInvalidBody) {
            const u32 next = island_member_next_[body];
#ifndef NDEBUG
            if (sleep_island_of_body_[body] != kInvalidIsland) {
                log::error(physics, "Body already belongs to a sleep island (body={} island={})", body,
                           sleep_island_of_body_[body]);
                std::terminate();
            }
#endif
            sleep_island_of_body_[body] = island_id;
            sleep_island_next_body_[body] = sleep_island_head_[island_id];
            sleep_island_head_[island_id] = body;
            ++member_count;
            body = next;
        }
        sleep_island_size_[island_id] = member_count;
    }

    [[nodiscard]] u32 wake_sleep_island_(const u32 island_id, std::span<u8> asleep,
                                          std::span<u32> sleep_timer) noexcept {
        if (island_id == kInvalidIsland || island_id >= sleep_island_head_.size()) {
            return 0u;
        }
        u32 body = sleep_island_head_[island_id];
        if (body == kInvalidBody) {
            return 0u;
        }

        u32 woken_body_count = 0u;
        while (body != kInvalidBody) {
            const u32 next = sleep_island_next_body_[body];
            asleep[body] = 0u;
            sleep_timer[body] = 0u;
            sleep_island_of_body_[body] = kInvalidIsland;
            sleep_island_next_body_[body] = kInvalidBody;
            ++woken_body_count;
            body = next;
        }

        sleep_island_head_[island_id] = kInvalidBody;
        sleep_island_size_[island_id] = 0u;
        sleep_island_free_ids_.push_back(island_id);
        return woken_body_count;
    }

    // Per-tick island scratch (union-find + component member lists).
    std::vector<u32> island_parent_{};
    std::vector<u8> island_rank_{};
    std::vector<u32> island_member_head_{};
    std::vector<u32> island_member_next_{};
    std::vector<u32> island_member_count_{};
    std::vector<u32> island_roots_{};
    // Persistent sleeping-island membership for island-level wake propagation.
    std::vector<u32> sleep_island_of_body_{};
    std::vector<u32> sleep_island_next_body_{};
    std::vector<u32> sleep_island_head_{};
    std::vector<u32> sleep_island_size_{};
    std::vector<u32> sleep_island_free_ids_{};
    // Per-body activity masks.  contact_activity_mask_ feeds sleep timers and
    // the resting-velocity clamp; constraint_activity_mask_ is kept separate
    // for diagnostics (constraint-only motion is not a resting-contact signal).
    std::vector<u8> contact_activity_mask_{};
    std::vector<u8> constraint_activity_mask_{};
    // Per-body consecutive resting-contact ticks used by velocity clamp hysteresis.
    std::vector<u8> clamp_rest_counter_{};
};

} // namespace javelin
