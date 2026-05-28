module;

#include <tracy/Tracy.hpp>

export module javelin.physics.body_graph;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.constraint_types;
import javelin.physics.types;
import javelin.scene.bodies;
import javelin.scene.shapes;

export namespace javelin {

// Owns the per-tick "topology" of the simulation: per-body bounds, the
// static/dynamic split and its two BVHs, the awake/moved-awake subsets, the
// per-tick connected-component view of contact + constraint edges, and the
// persistent sleep-island ledger that decides when settled groups sleep and
// when active edges wake them.
//
// The contract is organised around the three points in a tick where this
// state is touched, in temporal order:
//
//   prepare_collision_inputs(view)
//     - pre broad phase.  Rebuilds the bounds cache, refreshes the
//       static/dynamic split (rebuilding the static BVH if marked dirty),
//       builds the awake-dynamic mask, and folds the relevant bodies into
//       the dynamic BVH, recording the moved-awake subset.
//   register_contact_graph(view, manifolds) -> max island size
//     - between narrow phase and the contact solver.  Builds the contact +
//       constraint activity masks, runs union-find over dynamic-dynamic
//       edges, and wakes sleeping bodies that share an active edge with an
//       awake body.  The returned max island size feeds the contact
//       solver's adaptive iteration cap.
//   evolve_sleep(view)
//     - post solve.  Updates per-body sleep timers from final velocities +
//       contact activity, then puts every body in a fully-settled island to
//       sleep.
//
// Between phases 2 and 3 the resting-contact velocity clamp consumes
// `contact_activity_mask` and `clamp_rest_counter`; those accessors are
// exposed for that one external touchpoint.
//
// The finer per-stage methods (build_activity_masks, build_dynamic_islands,
// wake_with_active_edges, update_sleep_timers, put_settled_to_sleep) remain
// public test seams.  Production code uses register_contact_graph and
// evolve_sleep; tests use the finer seams to verify intermediate state.
//
// Lifecycle:
// - mark_dirty() forces a rebuild of the static/dynamic split (and the
//   static BVH) on the next prepare_collision_inputs.  It is set on
//   body-count changes and resets.
// - reserve(count) grows internal scratch up to `count` bodies.
// - clear(capacity) drops transient island scratch and resets the
//   persistent sleep ledger to `capacity`.  Called on reset and on
//   body-count changes.
struct BodyGraph final {
    struct WakeStats final {
        u32 woken_island_count{};
        u32 woken_body_count{};
    };

    struct SleepStats final {
        u32 slept_island_count{};
        u32 slept_body_count{};
    };

    // Sleep thresholds.  60 ticks = 1 s settling.  5 cm/s linear and
    // 0.10 rad/s angular are the rest cutoffs.
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

    // ---- Lifecycle ----

    void mark_dirty() noexcept { partition_dirty_ = true; }

    void reserve(const u32 count) {
        ZoneScopedN("BodyGraph reserve");
        bounds_cache_.reserve(count);
        dynamic_bvh_.reserve(count);
        static_bvh_.reserve(count);
        static_ids_.reserve(count);
        dynamic_ids_.reserve(count);
        awake_dynamic_ids_.reserve(count);
        awake_dynamic_mask_.reserve(count);
        moved_awake_dynamic_ids_.reserve(count);
        moved_awake_mask_.reserve(count);
        island_parent_.reserve(count);
        island_rank_.reserve(count);
        island_member_head_.reserve(count);
        island_member_next_.reserve(count);
        island_member_count_.reserve(count);
        island_roots_.reserve(count);
        sleep_island_of_body_.reserve(count);
        sleep_island_next_body_.reserve(count);
        sleep_island_head_.reserve(count);
        sleep_island_size_.reserve(count);
        sleep_island_free_ids_.reserve(count);
        contact_activity_mask_.reserve(count);
        constraint_activity_mask_.reserve(count);
        clamp_rest_counter_.reserve(count);
        if (sleep_island_of_body_.size() < count) {
            sleep_island_of_body_.resize(count, kInvalidIsland);
        }
        if (sleep_island_next_body_.size() < count) {
            sleep_island_next_body_.resize(count, kInvalidBody);
        }
        if (contact_activity_mask_.size() < count) {
            contact_activity_mask_.resize(count, 0u);
        }
        if (constraint_activity_mask_.size() < count) {
            constraint_activity_mask_.resize(count, 0u);
        }
        if (clamp_rest_counter_.size() < count) {
            clamp_rest_counter_.resize(count, 0u);
        }
    }

    // Drop transient island scratch and reset the persistent sleep ledger to
    // the body capacity.  Does not touch the partition / BVHs themselves;
    // mark_dirty() is what forces those to rebuild.
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

    // ---- Phase 1: pre-broad-phase ----

    // Rebuild bounds + partition + dynamic BVH for the upcoming tick.
    //
    // Reads:   view.position, view.orientation, view.shape_kind,
    //          view.shape_index, view.shapes, view.inv_mass, view.asleep
    // Writes:  bounds cache, static/dynamic split, awake mask, dynamic BVH,
    //          moved-awake subset, rebuilt_body_sets flag.
    //
    // The choice of BVH update set (full dynamic vs. awake-only) is made
    // internally based on whether the static/dynamic split was rebuilt this
    // tick: after a rebuild every dynamic leaf is refreshed so sleeping
    // bodies are present and in sync, otherwise only awake bodies are folded.
    void prepare_collision_inputs(const Bodies &view) {
        rebuild_bounds_(view);
        const bool rebuilt = refresh_partition_(view);
        const std::span<const u32> bvh_update_ids = rebuilt ? dynamic_ids() : awake_dynamic_ids();
        update_dynamic_bvh_(bvh_update_ids, view.count);
        rebuilt_body_sets_this_tick_ = rebuilt;
    }

    // True iff prepare_collision_inputs rebuilt the static/dynamic split this
    // tick.  The broad-phase incremental-vs-full-query policy uses this in
    // combination with the moved-awake ratio.
    [[nodiscard]] bool rebuilt_body_sets_this_tick() const noexcept { return rebuilt_body_sets_this_tick_; }

    // ---- Phase 2: post-narrow-phase, pre-solve ----

    // Build contact + constraint activity masks, union-find dynamic islands,
    // and wake any sleeping body that shares an active edge with an awake one.
    //
    // Reads:   view.inv_mass, view.constraints, manifolds.
    // Writes:  activity masks, island scratch, view.asleep, view.sleep_timer.
    // Returns: the largest dynamic island size, used by the contact solver's
    //          adaptive iteration cap.
    [[nodiscard]] u32 register_contact_graph(const Bodies &view,
                                              std::span<const ContactManifold> manifolds) {
        build_activity_masks(view.count, manifolds, view.constraints, view.inv_mass);
        const u32 max_island_size =
            build_dynamic_islands(view.count, view.inv_mass, manifolds, view.constraints, dynamic_ids());
        static_cast<void>(wake_with_active_edges(manifolds, view.constraints, view.inv_mass, view.asleep,
                                                  view.sleep_timer));
        return max_island_size;
    }

    // ---- Phase 3: post-solve ----

    // Update per-body sleep timers from final velocities + contact activity,
    // then put fully-settled dynamic islands to sleep.
    //
    // Reads:   view.velocity, view.angular_velocity, view.inv_mass, internal
    //          contact_activity_mask, island scratch.
    // Writes:  view.sleep_timer, view.asleep, persistent sleep-island ledger.
    void evolve_sleep(const Bodies &view) noexcept {
        update_sleep_timers(view.count, view.velocity, view.angular_velocity, view.inv_mass, view.sleep_timer,
                            view.asleep);
        static_cast<void>(put_settled_to_sleep(view.sleep_timer, view.asleep));
    }

    // ---- Phase 1 outputs (broad/narrow phase consumers) ----

    [[nodiscard]] std::span<const Aabb> bounds() const noexcept { return std::span<const Aabb>{bounds_cache_}; }
    [[nodiscard]] Aabb bounds_at(const u32 id) const noexcept { return bounds_cache_[id]; }
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

    // ---- Between-phase accessors (consumed by resting-velocity clamp) ----

    [[nodiscard]] std::span<const u8> contact_activity_mask(const u32 body_count) const noexcept {
        return std::span<const u8>{contact_activity_mask_.data(), body_count};
    }
    [[nodiscard]] std::span<u8> clamp_rest_counter(const u32 body_count) noexcept {
        return std::span<u8>{clamp_rest_counter_.data(), body_count};
    }

    // ---- Finer per-stage methods (test seams + reusable internals) ----
    //
    // register_contact_graph and evolve_sleep delegate to these.  They remain
    // public so existing unit tests can verify intermediate state.

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

  private:
    // ---- Phase 1 helpers ----

    void rebuild_bounds_(const Bodies &view) {
        ZoneScopedN("Physics build bounds cache");
        const u32 count = view.count;
        bounds_cache_.resize(count);
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
                bounds_cache_[i] = Aabb::from_sphere(view.position[i], sphere.radius);
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
                bounds_cache_[i] = Aabb{center - extents, center + extents};
            } break;
            }
        }
    }

    // Rebuilds the static/dynamic split if dirty, then builds the awake-dynamic
    // set.  Returns true iff the split was rebuilt this call.
    [[nodiscard]] bool refresh_partition_(const Bodies &view) {
        bool rebuilt = false;
        if (partition_dirty_) {
            rebuild_static_dynamic_split_(view);
            last_count_ = view.count;
            partition_dirty_ = false;
            rebuilt = true;
        }
        build_awake_dynamic_set_(view.count, view.asleep);
        return rebuilt;
    }

    void rebuild_static_dynamic_split_(const Bodies &view) {
        ZoneScopedN("BodyGraph rebuild body sets");
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
            static_bvh_.build(static_ids_, bounds());
        } else {
            static_bvh_.clear();
        }
    }

    void build_awake_dynamic_set_(const u32 body_count, std::span<const u8> asleep) {
        ZoneScopedN("BodyGraph build awake dynamic ids");
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

    void update_dynamic_bvh_(std::span<const u32> bvh_update_ids, const u32 body_count) {
        ZoneScopedN("BodyGraph update dynamic BVH");
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
            const bool moved = dynamic_bvh_.update(id, bounds_cache_[id]);
            if (!moved || awake_dynamic_mask_[id] == 0u) {
                continue;
            }
            moved_awake_mask_[id] = 1u;
            moved_awake_dynamic_ids_.push_back(id);
        }
    }

    // ---- Union-find helpers ----

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

    // ---- Sleep-island ledger helpers ----

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

    // ---- State ----

    // Phase 1: bounds + partition + BVHs + awake/moved sets.
    std::vector<Aabb> bounds_cache_{};
    DynamicBvh dynamic_bvh_{};
    StaticBvh static_bvh_{};
    std::vector<u32> static_ids_{};
    std::vector<u32> dynamic_ids_{};
    std::vector<u32> awake_dynamic_ids_{};
    std::vector<u8> awake_dynamic_mask_{};
    std::vector<u32> moved_awake_dynamic_ids_{};
    std::vector<u8> moved_awake_mask_{};
    bool partition_dirty_{true};
    bool rebuilt_body_sets_this_tick_{false};
    u32 last_count_{0};

    // Phase 2: per-tick island scratch (union-find + component member lists).
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
