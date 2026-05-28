module;

#include <tracy/Tracy.hpp>

export module javelin.physics.tick_pipeline;

import std;
import javelin.core.types;
import javelin.physics.aabb_debug;
import javelin.physics.body_partition;
import javelin.physics.bounds_builder;
import javelin.physics.broad_phase;
import javelin.physics.contact_debug;
import javelin.physics.debug_publisher;
import javelin.physics.integrate;
import javelin.physics.island_manager;
import javelin.physics.narrow_phase;
import javelin.physics.publish;
import javelin.physics.solve;
import javelin.physics.types;
import javelin.scene;
import javelin.scene.bodies;

namespace javelin::detail {

// Pair-wise material combination rules for contact resolution:
// - restitution: geometric mean preserves the elastic range [0,1] symmetrically.
// - friction: minimum gives the softer of the two surfaces (conservative slip threshold).
[[nodiscard]] inline f32 combined_restitution(const f32 a, const f32 b) noexcept { return std::sqrt(a * b); }
[[nodiscard]] inline f32 combined_friction(const f32 a, const f32 b) noexcept { return std::min(a, b); }

} // namespace javelin::detail

export namespace javelin {

// Per-tick parameters supplied by the surrounding driver.
struct TickParams final {
    f32 dt{1.0f / 60.0f};
    f32 gravity{-9.8f};
    f32 linear_damping{0.1f};
    f32 angular_damping{0.4f};
};

// Orchestrates one fixed simulation step.  Owns every per-tick stage
// (bounds, body partition, broad phase, narrow phase, islands, debug) and
// the small caches that connect them.  The step body is a sequence of
// stage calls — each stage has its own state and contract; the pipeline only
// threads outputs into inputs.
//
// Thread model: TickPipeline assumes a single owner thread.  The surrounding
// driver (PhysicsSystem) handles fixed-rate ticking, pause/step budget, and
// reset coordination.  This module is allocation-free on the hot path once
// reserve() has been called for the body count.
struct TickPipeline final {
    void init(Scene &scene) { debug_.reserve(scene.bodies().count); }

    // Stops the broad-phase worker threads.  Idempotent.
    void shutdown() noexcept { broad_phase_.stop(); }

    // Reset bookkeeping: drop manifolds and mark the partition dirty so the
    // next step rebuilds the static/dynamic split.  Also publishes an empty
    // snapshot to the debug channels so the renderer drops stale data.
    void handle_reset(const u64 step_id) {
        partition_.mark_dirty();
        clear_manifold_state_();
        debug_.publish_empty_on_reset(step_id);
    }

    // Step the simulation forward one tick.  next_step_id is the value
    // completed_simulation_steps() will return after this tick succeeds —
    // used to tag debug snapshots.  Returns false only when the scene has
    // no body view to step (the empty-scene degenerate case).
    bool step(Scene &scene, const TickParams &params, const u64 next_step_id) {
        Bodies view = scene.bodies();
        const u32 count = view.count;
        const f32 dt = params.dt;

        if (count != previous_body_count_) {
            partition_.mark_dirty();
            clear_manifold_state_();
            previous_body_count_ = count;
        }
        // Stage 0: ensure frame scratch and previous-manifold lookup are ready.
        ensure_capacity_(count);
        const HotPathCapacitySnapshot hot_capacity_before = capture_hot_path_capacity_snapshot_();
        narrow_phase_.prepare();

        // Stage 1: external forces and per-body bounds for broad phase.
        integrate_gravity_velocity(view.velocity, view.inv_mass, view.asleep, params.gravity, dt);
        f32 max_angular_speed_sq = 0.0f;
        {
            ZoneScopedN("Physics max angular speed scan");
            for (u32 i = 0; i < count; ++i) {
                max_angular_speed_sq = std::max(max_angular_speed_sq, view.angular_velocity[i].length_sq());
            }
        }
        TracyPlot("physics_max_angular_speed", std::sqrt(max_angular_speed_sq));
        bounds_.update(view);

        // Stage 2: broad phase candidate generation.
        const bool rebuilt_body_sets = partition_.refresh(view, bounds_.bounds());
        const std::span<const u32> dynamic_ids = partition_.dynamic_ids();
        const std::span<const u32> awake_dynamic_ids = partition_.awake_dynamic_ids();
        const std::span<const u8> awake_dynamic_mask = partition_.awake_dynamic_mask(count);
        TracyPlot("physics_dynamic_bodies", static_cast<i64>(dynamic_ids.size()));
        TracyPlot("physics_awake_dynamic_bodies", static_cast<i64>(awake_dynamic_ids.size()));
        TracyPlot("physics_sleeping_dynamic_bodies",
                  static_cast<i64>(dynamic_ids.size() - awake_dynamic_ids.size()));

        // Mutating phase: update dynamic BVH before read-only queries.
        // After body-set rebuild (count change / reset), refresh all dynamic ids once so
        // sleeping leaves are present and in sync.
        const std::span<const u32> bvh_update_ids = rebuilt_body_sets ? dynamic_ids : awake_dynamic_ids;
        partition_.update_dynamic_bvh(bvh_update_ids, count, bounds_.bounds());
        const std::span<const u32> moved_awake_dynamic_ids = partition_.moved_awake_dynamic_ids();
        const std::span<const u8> moved_awake_mask = partition_.moved_awake_mask(count);

        const f32 moved_awake_ratio = awake_dynamic_ids.empty()
                                          ? 0.0f
                                          : (static_cast<f32>(moved_awake_dynamic_ids.size()) /
                                             static_cast<f32>(awake_dynamic_ids.size()));
        const bool fallback_to_full_awake_query =
            rebuilt_body_sets || moved_awake_ratio >= kBroadPhaseIncrementalFallbackMovedRatio;
        const std::span<const u32> broad_phase_query_ids =
            fallback_to_full_awake_query ? awake_dynamic_ids : moved_awake_dynamic_ids;
        const std::span<const u8> broad_phase_query_mask =
            fallback_to_full_awake_query ? awake_dynamic_mask : moved_awake_mask;

        const auto broad_phase_result = broad_phase_.find_pairs({
            .dynamic_bvh = &partition_.dynamic_bvh(),
            .static_bvh = &partition_.static_bvh(),
            .bounds = bounds_.bounds(),
            .query_ids = broad_phase_query_ids,
            .query_mask = broad_phase_query_mask,
            .body_count = count,
            .awake_dynamic_mask = awake_dynamic_mask,
            .previous_manifolds = narrow_phase_.manifolds(),
            .include_carry_forward = !fallback_to_full_awake_query,
        });
        const std::span<const BodyPair> candidate_pairs = broad_phase_result.pairs;
#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
        broad_phase_.maybe_validate(partition_.dynamic_bvh(), partition_.static_bvh(), bounds_.bounds(),
                                    awake_dynamic_ids, awake_dynamic_mask, fallback_to_full_awake_query,
                                    next_step_id);
#endif
        TracyPlot("physics_moved_awake_dynamic_bodies", static_cast<i64>(moved_awake_dynamic_ids.size()));
        TracyPlot("physics_moved_awake_ratio", moved_awake_ratio);
        TracyPlot("physics_carried_pair_count", static_cast<i64>(broad_phase_result.carry_forward.carried_pair_count));
        TracyPlot("physics_validated_pair_count",
                  static_cast<i64>(broad_phase_result.carry_forward.validated_pair_count));
        TracyPlot("physics_broad_phase_full_query",
                  fallback_to_full_awake_query ? static_cast<i64>(1) : static_cast<i64>(0));
        TracyPlot("physics_pairs", static_cast<i64>(candidate_pairs.size()));

        // Stage 3: narrow phase manifolds + warm-start persistence refresh.
        const auto narrow_stats = narrow_phase_.run(view.position, view.orientation, view.shape_kind, view.shapes,
                                                    view.shape_index, view.inv_mass, candidate_pairs,
                                                    awake_dynamic_ids);
        const u32 manifold_count = narrow_stats.manifold_count;
        const u32 contact_point_count = narrow_stats.contact_point_count;
        {
            ZoneScopedN("Physics manifold statistics");
            const f32 avg_points_per_manifold =
                (manifold_count > 0u) ? (static_cast<f32>(contact_point_count) / static_cast<f32>(manifold_count))
                                      : 0.0f;
            const f32 warm_start_match_rate =
                (narrow_stats.next_point_count > 0u)
                    ? (static_cast<f32>(narrow_stats.matched_point_count) /
                       static_cast<f32>(narrow_stats.next_point_count))
                    : 0.0f;
            TracyPlot("physics_manifolds", static_cast<i64>(manifold_count));
            TracyPlot("physics_contact_points", static_cast<i64>(contact_point_count));
            TracyPlot("physics_avg_points_per_manifold", avg_points_per_manifold);
            TracyPlot("physics_warm_start_match_rate", warm_start_match_rate);
            TracyPlot("physics_dropped_points", static_cast<i64>(narrow_stats.dropped_point_count));
            TracyPlot("physics_axis_flip_count", static_cast<i64>(narrow_stats.axis_flip_count));
            TracyPlot("physics_persistence_cache_invalidations",
                      static_cast<i64>(narrow_stats.cache_invalidation_count));
            TracyPlot("physics_contacts", static_cast<i64>(contact_point_count));
        }

        // Stage 4: solve constraints, damp, integrate, and publish transforms.
        // Body b == kInvalidBody (ground plane) uses material id 0 (the default material).
        const std::span<ContactManifold> manifolds = narrow_phase_.manifolds();
        manifold_restitution_cache_.resize(manifold_count);
        manifold_friction_cache_.resize(manifold_count);
        {
            ZoneScopedN("Physics combine manifold materials");
            for (u32 i = 0; i < manifold_count; ++i) {
                const u32 a = manifolds[i].a;
                const u32 b = manifolds[i].b;
                const u32 mat_a = view.material[a].value;
                const u32 mat_b = (b != kInvalidBody) ? view.material[b].value : 0u;
                manifold_restitution_cache_[i] =
                    detail::combined_restitution(view.material_restitution[mat_a], view.material_restitution[mat_b]);
                manifold_friction_cache_[i] =
                    detail::combined_friction(view.material_friction[mat_a], view.material_friction[mat_b]);
            }
        }
        islands_.build_activity_masks(count, manifolds, view.constraints, view.inv_mass);
        const u32 max_dynamic_island_size =
            islands_.build_dynamic_islands(count, view.inv_mass, manifolds, view.constraints, partition_.dynamic_ids());
        static_cast<void>(
            islands_.wake_with_active_edges(manifolds, view.constraints, view.inv_mass, view.asleep, view.sleep_timer));

        const bool complex_contact_solve =
            max_dynamic_island_size >= kContactSolverComplexIslandSizeThreshold ||
            contact_point_count >= kContactSolverComplexPointCountThreshold;
        const ContactSolveConfig contact_solve_config{
            .adaptive_iteration_cap = complex_contact_solve,
            .max_iterations = complex_contact_solve ? kContactSolverComplexMaxIterations
                                                    : kContactSolverBaseMaxIterations,
            .min_iterations_before_adapt = kContactSolverAdaptiveMinIterations,
            .adaptive_impulse_epsilon = kContactSolverAdaptiveImpulseEpsilon,
        };
        TracyPlot("physics_contact_solver_adaptive_mode",
                  complex_contact_solve ? static_cast<i64>(1) : static_cast<i64>(0));
        TracyPlot("physics_contact_solver_max_iterations", static_cast<i64>(contact_solve_config.max_iterations));

        solve_contact_velocities(view.velocity, view.angular_velocity, view.inv_mass, view.inv_inertia,
                                 view.orientation, manifolds, dt, std::span<const f32>{manifold_restitution_cache_},
                                 std::span<const f32>{manifold_friction_cache_}, std::span<const u8>{view.asleep},
                                 contact_solve_config);
        solve_contact_penetration(view.position, view.orientation, view.inv_mass, view.inv_inertia, manifolds,
                                  std::span<const u8>{view.asleep});
        solve_distance_constraints(view.velocity, view.angular_velocity, view.inv_mass, view.inv_inertia,
                                   view.orientation, view.position, view.constraints, dt,
                                   std::span<const u8>{view.asleep});
        apply_linear_damping(view.velocity, view.inv_mass, view.asleep, params.linear_damping, dt);
        apply_angular_damping(view.angular_velocity, view.inv_mass, view.asleep, params.angular_damping, dt);
        clamp_resting_contact_velocities(view.velocity, view.angular_velocity, view.inv_mass,
                                         islands_.contact_activity_mask(count), view.asleep,
                                         islands_.clamp_rest_counter(count), kClampLinearSpeedThresholdSq,
                                         kClampAngularSpeedThresholdSq, kClampRestTickThreshold);
        islands_.update_sleep_timers(count, view.velocity, view.angular_velocity, view.inv_mass, view.sleep_timer,
                                     view.asleep);
        static_cast<void>(islands_.put_settled_to_sleep(view.sleep_timer, view.asleep));
        integrate_positions(view.position, view.velocity, view.inv_mass, view.asleep, dt);
        integrate_orientations(view.orientation, view.angular_velocity, view.inv_mass, view.asleep, dt);

        debug_.maybe_publish_contacts(view.position, view.orientation, manifolds, next_step_id);
        debug_.maybe_publish_aabbs(bounds_.bounds(), count, next_step_id);
        publish_poses(view.poses, view.position, view.orientation, view.asleep, count);
        publish_hot_path_capacity_growth_(hot_capacity_before);
        return true;
    }

    // Debug-channel passthroughs.  The orchestrator surfaces these as part of
    // PhysicsSystem's public API, but the underlying state lives in the
    // pipeline because it is mutated as part of every tick.
    void set_contact_debug_enabled(const bool enabled) noexcept { debug_.set_contact_enabled(enabled); }
    void set_aabb_debug_enabled(const bool enabled) noexcept { debug_.set_aabb_enabled(enabled); }
    [[nodiscard]] bool contact_debug_enabled() const noexcept { return debug_.contact_enabled(); }
    [[nodiscard]] bool aabb_debug_enabled() const noexcept { return debug_.aabb_enabled(); }
    [[nodiscard]] ContactDebugSnapshot contact_debug_snapshot() const noexcept { return debug_.contact_snapshot(); }
    [[nodiscard]] AabbDebugSnapshot aabb_debug_snapshot() const noexcept { return debug_.aabb_snapshot(); }

  private:
    struct HotPathCapacitySnapshot final {
        usize candidate_pairs_capacity{};
        usize manifold_capacity{};
        usize manifold_material_capacity{};
        usize broad_phase_worker_pair_capacity_sum{};
    };

    // Incremental broad-phase policy: fall back to a full awake query when the
    // moved-awake / awake ratio is high enough that the moved-only path stops
    // being a win.
    static constexpr f32 kBroadPhaseIncrementalFallbackMovedRatio = 0.60f;

    // Resting-contact velocity clamp parameters.  Clamp thresholds are intentionally
    // stricter than sleep thresholds: this removes solver jitter on sustained
    // resting contacts without killing legitimate low-speed motion.
    static constexpr f32 kClampLinearSpeedThreshold = 0.02f;
    static constexpr f32 kClampAngularSpeedThreshold = 0.04f;
    static constexpr f32 kClampLinearSpeedThresholdSq = kClampLinearSpeedThreshold * kClampLinearSpeedThreshold;
    static constexpr f32 kClampAngularSpeedThresholdSq = kClampAngularSpeedThreshold * kClampAngularSpeedThreshold;
    static constexpr u8 kClampRestTickThreshold = 4u;

    // Contact solver policy: deterministic baseline of 16 iterations, with
    // an adaptive cap of 20 for complex islands.
    static constexpr u32 kContactSolverBaseMaxIterations = 16u;
    static constexpr u32 kContactSolverComplexMaxIterations = 20u;
    static constexpr u32 kContactSolverAdaptiveMinIterations = 6u;
    static constexpr f32 kContactSolverAdaptiveImpulseEpsilon = 5e-5f;
    static constexpr u32 kContactSolverComplexIslandSizeThreshold = 12u;
    static constexpr u32 kContactSolverComplexPointCountThreshold = 48u;

    void ensure_capacity_(const u32 count) {
        ZoneScopedN("Physics ensure capacity");
        if (count <= capacity_) {
            broad_phase_.reserve(count);
            return;
        }
        capacity_ = count;
        partition_.reserve(count);
        bounds_.reserve(count);
        islands_.reserve(count);
        narrow_phase_.reserve(count);
        const usize manifold_reserve = static_cast<usize>(count) * NarrowPhaseStage::kManifoldReserveFactor;
        manifold_restitution_cache_.reserve(manifold_reserve);
        manifold_friction_cache_.reserve(manifold_reserve);
        broad_phase_.reserve(count);
    }

    void clear_manifold_state_() {
        narrow_phase_.clear();
        islands_.clear(capacity_);
    }

    [[nodiscard]] HotPathCapacitySnapshot capture_hot_path_capacity_snapshot_() const noexcept {
        HotPathCapacitySnapshot snapshot{};
        snapshot.candidate_pairs_capacity = broad_phase_.candidate_pairs_capacity();
        snapshot.manifold_capacity = narrow_phase_.manifold_capacity();
        snapshot.manifold_material_capacity = manifold_restitution_cache_.capacity();
        snapshot.broad_phase_worker_pair_capacity_sum = broad_phase_.worker_pair_capacity_sum();
        return snapshot;
    }

    void publish_hot_path_capacity_growth_(const HotPathCapacitySnapshot &before) const noexcept {
        const HotPathCapacitySnapshot after = capture_hot_path_capacity_snapshot_();
        i64 growth_events = 0;
        usize growth_bytes = 0;

        const auto accumulate_growth = [&](const usize before_capacity, const usize after_capacity,
                                           const usize elem_size) {
            if (after_capacity <= before_capacity) {
                return;
            }
            ++growth_events;
            growth_bytes += (after_capacity - before_capacity) * elem_size;
        };
        accumulate_growth(before.candidate_pairs_capacity, after.candidate_pairs_capacity, sizeof(BodyPair));
        accumulate_growth(before.manifold_capacity, after.manifold_capacity, sizeof(ContactManifold));
        accumulate_growth(before.manifold_material_capacity, after.manifold_material_capacity, sizeof(f32) * 2u);
        accumulate_growth(before.broad_phase_worker_pair_capacity_sum, after.broad_phase_worker_pair_capacity_sum,
                          sizeof(BodyPair));

        TracyPlot("physics_hot_capacity_growth_events", growth_events);
        TracyPlot("physics_hot_capacity_growth_bytes", static_cast<i64>(growth_bytes));
        TracyPlot("physics_candidate_pairs_capacity", static_cast<i64>(after.candidate_pairs_capacity));
        TracyPlot("physics_manifold_capacity", static_cast<i64>(after.manifold_capacity));
        TracyPlot("physics_broad_phase_worker_pair_capacity_sum",
                  static_cast<i64>(after.broad_phase_worker_pair_capacity_sum));
    }

    BodyPartition partition_{};
    BroadPhaseStage broad_phase_{};
    NarrowPhaseStage narrow_phase_{};
    IslandManager islands_{};
    BoundsBuilder bounds_{};
    DebugPublisher debug_{};

    // Per-manifold combined material properties, recomputed each tick before solve.
    std::vector<f32> manifold_restitution_cache_{};
    std::vector<f32> manifold_friction_cache_{};

    u32 previous_body_count_{0};
    u32 capacity_{0};
};

} // namespace javelin
