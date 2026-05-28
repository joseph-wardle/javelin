module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
import javelin.physics.aabb_debug;
import javelin.physics.body_graph;
import javelin.physics.broad_phase;
import javelin.physics.contact_debug;
import javelin.physics.debug_publisher;
import javelin.physics.integrate;
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

// Fixed-timestep physics driver.
//
// PhysicsSystem owns:
//   - the dedicated physics thread + its fixed-rate ticker,
//   - the simulation control state machine (pause / manual step budget / reset),
//   - the atomic configuration knobs callers tweak from other threads,
//   - the per-tick stage members (BodyGraph, broad/narrow phase, DebugPublisher,
//     and the small caches that connect them),
//   - and the inlined per-tick orchestration.
//
// DebugPublisher is held as a private member: its triple-buffered channel
// pair and edge-on-disable bookkeeping form a coherent threading contract
// worth encapsulating.  The six debug-related methods on PhysicsSystem
// forward one hop to DebugPublisher.
struct PhysicsSystem final {
    void init(Scene &scene) noexcept {
        scene_ = &scene;
        debug_publisher_.reserve(scene.bodies().count);
    }

    void set_gravity(const f32 gravity) noexcept { gravity_.store(gravity, std::memory_order_relaxed); }
    void set_linear_damping(const f32 damping) noexcept {
        linear_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void set_angular_damping(const f32 damping) noexcept {
        angular_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void set_contact_debug_enabled(const bool enabled) noexcept { debug_publisher_.set_contact_enabled(enabled); }
    void set_aabb_debug_enabled(const bool enabled) noexcept { debug_publisher_.set_aabb_enabled(enabled); }

    // Thread-safe control request consumed by the physics thread.
    // Reset is serviced even while simulation is paused.
    void request_reset() noexcept {
        reset_requested_.store(true, std::memory_order_release);
        simulation_control_cv_.notify_one();
    }

    // Pauses/resumes continuous fixed-rate ticking.
    // Resuming clears queued manual-step budget.
    void set_simulation_paused(const bool paused) noexcept {
        const bool previous = simulation_paused_.exchange(paused, std::memory_order_acq_rel);
        if (!paused) {
            pending_step_budget_.store(0u, std::memory_order_release);
        }
        if (previous != paused || paused) {
            simulation_control_cv_.notify_one();
        }
    }

    // Queues manual fixed ticks while paused.  No-op when simulation is running.
    void request_simulation_steps(const u32 count = 1u) noexcept {
        if (count == 0u || !simulation_paused_.load(std::memory_order_acquire)) {
            return;
        }
        u32 pending = pending_step_budget_.load(std::memory_order_relaxed);
        for (;;) {
            const u32 remaining_capacity = std::numeric_limits<u32>::max() - pending;
            const u32 next = pending + std::min(count, remaining_capacity);
            if (pending_step_budget_.compare_exchange_weak(pending, next, std::memory_order_acq_rel,
                                                          std::memory_order_relaxed)) {
                break;
            }
        }
        simulation_control_cv_.notify_one();
    }

    [[nodiscard]] f32 gravity() const noexcept { return gravity_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 linear_damping() const noexcept { return linear_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 angular_damping() const noexcept { return angular_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] bool contact_debug_enabled() const noexcept { return debug_publisher_.contact_enabled(); }
    [[nodiscard]] ContactDebugSnapshot contact_debug_snapshot() const noexcept {
        return debug_publisher_.contact_snapshot();
    }
    [[nodiscard]] bool aabb_debug_enabled() const noexcept { return debug_publisher_.aabb_enabled(); }
    [[nodiscard]] AabbDebugSnapshot aabb_debug_snapshot() const noexcept { return debug_publisher_.aabb_snapshot(); }
    [[nodiscard]] f32 last_tick_dt_ms() const noexcept { return last_tick_dt_ms_.load(std::memory_order_relaxed); }
    [[nodiscard]] bool simulation_paused() const noexcept {
        return simulation_paused_.load(std::memory_order_acquire);
    }
    [[nodiscard]] u32 pending_simulation_steps() const noexcept {
        return pending_step_budget_.load(std::memory_order_acquire);
    }
    [[nodiscard]] u64 completed_simulation_steps() const noexcept {
        return completed_sim_step_count_.load(std::memory_order_relaxed);
    }
    [[nodiscard]] static constexpr u32 fixed_step_hz() noexcept { return 60u; }
    [[nodiscard]] static constexpr f32 fixed_step_dt_seconds() noexcept { return kFixedStepDtSeconds; }

    [[nodiscard]] bool step_fixed() noexcept {
        if (thread_.joinable()) {
            log::warn(physics, "step_fixed ignored while threaded simulation is running");
            return false;
        }
        last_tick_dt_ms_.store(static_cast<f32>(kFixedStepDtMilliseconds), std::memory_order_relaxed);
        if (!run_one_tick_()) {
            return false;
        }
        completed_sim_step_count_.fetch_add(1u, std::memory_order_relaxed);
        return true;
    }

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }
        if (scene_ == nullptr) {
            log::warn(physics, "Starting without scene bound");
        }

        log::info(physics, "Starting physics system");
        log::info(physics, "Params gravity={} linear_damping={} angular_damping={}", gravity(), linear_damping(),
                  angular_damping());
        thread_ = std::jthread([this](const std::stop_token &stop_token) {
            tracy::SetThreadName("Physics");

            constexpr auto fixed_step_interval =
                std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<f64>(kFixedStepDtSeconds));
            FixedRateTicker ticker{fixed_step_interval};
            bool ticker_needs_resync = false;

            while (!stop_token.stop_requested()) {
                double tick_dt_ms = 0.0;
                if (simulation_paused_.load(std::memory_order_acquire)) {
                    // Paused mode: service resets and consume explicit step budget only.
                    ticker_needs_resync = true;
                    static_cast<void>(apply_pending_reset_());

                    if (consume_pending_step_()) {
                        tick_dt_ms = kFixedStepDtMilliseconds;
                    } else {
                        if (!wait_for_simulation_control_event_(stop_token)) {
                            break;
                        }
                        continue;
                    }
                } else {
                    // Running mode: resume phase-locked ticker after any pause interval.
                    if (ticker_needs_resync) {
                        ticker = FixedRateTicker{fixed_step_interval};
                        ticker_needs_resync = false;
                    }
                    const auto timing = ticker.wait_next(stop_token);
                    if (stop_token.stop_requested()) {
                        break;
                    }
                    tick_dt_ms = timing.interval_ms;
                }

                last_tick_dt_ms_.store(static_cast<f32>(tick_dt_ms), std::memory_order_relaxed);
                TracyPlot("physics_dt_ms", tick_dt_ms);

                {
                    ZoneScopedN("Physics tick");
                    if (run_one_tick_()) {
                        completed_sim_step_count_.fetch_add(1u, std::memory_order_relaxed);
                    }
                }
                const bool paused_for_plot = simulation_paused_.load(std::memory_order_relaxed);
                const u32 pending_steps_for_plot = pending_step_budget_.load(std::memory_order_relaxed);
                const u64 completed_steps_for_plot = completed_sim_step_count_.load(std::memory_order_relaxed);
                TracyPlot("physics_paused", paused_for_plot ? static_cast<i64>(1) : static_cast<i64>(0));
                TracyPlot("physics_pending_steps", static_cast<i64>(pending_steps_for_plot));
                TracyPlot("physics_completed_steps", tracy_counter_i64_(completed_steps_for_plot));

                FrameMarkNamed("Physics");
            }
        });
    }

    void stop() noexcept {
        if (thread_.joinable()) {
            log::info(physics, "Stopping physics system");
            thread_.request_stop();
            simulation_control_cv_.notify_all();
            thread_.join();
        }
        broad_phase_.stop();
    }

  private:
    static constexpr f32 kFixedStepDtSeconds = 1.0f / 60.0f;
    static constexpr double kFixedStepDtMilliseconds = 1000.0 / 60.0;

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

    struct HotPathCapacitySnapshot final {
        usize candidate_pairs_capacity{};
        usize manifold_capacity{};
        usize manifold_material_capacity{};
        usize broad_phase_worker_pair_capacity_sum{};
    };

    [[nodiscard]] static i64 tracy_counter_i64_(const u64 value) noexcept {
        return static_cast<i64>(std::min<u64>(value, static_cast<u64>(std::numeric_limits<i64>::max())));
    }

    // Sleeps only while paused and idle (no pending steps / no reset request).
    // Running mode never takes this path.
    [[nodiscard]] bool wait_for_simulation_control_event_(const std::stop_token &stop_token) {
        std::unique_lock lock(simulation_control_mutex_);
        simulation_control_cv_.wait(lock, [&] {
            return stop_token.stop_requested() || !simulation_paused_.load(std::memory_order_acquire) ||
                   pending_step_budget_.load(std::memory_order_acquire) > 0u ||
                   reset_requested_.load(std::memory_order_acquire);
        });
        return !stop_token.stop_requested();
    }

    [[nodiscard]] bool consume_pending_step_() noexcept {
        u32 pending = pending_step_budget_.load(std::memory_order_relaxed);
        while (pending > 0u) {
            if (pending_step_budget_.compare_exchange_weak(pending, pending - 1u, std::memory_order_acq_rel,
                                                           std::memory_order_relaxed)) {
                return true;
            }
        }
        return false;
    }

    // Services a pending reset request at a thread-safe tick boundary.
    [[nodiscard]] bool apply_pending_reset_() noexcept {
        if (!reset_requested_.exchange(false, std::memory_order_acq_rel)) {
            return false;
        }
        if (scene_ != nullptr) {
            scene_->reset_simulation();
            handle_reset_(completed_sim_step_count_.load(std::memory_order_relaxed));
        }
        return true;
    }

    // Reset bookkeeping: drop manifolds, mark the body graph dirty so the next
    // tick rebuilds the static/dynamic split, and publish an empty snapshot
    // to the debug channels so the renderer drops stale data.
    void handle_reset_(const u64 step_id) {
        graph_.mark_dirty();
        clear_manifold_state_();
        debug_publisher_.publish_empty_on_reset(step_id);
    }

    void ensure_capacity_(const u32 count) {
        ZoneScopedN("Physics ensure capacity");
        if (count <= capacity_) {
            broad_phase_.reserve(count);
            return;
        }
        capacity_ = count;
        graph_.reserve(count);
        narrow_phase_.reserve(count);
        const usize manifold_reserve = static_cast<usize>(count) * NarrowPhaseStage::kManifoldReserveFactor;
        manifold_restitution_cache_.reserve(manifold_reserve);
        manifold_friction_cache_.reserve(manifold_reserve);
        broad_phase_.reserve(count);
    }

    void clear_manifold_state_() {
        narrow_phase_.clear();
        graph_.clear(capacity_);
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

    // Step the simulation forward one tick.  Returns false only when the scene
    // is unbound or has no body view to step.
    [[nodiscard]] bool run_one_tick_() {
        if (scene_ == nullptr) {
            return false;
        }
        static_cast<void>(apply_pending_reset_());

        const f32 dt = kFixedStepDtSeconds;
        const f32 gravity = gravity_.load(std::memory_order_relaxed);
        const f32 linear_damping_value = linear_damping_.load(std::memory_order_relaxed);
        const f32 angular_damping_value = angular_damping_.load(std::memory_order_relaxed);
        const u64 next_step_id = completed_sim_step_count_.load(std::memory_order_relaxed) + 1u;

        Bodies view = scene_->bodies();
        const u32 count = view.count;

        if (count != previous_body_count_) {
            graph_.mark_dirty();
            clear_manifold_state_();
            previous_body_count_ = count;
        }
        // Stage 0: ensure frame scratch and previous-manifold lookup are ready.
        ensure_capacity_(count);
        const HotPathCapacitySnapshot hot_capacity_before = capture_hot_path_capacity_snapshot_();
        narrow_phase_.prepare();

        // Stage 1: external forces.
        integrate_gravity_velocity(view.velocity, view.inv_mass, view.asleep, gravity, dt);
        f32 max_angular_speed_sq = 0.0f;
        {
            ZoneScopedN("Physics max angular speed scan");
            for (u32 i = 0; i < count; ++i) {
                max_angular_speed_sq = std::max(max_angular_speed_sq, view.angular_velocity[i].length_sq());
            }
        }
        TracyPlot("physics_max_angular_speed", std::sqrt(max_angular_speed_sq));

        // Stage 2: prepare collision inputs (bounds + partition + dynamic BVH).
        graph_.prepare_collision_inputs(view);
        const bool rebuilt_body_sets = graph_.rebuilt_body_sets_this_tick();
        const std::span<const u32> dynamic_ids = graph_.dynamic_ids();
        const std::span<const u32> awake_dynamic_ids = graph_.awake_dynamic_ids();
        const std::span<const u8> awake_dynamic_mask = graph_.awake_dynamic_mask(count);
        const std::span<const u32> moved_awake_dynamic_ids = graph_.moved_awake_dynamic_ids();
        const std::span<const u8> moved_awake_mask = graph_.moved_awake_mask(count);
        TracyPlot("physics_dynamic_bodies", static_cast<i64>(dynamic_ids.size()));
        TracyPlot("physics_awake_dynamic_bodies", static_cast<i64>(awake_dynamic_ids.size()));
        TracyPlot("physics_sleeping_dynamic_bodies",
                  static_cast<i64>(dynamic_ids.size() - awake_dynamic_ids.size()));

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
            .dynamic_bvh = &graph_.dynamic_bvh(),
            .static_bvh = &graph_.static_bvh(),
            .bounds = graph_.bounds(),
            .query_ids = broad_phase_query_ids,
            .query_mask = broad_phase_query_mask,
            .body_count = count,
            .awake_dynamic_mask = awake_dynamic_mask,
            .previous_manifolds = narrow_phase_.manifolds(),
            .include_carry_forward = !fallback_to_full_awake_query,
        });
        const std::span<const BodyPair> candidate_pairs = broad_phase_result.pairs;
#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
        broad_phase_.maybe_validate(graph_.dynamic_bvh(), graph_.static_bvh(), graph_.bounds(),
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
        const u32 max_dynamic_island_size = graph_.register_contact_graph(view, manifolds);

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
        apply_linear_damping(view.velocity, view.inv_mass, view.asleep, linear_damping_value, dt);
        apply_angular_damping(view.angular_velocity, view.inv_mass, view.asleep, angular_damping_value, dt);
        clamp_resting_contact_velocities(view.velocity, view.angular_velocity, view.inv_mass,
                                         graph_.contact_activity_mask(count), view.asleep,
                                         graph_.clamp_rest_counter(count), kClampLinearSpeedThresholdSq,
                                         kClampAngularSpeedThresholdSq, kClampRestTickThreshold);
        graph_.evolve_sleep(view);
        integrate_positions(view.position, view.velocity, view.inv_mass, view.asleep, dt);
        integrate_orientations(view.orientation, view.angular_velocity, view.inv_mass, view.asleep, dt);

        debug_publisher_.maybe_publish_contacts(view.position, view.orientation, manifolds, next_step_id);
        debug_publisher_.maybe_publish_aabbs(graph_.bounds(), count, next_step_id);
        publish_poses(view.poses, view.position, view.orientation, view.asleep, count);
        publish_hot_path_capacity_growth_(hot_capacity_before);
        return true;
    }

    Scene *scene_{nullptr};

    // Per-tick stage members.
    BodyGraph graph_{};
    BroadPhaseStage broad_phase_{};
    NarrowPhaseStage narrow_phase_{};
    DebugPublisher debug_publisher_{};

    // Per-manifold combined material properties, recomputed each tick before solve.
    std::vector<f32> manifold_restitution_cache_{};
    std::vector<f32> manifold_friction_cache_{};

    u32 previous_body_count_{0};
    u32 capacity_{0};

    // Thread + control state.
    std::jthread thread_{};
    std::atomic<f32> gravity_{-9.8f};
    std::atomic<f32> linear_damping_{0.1f};
    std::atomic<f32> angular_damping_{0.4f};
    std::atomic<f32> last_tick_dt_ms_{0.0f};
    std::atomic<bool> reset_requested_{false};
    // Simulation control invariants:
    // - paused gate controls continuous ticking.
    // - pending_step_budget is decremented only by the physics thread.
    // - request_simulation_steps() only increments budget when paused.
    // - completed_sim_step_count is a monotonic diagnostic counter.
    std::atomic<bool> simulation_paused_{false};
    std::atomic<u32> pending_step_budget_{0u};
    std::atomic<u64> completed_sim_step_count_{0u};
    std::mutex simulation_control_mutex_{};
    std::condition_variable simulation_control_cv_{};
};

} // namespace javelin
