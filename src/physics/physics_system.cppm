module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
import javelin.math.quat;
import javelin.physics.aabb;
import javelin.physics.bvh_dynamic;
import javelin.physics.bvh_static;
import javelin.physics.broad_phase;
import javelin.physics.contact_debug;
import javelin.physics.integrate;
import javelin.physics.narrow_phase;
import javelin.physics.publish;
import javelin.physics.solve;
import javelin.physics.types;
import javelin.scene;
import javelin.scene.physics_materials;
import javelin.scene.physics_view;
import javelin.scene.shapes;

namespace javelin::detail {
// Pair-wise material combination rules for contact resolution:
// - restitution: geometric mean preserves the elastic range [0,1] symmetrically.
// - friction: minimum gives the softer of the two surfaces (conservative slip threshold).
[[nodiscard]] inline f32 combined_restitution(const f32 a, const f32 b) noexcept { return std::sqrt(a * b); }
[[nodiscard]] inline f32 combined_friction(const f32 a, const f32 b) noexcept { return std::min(a, b); }
} // namespace javelin::detail

export namespace javelin {
// Fixed-timestep physics driver.
// Tick pipeline:
// 1) update dynamic state (forces + bounds),
// 2) generate candidate pairs (broad phase),
// 3) build/refresh manifolds (narrow phase + persistence),
// 4) solve constraints and integrate,
// 5) publish debug snapshots + authoritative transforms.
struct PhysicsSystem final {
    void init(Scene &scene) noexcept {
        scene_ = &scene;

        // Contact debug payload budget:
        // - manifold reserve heuristic is 4*body_count.
        // - each manifold contributes up to 4 points.
        // - use a conservative floor to avoid tiny startup allocations.
        const u32 body_count = scene.physics_view().count;
        const u32 estimated_point_capacity = std::max<u32>(body_count * 16u, 64u);
        contact_debug_channel_.reserve(estimated_point_capacity);
        contact_debug_channel_.publish_empty(0u);
    }

    void set_gravity(const f32 gravity) noexcept { gravity_.store(gravity, std::memory_order_relaxed); }
    void set_restitution(const f32 restitution) noexcept { restitution_.store(restitution, std::memory_order_relaxed); }
    void set_friction(const f32 friction) noexcept { friction_.store(friction, std::memory_order_relaxed); }
    void set_linear_damping(const f32 damping) noexcept {
        linear_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void set_angular_damping(const f32 damping) noexcept {
        angular_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void set_contact_debug_enabled(const bool enabled) noexcept {
        contact_debug_enabled_.store(enabled, std::memory_order_release);
    }
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
    // Queues manual fixed ticks while paused.
    // No-op when simulation is running.
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
    [[nodiscard]] f32 restitution() const noexcept { return restitution_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 friction() const noexcept { return friction_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 linear_damping() const noexcept { return linear_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 angular_damping() const noexcept { return angular_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] bool contact_debug_enabled() const noexcept {
        return contact_debug_enabled_.load(std::memory_order_acquire);
    }
    [[nodiscard]] ContactDebugSnapshot contact_debug_snapshot() const noexcept { return contact_debug_channel_.snapshot(); }
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

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }
        if (scene_ == nullptr) {
            log::warn(physics, "Starting without scene bound");
        }

        log::info(physics, "Starting physics system");
        log::info(physics, "Params gravity={} restitution={} friction={} linear_damping={} angular_damping={}",
                  gravity(), restitution(), friction(), linear_damping(), angular_damping());
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
                    if (simulate_one_fixed_tick_()) {
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
        if (!thread_.joinable()) {
            log::warn(physics, "Stop ignored (not running)");
            return;
        }
        log::info(physics, "Stopping physics system");
        thread_.request_stop();
        simulation_control_cv_.notify_all();
        thread_.join();
        stop_broad_phase_workers_();
    }

  private:
    // Per-worker broad phase storage: scratch query buffers and local pair output.
    struct BroadPhaseWorker final {
        BroadPhaseScratch scratch{};
        std::vector<BodyPair> pairs{};

        void reserve(const u32 count, const u32 query_stack_factor, const u32 pair_factor) {
            scratch.reserve(count, query_stack_factor);
            pairs.reserve(static_cast<usize>(count) * pair_factor);
        }
    };

    // Immutable snapshot for one broad-phase dispatch.
    struct BroadPhaseJob final {
        std::span<const u32> dynamic_ids{};
        u32 dynamic_count{};
        u32 worker_count{};
        u32 chunk_size{};
    };

    Scene *scene_{nullptr};
    std::jthread thread_{};
    std::atomic<f32> gravity_{-9.8f};
    std::atomic<f32> restitution_{0.3f};
    std::atomic<f32> friction_{0.2f};
    std::atomic<f32> linear_damping_{0.1f};
    std::atomic<f32> angular_damping_{0.4f};
    std::atomic<bool> contact_debug_enabled_{false};
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
    bool static_dirty_{true};
    u32 last_count_{0};
    u32 capacity_{0};
    static constexpr f32 kFixedStepDtSeconds = 1.0f / 60.0f;
    static constexpr double kFixedStepDtMilliseconds = 1000.0 / 60.0;
    // Reserve factors are capacity planning heuristics for amortized O(1) growth.
    static constexpr u32 kQueryStackReserveFactor = 2;
    static constexpr u32 kPairReserveFactor = 8;
    static constexpr u32 kManifoldReserveFactor = 4;
    // Persistence thresholds in world-space meters.
    // A cached point is dropped when either threshold is exceeded.
    static constexpr f32 kPersistenceAnchorThreshold = 0.03f;
    static constexpr f32 kPersistenceAnchorThresholdSq = kPersistenceAnchorThreshold * kPersistenceAnchorThreshold;
    static constexpr f32 kPersistenceNormalBreakThreshold = 0.015f;
    static constexpr f32 kPersistenceTangentialDriftBreakThreshold = 0.025f;
    static constexpr f32 kPersistenceTangentialDriftBreakThresholdSq =
        kPersistenceTangentialDriftBreakThreshold * kPersistenceTangentialDriftBreakThreshold;
    static constexpr f32 kPersistenceMatchEps = 1e-6f;
    static constexpr u32 kBoxAxisFeatureTag = 1u << 10u;
    static constexpr u32 kBoxFaceFaceFeatureTag = 1u << 13u;
    DynamicBvh dynamic_bvh_{};
    StaticBvh static_bvh_{};
    std::vector<BodyPair> candidate_pairs_{};
    // Double-buffered manifold storage: current frame and next-frame write target.
    std::vector<ContactManifold> manifolds_{};
    std::vector<ContactManifold> next_manifolds_{};
    // Maps canonical body pair -> index in manifolds_ for persistence lookup.
    std::unordered_map<u64, u32> manifold_lookup_{};
    u32 broad_phase_worker_count_{0};
    std::vector<BroadPhaseWorker> broad_phase_workers_{};
    std::vector<std::thread> broad_phase_threads_{};
    std::vector<usize> broad_phase_pair_offsets_{};
    std::mutex broad_phase_mutex_{};
    std::condition_variable broad_phase_cv_{};
    std::condition_variable broad_phase_done_cv_{};
    BroadPhaseJob broad_phase_job_{};
    u64 broad_phase_job_id_{0};
    std::atomic<u32> broad_phase_jobs_remaining_{0};
    bool broad_phase_stop_{false};
    std::vector<u32> static_ids_{};
    std::vector<u32> dynamic_ids_{};
    std::vector<Aabb> bounds_cache_{};
    // Per-manifold combined material properties, recomputed each tick before solve.
    std::vector<f32> manifold_restitution_cache_{};
    std::vector<f32> manifold_friction_cache_{};
    // Byte mask indexed by body id; set when body participates in any active contact this tick.
    std::vector<u8> contact_activity_mask_{};
    ContactDebugChannel contact_debug_channel_{};
    // Physics-thread state: tracks enable->disable transitions so we can clear
    // stale snapshots once without paying per-tick writes while disabled.
    bool contact_debug_enabled_last_tick_{false};

    struct PersistenceRefreshStats final {
        u32 previous_point_count{};
        u32 next_point_count{};
        u32 matched_point_count{};
        u32 dropped_point_count{};
        // Optional diagnostic: manifold-level box-axis key changes across frames.
        u32 axis_flip_count{};
    };

    struct BoxAxisKey final {
        u8 type{};
        u8 i{};
        u8 j{};
    };

    [[nodiscard]] static i64 tracy_counter_i64_(const u64 value) noexcept {
        return static_cast<i64>(std::min<u64>(value, static_cast<u64>(std::numeric_limits<i64>::max())));
    }

    [[nodiscard]] u64 next_completed_step_id_() const noexcept {
        // This tick is counted by the outer loop immediately after
        // simulate_one_fixed_tick_() returns true.
        return completed_sim_step_count_.load(std::memory_order_relaxed) + 1u;
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
            static_dirty_ = true;
            clear_manifold_state_();
            contact_debug_channel_.publish_empty(completed_sim_step_count_.load(std::memory_order_relaxed));
            contact_debug_enabled_last_tick_ = false;
        }
        return true;
    }

    [[nodiscard]] bool simulate_one_fixed_tick_() {
        if (scene_ == nullptr) {
            return false;
        }

        static_cast<void>(apply_pending_reset_());

        // Material 0 is the fallback for any body with no explicit physics_material record.
        // Push the live global slider values into it so ImGui edits take effect immediately.
        scene_->set_physics_material(0u, PhysicsMaterial{
            .restitution = restitution_.load(std::memory_order_relaxed),
            .friction    = friction_.load(std::memory_order_relaxed),
        });

        PhysicsView view = scene_->physics_view();
        const u32 count = view.count;
        const f32 dt = kFixedStepDtSeconds;
        const f32 gravity = gravity_.load(std::memory_order_relaxed);
        const f32 linear_damping = linear_damping_.load(std::memory_order_relaxed);
        const f32 angular_damping = angular_damping_.load(std::memory_order_relaxed);

        if (count != last_count_) {
            static_dirty_ = true;
            clear_manifold_state_();
        }
        // Stage 0: ensure frame scratch and previous-manifold lookup are ready.
        ensure_capacity_(count);
        prepare_manifold_lookup_();

        // Stage 1: external forces and per-body bounds for broad phase.
        integrate_gravity_velocity(view.velocity, view.inv_mass, gravity, dt);
        f32 max_angular_speed_sq = 0.0f;
        for (u32 i = 0; i < count; ++i) {
            max_angular_speed_sq = std::max(max_angular_speed_sq, view.angular_velocity[i].length_sq());
        }
        TracyPlot("physics_max_angular_speed", std::sqrt(max_angular_speed_sq));
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
                const Vec3 extents = abs0 * box.half_extents.x + abs1 * box.half_extents.y + abs2 * box.half_extents.z;
                const Vec3 center = view.position[i];
                bounds_cache_[i] = Aabb{center - extents, center + extents};
            } break;
            }
        }

        if (static_dirty_) {
            rebuild_body_sets_(view);
            last_count_ = count;
            static_dirty_ = false;
        }

        // Stage 2: broad phase candidate generation.
        const std::span<const u32> dynamic_ids{dynamic_ids_.data(), dynamic_ids_.size()};
        // Mutating phase: update dynamic BVH before read-only queries.
        broad_phase_update_dynamic_bvh(dynamic_ids, dynamic_bvh_, bounds_cache_);
        // Read-only phase: query broad phase pairs.
        run_broad_phase_queries_(dynamic_ids);
        TracyPlot("physics_pairs", static_cast<i64>(candidate_pairs_.size()));

        // Stage 3: narrow phase manifolds + warm-start persistence refresh.
        narrow_phase_contacts(view.position, view.orientation, view.shape_kind, view.shapes, view.shape_index,
                              view.inv_mass, candidate_pairs_, manifolds_, manifold_lookup_, next_manifolds_);
        sort_manifold_points_(next_manifolds_);
        sort_manifolds_(next_manifolds_);
        const PersistenceRefreshStats persistence_stats = refresh_manifold_persistence_(view.position, view.orientation);
        manifolds_.swap(next_manifolds_);
        const u32 manifold_count = static_cast<u32>(manifolds_.size());
        const u32 contact_point_count = contact_point_count_(manifolds_);
        const f32 avg_points_per_manifold =
            (manifold_count > 0u) ? (static_cast<f32>(contact_point_count) / static_cast<f32>(manifold_count)) : 0.0f;
        const f32 warm_start_match_rate =
            (persistence_stats.next_point_count > 0u)
                ? (static_cast<f32>(persistence_stats.matched_point_count) /
                   static_cast<f32>(persistence_stats.next_point_count))
                : 0.0f;
        TracyPlot("physics_manifolds", static_cast<i64>(manifold_count));
        TracyPlot("physics_contact_points", static_cast<i64>(contact_point_count));
        TracyPlot("physics_avg_points_per_manifold", avg_points_per_manifold);
        TracyPlot("physics_warm_start_match_rate", warm_start_match_rate);
        TracyPlot("physics_dropped_points", static_cast<i64>(persistence_stats.dropped_point_count));
        TracyPlot("physics_axis_flip_count", static_cast<i64>(persistence_stats.axis_flip_count));
        TracyPlot("physics_contacts", static_cast<i64>(contact_point_count));

        // Stage 4: solve constraints, damp, integrate, and publish transforms.
        // Pre-compute per-manifold combined material properties.
        // Body b == kInvalidBody (ground plane) uses material id 0 (the default material).
        manifold_restitution_cache_.resize(manifold_count);
        manifold_friction_cache_.resize(manifold_count);
        for (u32 i = 0; i < manifold_count; ++i) {
            const u32 a = manifolds_[i].a;
            const u32 b = manifolds_[i].b;
            const u32 mat_a = view.material[a].value;
            const u32 mat_b = (b != kInvalidBody) ? view.material[b].value : 0u;
            manifold_restitution_cache_[i] =
                detail::combined_restitution(view.material_restitution[mat_a], view.material_restitution[mat_b]);
            manifold_friction_cache_[i] =
                detail::combined_friction(view.material_friction[mat_a], view.material_friction[mat_b]);
        }
        solve_contact_velocities(view.velocity, view.angular_velocity, view.inv_mass, view.inv_inertia, view.orientation,
                                 manifolds_, dt,
                                 std::span<const f32>{manifold_restitution_cache_},
                                 std::span<const f32>{manifold_friction_cache_});
        solve_contact_penetration(view.position, view.orientation, view.inv_mass, view.inv_inertia,
                                  std::span<const ContactManifold>{manifolds_});
        mark_bodies_with_active_contacts_(count, std::span<const ContactManifold>{manifolds_});
        apply_linear_damping(view.velocity, view.inv_mass, linear_damping, dt);
        apply_angular_damping(view.angular_velocity, view.inv_mass, angular_damping, dt);
        integrate_positions(view.position, view.velocity, view.inv_mass, dt);
        integrate_orientations(view.orientation, view.angular_velocity, view.inv_mass, dt);
        const bool publish_contact_debug = contact_debug_enabled_.load(std::memory_order_acquire);
        if (publish_contact_debug) {
            publish_contact_debug_snapshot_(view.position, view.orientation, std::span<const ContactManifold>{manifolds_},
                                            next_completed_step_id_());
        } else if (contact_debug_enabled_last_tick_) {
            contact_debug_channel_.publish_empty(next_completed_step_id_());
        }
        contact_debug_enabled_last_tick_ = publish_contact_debug;
        publish_poses(view.poses, view.position, view.orientation, count);
        return true;
    }

    void ensure_capacity_(const u32 count) {
        if (count <= capacity_) {
            ensure_broad_phase_workers_(count);
            return;
        }
        capacity_ = count;
        dynamic_bvh_.reserve(count);
        static_bvh_.reserve(count);
        bounds_cache_.reserve(count);
        static_ids_.reserve(count);
        dynamic_ids_.reserve(count);
        contact_activity_mask_.reserve(count);
        candidate_pairs_.reserve(static_cast<usize>(count) * kPairReserveFactor);
        const usize manifold_reserve = static_cast<usize>(count) * kManifoldReserveFactor;
        manifolds_.reserve(manifold_reserve);
        next_manifolds_.reserve(manifold_reserve);
        manifold_lookup_.reserve(manifold_reserve);
        manifold_restitution_cache_.reserve(manifold_reserve);
        manifold_friction_cache_.reserve(manifold_reserve);
        ensure_broad_phase_workers_(count);
    }

    void clear_manifold_state_() {
        manifolds_.clear();
        next_manifolds_.clear();
        manifold_lookup_.clear();
    }

    // Builds lookup for previous-frame manifolds keyed by canonical body pair.
    // Precondition: manifolds_ contains canonicalized pair ids.
    void prepare_manifold_lookup_() {
        next_manifolds_.clear();
        manifold_lookup_.clear();
        sort_manifold_points_(manifolds_);
        for (u32 i = 0; i < manifolds_.size(); ++i) {
            const ContactManifold &manifold = manifolds_[i];
            const auto [_, inserted] = manifold_lookup_.emplace(body_pair_key(manifold.a, manifold.b), i);
#ifndef NDEBUG
            if (!inserted) {
                log::error(physics, "Duplicate manifold pair key (a={} b={})", manifold.a, manifold.b);
                std::terminate();
            }
#endif
        }
    }

    void mark_bodies_with_active_contacts_(const u32 body_count, std::span<const ContactManifold> manifolds) {
        if (contact_activity_mask_.size() < body_count) {
            contact_activity_mask_.resize(body_count);
        }
        std::fill_n(contact_activity_mask_.begin(), body_count, static_cast<u8>(0u));

        for (const ContactManifold &manifold : manifolds) {
            if (manifold.point_count == 0u) {
                continue;
            }
#ifndef NDEBUG
            if (manifold.a >= body_count || (manifold.b != kInvalidBody && manifold.b >= body_count)) {
                log::error(physics, "Contact activity mask manifold id out of range (a={} b={} count={})", manifold.a,
                           manifold.b, body_count);
                std::terminate();
            }
#endif
            contact_activity_mask_[manifold.a] = 1u;
            if (manifold.b != kInvalidBody) {
                contact_activity_mask_[manifold.b] = 1u;
            }
        }
    }

    void sort_manifold_points_(std::vector<ContactManifold> &manifolds) {
        for (ContactManifold &manifold : manifolds) {
            sort_manifold_points(manifold);
        }
    }

    void publish_contact_debug_snapshot_(std::span<const Vec3> position, std::span<const Quat> orientation,
                                         std::span<const ContactManifold> manifolds, const u64 step_id) {
        const u32 point_count = contact_point_count_(manifolds);
        ContactDebugWrite out = contact_debug_channel_.write_contacts(point_count);

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
        contact_debug_channel_.publish(out_index, step_id);
    }

    [[nodiscard]] static bool body_pair_less_(const BodyPair lhs, const BodyPair rhs) noexcept {
        if (lhs.a != rhs.a) {
            return lhs.a < rhs.a;
        }
        return lhs.b < rhs.b;
    }

    [[nodiscard]] static bool body_pair_equal_(const BodyPair lhs, const BodyPair rhs) noexcept {
        return lhs.a == rhs.a && lhs.b == rhs.b;
    }

    // Canonicalizes pair order and removes duplicates to keep narrow phase deterministic.
    void normalize_and_sort_candidate_pairs_() {
        if (candidate_pairs_.empty()) {
            return;
        }

        for (BodyPair &pair : candidate_pairs_) {
            pair = canonical_body_pair(pair.a, pair.b);
        }

        std::sort(candidate_pairs_.begin(), candidate_pairs_.end(), body_pair_less_);
        const auto unique_end = std::unique(candidate_pairs_.begin(), candidate_pairs_.end(), body_pair_equal_);
        candidate_pairs_.erase(unique_end, candidate_pairs_.end());
    }

    [[nodiscard]] static bool manifold_less_(const ContactManifold &lhs, const ContactManifold &rhs) noexcept {
        if (lhs.a != rhs.a) {
            return lhs.a < rhs.a;
        }
        if (lhs.b != rhs.b) {
            return lhs.b < rhs.b;
        }
        if (lhs.manifold_feature_id != rhs.manifold_feature_id) {
            return lhs.manifold_feature_id < rhs.manifold_feature_id;
        }
        if (lhs.point_count != rhs.point_count) {
            return lhs.point_count < rhs.point_count;
        }

        const u32 point_count = std::min(lhs.point_count, rhs.point_count);
        for (u32 i = 0; i < point_count; ++i) {
            const u32 lhs_feature = lhs.points[i].feature_id;
            const u32 rhs_feature = rhs.points[i].feature_id;
            if (lhs_feature != rhs_feature) {
                return lhs_feature < rhs_feature;
            }
        }

        const u32 lhs_nx = ordered_float_key(lhs.normal.x);
        const u32 rhs_nx = ordered_float_key(rhs.normal.x);
        if (lhs_nx != rhs_nx) {
            return lhs_nx < rhs_nx;
        }
        const u32 lhs_ny = ordered_float_key(lhs.normal.y);
        const u32 rhs_ny = ordered_float_key(rhs.normal.y);
        if (lhs_ny != rhs_ny) {
            return lhs_ny < rhs_ny;
        }
        const u32 lhs_nz = ordered_float_key(lhs.normal.z);
        const u32 rhs_nz = ordered_float_key(rhs.normal.z);
        if (lhs_nz != rhs_nz) {
            return lhs_nz < rhs_nz;
        }

        return false;
    }

    void sort_manifolds_(std::vector<ContactManifold> &manifolds) {
        if (manifolds.size() <= 1u) {
            return;
        }
        std::sort(manifolds.begin(), manifolds.end(), manifold_less_);
    }

    [[nodiscard]] static u32 contact_point_count_(std::span<const ContactManifold> manifolds) noexcept {
        u32 count = 0u;
        for (const ContactManifold &manifold : manifolds) {
            count += manifold.point_count;
        }
        return count;
    }

    [[nodiscard]] static bool decode_box_axis_feature_id_(const u32 feature_id, BoxAxisKey &out) noexcept {
        if ((feature_id & kBoxAxisFeatureTag) == 0u) {
            return false;
        }
        // Face-face point ids are not valid manifold axis ids.
        if ((feature_id & kBoxFaceFaceFeatureTag) != 0u) {
            return false;
        }
        const u32 type = feature_id & 0x3u;
        if (type > 2u) {
            return false;
        }
        out.type = static_cast<u8>(type);
        out.i = static_cast<u8>((feature_id >> 2u) & 0x3u);
        out.j = static_cast<u8>((feature_id >> 4u) & 0x3u);
        return true;
    }

    [[nodiscard]] static bool manifold_axis_flipped_(const ContactManifold &previous_manifold,
                                                     const ContactManifold &next_manifold) noexcept {
        if (previous_manifold.point_count == 0u || next_manifold.point_count == 0u) {
            return false;
        }
        BoxAxisKey previous_axis{};
        BoxAxisKey next_axis{};
        if (!decode_box_axis_feature_id_(previous_manifold.manifold_feature_id, previous_axis) ||
            !decode_box_axis_feature_id_(next_manifold.manifold_feature_id, next_axis)) {
            return false;
        }
        return previous_axis.type != next_axis.type || previous_axis.i != next_axis.i || previous_axis.j != next_axis.j;
    }

    [[nodiscard]] static f32 local_anchor_match_distance_sq_(const ContactPoint &lhs, const ContactPoint &rhs,
                                                             const bool manifold_has_body_b) noexcept {
        const f32 anchor_delta_a_sq = (lhs.local_anchor_a - rhs.local_anchor_a).length_sq();
        if (!manifold_has_body_b) {
            return anchor_delta_a_sq;
        }
        const f32 anchor_delta_b_sq = (lhs.local_anchor_b - rhs.local_anchor_b).length_sq();
        return std::max(anchor_delta_a_sq, anchor_delta_b_sq);
    }

    static void reset_point_cache_(ContactPoint &point) noexcept {
        point.normal_impulse = 0.0f;
        point.tangent_impulse = Vec3{};
        point.persisted = false;
    }

    static void copy_point_cache_(ContactPoint &dst, const ContactPoint &src) noexcept {
        dst.normal_impulse = src.normal_impulse;
        dst.tangent_impulse = src.tangent_impulse;
        // This point was matched to a previous-frame point.
        dst.persisted = true;
    }

    // Drops cached impulses when anchor drift exceeds thresholds in the current manifold frame.
    [[nodiscard]] bool should_drop_persisted_point_(std::span<const Vec3> position, std::span<const Quat> orientation,
                                                    const ContactManifold &manifold, const ContactPoint &next_point,
                                                    const ContactPoint &previous_point) const noexcept {
        const u32 a = manifold.a;
        const Vec3 world_a_previous = position[a] + rotate(orientation[a], previous_point.local_anchor_a);
        const Vec3 world_a_next = position[a] + rotate(orientation[a], next_point.local_anchor_a);

        f32 normal_separation = 0.0f;
        f32 normal_drift = 0.0f;
        Vec3 tangential_delta{};
        if (manifold.b != kInvalidBody) {
            const u32 b = manifold.b;
            const Vec3 world_b_previous = position[b] + rotate(orientation[b], previous_point.local_anchor_b);
            const Vec3 world_b_next = position[b] + rotate(orientation[b], next_point.local_anchor_b);
            const Vec3 delta_previous = world_b_previous - world_a_previous;
            const Vec3 delta_next = world_b_next - world_a_next;

            const f32 normal_previous = dot(delta_previous, manifold.normal);
            const f32 normal_next = dot(delta_next, manifold.normal);
            normal_separation = normal_next;
            normal_drift = std::fabs(normal_next - normal_previous);

            const Vec3 tangential_previous = delta_previous - manifold.normal * normal_previous;
            const Vec3 tangential_next = delta_next - manifold.normal * normal_next;
            tangential_delta = tangential_next - tangential_previous;
        } else {
            const Vec3 delta = world_a_previous - world_a_next;
            const f32 normal_component = dot(delta, manifold.normal);
            normal_separation = std::fabs(normal_component);
            normal_drift = std::fabs(normal_component);
            tangential_delta = delta - manifold.normal * normal_component;
        }

        const bool normal_break_exceeded = normal_separation > kPersistenceNormalBreakThreshold;
        const bool normal_drift_exceeded = normal_drift > kPersistenceNormalBreakThreshold;
        const bool tangential_drift_exceeded =
            tangential_delta.length_sq() > kPersistenceTangentialDriftBreakThresholdSq;
        return normal_break_exceeded || normal_drift_exceeded || tangential_drift_exceeded;
    }

    void match_and_transfer_point_cache_(std::span<const Vec3> position, std::span<const Quat> orientation,
                                         ContactManifold &next_manifold,
                                         const ContactManifold &previous_manifold) const {
#ifndef NDEBUG
        if (next_manifold.point_count > kMaxManifoldPoints || previous_manifold.point_count > kMaxManifoldPoints) {
            log::error(physics, "Invalid manifold point_count during persistence refresh (next={} previous={})",
                       next_manifold.point_count, previous_manifold.point_count);
            std::terminate();
        }
#endif
        const bool manifold_has_body_b = next_manifold.b != kInvalidBody;
        std::array<bool, kMaxManifoldPoints> previous_used{};

        for (u32 i = 0; i < next_manifold.point_count; ++i) {
            reset_point_cache_(next_manifold.points[i]);
        }

        // Match order is deterministic: feature id pass first, then local-anchor fallback.
        auto try_match_point = [&](const u32 next_index, const bool match_feature_first) {
            ContactPoint &next_point = next_manifold.points[next_index];
            const bool next_has_feature = next_point.feature_id != kInvalidContactFeature;
            if (match_feature_first && !next_has_feature) {
                return false;
            }

            u32 best_previous = kMaxManifoldPoints;
            f32 best_metric = std::numeric_limits<f32>::infinity();
            for (u32 previous_index = 0; previous_index < previous_manifold.point_count; ++previous_index) {
                if (previous_used[previous_index]) {
                    continue;
                }
                const ContactPoint &previous_point = previous_manifold.points[previous_index];
                const bool previous_has_feature = previous_point.feature_id != kInvalidContactFeature;
                if (match_feature_first) {
                    if (!previous_has_feature || previous_point.feature_id != next_point.feature_id) {
                        continue;
                    }
                }

                const f32 metric = local_anchor_match_distance_sq_(next_point, previous_point, manifold_has_body_b);
                if (metric > kPersistenceAnchorThresholdSq) {
                    continue;
                }

                const bool better =
                    metric < best_metric - kPersistenceMatchEps ||
                    (std::fabs(metric - best_metric) <= kPersistenceMatchEps && previous_index < best_previous);
                if (better) {
                    best_metric = metric;
                    best_previous = previous_index;
                }
            }

            if (best_previous == kMaxManifoldPoints) {
                return false;
            }
            const ContactPoint &previous_point = previous_manifold.points[best_previous];
            if (should_drop_persisted_point_(position, orientation, next_manifold, next_point, previous_point)) {
                return false;
            }

            copy_point_cache_(next_point, previous_point);
            previous_used[best_previous] = true;
            return true;
        };

        for (u32 i = 0; i < next_manifold.point_count; ++i) {
            static_cast<void>(try_match_point(i, true));
        }
        for (u32 i = 0; i < next_manifold.point_count; ++i) {
            if (next_manifold.points[i].persisted) {
                continue;
            }
            static_cast<void>(try_match_point(i, false));
        }
    }

    // Refreshes per-point warm-start caches by matching next_manifolds_ against manifolds_.
    // Matching order is deterministic and stats are exposed for diagnostics.
    [[nodiscard]] PersistenceRefreshStats refresh_manifold_persistence_(std::span<const Vec3> position,
                                                                        std::span<const Quat> orientation) {
        PersistenceRefreshStats stats{
            .previous_point_count = contact_point_count_(manifolds_),
        };
        for (ContactManifold &next_manifold : next_manifolds_) {
            stats.next_point_count += next_manifold.point_count;
            const auto it = manifold_lookup_.find(body_pair_key(next_manifold.a, next_manifold.b));
            if (it == manifold_lookup_.end()) {
                for (u32 i = 0; i < next_manifold.point_count; ++i) {
                    reset_point_cache_(next_manifold.points[i]);
                }
                continue;
            }
#ifndef NDEBUG
            if (it->second >= manifolds_.size()) {
                log::error(physics, "Manifold lookup out of range (a={} b={} index={} size={})", next_manifold.a,
                           next_manifold.b, it->second, manifolds_.size());
                std::terminate();
            }
#endif
            const ContactManifold &previous_manifold = manifolds_[it->second];
#ifndef NDEBUG
            if (previous_manifold.a != next_manifold.a || previous_manifold.b != next_manifold.b) {
                log::error(physics,
                           "Manifold pair mismatch during persistence refresh (next=({}, {}) previous=({}, {}))",
                           next_manifold.a, next_manifold.b, previous_manifold.a, previous_manifold.b);
                std::terminate();
            }
#endif
            if (manifold_axis_flipped_(previous_manifold, next_manifold)) {
                ++stats.axis_flip_count;
            }
            match_and_transfer_point_cache_(position, orientation, next_manifold, previous_manifold);
            for (u32 i = 0; i < next_manifold.point_count; ++i) {
                stats.matched_point_count += next_manifold.points[i].persisted ? 1u : 0u;
            }
        }
        stats.dropped_point_count = (stats.previous_point_count > stats.matched_point_count)
                                        ? (stats.previous_point_count - stats.matched_point_count)
                                        : 0u;
        return stats;
    }

    void ensure_broad_phase_workers_(const u32 count) {
        if (broad_phase_worker_count_ == 0) {
            const u32 hw_threads = std::thread::hardware_concurrency();
            broad_phase_worker_count_ = (hw_threads > 0) ? hw_threads : 1u;
            broad_phase_workers_.resize(broad_phase_worker_count_);
            broad_phase_threads_.reserve(broad_phase_worker_count_ - 1u);
            broad_phase_pair_offsets_.reserve(static_cast<usize>(broad_phase_worker_count_) + 1u);
            start_broad_phase_workers_();
        }
        for (auto &worker : broad_phase_workers_) {
            worker.reserve(count, kQueryStackReserveFactor, kPairReserveFactor);
        }
    }

    void run_broad_phase_queries_(std::span<const u32> dynamic_ids) {
        ZoneScopedN("Physics broad phase parallel");
        candidate_pairs_.clear();
        const u32 dynamic_count = static_cast<u32>(dynamic_ids.size());
        if (dynamic_count == 0) {
            return;
        }

        // dynamic_ids are built in ascending order; assert in debug for safety.
#ifndef NDEBUG
        if (!std::is_sorted(dynamic_ids.begin(), dynamic_ids.end())) {
            log::error(physics, "Broad phase dynamic ids are not sorted");
            std::terminate();
        }
#endif

        // Worker pool: fixed thread count, contiguous chunks, deterministic merge by chunk order.
        const u32 worker_count = std::min(broad_phase_worker_count_, dynamic_count);
        const u32 chunk_size = (dynamic_count + worker_count - 1u) / worker_count;

        const BroadPhaseJob job{
            .dynamic_ids = dynamic_ids,
            .dynamic_count = dynamic_count,
            .worker_count = worker_count,
            .chunk_size = chunk_size,
        };

        if (worker_count > 1) {
            {
                std::lock_guard lock(broad_phase_mutex_);
                // BVHs are read-only for the duration of this job.
                broad_phase_job_ = job;
                broad_phase_jobs_remaining_.store(worker_count - 1u, std::memory_order_release);
                ++broad_phase_job_id_;
            }
            broad_phase_cv_.notify_all();
        }

        run_broad_phase_chunk_(job, 0);

        if (worker_count > 1) {
            std::unique_lock lock(broad_phase_mutex_);
            broad_phase_done_cv_.wait(
                lock, [&] { return broad_phase_jobs_remaining_.load(std::memory_order_acquire) == 0u; });
        }

        {
            ZoneScopedN("Physics broad phase merge");
            // Deterministic output: concatenate chunks in increasing worker index.
            broad_phase_pair_offsets_.resize(static_cast<usize>(worker_count) + 1u);
            broad_phase_pair_offsets_[0] = 0;
            for (u32 worker_index = 0; worker_index < worker_count; ++worker_index) {
                const usize count = broad_phase_workers_[worker_index].pairs.size();
                broad_phase_pair_offsets_[static_cast<usize>(worker_index) + 1u] =
                    broad_phase_pair_offsets_[worker_index] + count;
            }

            const usize total_pairs = broad_phase_pair_offsets_[worker_count];
            candidate_pairs_.resize(total_pairs);
            for (u32 worker_index = 0; worker_index < worker_count; ++worker_index) {
                auto &src = broad_phase_workers_[worker_index].pairs;
                const usize offset = broad_phase_pair_offsets_[worker_index];
                std::copy(src.begin(), src.end(), candidate_pairs_.data() + offset);
            }
        }
        normalize_and_sort_candidate_pairs_();

#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
        validate_broad_phase_pairs_(dynamic_ids);
#endif
    }

    void run_broad_phase_chunk_(const BroadPhaseJob &job, const u32 worker_index) {
        if (worker_index >= job.worker_count) {
            return;
        }
        // Worker owns one contiguous chunk of dynamic ids.
        const u32 begin = worker_index * job.chunk_size;
        if (begin >= job.dynamic_count) {
            broad_phase_workers_[worker_index].pairs.clear();
            return;
        }
        const u32 end = std::min(begin + job.chunk_size, job.dynamic_count);
        const std::span<const u32> chunk{job.dynamic_ids.data() + begin, end - begin};
        BroadPhaseWorker &worker = broad_phase_workers_[worker_index];
        broad_phase_generate_pairs(chunk, dynamic_bvh_, static_bvh_, bounds_cache_, worker.pairs, worker.scratch);
    }

    void broad_phase_worker_loop_(const u32 worker_index) {
        thread_local std::string name;
        name = "Physics BroadPhase " + std::to_string(worker_index);
        tracy::SetThreadName(name.c_str());
        u64 last_job = 0;
        for (;;) {
            BroadPhaseJob job{};
            {
                std::unique_lock lock(broad_phase_mutex_);
                broad_phase_cv_.wait(lock, [&] { return broad_phase_stop_ || broad_phase_job_id_ != last_job; });
                if (broad_phase_stop_) {
                    return;
                }
                last_job = broad_phase_job_id_;
                job = broad_phase_job_;
            }

            if (worker_index >= job.worker_count) {
                continue;
            }
            {
                ZoneScopedN("Physics broad phase worker");
                run_broad_phase_chunk_(job, worker_index);
            }
            if (broad_phase_jobs_remaining_.fetch_sub(1u, std::memory_order_acq_rel) == 1u) {
                std::lock_guard lock(broad_phase_mutex_);
                broad_phase_done_cv_.notify_one();
            }
        }
    }

    void start_broad_phase_workers_() {
        if (broad_phase_worker_count_ <= 1 || !broad_phase_threads_.empty()) {
            return;
        }
        broad_phase_stop_ = false;
        for (u32 worker_index = 1; worker_index < broad_phase_worker_count_; ++worker_index) {
            broad_phase_threads_.emplace_back([this, worker_index] { broad_phase_worker_loop_(worker_index); });
        }
        log::info(physics, "Broad phase workers={}", broad_phase_worker_count_);
    }

    void stop_broad_phase_workers_() {
        if (broad_phase_threads_.empty()) {
            return;
        }
        {
            std::lock_guard lock(broad_phase_mutex_);
            broad_phase_stop_ = true;
            ++broad_phase_job_id_;
        }
        broad_phase_cv_.notify_all();
        for (auto &thread : broad_phase_threads_) {
            thread.join();
        }
        broad_phase_threads_.clear();
        broad_phase_stop_ = false;
        log::info(physics, "Broad phase workers stopped");
    }

#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
    void validate_broad_phase_pairs_(std::span<const u32> dynamic_ids) {
        BroadPhaseWorker &worker = broad_phase_workers_[0];
        std::vector<BodyPair> expected{};
        expected.reserve(candidate_pairs_.size());
        broad_phase_generate_pairs(dynamic_ids, dynamic_bvh_, static_bvh_, bounds_cache_, expected, worker.scratch);

        auto normalize = [](std::vector<BodyPair> &pairs) {
            std::sort(pairs.begin(), pairs.end(), [](const BodyPair &lhs, const BodyPair &rhs) {
                if (lhs.a != rhs.a) {
                    return lhs.a < rhs.a;
                }
                return lhs.b < rhs.b;
            });
            pairs.erase(
                std::unique(pairs.begin(), pairs.end(),
                            [](const BodyPair &lhs, const BodyPair &rhs) { return lhs.a == rhs.a && lhs.b == rhs.b; }),
                pairs.end());
        };

        std::vector<BodyPair> actual = candidate_pairs_;
        normalize(expected);
        normalize(actual);
        if (expected != actual) {
            log::error(physics, "Broad phase validation failed expected={} actual={}", expected.size(), actual.size());
        }
    }
#endif

    void rebuild_body_sets_(const PhysicsView &view) {
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
            static_bvh_.build(static_ids_, bounds_cache_);
        } else {
            static_bvh_.clear();
        }
    }
};

} // namespace javelin
