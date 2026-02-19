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
import javelin.physics.integrate;
import javelin.physics.narrow_phase;
import javelin.physics.publish;
import javelin.physics.solve;
import javelin.physics.types;
import javelin.scene;
import javelin.scene.physics_view;
import javelin.scene.shapes;

export namespace javelin {
struct PhysicsSystem final {
    void init(Scene &scene) noexcept { scene_ = &scene; }

    void set_gravity(const f32 gravity) noexcept { gravity_.store(gravity, std::memory_order_relaxed); }
    void set_restitution(const f32 restitution) noexcept { restitution_.store(restitution, std::memory_order_relaxed); }
    void set_friction(const f32 friction) noexcept { friction_.store(friction, std::memory_order_relaxed); }
    void set_angular_damping(const f32 damping) noexcept {
        angular_damping_.store(std::max(damping, 0.0f), std::memory_order_relaxed);
    }
    void request_reset() noexcept { reset_requested_.store(true, std::memory_order_release); }

    [[nodiscard]] f32 gravity() const noexcept { return gravity_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 restitution() const noexcept { return restitution_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 friction() const noexcept { return friction_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 angular_damping() const noexcept { return angular_damping_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 last_tick_dt_ms() const noexcept { return last_tick_dt_ms_.load(std::memory_order_relaxed); }

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }
        if (scene_ == nullptr) {
            log::warn(physics, "Starting without scene bound");
        }

        log::info(physics, "Starting physics system");
        log::info(physics, "Params gravity={} restitution={} friction={} angular_damping={}", gravity(), restitution(),
                  friction(), angular_damping());
        thread_ = std::jthread([this](const std::stop_token &stop_token) {
            tracy::SetThreadName("Physics");

            constexpr auto delta_time =
                std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<f64>(1.0 / 60.0));
            FixedRateTicker ticker{delta_time};

            while (!stop_token.stop_requested()) {
                const auto t = ticker.wait_next(stop_token);
                const double dt_ms = t.interval_ms;
                last_tick_dt_ms_.store(static_cast<f32>(dt_ms), std::memory_order_relaxed);
                TracyPlot("physics_dt_ms", dt_ms);

                {
                    ZoneScopedN("Physics tick");
                    if (scene_ != nullptr) {
                        PhysicsView view = scene_->physics_view();
                        const u32 count = view.count;
                        const f32 dt = 1.0f / 60.0f;
                        const f32 gravity = gravity_.load(std::memory_order_relaxed);
                        const f32 restitution = restitution_.load(std::memory_order_relaxed);
                        const f32 friction = friction_.load(std::memory_order_relaxed);
                        const f32 angular_damping = angular_damping_.load(std::memory_order_relaxed);

                        if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
                            scene_->reset_simulation();
                            static_dirty_ = true;
                            clear_manifold_state_();
                        }

                        if (count != last_count_) {
                            static_dirty_ = true;
                            clear_manifold_state_();
                        }
                        ensure_capacity_(count);
                        prepare_manifold_lookup_();

                        accumulate_forces(view.velocity, view.inv_mass, gravity, dt);
                        integrate_predicted_positions(view.position, view.velocity, view.inv_mass, dt);
                        integrate_predicted_orientations(view.orientation, view.angular_velocity, view.inv_mass, dt);
                        f32 max_angular_speed_sq = 0.0f;
                        for (u32 i = 0; i < count; ++i) {
                            max_angular_speed_sq = std::max(max_angular_speed_sq, view.angular_velocity[i].length_sq());
                        }
                        TracyPlot("physics_max_angular_speed", std::sqrt(max_angular_speed_sq));
                        bounds_cache_.resize(count);
                        for (u32 i = 0; i < count; ++i) {
#ifndef NDEBUG
                            if (view.shape_index[i] >= view.shapes.size()) {
                                log::error(physics, "Shape index out of range (id={} shape_id={})", i,
                                           view.shape_index[i]);
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

                        if (static_dirty_) {
                            rebuild_body_sets_(view);
                            last_count_ = count;
                            static_dirty_ = false;
                        }

                        const std::span<const u32> dynamic_ids{dynamic_ids_.data(), dynamic_ids_.size()};
                        // Mutating phase: update dynamic BVH before read-only queries.
                        broad_phase_update_dynamic_bvh(dynamic_ids, dynamic_bvh_, bounds_cache_);
                        // Read-only phase: query broad phase pairs.
                        run_broad_phase_queries_(dynamic_ids);
                        TracyPlot("physics_pairs", static_cast<i64>(candidate_pairs_.size()));
                        narrow_phase_contacts(view.position, view.orientation, view.shape_kind, view.shapes,
                                              view.shape_index, view.inv_mass, candidate_pairs_, next_manifolds_);
                        manifolds_.swap(next_manifolds_);
                        build_legacy_contacts_from_manifolds_(view.orientation);
                        TracyPlot("physics_contacts", static_cast<i64>(legacy_contacts_.size()));
                        solve_contacts(view.position, view.velocity, view.angular_velocity, view.inv_mass,
                                       view.inv_inertia, view.orientation, legacy_contacts_, restitution, friction);
                        apply_angular_damping(view.angular_velocity, view.inv_mass, angular_damping, dt);
                        publish_poses(view.poses, view.position, view.orientation, count);
                    }
                }

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
        thread_.join();
        stop_broad_phase_workers_();
    }

  private:
    struct BroadPhaseWorker final {
        BroadPhaseScratch scratch{};
        std::vector<BodyPair> pairs{};

        void reserve(const u32 count, const u32 query_stack_factor, const u32 pair_factor) {
            scratch.reserve(count, query_stack_factor);
            pairs.reserve(static_cast<usize>(count) * pair_factor);
        }
    };

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
    std::atomic<f32> angular_damping_{0.4f};
    std::atomic<f32> last_tick_dt_ms_{0.0f};
    std::atomic<bool> reset_requested_{false};
    bool static_dirty_{true};
    u32 last_count_{0};
    u32 capacity_{0};
    static constexpr u32 kQueryStackReserveFactor = 2;
    static constexpr u32 kPairReserveFactor = 8;
    static constexpr u32 kManifoldReserveFactor = 4;
    static constexpr u32 kLegacyContactReserveFactor = kManifoldReserveFactor * kMaxManifoldPoints;
    DynamicBvh dynamic_bvh_{};
    StaticBvh static_bvh_{};
    std::vector<BodyPair> candidate_pairs_{};
    std::vector<ContactManifold> manifolds_{};
    std::vector<ContactManifold> next_manifolds_{};
    std::unordered_map<u64, u32> manifold_lookup_{};
    // Transitional single-point contacts for the legacy narrow-phase/solver path.
    std::vector<LegacyContact> legacy_contacts_{};
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
        candidate_pairs_.reserve(static_cast<usize>(count) * kPairReserveFactor);
        const usize manifold_reserve = static_cast<usize>(count) * kManifoldReserveFactor;
        manifolds_.reserve(manifold_reserve);
        next_manifolds_.reserve(manifold_reserve);
        manifold_lookup_.reserve(manifold_reserve);
        legacy_contacts_.reserve(static_cast<usize>(count) * kLegacyContactReserveFactor);
        ensure_broad_phase_workers_(count);
    }

    void clear_manifold_state_() {
        manifolds_.clear();
        next_manifolds_.clear();
        manifold_lookup_.clear();
    }

    void prepare_manifold_lookup_() {
        next_manifolds_.clear();
        manifold_lookup_.clear();
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

    void build_legacy_contacts_from_manifolds_(std::span<const Quat> orientation) {
        legacy_contacts_.clear();
        for (const ContactManifold &manifold : manifolds_) {
#ifndef NDEBUG
            if (manifold.point_count > kMaxManifoldPoints) {
                log::error(physics, "Invalid manifold point_count={} (a={} b={})", manifold.point_count, manifold.a,
                           manifold.b);
                std::terminate();
            }
#endif
            const bool has_b = manifold.b != kInvalidBody;
            for (u32 i = 0; i < manifold.point_count; ++i) {
                const ContactPoint &point = manifold.points[i];
                const Vec3 r_a = rotate(orientation[manifold.a], point.local_anchor_a);
                const Vec3 r_b = has_b ? rotate(orientation[manifold.b], point.local_anchor_b) : Vec3{};
                legacy_contacts_.push_back(LegacyContact{
                    .a = manifold.a,
                    .b = manifold.b,
                    .normal = manifold.normal,
                    .penetration = std::max(-point.separation, 0.0f),
                    .r_a = r_a,
                    .r_b = r_b,
                });
            }
        }
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

#if defined(JAVELIN_BROAD_PHASE_VALIDATE)
        validate_broad_phase_pairs_(dynamic_ids);
#endif
    }

    void run_broad_phase_chunk_(const BroadPhaseJob &job, const u32 worker_index) {
        if (worker_index >= job.worker_count) {
            return;
        }
        const u32 begin = worker_index * job.chunk_size;
        if (begin >= job.dynamic_count) {
            broad_phase_workers_[worker_index].pairs.clear();
            return;
        }
        const u32 end = std::min(begin + job.chunk_size, job.dynamic_count);
        const std::span<const u32> chunk{job.dynamic_ids.data() + begin, end - begin};
        BroadPhaseWorker &worker = broad_phase_workers_[worker_index];
        broad_phase_sphere_pairs(chunk, dynamic_bvh_, static_bvh_, bounds_cache_, worker.pairs, worker.scratch);
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
        broad_phase_sphere_pairs(dynamic_ids, dynamic_bvh_, static_bvh_, bounds_cache_, expected, worker.scratch);

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
