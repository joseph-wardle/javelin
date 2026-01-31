module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
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

export namespace javelin {
struct PhysicsSystem final {
    void init(Scene &scene) noexcept { scene_ = &scene; }

    void set_gravity(const f32 gravity) noexcept { gravity_.store(gravity, std::memory_order_relaxed); }
    void set_restitution(const f32 restitution) noexcept { restitution_.store(restitution, std::memory_order_relaxed); }
    void set_friction(const f32 friction) noexcept { friction_.store(friction, std::memory_order_relaxed); }
    void request_reset() noexcept { reset_requested_.store(true, std::memory_order_release); }

    [[nodiscard]] f32 gravity() const noexcept { return gravity_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 restitution() const noexcept { return restitution_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 friction() const noexcept { return friction_.load(std::memory_order_relaxed); }

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }
        if (scene_ == nullptr) {
            log::warn(physics, "Starting without scene bound");
        }

        log::info(physics, "Starting physics system");
        log::info(physics, "Params gravity={} restitution={} friction={}", gravity(), restitution(), friction());
        thread_ = std::jthread([this](const std::stop_token &stop_token) {
            tracy::SetThreadName("Physics");

            constexpr auto delta_time =
                std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<f64>(1.0 / 60.0));
            FixedRateTicker ticker{delta_time};

            while (!stop_token.stop_requested()) {
                const auto t = ticker.wait_next(stop_token);

                TracyPlot("physics_tick_interval_error_us", t.interval_error_us);

                {
                    ZoneScopedN("Physics tick");
                    if (scene_ != nullptr) {
                        PhysicsView view = scene_->physics_view();
                        const u32 count = view.count;
                        const f32 dt = 1.0f / 60.0f;
                        const f32 gravity = gravity_.load(std::memory_order_relaxed);
                        const f32 restitution = restitution_.load(std::memory_order_relaxed);
                        const f32 friction = friction_.load(std::memory_order_relaxed);

                        if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
                            scene_->reset_simulation();
                            static_dirty_ = true;
                        }

                        if (count != last_count_) {
                            static_dirty_ = true;
                        }
                        ensure_capacity_(count);

                        accumulate_forces(view.velocity, view.inv_mass, gravity, dt);
                        integrate_predicted_positions(view.position, view.velocity, view.inv_mass, dt);
                        bounds_cache_.resize(count);
                        for (u32 i = 0; i < count; ++i) {
                            bounds_cache_[i] = Aabb::from_sphere(view.position[i], view.sphere[i].radius);
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
                        narrow_phase_contacts(view.position, view.sphere, view.inv_mass, candidate_pairs_, contacts_);
                        solve_contacts(view.position, view.velocity, view.inv_mass, contacts_, restitution, friction);
                        publish_poses(view.poses, view.position, count);
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

    Scene *scene_{nullptr};
    std::jthread thread_{};
    std::atomic<f32> gravity_{-9.8f};
    std::atomic<f32> restitution_{0.3f};
    std::atomic<f32> friction_{0.2f};
    std::atomic<bool> reset_requested_{false};
    bool static_dirty_{true};
    u32 last_count_{0};
    u32 capacity_{0};
    static constexpr u32 kQueryStackReserveFactor = 2;
    static constexpr u32 kPairReserveFactor = 8;
    static constexpr u32 kContactReserveFactor = 4;
    DynamicBvh dynamic_bvh_{};
    StaticBvh static_bvh_{};
    std::vector<BodyPair> candidate_pairs_{};
    std::vector<Contact> contacts_{};
    u32 broad_phase_worker_count_{0};
    std::vector<BroadPhaseWorker> broad_phase_workers_{};
    std::vector<std::thread> broad_phase_threads_{};
    std::vector<usize> broad_phase_pair_offsets_{};
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
        contacts_.reserve(static_cast<usize>(count) * kContactReserveFactor);
        ensure_broad_phase_workers_(count);
    }

    void ensure_broad_phase_workers_(const u32 count) {
        if (broad_phase_worker_count_ == 0) {
            const u32 hw_threads = std::thread::hardware_concurrency();
            broad_phase_worker_count_ = (hw_threads > 0) ? hw_threads : 1u;
            broad_phase_workers_.resize(broad_phase_worker_count_);
            broad_phase_threads_.reserve(broad_phase_worker_count_ - 1u);
            broad_phase_pair_offsets_.reserve(static_cast<usize>(broad_phase_worker_count_) + 1u);
        }
        for (auto &worker : broad_phase_workers_) {
            worker.reserve(count, kQueryStackReserveFactor, kPairReserveFactor);
        }
    }

    void run_broad_phase_queries_(std::span<const u32> dynamic_ids) {
        candidate_pairs_.clear();
        const u32 dynamic_count = static_cast<u32>(dynamic_ids.size());
        if (dynamic_count == 0) {
            return;
        }

        const u32 worker_count = std::min(broad_phase_worker_count_, dynamic_count);
        const u32 chunk_size = (dynamic_count + worker_count - 1u) / worker_count;

        auto run_chunk = [&](const u32 worker_index) {
            const u32 begin = worker_index * chunk_size;
            if (begin >= dynamic_count) {
                broad_phase_workers_[worker_index].pairs.clear();
                return;
            }
            const u32 end = std::min(begin + chunk_size, dynamic_count);
            const std::span<const u32> chunk{dynamic_ids.data() + begin, end - begin};
            BroadPhaseWorker &worker = broad_phase_workers_[worker_index];
            broad_phase_sphere_pairs(chunk, dynamic_bvh_, static_bvh_, bounds_cache_, worker.pairs, worker.scratch);
        };

        broad_phase_threads_.clear();
        for (u32 worker_index = 1; worker_index < worker_count; ++worker_index) {
            broad_phase_threads_.emplace_back(run_chunk, worker_index);
        }
        run_chunk(0);
        for (auto &thread : broad_phase_threads_) {
            thread.join();
        }

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
