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
                        const f32 dt = 1.0f / 60.0f;
                        const f32 gravity = gravity_.load(std::memory_order_relaxed);
                        const f32 restitution = restitution_.load(std::memory_order_relaxed);
                        const f32 friction = friction_.load(std::memory_order_relaxed);

                        if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
                            scene_->reset_simulation();
                            static_dirty_ = true;
                        }

                        accumulate_forces(view.velocity, view.inv_mass, gravity, dt);
                        integrate_predicted_positions(view.position, view.velocity, view.inv_mass, dt);
                        bounds_cache_.resize(view.count);
                        for (u32 i = 0; i < view.count; ++i) {
                            bounds_cache_[i] = Aabb::from_sphere(view.position[i], view.sphere[i].radius);
                        }

                        if (static_dirty_) {
                            static_ids_.clear();
                            for (u32 i = 0; i < view.count; ++i) {
                                if (view.inv_mass[i] == 0.0f) {
                                    static_ids_.push_back(i);
                                }
                            }
                            if (!static_ids_.empty()) {
                                static_bvh_.build(static_ids_, bounds_cache_);
                            } else {
                                static_bvh_.clear();
                            }
                            static_dirty_ = false;
                        }

                        broad_phase_sphere_pairs(view.position, view.velocity, view.inv_mass, dt, dynamic_bvh_,
                                                 static_bvh_, bounds_cache_, candidate_pairs_, query_hits_);
                        narrow_phase_contacts(view.position, view.sphere, view.inv_mass, candidate_pairs_, contacts_);
                        solve_contacts(view.position, view.velocity, view.inv_mass, contacts_, restitution, friction);
                        publish_poses(view.poses, view.position, view.count);
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
    Scene *scene_{nullptr};
    std::jthread thread_{};
    std::atomic<f32> gravity_{-9.8f};
    std::atomic<f32> restitution_{0.3f};
    std::atomic<f32> friction_{0.2f};
    std::atomic<bool> reset_requested_{false};
    bool static_dirty_{true};
    DynamicBvh dynamic_bvh_{};
    StaticBvh static_bvh_{};
    std::vector<BodyPair> candidate_pairs_{};
    std::vector<Contact> contacts_{};
    std::vector<u32> query_hits_{};
    std::vector<u32> static_ids_{};
    std::vector<Aabb> bounds_cache_{};
};

} // namespace javelin
