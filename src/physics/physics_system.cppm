module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
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
    void set_reset_y(const f32 reset_y) noexcept { reset_y_.store(reset_y, std::memory_order_relaxed); }
    void set_spawn_y(const f32 spawn_y) noexcept { spawn_y_.store(spawn_y, std::memory_order_relaxed); }

    [[nodiscard]] f32 gravity() const noexcept { return gravity_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 reset_y() const noexcept { return reset_y_.load(std::memory_order_relaxed); }
    [[nodiscard]] f32 spawn_y() const noexcept { return spawn_y_.load(std::memory_order_relaxed); }

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }
        if (scene_ == nullptr) {
            log::warn(physics, "Starting without scene bound");
        }

        log::info(physics, "Starting physics system");
        log::info(physics, "Params gravity={} reset_y={} spawn_y={}", gravity(), reset_y(), spawn_y());
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

                        accumulate_forces(view.velocity, view.inv_mass, gravity, dt);
                        integrate_predicted_positions(view.position, view.velocity, view.inv_mass, dt);
                        broad_phase_sphere_pairs(view.count, candidate_pairs_);
                        narrow_phase_contacts(view.position, view.sphere, view.inv_mass, candidate_pairs_, contacts_);
                        solve_contacts(view.position, view.velocity, view.inv_mass, contacts_, restitution);
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
    std::atomic<f32> reset_y_{-10.0f};
    std::atomic<f32> spawn_y_{10.0f};
    std::vector<BodyPair> candidate_pairs_{};
    std::vector<Contact> contacts_{};
};

} // namespace javelin
