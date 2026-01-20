module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
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
                        const f32 reset_y = reset_y_.load(std::memory_order_relaxed);
                        const f32 spawn_y = spawn_y_.load(std::memory_order_relaxed);

                        // TEMP: simplified falling test for render/physics plumbing.
                        for (u32 i = 0; i < view.count; ++i) {
                            view.velocity[i].y += gravity * dt;
                            view.position[i].y += view.velocity[i].y * dt;

                            if (view.position[i].y < reset_y) {
                                view.position[i].y = spawn_y;
                                view.velocity[i].y = 0.0f;
                            }
                        }

                        auto out = view.poses.write_positions(view.count);
                        for (u32 i = 0; i < view.count; ++i) {
                            out[i] = view.position[i];
                        }
                        view.poses.publish();
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
    std::atomic<f32> reset_y_{-10.0f};
    std::atomic<f32> spawn_y_{10.0f};
};

} // namespace javelin
