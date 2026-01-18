module;

#include <tracy/Tracy.hpp>

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.core.time;
import javelin.core.types;
import javelin.scene;

export namespace javelin {

struct PhysicsSystem final {
    void init(Scene &scene) noexcept { scene_ = &scene; }

    void start() {
        if (thread_.joinable()) {
            log::warn(physics, "Start ignored (already running)");
            return;
        }

        log::info(physics, "Starting physics system");
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
                    // step_simulation_fixed(1.0 / 60.0);
                    // publish_snapshot();
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
};

} // namespace javelin
