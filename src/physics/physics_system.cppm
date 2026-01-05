module;

#include <tracy/Tracy.hpp>
#include <immintrin.h> // _mm_pause

export module javelin.physics.physics_system;

import std;
import javelin.core.logging;
import javelin.scene;

export namespace javelin {

    struct PhysicsSystem final {
        void init(Scene& scene) noexcept { scene_ = &scene; }

        void start() {
            if (thread_.joinable()) return;

            thread_ = std::jthread([this](std::stop_token st) {
                tracy::SetThreadName("Physics");
                // ZoneScopedN("Physics thread");

                using clock = std::chrono::steady_clock;

                const auto step = std::chrono::duration_cast<clock::duration>(
                    std::chrono::duration<double>(fixed_dt_seconds_)
                );

                auto next = clock::now();
                auto prev = next;

                while (!st.stop_requested()) {
                    next += step;

                    while (clock::now() < next) {
                        if (st.stop_requested()) return;
                        _mm_pause(); // busy-wait hint (x86_64)
                    }

                    const auto now = clock::now();
                    const double wall_dt = std::chrono::duration<double>(now - prev).count();
                    prev = now;

                    if (wall_dt > 0.0) TracyPlot("physics_hz", 1.0 / wall_dt);

                    {
                        ZoneScopedN("Physics tick");
                        // TODO: drain actions
                        // TODO: step simulation
                        // TODO: publish snapshot
                        log::info("Physics tick");
                    }

                    FrameMarkNamed("Physics");
                }
            });
        }

        void stop() noexcept {
            if (!thread_.joinable()) return;
            thread_.request_stop();
            thread_.join();
        }

    private:
        Scene* scene_{nullptr};
        double fixed_dt_seconds_{1.0 / 60.0};
        std::jthread thread_{};
    };

} // namespace javelin
