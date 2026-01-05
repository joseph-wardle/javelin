module;

#include <tracy/Tracy.hpp>

export module javelin.render.render_system;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.scene;
import javelin.platform.window;

export namespace javelin {

    struct RenderSystem final {
        void init(const Scene& scene) noexcept {
            scene_ = &scene;
        }

        void start(const WindowHandle window) noexcept {
            if (thread_.joinable()) return; // already running

            window_ = window;

            thread_ = std::jthread([this](const std::stop_token& st) {
                tracy::SetThreadName("Render");

                // ZoneScopedN("Render thread");

                // - glfwMakeContextCurrent(window_.native);
                // - gladLoadGLLoader(...);
                // - FrameMark; after swap/present (canonical)

                using clock = std::chrono::steady_clock;
                auto prev = clock::now();

                while (!st.stop_requested()) {
                    ZoneScopedN("Render frame");

                    const auto now = clock::now();
                    const f64 dt = std::chrono::duration<f64>(now - prev).count();
                    prev = now;

                    if (dt > 0.0) {
                        TracyPlot("render_fps", 1.0 / dt);
                        TracyPlot("render_dt_ms", dt * 1000.0);
                    }

                    log::info("Render tick");

                    FrameMark;
                }

                // glfwMakeContextCurrent(nullptr);
            });
        }

        void stop() noexcept {
            if (!thread_.joinable()) return;
            thread_.request_stop();
            thread_.join();
        }

    private:
        const Scene* scene_{nullptr};
        WindowHandle window_{};
        std::jthread thread_{};
    };

} // namespace javelin
