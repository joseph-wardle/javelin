module;

#include <tracy/Tracy.hpp>

export module javelin.core.app;

import std;
import javelin.physics.physics_system;
import javelin.platform;
import javelin.render.render_system;
import javelin.scene;

export namespace javelin {

struct App final {
    Platform platform{};
    RenderSystem renderer{};
    PhysicsSystem physics{};
    Scene scene{};

    void run(const std::filesystem::path &scene_path) {
        tracy::SetThreadName("Main");
        ZoneScoped;

        platform.init();
        scene = Scene::load_scene_from_disk(scene_path);

        renderer.init_cpu(scene);
        renderer.init_gpu(platform.window_handle());

        physics.init(scene);
        physics.start();

        using clock = std::chrono::steady_clock;
        auto prev = clock::now();

        while (!platform.quit_requested()) {
            ZoneScopedN("Frame");

            platform.poll_events();

            const auto now = clock::now();
            const double dt = std::chrono::duration<double>(now - prev).count();
            prev = now;

            // TODO: build Actions from input (and optionally push commands/settings to physics)
            // actions = input.map(platform.input_state());

            renderer.render_frame(dt);

            FrameMark;
        }

        physics.stop();
        renderer.shutdown();
        platform.shutdown();
    }
};

} // namespace javelin
