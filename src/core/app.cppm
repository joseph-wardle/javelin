module;

#include <tracy/Tracy.hpp>

export module javelin.core.app;

import std;
import javelin.core.time;
import javelin.physics.physics_system;
import javelin.platform;
import javelin.render.render_system;
import javelin.scene;

export namespace javelin {
    struct App {
        Platform platform;
        RenderSystem renderer;
        PhysicsSystem physics;
        Scene scene;

        void run(const std::filesystem::path& scene_path) {
            ZoneScoped;
            platform.init();

            scene = load_scene_from_disk(scene_path);

            renderer.init(scene);                      // CPU only
            renderer.start(platform.window_handle());  // spawn render thread

            physics.init(scene);  // CPU-only
            physics.start();      // spawn physics thread

            while (!platform.quit_requested()) {
                ZoneScopedN("Main");
                platform.poll_events();

                // build actions here later

                FrameMarkNamed("App");
            }

            physics.stop();
            renderer.stop();
            platform.shutdown();
        }
    };
}
