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
        FixedStepClock clock;

        void init(const std::filesystem::path& scene_path) {
            platform.init();
            renderer.init();
            physics.init();

            scene = load_scene_from_disk(scene_path);

            physics.build_runtime(scene);
            renderer.build_runtime(scene);
        }

        void run() {
            while (!platform.quit_requested()) {
                platform.poll_events();
                // scene.presentation.camera.publish(platform.camera_state()); gonna need better input handling

                clock.advance(platform.time_seconds());

                while (clock.consume_step()) {
                    physics.step(clock.delta());
                    //physics.publish_snapshot(scene.presentation); // needs to be atomic
                }

                shutdown();
            }
        }

        void shutdown() {
            renderer.shutdown();
            physics.shutdown();
            platform.shutdown();
        }
    };
}
