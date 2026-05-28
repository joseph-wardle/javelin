module;

#include <tracy/Tracy.hpp>

export module javelin.core.app;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.physics.physics_system;
import javelin.platform;
import javelin.render.frame_exporter;
import javelin.render.render_system;
import javelin.scene;

export namespace javelin {

struct OfflineRenderConfig final {
  std::filesystem::path output_dir{"capture/offline"};
  i32 width{1920};
  i32 height{1080};
  u32 frame_count{600};
  f32 orbit_start_seconds{0.0f};
  bool draw_sleep_state{false};
};

struct App final {
  Platform platform{};
  RenderSystem renderer{};
  PhysicsSystem physics{};
  Scene scene{};

  void run(const std::filesystem::path &scene_path) {
    tracy::SetThreadName("Main");
    ZoneScoped;

    log::info(app, "Start scene={}", scene_path.string());

    platform.init(PlatformConfig{.fullscreen = true});
    auto loaded_scene = Scene::from_path(scene_path);
    if (!loaded_scene) {
      log::critical(app, "Failed to load scene: {}", loaded_scene.error().formatted());
    }
    scene = std::move(*loaded_scene);

    renderer.init_cpu(scene, physics);
    renderer.init_gpu(platform.window_handle());

    physics.init(scene);
    physics.start();

    using clock = std::chrono::steady_clock;
    auto prev = clock::now();

    while (!platform.quit_requested()) {
      ZoneScopedN("Frame");

      auto &input = platform.input_state();
      input.begin_frame();
      platform.poll_events();

      const auto now = clock::now();
      const double dt = std::chrono::duration<double>(now - prev).count();
      prev = now;

      // TODO: build Actions from input (and optionally push commands/settings
      // to physics) actions = input.map(platform.input_state());

      renderer.render_frame(dt, input);

      FrameMark;
    }

    physics.stop();
    renderer.shutdown();
    platform.shutdown();

    log::info(app, "Shutting down app");
  }

  void render_offline(const std::filesystem::path &scene_path,
                      const OfflineRenderConfig &config) {
    tracy::SetThreadName("Main");
    ZoneScoped;

    if (config.frame_count == 0u) {
      log::critical(app, "Offline render requires frame_count > 0");
    }
    if (config.width <= 0 || config.height <= 0) {
      log::critical(app,
                    "Offline render requires positive dimensions (got {}x{})",
                    config.width, config.height);
    }

    log::info(app, "Offline render scene={} frames={} size={}x{} output={}",
              scene_path.string(), config.frame_count, config.width,
              config.height, config.output_dir.string());

    platform.init(PlatformConfig{
        .width = config.width,
        .height = config.height,
        .visible = false,
    });
    auto loaded_scene = Scene::from_path(scene_path);
    if (!loaded_scene) {
      log::critical(app, "Failed to load scene: {}", loaded_scene.error().formatted());
    }
    scene = std::move(*loaded_scene);

    renderer.init_cpu(scene, physics);
    renderer.init_gpu(platform.window_handle(),
                      RenderGpuConfig{.ui_enabled = false,
                                      .fixed_world_camera = true,
                                      .recording_orbit_start_seconds =
                                          config.orbit_start_seconds,
                                      .draw_sleep_state =
                                          config.draw_sleep_state});

    physics.init(scene);

    const auto extent = renderer.extent();
    if (!extent.is_valid()) {
      log::critical(app, "Offline render framebuffer has invalid extent {}x{}",
                    extent.width, extent.height);
    }

    FrameExporter exporter{};
    if (auto r = exporter.init(config.output_dir, extent,
                               PhysicsSystem::fixed_step_hz());
        !r) {
      log::critical(app, "Offline render failed: {}", r.error());
    }

    const double fixed_dt =
        static_cast<double>(PhysicsSystem::fixed_step_dt_seconds());

    for (u32 frame_index = 0u; frame_index < config.frame_count;
         ++frame_index) {
      const double render_dt = (frame_index == 0u) ? 0.0 : fixed_dt;
      renderer.render_offline_frame(render_dt, 1.0f);

      if (auto r = exporter.capture(renderer, frame_index); !r) {
        log::critical(app, "Offline render failed: {}", r.error());
      }

      const u32 completed_frames = frame_index + 1u;
      if (completed_frames == config.frame_count ||
          (completed_frames % PhysicsSystem::fixed_step_hz()) == 0u) {
        log::info(app, "Offline render progress {}/{}", completed_frames,
                  config.frame_count);
      }

      if (completed_frames < config.frame_count && !physics.step_fixed()) {
        log::critical(
            app, "Offline render failed to advance fixed simulation step {}",
            completed_frames);
      }
    }

    physics.stop();
    renderer.shutdown();
    platform.shutdown();

    log::info(app, "Offline render complete frames={} output={} fps={}",
              config.frame_count, config.output_dir.string(),
              PhysicsSystem::fixed_step_hz());
  }
};

} // namespace javelin
