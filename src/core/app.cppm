module;

#include <tracy/Tracy.hpp>

export module javelin.core.app;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.physics.physics_system;
import javelin.platform;
import javelin.render.render_system;
import javelin.scene;

namespace javelin::detail {

[[nodiscard]] std::filesystem::path
offline_frame_path(const std::filesystem::path &output_dir,
                   const u32 frame_index) {
  return output_dir / std::format("frame_{:06}.ppm", frame_index);
}

[[nodiscard]] std::expected<void, std::string>
ensure_output_dir(const std::filesystem::path &output_dir) {
  std::error_code ec{};
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    return std::unexpected(
        std::format("Unable to create output directory '{}': {}",
                    output_dir.string(), ec.message()));
  }
  return {};
}

[[nodiscard]] std::expected<void, std::string>
write_ppm_frame(const std::filesystem::path &path, const i32 width,
                const i32 height, std::span<const u8> rgba) {
  std::ofstream out{path, std::ios::binary | std::ios::trunc};
  if (!out.is_open()) {
    return std::unexpected(
        std::format("Unable to open frame output '{}'", path.string()));
  }

  out << "P6\n" << width << ' ' << height << "\n255\n";

  const usize row_rgba_size = static_cast<usize>(width) * 4u;
  std::vector<char> row_rgb(static_cast<usize>(width) * 3u);
  for (i32 y = height - 1; y >= 0; --y) {
    const usize src_row = static_cast<usize>(y) * row_rgba_size;
    for (i32 x = 0; x < width; ++x) {
      const usize src = src_row + static_cast<usize>(x) * 4u;
      const usize dst = static_cast<usize>(x) * 3u;
      row_rgb[dst + 0u] = static_cast<char>(rgba[src + 0u]);
      row_rgb[dst + 1u] = static_cast<char>(rgba[src + 1u]);
      row_rgb[dst + 2u] = static_cast<char>(rgba[src + 2u]);
    }
    out.write(row_rgb.data(), static_cast<std::streamsize>(row_rgb.size()));
    if (!out) {
      return std::unexpected(
          std::format("I/O error while writing '{}'", path.string()));
    }
  }

  return {};
}

[[nodiscard]] std::expected<void, std::string>
write_ffmpeg_hint(const std::filesystem::path &output_dir, const u32 fps) {
  const auto hint_path = output_dir / "ffmpeg_command.txt";
  std::ofstream out{hint_path, std::ios::trunc};
  if (!out.is_open()) {
    return std::unexpected(
        std::format("Unable to open hint file '{}'", hint_path.string()));
  }

  out << "ffmpeg -framerate " << fps
      << " -i frame_%06d.ppm -c:v libx264 -pix_fmt yuv420p output.mp4\n";
  if (!out) {
    return std::unexpected(
        std::format("I/O error while writing '{}'", hint_path.string()));
  }
  return {};
}

} // namespace javelin::detail

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
    scene = Scene::load_scene_from_disk(scene_path);

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

    const auto dir_result = detail::ensure_output_dir(config.output_dir);
    if (!dir_result) {
      log::critical(app, "Offline render failed: {}", dir_result.error());
    }
    const auto hint_result = detail::write_ffmpeg_hint(
        config.output_dir, PhysicsSystem::fixed_step_hz());
    if (!hint_result) {
      log::critical(app, "Offline render failed: {}", hint_result.error());
    }

    platform.init(PlatformConfig{
        .width = config.width,
        .height = config.height,
        .visible = false,
    });
    scene = Scene::load_scene_from_disk(scene_path);

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

    std::vector<u8> frame_rgba(static_cast<usize>(extent.width) *
                               static_cast<usize>(extent.height) *
                               static_cast<usize>(4u));
    const double fixed_dt =
        static_cast<double>(PhysicsSystem::fixed_step_dt_seconds());

    for (u32 frame_index = 0u; frame_index < config.frame_count;
         ++frame_index) {
      const double render_dt = (frame_index == 0u) ? 0.0 : fixed_dt;
      renderer.render_offline_frame(render_dt, 1.0f);

      if (!renderer.readback_rgba8(frame_rgba)) {
        log::critical(app, "Offline render failed to read back frame {}",
                      frame_index);
      }

      const auto frame_path =
          detail::offline_frame_path(config.output_dir, frame_index);
      const auto write_result = detail::write_ppm_frame(
          frame_path, extent.width, extent.height, frame_rgba);
      if (!write_result) {
        log::critical(app, "Offline render failed: {}", write_result.error());
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
