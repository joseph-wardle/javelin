import std;

import javelin.core.app;
import javelin.core.types;

using namespace javelin;

namespace {
constexpr std::string_view kDefaultScenePath =
    "assets/scenes/samples/stack_boxes.jvscene";

void print_usage(const char *exe, const OfflineRenderConfig &offline_defaults) {
  std::println("Javelin sandbox");
  std::println("Usage:");
  std::println("  {} [scene.jvscene]", exe);
  std::println("  {} offline-render [scene.jvscene] [--frames=N] "
               "[--output-dir=DIR] [--width=N] [--height=N]",
               exe);
  std::println("");
  std::println("Offline render defaults:");
  std::println("  scene={}", kDefaultScenePath);
  std::println("  --frames={}", offline_defaults.frame_count);
  std::println("  --output-dir={}", offline_defaults.output_dir.string());
  std::println("  --width={}", offline_defaults.width);
  std::println("  --height={}", offline_defaults.height);
}

[[nodiscard]] bool parse_u32_arg(const std::string_view text, u32 &value) {
  const char *const begin = text.data();
  const char *const end = text.data() + text.size();
  auto [ptr, ec] = std::from_chars(begin, end, value);
  return ec == std::errc{} && ptr == end;
}

[[nodiscard]] bool parse_i32_arg(const std::string_view text, i32 &value) {
  const char *const begin = text.data();
  const char *const end = text.data() + text.size();
  auto [ptr, ec] = std::from_chars(begin, end, value);
  return ec == std::errc{} && ptr == end;
}

[[nodiscard]] bool split_key_value_arg(const std::string_view arg,
                                       std::string_view &key,
                                       std::string_view &value) {
  if (!arg.starts_with("--")) {
    return false;
  }

  const usize eq = arg.find('=');
  if (eq == std::string_view::npos || eq <= 2u || eq + 1u >= arg.size()) {
    return false;
  }

  key = arg.substr(2u, eq - 2u);
  value = arg.substr(eq + 1u);
  return true;
}

[[nodiscard]] bool parse_offline_arg(const std::string_view arg,
                                     OfflineRenderConfig &config) {
  std::string_view key{};
  std::string_view value{};
  if (!split_key_value_arg(arg, key, value)) {
    return false;
  }

  if (key == "frames") {
    return parse_u32_arg(value, config.frame_count);
  }
  if (key == "output-dir") {
    config.output_dir = value;
    return !config.output_dir.empty();
  }
  if (key == "width") {
    return parse_i32_arg(value, config.width);
  }
  if (key == "height") {
    return parse_i32_arg(value, config.height);
  }

  return false;
}
} // namespace

int main(int argc, char **argv) {
  const OfflineRenderConfig offline_defaults{};

  if (argc > 1 && argv[1] != nullptr) {
    const std::string_view first_arg{argv[1]};
    if (first_arg == "-h" || first_arg == "--help") {
      print_usage(argv[0], offline_defaults);
      return 0;
    }
    if (first_arg == "offline-render") {
      OfflineRenderConfig config = offline_defaults;
      std::filesystem::path scene_path =
          std::filesystem::path{kDefaultScenePath};
      bool scene_explicit = false;

      for (int i = 2; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (arg == "-h" || arg == "--help") {
          print_usage(argv[0], config);
          return 0;
        }
        if (arg.starts_with("--")) {
          if (!parse_offline_arg(arg, config)) {
            std::cerr << "Unrecognized offline-render argument: " << arg
                      << "\n";
            print_usage(argv[0], offline_defaults);
            return 1;
          }
          continue;
        }
        if (!scene_explicit) {
          scene_path = arg;
          scene_explicit = true;
          continue;
        }

        std::cerr << "Too many positional arguments for offline-render.\n";
        print_usage(argv[0], offline_defaults);
        return 1;
      }

      if (config.frame_count == 0u) {
        std::cerr << "--frames must be > 0.\n";
        return 1;
      }
      if (config.width <= 0 || config.height <= 0) {
        std::cerr << "--width and --height must both be > 0.\n";
        return 1;
      }
      if (config.output_dir.empty()) {
        std::cerr << "--output-dir must not be empty.\n";
        return 1;
      }

      App app{};
      app.render_offline(scene_path, config);
      return 0;
    }
  }

  std::filesystem::path scene_path = std::filesystem::path{kDefaultScenePath};
  if (argc > 1 && argv[1] != nullptr) {
    scene_path = argv[1];
  }

  App app{};
  app.run(scene_path);

  return 0;
}
