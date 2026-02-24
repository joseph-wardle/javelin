import std;

import javelin.scene;
import javelin.scene.scene_file;

using namespace javelin;

namespace {
[[nodiscard]] std::expected<std::string, std::string> read_file_text(const std::filesystem::path &path) {
    std::ifstream in{path, std::ios::binary};
    if (!in.is_open()) {
        return std::unexpected(std::format("Unable to open file '{}'", path.string()));
    }

    in.seekg(0, std::ios::end);
    const std::streampos end = in.tellg();
    if (end < 0) {
        return std::unexpected(std::format("Unable to size file '{}'", path.string()));
    }
    std::string text(static_cast<std::size_t>(end), '\0');
    in.seekg(0, std::ios::beg);
    if (!text.empty()) {
        in.read(text.data(), static_cast<std::streamsize>(text.size()));
    }
    if (!in) {
        return std::unexpected(std::format("I/O error while reading '{}'", path.string()));
    }
    return text;
}

[[nodiscard]] std::filesystem::path make_temp_scene_path(const std::string_view label) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::string name = std::format("javelin_scene_tool_{}_{}.jvscene", label, now);
    return std::filesystem::temp_directory_path() / name;
}

void print_usage() {
    std::cerr << "Usage:\n";
    std::cerr << "  javelin_scene_tool normalize <input.jvscene> <output.jvscene>\n";
    std::cerr << "  javelin_scene_tool verify-roundtrip <input.jvscene>\n";
    std::cerr << "  javelin_scene_tool scene-export <input.jvscene> <output.jvscene>\n";
}

int normalize_scene(const std::filesystem::path &input, const std::filesystem::path &output) {
    const auto scene = SceneFile::load(input);
    if (!scene) {
        std::cerr << format_scene_file_error(scene.error()) << '\n';
        return 1;
    }
    const auto saved = scene->save(output);
    if (!saved) {
        std::cerr << format_scene_file_error(saved.error()) << '\n';
        return 1;
    }
    std::cout << "Normalized scene: " << input.string() << " -> " << output.string() << '\n';
    return 0;
}

int verify_roundtrip(const std::filesystem::path &input) {
    const auto loaded = SceneFile::load(input);
    if (!loaded) {
        std::cerr << format_scene_file_error(loaded.error()) << '\n';
        return 1;
    }

    const std::filesystem::path canonical_a = make_temp_scene_path("canonical_a");
    const std::filesystem::path canonical_b = make_temp_scene_path("canonical_b");

    const auto saved_a = loaded->save(canonical_a);
    if (!saved_a) {
        std::cerr << format_scene_file_error(saved_a.error()) << '\n';
        return 1;
    }

    const auto reloaded = SceneFile::load(canonical_a);
    if (!reloaded) {
        std::cerr << format_scene_file_error(reloaded.error()) << '\n';
        return 1;
    }
    const auto saved_b = reloaded->save(canonical_b);
    if (!saved_b) {
        std::cerr << format_scene_file_error(saved_b.error()) << '\n';
        return 1;
    }

    const auto text_a = read_file_text(canonical_a);
    if (!text_a) {
        std::cerr << text_a.error() << '\n';
        return 1;
    }
    const auto text_b = read_file_text(canonical_b);
    if (!text_b) {
        std::cerr << text_b.error() << '\n';
        return 1;
    }

    if (*text_a != *text_b) {
        std::cerr << "Roundtrip is not deterministic for '" << input.string() << "'\n";
        std::cerr << "  first : " << canonical_a.string() << '\n';
        std::cerr << "  second: " << canonical_b.string() << '\n';
        return 2;
    }

    std::error_code ec{};
    std::filesystem::remove(canonical_a, ec);
    std::filesystem::remove(canonical_b, ec);
    std::cout << "Roundtrip deterministic: " << input.string() << '\n';
    return 0;
}

int scene_export(const std::filesystem::path &input, const std::filesystem::path &output) {
    Scene scene = Scene::load_scene_from_disk(input);
    const auto saved = scene.save_scene_to_disk(output);
    if (!saved) {
        std::cerr << format_scene_file_error(saved.error()) << '\n';
        return 1;
    }
    std::cout << "Exported scene via runtime contract: " << input.string() << " -> " << output.string() << '\n';
    return 0;
}
} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argv[1] == nullptr) {
        print_usage();
        return 1;
    }

    const std::string_view command = argv[1];
    if (command == "normalize") {
        if (argc != 4 || argv[2] == nullptr || argv[3] == nullptr) {
            print_usage();
            return 1;
        }
        return normalize_scene(argv[2], argv[3]);
    }
    if (command == "verify-roundtrip") {
        if (argc != 3 || argv[2] == nullptr) {
            print_usage();
            return 1;
        }
        return verify_roundtrip(argv[2]);
    }
    if (command == "scene-export") {
        if (argc != 4 || argv[2] == nullptr || argv[3] == nullptr) {
            print_usage();
            return 1;
        }
        return scene_export(argv[2], argv[3]);
    }

    print_usage();
    return 1;
}
