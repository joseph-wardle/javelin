export module javelin.bench.json;

import std;

import javelin.core.types;

namespace javelin::bench {

export [[nodiscard]] bool open_json_output(const std::string_view output_path, std::ofstream &stream,
                                           std::ostream &error_stream = std::cerr) {
    namespace fs = std::filesystem;

    const fs::path path{output_path};
    std::error_code error{};
    if (path.has_parent_path()) {
        fs::create_directories(path.parent_path(), error);
        if (error) {
            error_stream << "Failed to create directory '" << path.parent_path().string()
                         << "' for JSON output: " << error.message() << '\n';
            return false;
        }
    }

    stream = std::ofstream{path};
    if (!stream.is_open()) {
        error_stream << "Failed to open JSON output file: " << path.string() << '\n';
        return false;
    }

    stream << std::setprecision(17);
    return true;
}

export void write_json_f64_array(std::ostream &stream, const std::span<const f64> values) {
    stream << "[";
    for (usize i = 0u; i < values.size(); ++i) {
        if (i > 0u) {
            stream << ", ";
        }
        stream << values[i];
    }
    stream << "]";
}

} // namespace javelin::bench
