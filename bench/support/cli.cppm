export module javelin.bench.cli;

import std;

import javelin.core.types;

namespace javelin::bench {

export struct ParsedArg final {
    std::string_view key{};
    std::string_view value{};
};

export [[nodiscard]] bool is_help_arg(const std::string_view arg) noexcept { return arg == "--help" || arg == "-h"; }

export [[nodiscard]] bool split_key_value_arg(const std::string_view arg, ParsedArg &out) noexcept {
    if (!arg.starts_with("--")) {
        return false;
    }
    const usize eq = arg.find('=');
    if (eq == std::string_view::npos || eq <= 2u || eq + 1u >= arg.size()) {
        return false;
    }

    out.key = arg.substr(2u, eq - 2u);
    out.value = arg.substr(eq + 1u);
    return true;
}

export [[nodiscard]] bool parse_u32(const std::string_view text, u32 &out) noexcept {
    if (text.empty()) {
        return false;
    }

    u32 value = 0u;
    const char *begin = text.data();
    const char *end = begin + text.size();
    const std::from_chars_result result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return false;
    }

    out = value;
    return true;
}

export [[nodiscard]] bool parse_f32(const std::string_view text, f32 &out) {
    if (text.empty()) {
        return false;
    }

    std::string token{text};
    char *end = nullptr;
    const f32 value = std::strtof(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }

    out = value;
    return true;
}

export [[nodiscard]] bool parse_u32_csv(std::string_view text, std::vector<u32> &out) noexcept {
    out.clear();
    while (!text.empty()) {
        const usize comma = text.find(',');
        const std::string_view token = (comma == std::string_view::npos) ? text : text.substr(0u, comma);

        u32 value = 0u;
        if (!parse_u32(token, value) || value == 0u) {
            out.clear();
            return false;
        }

        out.push_back(value);
        if (comma == std::string_view::npos) {
            break;
        }
        text.remove_prefix(comma + 1u);
    }

    return !out.empty();
}

} // namespace javelin::bench
