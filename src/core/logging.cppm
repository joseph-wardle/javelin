module;

#if defined(TRACY_ENABLE)
#include <tracy/Tracy.hpp>
#endif

export module javelin.core.logging;

import std;
import javelin.core.types;

export namespace javelin::log {

enum class Level : u8 { trace = 0, debug, info, warn, error, critical, off };

enum class Tag : u8 { app, platform, scene, render, physics, input, none };

inline constexpr Tag app = Tag::app;
inline constexpr Tag platform = Tag::platform;
inline constexpr Tag scene = Tag::scene;
inline constexpr Tag render = Tag::render;
inline constexpr Tag physics = Tag::physics;
inline constexpr Tag input = Tag::input;

inline constexpr usize kTagWidth = 8;

[[nodiscard]] consteval std::string_view to_string(const Level level) noexcept {
    switch (level) {
    case Level::trace:
        return "TRACE";
    case Level::debug:
        return "DEBUG";
    case Level::info:
        return "INFO";
    case Level::warn:
        return "WARN";
    case Level::error:
        return "ERROR";
    case Level::critical:
        return "CRIT";
    case Level::off:
        return "OFF";
    }
    return "UNKNOWN";
}

[[nodiscard]] constexpr std::string_view to_string(const Tag tag) noexcept {
    switch (tag) {
    case Tag::app:
        return "app";
    case Tag::platform:
        return "platform";
    case Tag::scene:
        return "scene";
    case Tag::render:
        return "render";
    case Tag::physics:
        return "physics";
    case Tag::input:
        return "input";
    case Tag::none:
        return "";
    }
    return "";
}

#if !defined(JAVELIN_LOG_LEVEL)
inline constexpr auto compile_time_level = Level::info;
#else
static_assert(JAVELIN_LOG_LEVEL >= 0);
static_assert(JAVELIN_LOG_LEVEL <= 6);
inline constexpr Level compile_time_level = static_cast<Level>(JAVELIN_LOG_LEVEL);
#endif

using Sink = void (*)(std::string_view);

namespace detail {
using clock = std::chrono::steady_clock;

[[nodiscard]] inline const clock::time_point &start_time() noexcept {
    static const clock::time_point t0 = clock::now();
    return t0;
}

inline void default_sink(std::string_view s) noexcept {
    std::cerr.write(s.data(), static_cast<std::streamsize>(s.size()));
}

inline std::atomic<Sink> g_sink{&default_sink};

[[nodiscard]] inline Sink sink() noexcept { return g_sink.load(std::memory_order_acquire); }

[[nodiscard]] inline std::string_view trim_tag(std::string_view tag) noexcept {
    const usize size = static_cast<usize>(tag.size());
    if (size <= kTagWidth) {
        return tag;
    }
    return std::string_view{tag.data(), kTagWidth};
}

inline void tracy_message(const Level lvl, std::string_view s) noexcept {
#if defined(TRACY_ENABLE)
    TracyMessage(s.data(), s.size());
#else
    (void)lvl;
    (void)s;
#endif
}
} // namespace detail

inline void set_sink(const Sink s) noexcept {
    detail::g_sink.store(s ? s : &detail::default_sink, std::memory_order_release);
}

template <Level level, class... Args> void log(std::format_string<Args...> fmt, Args &&...args) {
    if constexpr (level < compile_time_level)
        return;

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(detail::clock::now() - detail::start_time()).count();

    thread_local std::string line;
    line.clear();
    line.reserve(256);

    std::format_to(std::back_inserter(line), "[{:>6} ms] [{:^6}] [{:<{}}] ", ms, to_string(level), "", kTagWidth);
    std::format_to(std::back_inserter(line), fmt, std::forward<Args>(args)...);
    line.push_back('\n');

    detail::tracy_message(level, line);
    detail::sink()(line);
}

template <Level level, class... Args> void log(const Tag tag, std::format_string<Args...> fmt, Args &&...args) {
    if constexpr (level < compile_time_level)
        return;

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(detail::clock::now() - detail::start_time()).count();

    thread_local std::string line;
    line.clear();
    line.reserve(256);

    const std::string_view trimmed = detail::trim_tag(to_string(tag));
    std::format_to(std::back_inserter(line), "[{:>6} ms] [{:^6}] [{:<{}}] ", ms, to_string(level), trimmed, kTagWidth);
    std::format_to(std::back_inserter(line), fmt, std::forward<Args>(args)...);
    line.push_back('\n');

    detail::tracy_message(level, line);
    detail::sink()(line);
}

template <class... A> void trace(std::format_string<A...> f, A &&...a) { log<Level::trace>(f, std::forward<A>(a)...); }
template <class... A> void debug(std::format_string<A...> f, A &&...a) { log<Level::debug>(f, std::forward<A>(a)...); }
template <class... A> void info(std::format_string<A...> f, A &&...a) { log<Level::info>(f, std::forward<A>(a)...); }
template <class... A> void warn(std::format_string<A...> f, A &&...a) { log<Level::warn>(f, std::forward<A>(a)...); }
template <class... A> void error(std::format_string<A...> f, A &&...a) { log<Level::error>(f, std::forward<A>(a)...); }
template <class... A> void critical(std::format_string<A...> f, A &&...a) {
    log<Level::critical>(f, std::forward<A>(a)...);
    std::exit(1);
}

template <class... A> void trace(const Tag tag, std::format_string<A...> f, A &&...a) {
    log<Level::trace>(tag, f, std::forward<A>(a)...);
}
template <class... A> void debug(const Tag tag, std::format_string<A...> f, A &&...a) {
    log<Level::debug>(tag, f, std::forward<A>(a)...);
}
template <class... A> void info(const Tag tag, std::format_string<A...> f, A &&...a) {
    log<Level::info>(tag, f, std::forward<A>(a)...);
}
template <class... A> void warn(const Tag tag, std::format_string<A...> f, A &&...a) {
    log<Level::warn>(tag, f, std::forward<A>(a)...);
}
template <class... A> void error(const Tag tag, std::format_string<A...> f, A &&...a) {
    log<Level::error>(tag, f, std::forward<A>(a)...);
}
template <class... A> void critical(const Tag tag, std::format_string<A...> f, A &&...a) {
    log<Level::critical>(tag, f, std::forward<A>(a)...);
    std::exit(1);
}

} // namespace javelin::log

export namespace javelin {
using log::app;
using log::input;
using log::physics;
using log::platform;
using log::render;
using log::scene;
} // namespace javelin
