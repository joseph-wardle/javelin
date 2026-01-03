export module javelin.core.logging;

import std;
import javelin.core.types;

export namespace javelin::log {

enum class Level : u8 { trace = 0, debug, info, warn, error, critical, off };

[[nodiscard]] consteval std::string_view to_string(const Level level) noexcept {
    switch (level) {
        case Level::trace:    return "TRACE";
        case Level::debug:    return "DEBUG";
        case Level::info:     return "INFO";
        case Level::warn:     return "WARN";
        case Level::error:    return "ERROR";
        case Level::critical: return "CRIT";
        case Level::off:      return "OFF";
    }
    return "UNKNOWN";
}

#if !defined(JAVELIN_LOG_LEVEL)
inline constexpr auto compile_time_level = Level::info;
#else
static_assert(JAVELIN_LOG_LEVEL >= 0, "JAVELIN_LOG_LEVEL must be in [0, 6].");
static_assert(JAVELIN_LOG_LEVEL <= 6, "JAVELIN_LOG_LEVEL must be in [0, 6].");
inline constexpr Level compile_time_level = static_cast<Level>(JAVELIN_LOG_LEVEL);
#endif

using Sink = void(*)(std::string_view);

namespace detail {
    using clock = std::chrono::steady_clock;

    [[nodiscard]] inline const clock::time_point& start_time() noexcept {
        static const clock::time_point t0 = clock::now();
        return t0;
    }

    inline void default_sink(std::string_view s) {
        std::cerr.write(s.data(), static_cast<std::streamsize>(s.size()));
    }

    inline std::atomic g_sink{&default_sink};

    [[nodiscard]] inline Sink sink() noexcept {
        return g_sink.load(std::memory_order_acquire);
    }
} // namespace detail

inline void set_sink(const Sink s) noexcept {
    detail::g_sink.store(s ? s : &detail::default_sink, std::memory_order_release);
}

template<Level level, class... Args>
void log(std::format_string<Args...> fmt, Args&&... args) {
    if constexpr (level < compile_time_level) return;

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        detail::clock::now() - detail::start_time()
    ).count();

    thread_local std::string line;
    line.clear();
    line.reserve(256);

    std::format_to(std::back_inserter(line), "[{:>8} ms] [{:^7}] ", ms, to_string(level));
    std::format_to(std::back_inserter(line), fmt, std::forward<Args>(args)...);
    line.push_back('\n');

    detail::sink()(line);
}

template<class... A>
void trace(std::format_string<A...> f, A&&... a)
{ log<Level::trace>(f, std::forward<A>(a)...); }

template<class... A>
void debug(std::format_string<A...> f, A&&... a)
{ log<Level::debug>(f, std::forward<A>(a)...); }

template<class... A>
void info(std::format_string<A...> f, A&&... a)
{ log<Level::info>(f, std::forward<A>(a)...); }

template<class... A>
void warn(std::format_string<A...> f, A&&... a)
{ log<Level::warn>(f, std::forward<A>(a)...); }

template<class... A>
void error(std::format_string<A...> f, A&&... a)
{ log<Level::error>(f, std::forward<A>(a)...); }

template<class... A>
void critical(std::format_string<A...> f, A&&... a)
{ log<Level::critical>(f, std::forward<A>(a)...); }

} // namespace javelin::log
