export module javelin.bench.timer;

import std;

import javelin.core.types;

namespace javelin::bench {

export [[nodiscard]] f64 duration_ns(const std::chrono::nanoseconds duration) noexcept {
    return static_cast<f64>(duration.count());
}

export [[nodiscard]] f64 duration_us(const std::chrono::nanoseconds duration) noexcept {
    return duration_ns(duration) / 1000.0;
}

export [[nodiscard]] f64 duration_ms(const std::chrono::nanoseconds duration) noexcept {
    return duration_ns(duration) / 1'000'000.0;
}

export [[nodiscard]] f64 ns_per_item(const std::chrono::nanoseconds duration, const u64 item_count) noexcept {
    if (item_count == 0u) {
        return std::numeric_limits<f64>::quiet_NaN();
    }
    return duration_ns(duration) / static_cast<f64>(item_count);
}

export [[nodiscard]] f64 us_per_iteration(const std::chrono::nanoseconds duration, const u64 iteration_count) noexcept {
    if (iteration_count == 0u) {
        return std::numeric_limits<f64>::quiet_NaN();
    }
    return duration_us(duration) / static_cast<f64>(iteration_count);
}

} // namespace javelin::bench
