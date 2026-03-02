export module javelin.core.time;

import std;
import javelin.core.types;

export namespace javelin {

using SteadyClock = std::chrono::steady_clock;

namespace detail {
[[nodiscard]] inline u64 now_ns() noexcept {
    const auto now = SteadyClock::now().time_since_epoch();
    return static_cast<u64>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}
} // namespace detail

[[nodiscard]] inline double to_us(const std::chrono::nanoseconds ns) noexcept {
    return static_cast<double>(ns.count()) / 1000.0;
}

[[nodiscard]] inline double to_ms(const std::chrono::nanoseconds ns) noexcept {
    return static_cast<double>(ns.count()) / 1'000'000.0;
}

struct FixedRateTicker final {
    using clock = SteadyClock;
    using duration = clock::duration;
    using time_point = clock::time_point;

    explicit FixedRateTicker(const duration dt) noexcept
        : dt_{dt}, next_{clock::now() + dt_}, prev_wake_{clock::now()} {}

    struct TickTiming {
        double interval_ms{};       // actual interval in milliseconds
        double interval_error_us{}; // (actual_interval - dt) in microseconds
    };

    [[nodiscard]] TickTiming wait_next(const std::stop_token &st) noexcept {
        while (!st.stop_requested() && clock::now() < next_) {
            std::this_thread::sleep_until(next_);
        }

        const time_point wake = clock::now();
        const duration actual = wake - prev_wake_;
        const duration error = actual - dt_;

        prev_wake_ = wake;
        next_ += dt_; // phase-locked

        const auto actual_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(actual);
        const auto error_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(error);
        return TickTiming{
            .interval_ms = to_ms(actual_ns),
            .interval_error_us = to_us(error_ns),
        };
    }

  private:
    duration dt_{};
    time_point next_{};
    time_point prev_wake_{};
};

} // namespace javelin
