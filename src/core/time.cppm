export module javelin.core.time;

import javelin.core.types;

export namespace javelin {
    struct FixedStepClock {
        f64 accumulated_time_;
        f64 delta_;

        void advance(const f64 time_seconds) noexcept {
            accumulated_time_ += time_seconds;
        }

        [[nodiscard]] bool consume_step() noexcept {
            if (accumulated_time_ >= delta_) {
                accumulated_time_ -= delta_;
                return true;
            }
            return false;
        }

        [[nodiscard]] constexpr f64 delta() const noexcept {
            return delta_;
        }
    };
}