export module javelin.core.time;

import javelin.core.types;

export namespace javelin {
    struct FixedStepClock {
        f64 delta{1.0 / 60.0};
        f64 accumulated_time{0.0};
        f64 prev_time{0.0};
        bool started{false};

        void advance(const f64 now_seconds) noexcept {
            if (!started) {
                prev_time = now_seconds;
                started = true;
                return;
            }

            f64 frame_dt = now_seconds - prev_time;
            prev_time = now_seconds;

            if (frame_dt < 0.0) frame_dt = 0.0;
            if (frame_dt > 0.25) frame_dt = 0.25; // hitch guard

            accumulated_time += frame_dt;
        }

        [[nodiscard]] bool consume_step() noexcept {
            if (accumulated_time >= delta) {
                accumulated_time -= delta;
                return true;
            }
            return false;
        }
    };
}