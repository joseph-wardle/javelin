export module javelin.platform;

import javelin.core.types;

export namespace javelin {
    struct Platform {
        void init() {}
        [[nodiscard]] bool quit_requested() const { return false; }
        void poll_events() {}
        [[nodiscard]] f64 time_seconds() const { return 0.016; }
        void shutdown() {}
    };
}