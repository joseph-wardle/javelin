export module javelin.platform;

export import javelin.platform.window;

import javelin.core.types;

export namespace javelin {
    struct Platform final {
        void init() {}
        [[nodiscard]] WindowHandle window_handle() const noexcept { return window_; }
        [[nodiscard]] bool quit_requested() const { return quit_; }
        void poll_events() {}
        void shutdown() {}

    private:
        WindowHandle window_{};
        bool quit_{false};
    };
}