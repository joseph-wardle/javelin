export module javelin.platform.window;

import javelin.core.types;

export namespace javelin {

    struct GLFWwindow;

    struct WindowHandle final {
        GLFWwindow* native{};

        [[nodiscard]] constexpr explicit operator bool() const noexcept {
            return native != nullptr;
        }
    };
} // namespace javelin
