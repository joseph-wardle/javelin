module;
#include <GLFW/glfw3.h>
export module javelin.platform.window;

import javelin.core.types;

export namespace javelin {

    struct WindowHandle final {
        GLFWwindow* native{};

        [[nodiscard]] constexpr explicit operator bool() const noexcept {
            return native != nullptr;
        }
    };

} // namespace javelin
