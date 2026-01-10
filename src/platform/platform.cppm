module;
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
export module javelin.platform;

import javelin.core.logging;
export import javelin.platform.window;

export namespace javelin {
    struct Platform final {
        void init() {
            glfwSetErrorCallback([](int code, const char* desc) {
                log::error("[glfw] error {}: {}", code, desc ? desc : "(null)");
            });

            if (glfwInit() != GLFW_TRUE) {
                log::critical("{}", "glfwInit failed");
            }

            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if defined(__APPLE__)
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
#if !defined(NDEBUG)
            glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif

            window_.native = glfwCreateWindow(1280, 720, "javelin", nullptr, nullptr);
            if (!window_.native) {
                glfwTerminate();
                log::critical("{}", "glfwCreateWindow failed");
            }

            glfwSetWindowUserPointer(window_.native, this);
            glfwSetWindowCloseCallback(window_.native, [](GLFWwindow* w) {
                if (auto* self = static_cast<Platform*>(glfwGetWindowUserPointer(w))) self->quit_ = true;
            });

            if constexpr (false) { // vsync
                glfwMakeContextCurrent(window_.native);
                glfwSwapInterval(1);
                glfwMakeContextCurrent(nullptr);
            }

            quit_ = false;
        }

        [[nodiscard]] WindowHandle window_handle() const noexcept { return window_; }

        [[nodiscard]] bool quit_requested() const noexcept {
            if (quit_) return true;
            return window_.native && glfwWindowShouldClose(window_.native);
        }

        void poll_events() noexcept {
            glfwPollEvents();
            if (window_.native && glfwWindowShouldClose(window_.native)) {
                quit_ = true;
            }
        }

        void shutdown() noexcept {
            if (window_.native) {
                glfwDestroyWindow(window_.native);
                window_.native = nullptr;
            }
            glfwTerminate();
            quit_ = true;
        }

    private:
        WindowHandle window_{};
        bool quit_{false};
    };

} // namespace javelin
