module;
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
export module javelin.platform;

import javelin.core.logging;
import javelin.core.types;
export import javelin.platform.input;
export import javelin.platform.window;

export namespace javelin {
struct Platform final {
    void init() {
        log::info("[platform] init");
        glfwSetErrorCallback(
            [](int code, const char *desc) {
                log::error("[platform][glfw] error {}: {}", code, desc ? desc : "(null)");
            });

        if (glfwInit() != GLFW_TRUE) {
            log::critical("[platform] glfwInit failed");
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
            log::critical("[platform] glfwCreateWindow failed");
        }
        log::info("[platform] window created 1280x720");

        glfwSetWindowUserPointer(window_.native, this);
        glfwSetKeyCallback(window_.native, [](GLFWwindow *w, const int key, const int, const int action,
                                              const int) {
            if (auto *self = static_cast<Platform *>(glfwGetWindowUserPointer(w))) {
                const bool down = action != GLFW_RELEASE;
                switch (key) {
                case GLFW_KEY_W:
                    self->input_.set_key(InputKey::w, down);
                    break;
                case GLFW_KEY_A:
                    self->input_.set_key(InputKey::a, down);
                    break;
                case GLFW_KEY_S:
                    self->input_.set_key(InputKey::s, down);
                    break;
                case GLFW_KEY_D:
                    self->input_.set_key(InputKey::d, down);
                    break;
                case GLFW_KEY_Q:
                    self->input_.set_key(InputKey::q, down);
                    break;
                case GLFW_KEY_E:
                    self->input_.set_key(InputKey::e, down);
                    break;
                case GLFW_KEY_LEFT_SHIFT:
                case GLFW_KEY_RIGHT_SHIFT:
                    self->input_.set_key(InputKey::shift, down);
                    break;
                case GLFW_KEY_LEFT_CONTROL:
                case GLFW_KEY_RIGHT_CONTROL:
                    self->input_.set_key(InputKey::ctrl, down);
                    break;
                default:
                    break;
                }
            }
        });
        glfwSetMouseButtonCallback(window_.native, [](GLFWwindow *w, const int button, const int action, const int) {
            if (auto *self = static_cast<Platform *>(glfwGetWindowUserPointer(w))) {
                if (button == GLFW_MOUSE_BUTTON_RIGHT) {
                    self->input_.set_mouse_right(action != GLFW_RELEASE);
                }
            }
        });
        glfwSetCursorPosCallback(window_.native, [](GLFWwindow *w, const double x, const double y) {
            if (auto *self = static_cast<Platform *>(glfwGetWindowUserPointer(w))) {
                self->input_.add_cursor_pos(static_cast<f32>(x), static_cast<f32>(y));
            }
        });
        glfwSetScrollCallback(window_.native, [](GLFWwindow *w, const double, const double y) {
            if (auto *self = static_cast<Platform *>(glfwGetWindowUserPointer(w))) {
                self->input_.add_scroll(static_cast<f32>(y));
            }
        });
        glfwSetWindowFocusCallback(window_.native, [](GLFWwindow *w, const int focused) {
            if (focused == GLFW_FALSE) {
                if (auto *self = static_cast<Platform *>(glfwGetWindowUserPointer(w))) {
                    self->input_.reset_cursor();
                }
            }
        });
        glfwSetWindowCloseCallback(window_.native, [](GLFWwindow *w) {
            if (auto *self = static_cast<Platform *>(glfwGetWindowUserPointer(w)))
                self->quit_ = true;
        });

        if constexpr (false) { // vsync
            glfwMakeContextCurrent(window_.native);
            glfwSwapInterval(1);
            glfwMakeContextCurrent(nullptr);
        }

        quit_ = false;
    }

    [[nodiscard]] WindowHandle window_handle() const noexcept { return window_; }
    [[nodiscard]] InputState &input_state() noexcept { return input_; }
    [[nodiscard]] const InputState &input_state() const noexcept { return input_; }

    [[nodiscard]] bool quit_requested() const noexcept {
        if (quit_)
            return true;
        return window_.native && glfwWindowShouldClose(window_.native);
    }

    void poll_events() noexcept {
        glfwPollEvents();
        if (window_.native && glfwWindowShouldClose(window_.native)) {
            quit_ = true;
        }
    }

    void shutdown() noexcept {
        log::info("[platform] shutdown");
        if (window_.native) {
            glfwDestroyWindow(window_.native);
            window_.native = nullptr;
        }
        glfwTerminate();
        quit_ = true;
    }

  private:
    WindowHandle window_{};
    InputState input_{};
    bool quit_{false};
};

} // namespace javelin
