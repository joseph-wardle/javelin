export module javelin.platform.input;

import javelin.core.types;
import javelin.math.vec2;

export namespace javelin {

enum struct InputKey : u8 {
    w,
    a,
    s,
    d,
    q,
    e,
    shift,
    ctrl,
};

struct InputFrame final {
    // held (digital)
    bool key_w{};
    bool key_a{};
    bool key_s{};
    bool key_d{};
    bool key_q{};
    bool key_e{};
    bool key_shift{};
    bool key_ctrl{};
    bool mouse_right{};

    // per-frame deltas
    Vec2 mouse_delta{};
    f32 scroll_steps{};

    // UI capture hints (set by render thread after ImGui updates)
    bool ui_wants_mouse{};
    bool ui_wants_keyboard{};
};

struct InputState final {
    void begin_frame() noexcept {
        frame_.mouse_delta = {};
        frame_.scroll_steps = 0.0f;
        frame_.ui_wants_mouse = false;
        frame_.ui_wants_keyboard = false;
    }

    void end_frame(const bool ui_wants_mouse, const bool ui_wants_keyboard) noexcept {
        frame_.ui_wants_mouse = ui_wants_mouse;
        frame_.ui_wants_keyboard = ui_wants_keyboard;
    }

    void set_key(const InputKey key, const bool down) noexcept {
        switch (key) {
        case InputKey::w:
            frame_.key_w = down;
            break;
        case InputKey::a:
            frame_.key_a = down;
            break;
        case InputKey::s:
            frame_.key_s = down;
            break;
        case InputKey::d:
            frame_.key_d = down;
            break;
        case InputKey::q:
            frame_.key_q = down;
            break;
        case InputKey::e:
            frame_.key_e = down;
            break;
        case InputKey::shift:
            frame_.key_shift = down;
            break;
        case InputKey::ctrl:
            frame_.key_ctrl = down;
            break;
        }
    }

    void set_mouse_right(const bool down) noexcept { frame_.mouse_right = down; }

    void add_cursor_pos(const f32 x, const f32 y) noexcept {
        const Vec2 pos{x, y};
        if (cursor_valid_) {
            frame_.mouse_delta += pos - cursor_pos_;
        }
        cursor_pos_ = pos;
        cursor_valid_ = true;
    }

    void add_scroll(const f32 steps) noexcept { frame_.scroll_steps += steps; }

    void reset_cursor() noexcept { cursor_valid_ = false; }

    [[nodiscard]] const InputFrame &frame() const noexcept { return frame_; }

  private:
    InputFrame frame_{};
    Vec2 cursor_pos_{};
    bool cursor_valid_{false};
};

} // namespace javelin
