export module javelin.platform.input;

import javelin.core.types;
import javelin.math.vec2;

export namespace javelin {

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

} // namespace javelin
