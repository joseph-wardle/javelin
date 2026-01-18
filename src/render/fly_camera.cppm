export module javelin.render.fly_camera;

import std;

import javelin.core.types;
import javelin.math.vec3;
import javelin.platform.input;
import javelin.scene.camera;

export namespace javelin {

enum struct CursorMode : u8 { normal, captured };

struct FlyCameraTuning final {
    f32 mouse_sensitivity{0.0025f}; // rad per pixel-ish
    f32 base_speed{4.0f};           // units/s
    f32 speed_min{0.25f};
    f32 speed_max{80.0f};
    f32 shift_multiplier{4.0f};
    f32 ctrl_multiplier{0.25f};
    f32 scroll_log_step{0.1f}; // speed *= exp(scroll * step)
    f32 pitch_limit{1.55334f}; // ~89 degrees
};

struct FlyCameraController final {
    FlyCameraTuning tuning{};
    f32 speed{tuning.base_speed};

    // Avoid a giant mouse delta the first frame you capture.
    bool ignore_next_mouse_delta{true};

    [[nodiscard]] CursorMode update(CameraState &camera, const InputFrame &in, f32 dt_seconds) noexcept;
};

CursorMode FlyCameraController::update(CameraState &camera, const InputFrame &in, f32 dt_seconds) noexcept {
    if (in.ui_wants_mouse || in.ui_wants_keyboard) {
        ignore_next_mouse_delta = true;
        return CursorMode::normal;
    }

    if (!in.mouse_right) {
        ignore_next_mouse_delta = true;
        return CursorMode::normal;
    }

    if (in.scroll_steps != 0.0f) {
        const f32 mul = std::exp(in.scroll_steps * tuning.scroll_log_step);
        speed = std::clamp(speed * mul, tuning.speed_min, tuning.speed_max);
    }

    if (ignore_next_mouse_delta) {
        ignore_next_mouse_delta = false;
    } else {
        camera.yaw_radians += in.mouse_delta.x * tuning.mouse_sensitivity;
        camera.pitch_radians -= in.mouse_delta.y * tuning.mouse_sensitivity;
        camera.pitch_radians = std::clamp(camera.pitch_radians, -tuning.pitch_limit, tuning.pitch_limit);
    }

    // Movement in camera space
    Vec3 move{};
    if (in.key_w)
        move.z += 1.0f;
    if (in.key_s)
        move.z -= 1.0f;
    if (in.key_d)
        move.x += 1.0f;
    if (in.key_a)
        move.x -= 1.0f;
    if (in.key_e)
        move.y += 1.0f;
    if (in.key_q)
        move.y -= 1.0f;

    const f32 speed_mul =
        (in.key_shift ? tuning.shift_multiplier : 1.0f) * (in.key_ctrl ? tuning.ctrl_multiplier : 1.0f);

    if (move.try_normalize()) {
        const CameraBasis basis = camera_basis(camera);
        const Vec3 world_up = Vec3::unit_y();
        const Vec3 delta = (basis.forward * move.z) + (basis.right * move.x) + (world_up * move.y);

        camera.position += delta * (speed * speed_mul * dt_seconds);
    }

    return CursorMode::captured;
}

} // namespace javelin
