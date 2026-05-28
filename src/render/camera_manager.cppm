export module javelin.render.camera_manager;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math;
import javelin.render.camera_controller;
import javelin.render.fly_camera;
import javelin.render.orbit_camera;
import javelin.render.render_context;
import javelin.platform.input;
import javelin.scene;
import javelin.scene.camera;

export namespace javelin {

// Coordinates the active camera controller (orbit vs. fly) and exposes the
// per-frame FrameCamera (view/proj/view_proj) used by the render pipeline.
// Knows nothing about windows or GLFW — returns CursorMode and lets the caller
// apply it.
struct CameraManager final {
    void init_default_pose() noexcept {
        orbit_camera_ = {};
        fly_camera_ = {};
        active_camera_ = &orbit_camera_;
        camera_.position = {0.0f, 5.0f, 5.0f};
        camera_.yaw_radians = 0.0f;
        camera_.pitch_radians = -0.35f;
        fixed_world_camera_ = false;
    }

    // Called once the scene and framebuffer extent are known. Chooses between a
    // fixed recording orbit (deterministic playback) and a scene-framed orbit.
    void configure_for_scene(const Scene &scene, const i32 extent_width,
                             const i32 extent_height,
                             const bool fixed_world_camera,
                             const f32 recording_orbit_start_seconds) noexcept {
        fixed_world_camera_ = fixed_world_camera;
        if (fixed_world_camera) {
            set_fixed_world_orbit_(recording_orbit_start_seconds);
            log::info(render, "Recording orbit active; using fixed world-space anchor");
            return;
        }
        const f32 aspect = (extent_height > 0)
                               ? static_cast<f32>(extent_width) /
                                     static_cast<f32>(extent_height)
                               : 1.0f;
        orbit_camera_.frame_scene(camera_, scene.bodies(), aspect);
        log::info(render, "Presentation orbit active; hold RMB to take control");
    }

    // Advance the active controller for one frame, handling the orbit→fly
    // handoff when the user starts holding RMB outside an ImGui window.
    [[nodiscard]] CursorMode update_interactive(const InputFrame &frame_input,
                                                const f32 dt) noexcept {
        if (active_camera_ == &orbit_camera_ && !frame_input.ui_wants_mouse &&
            !frame_input.ui_wants_keyboard && frame_input.mouse_right) {
            active_camera_ = &fly_camera_;
            log::info(render, "Fly camera enabled");
        }
        return active_camera_->update(camera_, frame_input, dt);
    }

    // Offline / headless: only orbit animates; fly is purely input-driven so it
    // would freeze without an InputFrame. Always returns CursorMode::normal.
    CursorMode update_offline(const f32 dt) noexcept {
        if (fixed_world_camera_ || active_camera_ == &orbit_camera_) {
            static_cast<void>(orbit_camera_.update(camera_, InputFrame{}, dt));
        }
        return CursorMode::normal;
    }

    [[nodiscard]] FrameCamera frame_camera(const i32 extent_width,
                                           const i32 extent_height) const noexcept {
        const f32 aspect = (extent_height > 0)
                               ? static_cast<f32>(extent_width) /
                                     static_cast<f32>(extent_height)
                               : 1.0f;
        const auto view = camera_view(camera_);
        const auto proj = camera_proj(camera_.lens, aspect);
        return FrameCamera{.view = view, .proj = proj, .view_proj = proj * view};
    }

    [[nodiscard]] const CameraState &state() const noexcept { return camera_; }

private:
    static constexpr Vec3 kRecordingOrbitFocusPoint{0.0f, 5.0f, 0.0f};
    static constexpr f32 kRecordingCameraYawRadians = 0.75f;
    static constexpr f32 kRecordingCameraPitchRadians = -0.35f;
    static constexpr f32 kRecordingCameraDistance = 16.0f;

    OrbitCameraController orbit_camera_{};
    FlyCameraController fly_camera_{};
    CameraController *active_camera_{&orbit_camera_};
    CameraState camera_{};
    bool fixed_world_camera_{false};

    void set_fixed_world_orbit_(const f32 start_seconds) noexcept {
        orbit_camera_.tuning.initial_yaw_radians = kRecordingCameraYawRadians;
        orbit_camera_.tuning.pitch_radians = kRecordingCameraPitchRadians;
        orbit_camera_.focus_point = kRecordingOrbitFocusPoint;
        orbit_camera_.distance = kRecordingCameraDistance;
        orbit_camera_.yaw_radians = orbit_camera_.tuning.initial_yaw_radians +
                                    orbit_camera_.tuning.yaw_speed_radians *
                                        std::max(start_seconds, 0.0f);
        orbit_camera_.framed = true;
        static_cast<void>(orbit_camera_.update(camera_, InputFrame{}, 0.0f));
        active_camera_ = &orbit_camera_;
    }
};

} // namespace javelin
