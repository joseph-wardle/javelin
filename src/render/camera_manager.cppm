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
//
// Transitions are one-way: the manager starts in orbit and switches to fly the
// first frame the user starts holding RMB outside an ImGui window.  Once in
// fly mode, the manager stays there for the session, so the orbit controller's
// state is not preserved across the transition.
struct CameraManager final {
    void init_default_pose() noexcept {
        active_camera_ = OrbitCameraController{};
        camera_.position = {0.0f, 5.0f, 5.0f};
        camera_.yaw_radians = 0.0f;
        camera_.pitch_radians = -0.35f;
    }

    // Called once the scene and framebuffer extent are known. Chooses between a
    // fixed recording orbit (deterministic playback) and a scene-framed orbit.
    void configure_for_scene(const Scene &scene, const i32 extent_width,
                             const i32 extent_height,
                             const bool fixed_world_camera,
                             const f32 recording_orbit_start_seconds) noexcept {
        if (fixed_world_camera) {
            set_fixed_world_orbit_(recording_orbit_start_seconds);
            log::info(render, "Recording orbit active; using fixed world-space anchor");
            return;
        }
        const f32 aspect = (extent_height > 0)
                               ? static_cast<f32>(extent_width) /
                                     static_cast<f32>(extent_height)
                               : 1.0f;
        OrbitCameraController orbit{};
        orbit.frame_scene(camera_, scene.bodies(), aspect);
        active_camera_ = std::move(orbit);
        log::info(render, "Presentation orbit active; hold RMB to take control");
    }

    // Advance the active controller for one frame, handling the orbit→fly
    // handoff when the user starts holding RMB outside an ImGui window.
    [[nodiscard]] CursorMode update_interactive(const InputFrame &frame_input,
                                                const f32 dt) noexcept {
        if (std::holds_alternative<OrbitCameraController>(active_camera_) &&
            !frame_input.ui_wants_mouse && !frame_input.ui_wants_keyboard &&
            frame_input.mouse_right) {
            active_camera_ = FlyCameraController{};
            log::info(render, "Fly camera enabled");
        }
        return std::visit([&](auto &controller) { return controller.update(camera_, frame_input, dt); },
                          active_camera_);
    }

    // Offline / headless: only orbit animates; fly is purely input-driven so it
    // would freeze without an InputFrame. Always returns CursorMode::normal.
    CursorMode update_offline(const f32 dt) noexcept {
        if (auto *orbit = std::get_if<OrbitCameraController>(&active_camera_)) {
            static_cast<void>(orbit->update(camera_, InputFrame{}, dt));
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

    std::variant<OrbitCameraController, FlyCameraController> active_camera_{OrbitCameraController{}};
    CameraState camera_{};

    void set_fixed_world_orbit_(const f32 start_seconds) noexcept {
        OrbitCameraController orbit{};
        orbit.tuning.initial_yaw_radians = kRecordingCameraYawRadians;
        orbit.tuning.pitch_radians = kRecordingCameraPitchRadians;
        orbit.focus_point = kRecordingOrbitFocusPoint;
        orbit.distance = kRecordingCameraDistance;
        orbit.yaw_radians = orbit.tuning.initial_yaw_radians +
                            orbit.tuning.yaw_speed_radians * std::max(start_seconds, 0.0f);
        orbit.framed = true;
        static_cast<void>(orbit.update(camera_, InputFrame{}, 0.0f));
        active_camera_ = std::move(orbit);
    }
};

} // namespace javelin
