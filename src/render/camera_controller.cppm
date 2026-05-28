export module javelin.render.camera_controller;

import javelin.core.types;
import javelin.platform.input;
import javelin.scene.camera;

export namespace javelin {

// Whether the OS cursor should be captured (hidden + locked to window) for the active
// camera mode this frame.  Drives the GLFW cursor mode toggle in RenderSystem.
enum struct CursorMode : u8 { normal, captured };

// Polymorphic camera interface.  Each concrete controller (orbit, fly, ...) advances
// the camera state for one frame given the latest input and dt, and tells the caller
// whether the cursor should be captured.
//
// Orbit-style cameras ignore the input frame entirely; fly-style cameras consume it.
// Either way the caller can dispatch through this base without caring which kind is
// currently active.
struct CameraController {
    CameraController() = default;
    CameraController(const CameraController &) = default;
    CameraController(CameraController &&) = default;
    CameraController &operator=(const CameraController &) = default;
    CameraController &operator=(CameraController &&) = default;
    virtual ~CameraController() = default;

    [[nodiscard]] virtual CursorMode update(CameraState &camera, const InputFrame &input, f32 dt_seconds) noexcept = 0;
};

} // namespace javelin
