export module javelin.render.render_context;

import javelin.core.types;
import javelin.physics.contact_debug;
import javelin.render.types;
import javelin.render.render_targets;
import javelin.math.mat4;
import javelin.scene.render_view;
import javelin.scene.pose_channel;

export namespace javelin {

struct FrameCamera final {
    Mat4 view;
    Mat4 proj;
    Mat4 view_proj;
    // optional: position, forward, etc.
};

struct DebugToggles final {
    bool draw_grid{true};
    bool draw_contacts{false};
    bool apply_color_transform{true};
    // later: show_aabbs, show_joint_limits, etc.
};

struct RenderContext final {
    Extent2D extent;
    FrameCamera camera;

    RenderView view;
    PoseSnapshot poses;
    ContactDebugSnapshot contacts;
    f32 pose_alpha{1.0f};

    RenderTargets &targets;
    const DebugToggles &debug;
};

} // namespace javelin
