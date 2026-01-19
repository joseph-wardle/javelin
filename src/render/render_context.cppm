export module javelin.render.render_context;

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
    bool draw_debug{true};
    bool draw_wireframe{false};
    bool apply_color_transform{true};
    // later: show_aabbs, show_contacts, etc.
};

struct RenderContext final {
    Extent2D extent;
    FrameCamera camera;

    RenderView view;
    PoseSnapshot poses;

    RenderTargets &targets;
    const DebugToggles &debug;
};

} // namespace javelin
