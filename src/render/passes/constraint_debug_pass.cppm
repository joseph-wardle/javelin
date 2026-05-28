export module javelin.render.passes.constraint_debug_pass;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.constraint_types;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.passes.line_overlay_pass;
import javelin.scene.pose_channel;

export namespace javelin {

// Per-constraint debug line builder for LineOverlayPass.
//
// Renders a line between the world-space anchor points of each distance constraint.
// Rigid constraints (compliance = 0): icy blue-white — stiff rod.
// Soft constraints  (compliance > 0): warm orange     — elastic spring.
struct ConstraintOverlayBuilder final {
    struct Settings final {
        // Alpha of constraint lines blended over the scene.
        f32 alpha{0.85f};
    };

    static constexpr std::string_view kLabel{"Constraint debug"};
    static constexpr f32 kLineWidthPx = 2.0f;

    // Rigid rod — icy blue-white; evokes a stiff wire or steel cable.
    static constexpr Vec3 kRigidColor = linear_srgb_to_acescg(Vec3{0.75f, 0.90f, 1.00f});
    // Elastic spring — warm orange; high contrast against the rigid color and the
    // sleep/contact debug overlays.
    static constexpr Vec3 kSoftColor = linear_srgb_to_acescg(Vec3{1.00f, 0.55f, 0.10f});

    [[nodiscard]] static bool enabled(const RenderContext &ctx) noexcept {
        return ctx.debug.draw_constraints && !ctx.view.constraints.empty();
    }
    [[nodiscard]] static f32 alpha(const Settings &settings) noexcept { return settings.alpha; }

    static LineOverlayCounts fill(const RenderContext &ctx, const Settings &, std::vector<LineOverlayVertex> &out) {
        const u32 body_count = ctx.poses.count;
        out.reserve(ctx.view.constraints.size() * 2u);

        for (const DistanceConstraint &c : ctx.view.constraints) {
            if (c.body_a >= body_count || c.body_b >= body_count) {
                continue;
            }
            const PoseSample pose_a = sample_pose(ctx.poses, c.body_a, ctx.pose_alpha);
            const PoseSample pose_b = sample_pose(ctx.poses, c.body_b, ctx.pose_alpha);

            const Vec3 world_a = pose_a.position + rotate(pose_a.orientation, c.anchor_a);
            const Vec3 world_b = pose_b.position + rotate(pose_b.orientation, c.anchor_b);

            const Vec3 color = (c.compliance == 0.0f) ? kRigidColor : kSoftColor;
            out.push_back(LineOverlayVertex{.position = world_a, .color = color});
            out.push_back(LineOverlayVertex{.position = world_b, .color = color});
        }

        return LineOverlayCounts{.point_count = 0, .line_count = static_cast<i32>(out.size())};
    }
};

using ConstraintDebugPass = LineOverlayPass<ConstraintOverlayBuilder>;

} // namespace javelin
