export module javelin.render.passes.velocity_debug_pass;

import std;

import javelin.core.types;
import javelin.math;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.passes.line_overlay_pass;
import javelin.scene.pose_channel;

export namespace javelin {

// Per-body velocity arrow builder for LineOverlayPass.
//
// Linear velocity  (bright green): arrow from body center in the direction of travel,
//   scaled so 1 m/s → scale metres.  Conveys speed and direction at a glance.
// Angular velocity (magenta):      arrow along the spin axis, length proportional to
//   angular speed.  Drawn from the same origin as the linear vector.
//
// Static bodies (inv_mass == 0) and bodies with negligible velocity are skipped.
struct VelocityOverlayBuilder final {
    struct Settings final {
        // Seconds of look-ahead encoded as arrow length.
        // arrow_length = |velocity| * scale, so 0.1 s means 10 m/s produces a 1 m arrow.
        f32 scale{0.1f};
        // Alpha of velocity lines blended over the scene.
        f32 alpha{0.9f};
    };

    static constexpr std::string_view kLabel{"Velocity debug"};
    static constexpr f32 kLineWidthPx = 2.0f;

    // Linear velocity arrow — bright green; universally associated with forward motion.
    static constexpr Vec3 kLinearColor = linear_srgb_to_acescg(Vec3{0.20f, 1.00f, 0.30f});
    // Angular velocity arrow — magenta; visually distinct from linear and from all other
    // debug overlays (contacts=red, aabbs=green-ish, constraints=blue/orange).
    static constexpr Vec3 kAngularColor = linear_srgb_to_acescg(Vec3{1.00f, 0.20f, 0.90f});
    // Vectors shorter than this (in m or rad/s * scale) are not drawn; avoids visual noise
    // from nearly-resting bodies that technically have non-zero velocity.
    static constexpr f32 kMinEpsilon = 1e-3f;

    [[nodiscard]] static bool enabled(const RenderContext &ctx) noexcept { return ctx.debug.draw_velocities; }
    [[nodiscard]] static f32 alpha(const Settings &settings) noexcept { return settings.alpha; }

    static LineOverlayCounts fill(const RenderContext &ctx, const Settings &settings,
                                  std::vector<LineOverlayVertex> &out) {
        const u32 body_count = ctx.poses.count;
        out.reserve(body_count * 4u);

        for (u32 i = 0; i < body_count; ++i) {
            // Static bodies (inv_mass == 0) never move; skip them entirely.
            if (i < ctx.view.inv_mass.size() && ctx.view.inv_mass[i] == 0.0f) {
                continue;
            }

            const PoseSample pose = sample_pose(ctx.poses, i, ctx.pose_alpha);

            if (i < ctx.view.velocity.size()) {
                const Vec3 v = ctx.view.velocity[i];
                if (dot(v, v) >= kMinEpsilon * kMinEpsilon) {
                    out.push_back(LineOverlayVertex{.position = pose.position, .color = kLinearColor});
                    out.push_back(
                        LineOverlayVertex{.position = pose.position + v * settings.scale, .color = kLinearColor});
                }
            }

            if (i < ctx.view.angular_velocity.size()) {
                const Vec3 w = ctx.view.angular_velocity[i];
                if (dot(w, w) >= kMinEpsilon * kMinEpsilon) {
                    out.push_back(LineOverlayVertex{.position = pose.position, .color = kAngularColor});
                    out.push_back(
                        LineOverlayVertex{.position = pose.position + w * settings.scale, .color = kAngularColor});
                }
            }
        }

        return LineOverlayCounts{.point_count = 0, .line_count = static_cast<i32>(out.size())};
    }
};

using VelocityDebugPass = LineOverlayPass<VelocityOverlayBuilder>;

} // namespace javelin
