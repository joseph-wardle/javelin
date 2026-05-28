export module javelin.render.passes.contact_debug_pass;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.contact_debug;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.passes.line_overlay_pass;

export namespace javelin {

// Per-contact debug vertex builder for LineOverlayPass.
//
// For each contact point, emits one GL_POINTS vertex (color depends on whether the
// contact has persisted since the previous tick) and a GL_LINES pair for the normal arrow.
struct ContactOverlayBuilder final {
    struct Settings final {
        // World-space length of visualized contact normals.
        f32 normal_length{0.25f};
        // Raster point size in pixels.
        f32 point_size_px{7.0f};
        // Line/point alpha blended over the scene.
        f32 alpha{0.95f};
    };

    static constexpr std::string_view kLabel{"Contact debug"};
    static constexpr f32 kLineWidthPx = 3.0f;
    static constexpr bool kDrawsPoints = true;

    static constexpr Vec3 kPointColorNew = linear_srgb_to_acescg(Vec3{1.0f, 0.20f, 0.10f});
    static constexpr Vec3 kPointColorPersisted = linear_srgb_to_acescg(Vec3{0.30f, 1.0f, 0.30f});
    static constexpr Vec3 kNormalColor = linear_srgb_to_acescg(Vec3{0.20f, 0.65f, 1.0f});

    [[nodiscard]] static bool enabled(const RenderContext &ctx) noexcept {
        return ctx.debug.draw_contacts && ctx.contacts.count > 0u;
    }

    [[nodiscard]] static f32 alpha(const Settings &settings) noexcept { return settings.alpha; }
    [[nodiscard]] static f32 point_size_px(const Settings &settings) noexcept { return settings.point_size_px; }

    static LineOverlayCounts fill(const RenderContext &ctx, const Settings &settings,
                                  std::vector<LineOverlayVertex> &out) {
        const usize contact_count = static_cast<usize>(ctx.contacts.count);
        const usize point_count = contact_count;
        const usize line_count = contact_count * 2u;
        out.resize(point_count + line_count);

        // Layout: [all point vertices][all line segment vertices].
        usize line_write = point_count;
        for (usize i = 0; i < contact_count; ++i) {
            const Vec3 position = ctx.contacts.points[i];
            Vec3 normal = ctx.contacts.normals[i];
            if (!normal.try_normalize()) {
                normal = Vec3::unit_y();
            }

            const bool persisted = ctx.contacts.persisted[i] != 0u;
            const Vec3 point_color = persisted ? kPointColorPersisted : kPointColorNew;
            const Vec3 normal_tip = position + normal * settings.normal_length;

            out[i] = LineOverlayVertex{.position = position, .color = point_color};
            out[line_write + 0u] = LineOverlayVertex{.position = position, .color = kNormalColor};
            out[line_write + 1u] = LineOverlayVertex{.position = normal_tip, .color = kNormalColor};
            line_write += 2u;
        }

        return LineOverlayCounts{
            .point_count = static_cast<i32>(point_count),
            .line_count = static_cast<i32>(line_count),
        };
    }
};

using ContactDebugPass = LineOverlayPass<ContactOverlayBuilder>;

} // namespace javelin
