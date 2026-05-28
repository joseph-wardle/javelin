export module javelin.render.passes.aabb_debug_pass;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.aabb;
import javelin.physics.aabb_debug;
import javelin.render.color;
import javelin.render.render_context;
import javelin.render.passes.line_overlay_pass;

export namespace javelin {

// AABB wireframe debug builder for LineOverlayPass.
// Emits 12 edges (24 vertices) per body bounds — drawn as an x-ray overlay so
// mismatches between physics bounds and rendered geometry are easy to spot.
struct AabbOverlayBuilder final {
    struct Settings final {
        // Alpha of wireframe lines blended over the scene.
        f32 alpha{0.75f};
    };

    static constexpr std::string_view kLabel{"AABB debug"};
    static constexpr f32 kLineWidthPx = 1.0f;

    // Amber wireframe — distinct from contact debug colors (red/green points, blue normals).
    static constexpr Vec3 kWireframeColor = linear_srgb_to_acescg(Vec3{0.9f, 0.7f, 0.1f});

    // 12 edges × 2 endpoints = 24 vertices per AABB wireframe box.
    static constexpr u32 kVerticesPerAabb = 24u;

    // Each row is one edge: indices into the 8-corner local array below.
    // Corners are numbered:
    //   0:(min,min,min)  1:(max,min,min)  2:(max,max,min)  3:(min,max,min)  (near face, z=min)
    //   4:(min,min,max)  5:(max,min,max)  6:(max,max,max)  7:(min,max,max)  (far  face, z=max)
    static constexpr std::array<std::array<u8, 2>, 12> kEdges = {{
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // near face
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // far face
        {0, 4}, {1, 5}, {2, 6}, {3, 7}, // lateral edges
    }};

    [[nodiscard]] static bool enabled(const RenderContext &ctx) noexcept {
        return ctx.debug.draw_aabbs && ctx.aabbs.count > 0u;
    }
    [[nodiscard]] static f32 alpha(const Settings &settings) noexcept { return settings.alpha; }

    static LineOverlayCounts fill(const RenderContext &ctx, const Settings &, std::vector<LineOverlayVertex> &out) {
        const usize aabb_count = static_cast<usize>(ctx.aabbs.count);
        out.resize(aabb_count * kVerticesPerAabb);

        usize write = 0;
        for (usize i = 0; i < aabb_count; ++i) {
            const Aabb &box = ctx.aabbs.aabbs[i];
            const Vec3 corners[8] = {
                {box.min.x, box.min.y, box.min.z}, {box.max.x, box.min.y, box.min.z},
                {box.max.x, box.max.y, box.min.z}, {box.min.x, box.max.y, box.min.z},
                {box.min.x, box.min.y, box.max.z}, {box.max.x, box.min.y, box.max.z},
                {box.max.x, box.max.y, box.max.z}, {box.min.x, box.max.y, box.max.z},
            };
            for (const std::array<u8, 2> &edge : kEdges) {
                out[write++] = LineOverlayVertex{.position = corners[edge[0]], .color = kWireframeColor};
                out[write++] = LineOverlayVertex{.position = corners[edge[1]], .color = kWireframeColor};
            }
        }

        return LineOverlayCounts{.point_count = 0, .line_count = static_cast<i32>(out.size())};
    }
};

using AabbDebugPass = LineOverlayPass<AabbOverlayBuilder>;

} // namespace javelin
