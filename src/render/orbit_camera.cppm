export module javelin.render.orbit_camera;

import std;

import javelin.core.types;
import javelin.math.quat;
import javelin.scene.camera;
import javelin.scene.render_view;
import javelin.scene.shapes;

export namespace javelin {
namespace detail {

struct SceneBounds final {
    Vec3 min{};
    Vec3 max{};
    bool valid{};
};

[[nodiscard]] inline Vec3 abs_vec3(const Vec3 v) noexcept {
    return Vec3{std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)};
}

[[nodiscard]] inline Vec3 shape_world_half_extents(const ShapeData &shape, const Quat &orientation) noexcept {
    switch (shape.kind) {
    case ShapeKind::sphere:
        return Vec3{shape_sphere(shape).radius};
    case ShapeKind::box: {
        const Vec3 half_extents = shape_box(shape).half_extents;
        const Mat3 basis = to_mat3(orientation);
        const Vec3 axis_x = abs_vec3(basis.col(0)) * half_extents.x;
        const Vec3 axis_y = abs_vec3(basis.col(1)) * half_extents.y;
        const Vec3 axis_z = abs_vec3(basis.col(2)) * half_extents.z;
        return axis_x + axis_y + axis_z;
    }
    }
    return Vec3{};
}

[[nodiscard]] inline SceneBounds compute_scene_bounds(const RenderView &view) noexcept {
    SceneBounds bounds{};

    const usize count = std::min({view.alive.size(), view.shape_index.size(), view.position.size(), view.orientation.size()});

    for (usize i = 0; i < count; ++i) {
        if (view.alive[i] == 0u) {
            continue;
        }

        const u32 shape_index = view.shape_index[i];
        if (shape_index >= view.shapes.size()) {
            continue;
        }

        const Vec3 center = view.position[i];
        const Vec3 half_extents = shape_world_half_extents(view.shapes[shape_index], view.orientation[i]);
        const Vec3 world_min = center - half_extents;
        const Vec3 world_max = center + half_extents;

        if (!bounds.valid) {
            bounds.min = world_min;
            bounds.max = world_max;
            bounds.valid = true;
            continue;
        }

        bounds.min = min(bounds.min, world_min);
        bounds.max = max(bounds.max, world_max);
    }

    return bounds;
}

[[nodiscard]] inline f32 fit_distance_for_radius(const CameraLens &lens, const f32 aspect, const f32 radius,
                                                 const f32 padding) noexcept {
    const f32 clamped_aspect = std::max(aspect, 0.1f);
    const f32 half_fov_y = lens.fov_y_radians * 0.5f;
    const f32 half_fov_x = std::atan(std::tan(half_fov_y) * clamped_aspect);
    const f32 limiting_half_fov = std::min(half_fov_x, half_fov_y);
    const f32 sin_half_fov = std::max(std::sin(limiting_half_fov), 0.1f);
    const f32 padded_radius = radius * std::max(padding, 1.0f);
    return padded_radius / sin_half_fov;
}

} // namespace detail

struct OrbitCameraTuning final {
    f32 initial_yaw_radians{0.75f};
    f32 pitch_radians{-0.35f};
    f32 yaw_speed_radians{0.12f};
    f32 framing_padding{1.25f};
    f32 distance_scale{0.5f};
    f32 min_scene_radius{1.0f};
    f32 min_distance{4.0f};
    f32 fallback_distance{8.0f};
};

struct OrbitCameraController final {
    OrbitCameraTuning tuning{};
    Vec3 focus_point{};
    f32 distance{8.0f};
    f32 yaw_radians{tuning.initial_yaw_radians};
    bool framed{};

    void frame_scene(CameraState &camera, const RenderView &view, f32 aspect) noexcept;
    void update(CameraState &camera, f32 dt_seconds) noexcept;
};

void OrbitCameraController::frame_scene(CameraState &camera, const RenderView &view, const f32 aspect) noexcept {
    yaw_radians = tuning.initial_yaw_radians;
    framed = true;

    const detail::SceneBounds bounds = detail::compute_scene_bounds(view);
    if (!bounds.valid) {
        focus_point = Vec3{};
        distance = std::max(tuning.min_distance, tuning.fallback_distance);
        update(camera, 0.0f);
        return;
    }

    const Vec3 center = (bounds.min + bounds.max) * 0.5f;
    const Vec3 extents = (bounds.max - bounds.min) * 0.5f;
    const f32 radius = std::max(extents.length(), tuning.min_scene_radius);

    focus_point = center;
    const f32 fitted_distance =
        detail::fit_distance_for_radius(camera.lens, aspect, radius, tuning.framing_padding);
    distance = std::max(tuning.min_distance, fitted_distance * std::max(tuning.distance_scale, 0.1f));
    update(camera, 0.0f);
}

void OrbitCameraController::update(CameraState &camera, const f32 dt_seconds) noexcept {
    if (!framed) {
        return;
    }

    constexpr f32 tau = 6.28318530717958647692f;
    yaw_radians = std::remainder(yaw_radians + tuning.yaw_speed_radians * std::max(dt_seconds, 0.0f), tau);

    CameraState orbit_pose{};
    orbit_pose.yaw_radians = yaw_radians;
    orbit_pose.pitch_radians = tuning.pitch_radians;

    const Vec3 forward = camera_basis(orbit_pose).forward;
    camera.position = focus_point - forward * distance;
    [[maybe_unused]] const bool aligned = camera_look_at(camera, focus_point);
}

} // namespace javelin
