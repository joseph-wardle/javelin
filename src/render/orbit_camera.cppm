export module javelin.render.orbit_camera;

import std;

import javelin.core.types;
import javelin.math;
import javelin.platform.input;
import javelin.render.camera_controller;
import javelin.scene.camera;
import javelin.scene.bodies;
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

[[nodiscard]] inline SceneBounds compute_scene_bounds(const BodiesRead &view) noexcept {
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

} // namespace detail

struct OrbitCameraTuning final {
    f32 initial_yaw_radians{0.75f};
    f32 pitch_radians{-0.35f};
    f32 yaw_speed_radians{0.12f};
    f32 fixed_distance{16.0f};
};

struct OrbitCameraController final : CameraController {
    OrbitCameraTuning tuning{};
    Vec3 focus_point{};
    f32 distance{8.0f};
    f32 yaw_radians{tuning.initial_yaw_radians};
    bool framed{};

    void frame_scene(CameraState &camera, const BodiesRead &view, f32 aspect) noexcept;
    [[nodiscard]] CursorMode update(CameraState &camera, const InputFrame &input, f32 dt_seconds) noexcept override;
};

void OrbitCameraController::frame_scene(CameraState &camera, const BodiesRead &view, const f32 aspect) noexcept {
    (void)aspect;
    yaw_radians = tuning.initial_yaw_radians;
    framed = true;

    const detail::SceneBounds bounds = detail::compute_scene_bounds(view);
    if (!bounds.valid) {
        focus_point = Vec3{};
        distance = tuning.fixed_distance;
        static_cast<void>(update(camera, InputFrame{}, 0.0f));
        return;
    }

    const Vec3 center = (bounds.min + bounds.max) * 0.5f;

    focus_point = center;
    distance = tuning.fixed_distance;
    static_cast<void>(update(camera, InputFrame{}, 0.0f));
}

CursorMode OrbitCameraController::update(CameraState &camera, const InputFrame &, const f32 dt_seconds) noexcept {
    if (!framed) {
        return CursorMode::normal;
    }

    constexpr f32 tau = 6.28318530717958647692f;
    yaw_radians = std::remainder(yaw_radians + tuning.yaw_speed_radians * std::max(dt_seconds, 0.0f), tau);

    CameraState orbit_pose{};
    orbit_pose.yaw_radians = yaw_radians;
    orbit_pose.pitch_radians = tuning.pitch_radians;

    const Vec3 forward = camera_basis(orbit_pose).forward;
    camera.position = focus_point - forward * distance;
    [[maybe_unused]] const bool aligned = camera_look_at(camera, focus_point);
    return CursorMode::normal;
}

} // namespace javelin
