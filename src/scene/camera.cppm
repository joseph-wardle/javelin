export module javelin.scene.camera;

import std;

import javelin.core.types;
import javelin.math.vec3;
import javelin.math.mat4;

export namespace javelin {

struct CameraLens final {
    f32 fov_y_radians{1.0f};
    f32 near_z{0.05f};
    f32 far_z{500.0f};
};

struct CameraState final {
    Vec3 position{};
    f32 yaw_radians{};   // yaw around world up
    f32 pitch_radians{}; // pitch around camera right
    CameraLens lens{};
};

struct CameraBasis final {
    Vec3 forward{};
    Vec3 right{};
    Vec3 up{};
};

[[nodiscard]] CameraBasis camera_basis(const CameraState &camera) noexcept;
[[nodiscard]] Mat4 camera_view(const CameraState &camera) noexcept;
[[nodiscard]] Mat4 camera_proj(const CameraLens &lens, f32 aspect) noexcept;

[[nodiscard]] CameraBasis camera_basis(const CameraState &camera) noexcept {
    const f32 cy = std::cos(camera.yaw_radians);
    const f32 sy = std::sin(camera.yaw_radians);
    const f32 cp = std::cos(camera.pitch_radians);
    const f32 sp = std::sin(camera.pitch_radians);

    const Vec3 forward{sy * cp, sp, -cy * cp};
    const Vec3 world_up = Vec3::unit_y();

    Vec3 right = cross(forward, world_up);
    if (!right.try_normalize()) {
        right = Vec3::unit_x();
    }
    const Vec3 up = cross(right, forward);

    return CameraBasis{.forward = forward, .right = right, .up = up};
}

[[nodiscard]] Mat4 camera_view(const CameraState &camera) noexcept {
    const CameraBasis basis = camera_basis(camera);
    const Vec3 position = camera.position;

    return Mat4::from_rows(Vec4{basis.right.x, basis.right.y, basis.right.z, -dot(basis.right, position)},
                           Vec4{basis.up.x, basis.up.y, basis.up.z, -dot(basis.up, position)},
                           Vec4{-basis.forward.x, -basis.forward.y, -basis.forward.z, dot(basis.forward, position)},
                           Vec4{0.0f, 0.0f, 0.0f, 1.0f});
}

[[nodiscard]] Mat4 camera_proj(const CameraLens &lens, f32 aspect) noexcept {
    const f32 f = 1.0f / std::tan(lens.fov_y_radians * 0.5f);
    const f32 inv_nf = 1.0f / (lens.near_z - lens.far_z);

    return Mat4::from_rows(
        Vec4{f / aspect, 0.0f, 0.0f, 0.0f}, Vec4{0.0f, f, 0.0f, 0.0f},
        Vec4{0.0f, 0.0f, (lens.far_z + lens.near_z) * inv_nf, 2.0f * lens.far_z * lens.near_z * inv_nf},
        Vec4{0.0f, 0.0f, -1.0f, 0.0f});
}

} // namespace javelin
