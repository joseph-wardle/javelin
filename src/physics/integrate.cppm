module;

#include <tracy/Tracy.hpp>

export module javelin.physics.integrate;

import std;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;

namespace javelin::detail {
// Skip orientation integration when |angular_velocity|² < kAngularVelocityEpsSq
// (i.e., |ω| < 1e-6 rad/s).  Avoids computing a near-identity quaternion step
// that would be normalised away immediately and contributes only floating-point noise.
inline constexpr f32 kAngularVelocityEpsSq = 1e-12f;
}

export namespace javelin {

// Integration contract:
// - all spans in each function address the same body index domain.
// - inv_mass[i] == 0 marks static/kinematic bodies that are skipped.
// - asleep[i] != 0 marks sleeping bodies that are skipped (functions that accept asleep).
// - loops are single-pass, contiguous, and allocation-free.
void integrate_gravity_velocity(std::span<Vec3> velocity, std::span<const f32> inv_mass,
                                std::span<const u8> asleep, const f32 gravity, const f32 dt) noexcept {
    ZoneScopedN("Physics accumulate forces");
#ifndef NDEBUG
    if (velocity.size() != inv_mass.size() || velocity.size() != asleep.size()) {
        std::terminate();
    }
#endif
    const u32 count = static_cast<u32>(velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
            continue;
        }
        velocity[i].y += gravity * dt;
    }
}

void apply_linear_damping(std::span<Vec3> velocity, std::span<const f32> inv_mass, std::span<const u8> asleep,
                          const f32 damping, const f32 dt) noexcept {
    ZoneScopedN("Physics linear damping");
#ifndef NDEBUG
    if (velocity.size() != inv_mass.size() || velocity.size() != asleep.size()) {
        std::terminate();
    }
#endif
    if (damping <= 0.0f) {
        return;
    }
    const f32 scale = std::clamp(1.0f - damping * dt, 0.0f, 1.0f);
    const u32 count = static_cast<u32>(velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
            continue;
        }
        velocity[i] *= scale;
    }
}

void integrate_positions(std::span<Vec3> position, std::span<const Vec3> velocity, std::span<const f32> inv_mass,
                         std::span<const u8> asleep, const f32 dt) noexcept {
    ZoneScopedN("Physics integrate positions");
#ifndef NDEBUG
    if (position.size() != velocity.size() || position.size() != inv_mass.size() ||
        position.size() != asleep.size()) {
        std::terminate();
    }
#endif
    const u32 count = static_cast<u32>(position.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
            continue;
        }
        position[i] += velocity[i] * dt;
    }
}

void integrate_orientations(std::span<Quat> orientation, std::span<const Vec3> angular_velocity,
                            std::span<const f32> inv_mass, std::span<const u8> asleep, const f32 dt) noexcept {
    ZoneScopedN("Physics integrate orientations");
#ifndef NDEBUG
    if (orientation.size() != angular_velocity.size() || orientation.size() != inv_mass.size() ||
        orientation.size() != asleep.size()) {
        std::terminate();
    }
#endif
    const u32 count = static_cast<u32>(orientation.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
            continue;
        }
        const Vec3 angular_velocity_world = angular_velocity[i];
        if (angular_velocity_world.length_sq() <= detail::kAngularVelocityEpsSq) {
            continue;
        }

        Quat orientation_world = orientation[i];
        // q' = 0.5 * omega * q, where omega = (wx, wy, wz, 0) in world space.
        const Quat angular_velocity_quat{angular_velocity_world.x, angular_velocity_world.y, angular_velocity_world.z,
                                         0.0f};
        const Quat orientation_derivative = angular_velocity_quat * orientation_world;
        orientation_world = orientation_world + orientation_derivative * (0.5f * dt);
        orientation_world.try_normalize();
        orientation[i] = orientation_world;
    }
}

void apply_angular_damping(std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                           std::span<const u8> asleep, const f32 damping, const f32 dt) noexcept {
    ZoneScopedN("Physics angular damping");
#ifndef NDEBUG
    if (angular_velocity.size() != inv_mass.size() || angular_velocity.size() != asleep.size()) {
        std::terminate();
    }
#endif
    if (damping <= 0.0f) {
        return;
    }
    const f32 scale = std::clamp(1.0f - damping * dt, 0.0f, 1.0f);
    const u32 count = static_cast<u32>(angular_velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
            continue;
        }
        angular_velocity[i] *= scale;
    }
}

} // namespace javelin
