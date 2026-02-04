module;

#include <tracy/Tracy.hpp>

export module javelin.physics.integrate;

import std;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;

export namespace javelin {

void accumulate_forces(std::span<Vec3> velocity, std::span<const f32> inv_mass, const f32 gravity,
                       const f32 dt) noexcept {
    ZoneScopedN("Physics accumulate forces");
    const u32 count = static_cast<u32>(velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        velocity[i].y += gravity * dt;
    }
}

void integrate_predicted_positions(std::span<Vec3> position, std::span<const Vec3> velocity,
                                   std::span<const f32> inv_mass, const f32 dt) noexcept {
    ZoneScopedN("Physics integrate positions");
    const u32 count = static_cast<u32>(position.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        position[i] += velocity[i] * dt;
    }
}

void integrate_predicted_orientations(std::span<Quat> orientation, std::span<const Vec3> angular_velocity,
                                      std::span<const f32> inv_mass, const f32 dt) noexcept {
    ZoneScopedN("Physics integrate orientations");
    const u32 count = static_cast<u32>(orientation.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        const Vec3 w = angular_velocity[i];
        if (w.length_sq() <= 1e-12f) {
            continue;
        }

        Quat q = orientation[i];
        // q' = 0.5 * omega * q, where omega = (wx, wy, wz, 0) in world space.
        const Quat omega{w.x, w.y, w.z, 0.0f};
        const Quat dq = omega * q;
        q = q + dq * (0.5f * dt);
        q.try_normalize();
        orientation[i] = q;
    }
}

void apply_angular_damping(std::span<Vec3> angular_velocity, std::span<const f32> inv_mass, const f32 damping,
                           const f32 dt) noexcept {
    ZoneScopedN("Physics angular damping");
    if (damping <= 0.0f) {
        return;
    }
    const f32 scale = std::clamp(1.0f - damping * dt, 0.0f, 1.0f);
    const u32 count = static_cast<u32>(angular_velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        angular_velocity[i] *= scale;
    }
}

} // namespace javelin
