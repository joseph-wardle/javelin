module;

#include <tracy/Tracy.hpp>

export module javelin.physics.integrate;

import std;
import javelin.core.types;
import javelin.math;

namespace javelin::detail {
// Skip orientation integration when |angular_velocity|² < kAngularVelocityEpsSq
// (i.e., |ω| < 1e-6 rad/s).  Avoids computing a near-identity quaternion step
// that would be normalised away immediately and contributes only floating-point noise.
inline constexpr f32 kAngularVelocityEpsSq = 1e-12f;
} // namespace javelin::detail

export namespace javelin {

// Integration contract:
// - all spans in each function address the same body index domain.
// - inv_mass[i] == 0 marks static/kinematic bodies that are skipped.
// - asleep[i] != 0 marks sleeping bodies that are skipped (functions that accept asleep).
// - loops are single-pass, contiguous, and allocation-free.
void integrate_gravity_velocity(std::span<Vec3> velocity, std::span<const f32> inv_mass, std::span<const u8> asleep,
                                const f32 gravity, const f32 dt) noexcept {
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
    // Continuous drag integrates to v *= exp(-damping * dt). For fixed small dt we use
    // the first-order form (1 - damping * dt), then clamp to [0, 1] for stable decay.
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
    if (position.size() != velocity.size() || position.size() != inv_mass.size() || position.size() != asleep.size()) {
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

// Clamps near-zero velocities to zero for awake dynamic bodies in contact.
// Called every tick after the solver and damping, before integration.
//
// This helper uses a small per-body consecutive-tick counter before clamping.
// The hysteresis avoids zeroing legitimate slow motion that briefly crosses the
// speed thresholds (for example pendulum apex motion), while still removing
// persistent solver jitter on settled stacks.
void clamp_resting_contact_velocities(std::span<Vec3> velocity, std::span<Vec3> angular_velocity,
                                      std::span<const f32> inv_mass, std::span<const u8> in_contact,
                                      std::span<const u8> asleep, std::span<u8> consecutive_rest_ticks,
                                      const f32 linear_speed_sq_threshold, const f32 angular_speed_sq_threshold,
                                      const u8 min_rest_ticks_to_clamp) noexcept {
    ZoneScopedN("Physics clamp resting velocities");
#ifndef NDEBUG
    if (velocity.size() != angular_velocity.size() || velocity.size() != inv_mass.size() ||
        velocity.size() != in_contact.size() || velocity.size() != asleep.size() ||
        velocity.size() != consecutive_rest_ticks.size()) {
        std::terminate();
    }
#endif
    if (min_rest_ticks_to_clamp == 0u) {
        return;
    }

    const u32 count = static_cast<u32>(velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u || in_contact[i] == 0u) {
            consecutive_rest_ticks[i] = 0u;
            continue;
        }

        const bool under_threshold = velocity[i].length_sq() <= linear_speed_sq_threshold &&
                                     angular_velocity[i].length_sq() <= angular_speed_sq_threshold;
        if (!under_threshold) {
            consecutive_rest_ticks[i] = 0u;
            continue;
        }

        const u8 next_ticks = static_cast<u8>(std::min<u32>(static_cast<u32>(consecutive_rest_ticks[i]) + 1u, 255u));
        consecutive_rest_ticks[i] = next_ticks;
        if (next_ticks >= min_rest_ticks_to_clamp) {
            velocity[i] = Vec3{};
            angular_velocity[i] = Vec3{};
        }
    }
}

void apply_angular_damping(std::span<Vec3> angular_velocity, std::span<const f32> inv_mass, std::span<const u8> asleep,
                           const f32 damping, const f32 dt) noexcept {
    ZoneScopedN("Physics angular damping");
#ifndef NDEBUG
    if (angular_velocity.size() != inv_mass.size() || angular_velocity.size() != asleep.size()) {
        std::terminate();
    }
#endif
    if (damping <= 0.0f) {
        return;
    }
    // Continuous drag integrates to omega *= exp(-damping * dt). For fixed small dt we use
    // the first-order form (1 - damping * dt), then clamp to [0, 1] for stable decay.
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
