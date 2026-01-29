module;

#include <tracy/Tracy.hpp>

export module javelin.physics.solve;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.types;

namespace javelin::detail {
inline constexpr f32 kPositionSlop = 0.01f;
inline constexpr f32 kPositionCorrectionPercent = 0.8f;
inline constexpr u32 kSolverIterations = 8;
inline constexpr f32 kTangentEpsSq = 1e-8f;
} // namespace javelin::detail

export namespace javelin {

void solve_contacts(std::span<Vec3> position, std::span<Vec3> velocity, std::span<const f32> inv_mass,
                    std::span<const Contact> contacts, const f32 restitution, const f32 friction) {
    ZoneScopedN("Physics solve");
    if (contacts.empty()) {
        return;
    }

    for (u32 iteration = 0; iteration < detail::kSolverIterations; ++iteration) {
        for (const Contact contact : contacts) {
            const u32 a = contact.a;
            const u32 b = contact.b;
            const bool has_b = (b != kInvalidBody);
            const f32 inv_mass_a = inv_mass[a];
            const f32 inv_mass_b = has_b ? inv_mass[b] : 0.0f;
            const f32 inv_mass_sum = inv_mass_a + inv_mass_b;
            if (inv_mass_sum <= 0.0f) {
                continue;
            }

            const f32 correction_mag =
                std::max(contact.penetration - detail::kPositionSlop, 0.0f) *
                detail::kPositionCorrectionPercent / inv_mass_sum;
            if (correction_mag > 0.0f) {
                const Vec3 correction = contact.normal * correction_mag;
                position[a] -= correction * inv_mass_a;
                if (has_b) {
                    position[b] += correction * inv_mass_b;
                }
            }

            const Vec3 velocity_b = has_b ? velocity[b] : Vec3{};
            const Vec3 relative = velocity_b - velocity[a];
            const f32 vn = dot(relative, contact.normal);
            if (vn > 0.0f) {
                continue;
            }

            const f32 impulse_mag = -(1.0f + restitution) * vn / inv_mass_sum;
            const Vec3 impulse = contact.normal * impulse_mag;
            velocity[a] -= impulse * inv_mass_a;
            if (has_b) {
                velocity[b] += impulse * inv_mass_b;
            }

            const Vec3 velocity_b_after = has_b ? velocity[b] : Vec3{};
            const Vec3 relative_after = velocity_b_after - velocity[a];
            const Vec3 tangent_velocity = relative_after - contact.normal * dot(relative_after, contact.normal);
            const f32 tangent_speed_sq = tangent_velocity.length_sq();
            if (tangent_speed_sq <= detail::kTangentEpsSq) {
                continue;
            }

            const f32 tangent_speed = std::sqrt(tangent_speed_sq);
            const Vec3 tangent = tangent_velocity / tangent_speed;
            f32 friction_impulse_mag = -dot(relative_after, tangent) / inv_mass_sum;
            const f32 max_friction = friction * impulse_mag;
            friction_impulse_mag = std::clamp(friction_impulse_mag, -max_friction, max_friction);

            const Vec3 friction_impulse = tangent * friction_impulse_mag;
            velocity[a] -= friction_impulse * inv_mass_a;
            if (has_b) {
                velocity[b] += friction_impulse * inv_mass_b;
            }
        }
    }
}

} // namespace javelin
