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

void solve_contacts(std::span<Vec3> position, std::span<Vec3> velocity, std::span<Vec3> angular_velocity,
                    std::span<const f32> inv_mass, std::span<const Vec3> inv_inertia, std::span<const Contact> contacts,
                    const f32 restitution, const f32 friction) {
    ZoneScopedN("Physics solve");
    if (contacts.empty()) {
        return;
    }

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

        const f32 correction_mag = std::max(contact.penetration - detail::kPositionSlop, 0.0f) *
                                   detail::kPositionCorrectionPercent / inv_mass_sum;
        if (correction_mag > 0.0f) {
            const Vec3 correction = contact.normal * correction_mag;
            position[a] -= correction * inv_mass_a;
            if (has_b) {
                position[b] += correction * inv_mass_b;
            }
        }
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

            const Vec3 r_a = contact.r_a;
            const Vec3 r_b = contact.r_b;
            const Vec3 ang_a = angular_velocity[a];
            const Vec3 ang_b = has_b ? angular_velocity[b] : Vec3{};
            const Vec3 vel_a = velocity[a] + cross(ang_a, r_a);
            const Vec3 vel_b = has_b ? (velocity[b] + cross(ang_b, r_b)) : Vec3{};
            const Vec3 relative = vel_b - vel_a;
            const f32 vn = dot(relative, contact.normal);
            if (vn > 0.0f) {
                continue;
            }

            const Vec3 inv_inertia_a = inv_inertia[a];
            const Vec3 inv_inertia_b = has_b ? inv_inertia[b] : Vec3{};
            const Vec3 ra_cross_n = cross(r_a, contact.normal);
            const Vec3 rb_cross_n = cross(r_b, contact.normal);
            const f32 ang_term_a = dot(hadamard(inv_inertia_a, ra_cross_n), ra_cross_n);
            const f32 ang_term_b = has_b ? dot(hadamard(inv_inertia_b, rb_cross_n), rb_cross_n) : 0.0f;
            const f32 impulse_denom = inv_mass_sum + ang_term_a + ang_term_b;
            if (impulse_denom <= 0.0f) {
                continue;
            }

            const f32 impulse_mag = -(1.0f + restitution) * vn / impulse_denom;
            const Vec3 impulse = contact.normal * impulse_mag;
            velocity[a] -= impulse * inv_mass_a;
            if (has_b) {
                velocity[b] += impulse * inv_mass_b;
            }
            angular_velocity[a] -= hadamard(inv_inertia_a, cross(r_a, impulse));
            if (has_b) {
                angular_velocity[b] += hadamard(inv_inertia_b, cross(r_b, impulse));
            }

            const Vec3 ang_a_after = angular_velocity[a];
            const Vec3 ang_b_after = has_b ? angular_velocity[b] : Vec3{};
            const Vec3 vel_a_after = velocity[a] + cross(ang_a_after, r_a);
            const Vec3 vel_b_after = has_b ? (velocity[b] + cross(ang_b_after, r_b)) : Vec3{};
            const Vec3 relative_after = vel_b_after - vel_a_after;
            const Vec3 tangent_velocity = relative_after - contact.normal * dot(relative_after, contact.normal);
            const f32 tangent_speed_sq = tangent_velocity.length_sq();
            if (tangent_speed_sq <= detail::kTangentEpsSq) {
                continue;
            }

            const f32 tangent_speed = std::sqrt(tangent_speed_sq);
            const Vec3 tangent = tangent_velocity / tangent_speed;
            const Vec3 ra_cross_t = cross(r_a, tangent);
            const Vec3 rb_cross_t = cross(r_b, tangent);
            const f32 ang_t_a = dot(hadamard(inv_inertia_a, ra_cross_t), ra_cross_t);
            const f32 ang_t_b = has_b ? dot(hadamard(inv_inertia_b, rb_cross_t), rb_cross_t) : 0.0f;
            const f32 friction_denom = inv_mass_sum + ang_t_a + ang_t_b;
            if (friction_denom <= 0.0f) {
                continue;
            }
            f32 friction_impulse_mag = -dot(relative_after, tangent) / friction_denom;
            const f32 max_friction = friction * impulse_mag;
            friction_impulse_mag = std::clamp(friction_impulse_mag, -max_friction, max_friction);

            const Vec3 friction_impulse = tangent * friction_impulse_mag;
            velocity[a] -= friction_impulse * inv_mass_a;
            if (has_b) {
                velocity[b] += friction_impulse * inv_mass_b;
            }
            angular_velocity[a] -= hadamard(inv_inertia_a, cross(r_a, friction_impulse));
            if (has_b) {
                angular_velocity[b] += hadamard(inv_inertia_b, cross(r_b, friction_impulse));
            }
        }
    }
}

} // namespace javelin
