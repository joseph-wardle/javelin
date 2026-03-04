module;

#include <tracy/Tracy.hpp>

export module javelin.physics.solve;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.constraint_types;
import javelin.physics.types;

namespace javelin::detail {
// Solver contract:
// - manifold normals/anchors are authored by narrow phase in canonical orientation.
// - this module performs iterative projected Gauss-Seidel in velocity space,
//   then a positional correction pass for residual penetration.
// - only dynamic bodies (inv_mass > 0) are moved by impulses/corrections.
// 3D manifold stacks converge more slowly than single-point contacts.

// Iteration counts:
//   kSolverIterations         — velocity PGS passes per tick.  16 converges well
//                               for moderate stacks (≤ ~8 bodies) at 60 Hz.
//   kPositionSolverIterations — geometric correction passes after the velocity solve.
//                               4 removes residual penetration without over-stiffening
//                               resting contacts.
inline constexpr u32 kSolverIterations = 16;
inline constexpr u32 kPositionSolverIterations = 4;

// Degenerate-contact guards: skip impulse or correction when effective mass or dt
// is below these epsilons to avoid divide-by-zero and NaN propagation.
inline constexpr f32 kMassEps = 1e-8f;
inline constexpr f32 kDtEps = 1e-8f;

// Baumgarte stabilisation injects a closing velocity to resolve penetration:
//   kPenetrationBiasFactor — fraction of penetration depth resolved per tick (20%).
//                            Lower values are smoother but allow more sinking.
//   kPenetrationSlop       — 5 mm of free penetration before bias activates;
//                            suppresses micro-jitter on resting contacts.
//   kMaxPenetrationBias    — cap on injected correction velocity (m/s); prevents
//                            explosive separation from very deep interpenetrations.
inline constexpr f32 kPenetrationBiasFactor = 0.2f;
inline constexpr f32 kPenetrationSlop = 0.005f;
inline constexpr f32 kMaxPenetrationBias = 2.0f;

// Position solver: one-shot geometric correction applied after the velocity pass.
//   kPositionSlop             — 1 mm contacts are not corrected; avoids fighting
//                               resting-contact noise with position moves.
//   kPositionCorrectionFactor — 20% of residual error corrected per iteration;
//                               under-relaxed to prevent oscillation.
//   kAngularCorrectionEpsSq   — skip quaternion update when |Δθ|² < 1e-12
//                               (i.e. |Δθ| < 1e-6 rad) to avoid normalising noise.
inline constexpr f32 kPositionSlop = 0.001f;
inline constexpr f32 kPositionCorrectionFactor = 0.2f;
inline constexpr f32 kAngularCorrectionEpsSq = 1e-12f;

// Restitution is only applied when the closing speed exceeds 1 m/s.
// Below this threshold the contact is treated as resting and should not bounce.
inline constexpr f32 kRestitutionVelocityThreshold = 1.0f;

[[nodiscard]] inline Vec3 to_body_space(const Quat q, const Vec3 v) noexcept { return rotate(inverse_unit(q), v); }

[[nodiscard]] inline Vec3 to_world_space(const Quat q, const Vec3 v) noexcept { return rotate(q, v); }

[[nodiscard]] inline f32 inv_inertia_term(const Vec3 inv_inertia_body, const Quat q, const Vec3 v_world) noexcept {
    const Vec3 v_body = to_body_space(q, v_world);
    return dot(hadamard(inv_inertia_body, v_body), v_body);
}

[[nodiscard]] inline Vec3 apply_inv_inertia(const Vec3 inv_inertia_body, const Quat q, const Vec3 v_world) noexcept {
    const Vec3 v_body = to_body_space(q, v_world);
    const Vec3 scaled = hadamard(inv_inertia_body, v_body);
    return to_world_space(q, scaled);
}

// Per-contact-point pre-step data reused across solver iterations.
struct SolverPoint final {
    u32 manifold_index{};
    u32 point_index{};
    u32 a{};
    u32 b{kInvalidBody};
    Vec3 normal{};
    Vec3 tangent_u{};
    Vec3 tangent_v{};
    Vec3 r_a{};
    Vec3 r_b{};
    f32 normal_mass{};
    f32 tangent_mass_u{};
    f32 tangent_mass_v{};
    f32 velocity_bias{};
    f32 friction_coeff{}; // combined per-contact friction (min of two body materials)
};

[[nodiscard]] inline bool has_body_b(const SolverPoint &point) noexcept { return point.b != kInvalidBody; }

[[nodiscard]] inline f32 restitution_bias(const f32 normal_velocity, const f32 restitution_coeff) noexcept {
    if (normal_velocity >= -kRestitutionVelocityThreshold) {
        return 0.0f;
    }
    return -restitution_coeff * normal_velocity;
}

[[nodiscard]] inline bool has_warm_start_impulse(const ContactPoint &point) noexcept {
    return std::fabs(point.normal_impulse) > kMassEps || point.tangent_impulse.length_sq() > kMassEps * kMassEps;
}

[[nodiscard]] inline Vec3 project_and_clamp_tangent_impulse(const Vec3 cached_tangent_impulse, const Vec3 normal,
                                                            const f32 normal_impulse,
                                                            const f32 friction_coeff) noexcept {
    if (friction_coeff <= 0.0f || normal_impulse <= kMassEps) {
        return Vec3{};
    }

    // Contact normals rotate between frames; remove any leaked normal component from cached friction.
    Vec3 tangent_impulse = cached_tangent_impulse - normal * dot(cached_tangent_impulse, normal);
    const f32 tangent_impulse_sq = tangent_impulse.length_sq();
    if (tangent_impulse_sq <= kMassEps * kMassEps) {
        return Vec3{};
    }

    const f32 max_friction = friction_coeff * normal_impulse;
    const f32 max_friction_sq = max_friction * max_friction;
    if (tangent_impulse_sq > max_friction_sq && max_friction_sq > kMassEps * kMassEps) {
        tangent_impulse *= max_friction / std::sqrt(tangent_impulse_sq);
    }
    return tangent_impulse;
}

[[nodiscard]] inline std::pair<Vec3, Vec3> contact_tangent_frame(const Vec3 normal) noexcept {
    // Deterministic orthonormal basis from normal.
    Vec3 tangent_u =
        (std::fabs(normal.x) >= 0.57735026919f) ? Vec3{normal.y, -normal.x, 0.0f} : Vec3{0.0f, normal.z, -normal.y};
    if (!tangent_u.try_normalize()) {
        tangent_u = Vec3::unit_x();
    }
    Vec3 tangent_v = cross(normal, tangent_u);
    if (!tangent_v.try_normalize()) {
        tangent_v = cross(normal, Vec3::unit_y()).normalized_or_zero();
    }
    return std::pair{tangent_u, tangent_v};
}

[[nodiscard]] inline f32 axis_effective_mass_inverse(const SolverPoint &point, std::span<const f32> inv_mass,
                                                     std::span<const Vec3> inv_inertia_body,
                                                     std::span<const Quat> orientation, const Vec3 axis) noexcept {
    const f32 inv_mass_a = inv_mass[point.a];
    const f32 inv_mass_b = has_body_b(point) ? inv_mass[point.b] : 0.0f;
    const f32 inv_mass_sum = inv_mass_a + inv_mass_b;
    if (inv_mass_sum <= kMassEps) {
        return 0.0f;
    }

    const Vec3 ra_cross_axis = cross(point.r_a, axis);
    const f32 ang_term_a = inv_inertia_term(inv_inertia_body[point.a], orientation[point.a], ra_cross_axis);
    f32 ang_term_b = 0.0f;
    if (has_body_b(point)) {
        const Vec3 rb_cross_axis = cross(point.r_b, axis);
        ang_term_b = inv_inertia_term(inv_inertia_body[point.b], orientation[point.b], rb_cross_axis);
    }

    const f32 denom = inv_mass_sum + ang_term_a + ang_term_b;
    return (denom > kMassEps) ? (1.0f / denom) : 0.0f;
}

[[nodiscard]] inline f32 axis_effective_mass_inverse(const u32 a, const u32 b, const Vec3 r_a, const Vec3 r_b,
                                                     std::span<const f32> inv_mass,
                                                     std::span<const Vec3> inv_inertia_body,
                                                     std::span<const Quat> orientation, const Vec3 axis) noexcept {
    const f32 inv_mass_a = inv_mass[a];
    const f32 inv_mass_b = (b != kInvalidBody) ? inv_mass[b] : 0.0f;
    const f32 inv_mass_sum = inv_mass_a + inv_mass_b;
    if (inv_mass_sum <= kMassEps) {
        return 0.0f;
    }

    const Vec3 ra_cross_axis = cross(r_a, axis);
    const f32 ang_term_a = inv_inertia_term(inv_inertia_body[a], orientation[a], ra_cross_axis);
    f32 ang_term_b = 0.0f;
    if (b != kInvalidBody) {
        const Vec3 rb_cross_axis = cross(r_b, axis);
        ang_term_b = inv_inertia_term(inv_inertia_body[b], orientation[b], rb_cross_axis);
    }

    const f32 denom = inv_mass_sum + ang_term_a + ang_term_b;
    return (denom > kMassEps) ? (1.0f / denom) : 0.0f;
}

inline void apply_orientation_correction(Quat &orientation, const Vec3 delta_angle) noexcept {
    if (delta_angle.length_sq() <= kAngularCorrectionEpsSq) {
        return;
    }
    const Quat omega{delta_angle.x, delta_angle.y, delta_angle.z, 0.0f};
    orientation = orientation + (omega * orientation) * 0.5f;
    orientation.try_normalize();
}

[[nodiscard]] inline Vec3 relative_velocity_at_point(const SolverPoint &point, std::span<const Vec3> velocity,
                                                     std::span<const Vec3> angular_velocity) noexcept {
    const Vec3 v_a = velocity[point.a] + cross(angular_velocity[point.a], point.r_a);
    if (!has_body_b(point)) {
        return -v_a;
    }
    const Vec3 v_b = velocity[point.b] + cross(angular_velocity[point.b], point.r_b);
    return v_b - v_a;
}

inline void apply_impulse_at_point(const SolverPoint &point, const Vec3 impulse, std::span<Vec3> velocity,
                                   std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                                   std::span<const Vec3> inv_inertia_body, std::span<const Quat> orientation) noexcept {
    velocity[point.a] -= impulse * inv_mass[point.a];
    angular_velocity[point.a] -=
        apply_inv_inertia(inv_inertia_body[point.a], orientation[point.a], cross(point.r_a, impulse));

    if (!has_body_b(point)) {
        return;
    }

    velocity[point.b] += impulse * inv_mass[point.b];
    angular_velocity[point.b] +=
        apply_inv_inertia(inv_inertia_body[point.b], orientation[point.b], cross(point.r_b, impulse));
}

inline void warm_start_solver_point(const SolverPoint &solver_point, ContactPoint &point, std::span<Vec3> velocity,
                                    std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                                    std::span<const Vec3> inv_inertia_body,
                                    std::span<const Quat> orientation) noexcept {
    point.normal_impulse = std::max(point.normal_impulse, 0.0f);
    point.tangent_impulse = project_and_clamp_tangent_impulse(point.tangent_impulse, solver_point.normal,
                                                              point.normal_impulse, solver_point.friction_coeff);
    if (!has_warm_start_impulse(point)) {
        return;
    }
    const Vec3 impulse = solver_point.normal * point.normal_impulse + point.tangent_impulse;
    apply_impulse_at_point(solver_point, impulse, velocity, angular_velocity, inv_mass, inv_inertia_body, orientation);
}

[[nodiscard]] inline std::vector<SolverPoint> &solver_points_cache() {
    static thread_local std::vector<SolverPoint> cache{};
    return cache;
}

// Iteration count for the bilateral distance constraint velocity solve.
// 8 passes converge a rigid rod well at 60 Hz without over-burdening the
// contact solver budget (which runs 16 passes of its own).
inline constexpr u32 kConstraintIterations = 8;

// Baumgarte stabilisation factor for distance constraint position error:
// injects (factor / dt) * C_pos as a target closing velocity to correct drift.
// 20% per tick matches the contact penetration bias factor.
inline constexpr f32 kConstraintBiasFactor = 0.2f;

// Anchor-point distance below which the constraint axis is considered degenerate
// (anchors coincident within ~0.1 mm).  Skip rather than dividing by near-zero.
inline constexpr f32 kConstraintAxisEpsSq = 1e-8f;

// Per-constraint world-space geometry, precomputed once per tick before the
// velocity iteration loop.  Positions and orientations are not modified during
// the velocity solve so this data is invariant across all iterations.
struct ConstraintGeometry final {
    u32 a{};
    u32 b{};
    Vec3 r_a{};        // world-space anchor offset from body A COM (metres)
    Vec3 r_b{};        // world-space anchor offset from body B COM (metres)
    Vec3 n{};          // unit axis from anchor_a toward anchor_b
    Vec3 ra_cross_n{}; // r_a × n; precomputed to avoid cross in the inner loop
    Vec3 rb_cross_n{}; // r_b × n; precomputed to avoid cross in the inner loop
    f32 C_pos{};       // position violation: dist − rest_length (metres)
    f32 k_eff{};       // 1 / (w_eff + alpha_tilde); effective impulse scale
};

[[nodiscard]] inline std::vector<ConstraintGeometry> &constraint_geometry_cache() {
    static thread_local std::vector<ConstraintGeometry> cache{};
    return cache;
}
} // namespace javelin::detail

export namespace javelin {

void solve_contact_velocities(std::span<Vec3> velocity, std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                              std::span<const Vec3> inv_inertia_body, std::span<const Quat> orientation,
                              std::span<ContactManifold> manifolds, const f32 dt,
                              std::span<const f32> manifold_restitution, std::span<const f32> manifold_friction,
                              std::span<const u8> asleep) {
    ZoneScopedN("Physics solve");
#ifndef NDEBUG
    if (velocity.size() != angular_velocity.size() || velocity.size() != inv_mass.size() ||
        velocity.size() != inv_inertia_body.size() || velocity.size() != orientation.size() ||
        velocity.size() != asleep.size()) {
        log::error(physics,
                   "Velocity solver span size mismatch (vel={} ang_vel={} inv_mass={} inv_inertia={} orientation={} "
                   "asleep={})",
                   velocity.size(), angular_velocity.size(), inv_mass.size(), inv_inertia_body.size(),
                   orientation.size(), asleep.size());
        std::terminate();
    }
#endif
    if (manifolds.empty()) {
        return;
    }

    const u32 body_count = static_cast<u32>(velocity.size());
    const f32 inv_dt = (dt > detail::kDtEps) ? (1.0f / dt) : 0.0f;

    usize point_count = 0;
    for (u32 manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
        const ContactManifold &manifold = manifolds[manifold_index];
#ifndef NDEBUG
        if (manifold.point_count > kMaxManifoldPoints) {
            log::error(physics, "Solver manifold has invalid point_count={} (index={} a={} b={})", manifold.point_count,
                       manifold_index, manifold.a, manifold.b);
            std::terminate();
        }
        if (manifold.a >= body_count || (manifold.b != kInvalidBody && manifold.b >= body_count)) {
            log::error(physics, "Solver manifold body id out of range (index={} a={} b={} count={})", manifold_index,
                       manifold.a, manifold.b, body_count);
            std::terminate();
        }
#endif
        point_count += manifold.point_count;
    }
    if (point_count == 0u) {
        return;
    }

    std::vector<detail::SolverPoint> &solver_points = detail::solver_points_cache();
    solver_points.clear();
    solver_points.reserve(point_count);

    // Pre-step: world anchors, tangent basis, effective masses, and velocity bias.
    // Skip manifolds where both participants are asleep: a body is only still asleep
    // if all its contacts are ground/static/other-sleeping (wake_sleeping_bodies_with_contacts_
    // already cleared asleep for any body touched by an awake dynamic neighbour).
    for (u32 manifold_index = 0; manifold_index < manifolds.size(); ++manifold_index) {
        ContactManifold &manifold = manifolds[manifold_index];
        if (manifold.point_count == 0u) {
            continue;
        }
        if (asleep[manifold.a] != 0u) {
            continue;
        }
        const f32 rest_coeff = std::clamp(manifold_restitution[manifold_index], 0.0f, 1.0f);
        const f32 fric_coeff = std::max(manifold_friction[manifold_index], 0.0f);
        Vec3 normal = manifold.normal;
        if (!normal.try_normalize()) {
#ifndef NDEBUG
            log::error(physics, "Solver manifold has invalid normal (index={} a={} b={})", manifold_index, manifold.a,
                       manifold.b);
            std::terminate();
#else
            continue;
#endif
        }
        const auto [tangent_u, tangent_v] = detail::contact_tangent_frame(normal);

        for (u32 point_index = 0; point_index < manifold.point_count; ++point_index) {
            ContactPoint &point = manifold.points[point_index];

            detail::SolverPoint solver_point{
                .manifold_index = manifold_index,
                .point_index = point_index,
                .a = manifold.a,
                .b = manifold.b,
                .normal = normal,
                .tangent_u = tangent_u,
                .tangent_v = tangent_v,
                .r_a = rotate(orientation[manifold.a], point.local_anchor_a),
                .r_b = (manifold.b != kInvalidBody) ? rotate(orientation[manifold.b], point.local_anchor_b) : Vec3{},
                .friction_coeff = fric_coeff,
            };

            solver_point.normal_mass =
                detail::axis_effective_mass_inverse(solver_point, inv_mass, inv_inertia_body, orientation, normal);
            solver_point.tangent_mass_u =
                detail::axis_effective_mass_inverse(solver_point, inv_mass, inv_inertia_body, orientation, tangent_u);
            solver_point.tangent_mass_v =
                detail::axis_effective_mass_inverse(solver_point, inv_mass, inv_inertia_body, orientation, tangent_v);

            const f32 separation = point.separation + detail::kPenetrationSlop;
            const f32 penetration_bias =
                (inv_dt > 0.0f) ? (-detail::kPenetrationBiasFactor * inv_dt * std::min(separation, 0.0f)) : 0.0f;
            const f32 clamped_penetration_bias = std::min(penetration_bias, detail::kMaxPenetrationBias);

            const Vec3 relative_velocity = detail::relative_velocity_at_point(solver_point, velocity, angular_velocity);
            const f32 normal_velocity = dot(relative_velocity, normal);
            const f32 restitution_velocity_bias = detail::restitution_bias(normal_velocity, rest_coeff);
            solver_point.velocity_bias = std::max(clamped_penetration_bias, restitution_velocity_bias);

            solver_points.push_back(solver_point);
        }
    }

    // Warm start: apply accumulated normal impulse plus projected/clamped world-space friction impulse.
    for (const detail::SolverPoint &solver_point : solver_points) {
        ContactPoint &point = manifolds[solver_point.manifold_index].points[solver_point.point_index];
        detail::warm_start_solver_point(solver_point, point, velocity, angular_velocity, inv_mass, inv_inertia_body,
                                        orientation);
    }

    // Iterative projected Gauss-Seidel.
    for (u32 iteration = 0; iteration < detail::kSolverIterations; ++iteration) {
        for (const detail::SolverPoint &solver_point : solver_points) {
            ContactPoint &point = manifolds[solver_point.manifold_index].points[solver_point.point_index];

            Vec3 relative_velocity = detail::relative_velocity_at_point(solver_point, velocity, angular_velocity);
            const f32 vn = dot(relative_velocity, solver_point.normal);
            const f32 delta_normal = solver_point.normal_mass * (solver_point.velocity_bias - vn);
            const f32 old_normal_impulse = point.normal_impulse;
            point.normal_impulse = std::max(old_normal_impulse + delta_normal, 0.0f);
            const f32 applied_normal_impulse = point.normal_impulse - old_normal_impulse;
            if (std::fabs(applied_normal_impulse) > detail::kMassEps) {
                detail::apply_impulse_at_point(solver_point, solver_point.normal * applied_normal_impulse, velocity,
                                               angular_velocity, inv_mass, inv_inertia_body, orientation);
            }

            relative_velocity = detail::relative_velocity_at_point(solver_point, velocity, angular_velocity);
            const f32 vt_u = dot(relative_velocity, solver_point.tangent_u);
            const f32 vt_v = dot(relative_velocity, solver_point.tangent_v);
            const Vec3 old_tangent_impulse = point.tangent_impulse;
            const f32 old_tangent_u = dot(old_tangent_impulse, solver_point.tangent_u);
            const f32 old_tangent_v = dot(old_tangent_impulse, solver_point.tangent_v);

            f32 new_tangent_u = old_tangent_u - solver_point.tangent_mass_u * vt_u;
            f32 new_tangent_v = old_tangent_v - solver_point.tangent_mass_v * vt_v;

            // Coulomb friction on the accumulated tangent impulse vector.
            const f32 max_friction = solver_point.friction_coeff * point.normal_impulse;
            const f32 max_friction_sq = max_friction * max_friction;
            const f32 tangent_impulse_sq = new_tangent_u * new_tangent_u + new_tangent_v * new_tangent_v;
            if (tangent_impulse_sq > max_friction_sq && tangent_impulse_sq > detail::kMassEps * detail::kMassEps) {
                const f32 scale = max_friction / std::sqrt(tangent_impulse_sq);
                new_tangent_u *= scale;
                new_tangent_v *= scale;
            }

            const Vec3 new_tangent_impulse =
                solver_point.tangent_u * new_tangent_u + solver_point.tangent_v * new_tangent_v;
            point.tangent_impulse = new_tangent_impulse;

            const Vec3 applied_tangent_impulse = new_tangent_impulse - old_tangent_impulse;
            if (applied_tangent_impulse.length_sq() > detail::kMassEps * detail::kMassEps) {
                detail::apply_impulse_at_point(solver_point, applied_tangent_impulse, velocity, angular_velocity,
                                               inv_mass, inv_inertia_body, orientation);
            }
        }
    }
}

void solve_contact_penetration(std::span<Vec3> position, std::span<Quat> orientation, std::span<const f32> inv_mass,
                               std::span<const Vec3> inv_inertia_body, std::span<const ContactManifold> manifolds,
                               std::span<const u8> asleep) {
    ZoneScopedN("Physics solve positions");
#ifndef NDEBUG
    if (position.size() != orientation.size() || position.size() != inv_mass.size() ||
        position.size() != inv_inertia_body.size() || position.size() != asleep.size()) {
        log::error(
            physics,
            "Position solver span size mismatch (position={} orientation={} inv_mass={} inv_inertia={} asleep={})",
            position.size(), orientation.size(), inv_mass.size(), inv_inertia_body.size(), asleep.size());
        std::terminate();
    }
#endif
    if (manifolds.empty()) {
        return;
    }

    const u32 body_count = static_cast<u32>(position.size());
    for (u32 iteration = 0; iteration < detail::kPositionSolverIterations; ++iteration) {
        bool any_correction = false;

        for (const ContactManifold &manifold : manifolds) {
            if (manifold.point_count == 0u) {
                continue;
            }
            if (asleep[manifold.a] != 0u) {
                continue;
            }
#ifndef NDEBUG
            if (manifold.a >= body_count || (manifold.b != kInvalidBody && manifold.b >= body_count)) {
                log::error(physics, "Position solver manifold body id out of range (a={} b={} count={})", manifold.a,
                           manifold.b, body_count);
                std::terminate();
            }
#endif

            Vec3 normal = manifold.normal;
            if (!normal.try_normalize()) {
#ifndef NDEBUG
                log::error(physics, "Position solver manifold has invalid normal (a={} b={})", manifold.a, manifold.b);
                std::terminate();
#else
                continue;
#endif
            }

            const u32 a = manifold.a;
            const bool has_body_b = manifold.b != kInvalidBody;
            const u32 b = has_body_b ? manifold.b : kInvalidBody;

            for (u32 point_index = 0; point_index < manifold.point_count; ++point_index) {
                const ContactPoint &contact_point = manifold.points[point_index];
                const Vec3 r_a = rotate(orientation[a], contact_point.local_anchor_a);
                const Vec3 world_a = position[a] + r_a;

                Vec3 r_b{};
                Vec3 world_b{};
                f32 separation = 0.0f;
                if (has_body_b) {
                    r_b = rotate(orientation[b], contact_point.local_anchor_b);
                    world_b = position[b] + r_b;
                    separation = dot(world_b - world_a, normal);
                } else {
                    // Ground manifold uses body B as an implicit static plane through the origin.
                    separation = dot(-world_a, normal);
                }

                const f32 constraint = separation + detail::kPositionSlop;
                if (constraint >= 0.0f) {
                    continue;
                }

                const f32 normal_mass = detail::axis_effective_mass_inverse(a, b, r_a, r_b, inv_mass, inv_inertia_body,
                                                                            std::span<const Quat>{orientation}, normal);
                if (normal_mass <= detail::kMassEps) {
                    continue;
                }

                const f32 delta_lambda = -detail::kPositionCorrectionFactor * constraint * normal_mass;
                if (delta_lambda <= detail::kMassEps) {
                    continue;
                }
                const Vec3 correction = normal * delta_lambda;

                position[a] -= correction * inv_mass[a];
                const Vec3 angular_correction_a =
                    detail::apply_inv_inertia(inv_inertia_body[a], orientation[a], cross(r_a, correction));
                detail::apply_orientation_correction(orientation[a], -angular_correction_a);

                if (has_body_b) {
                    position[b] += correction * inv_mass[b];
                    const Vec3 angular_correction_b =
                        detail::apply_inv_inertia(inv_inertia_body[b], orientation[b], cross(r_b, correction));
                    detail::apply_orientation_correction(orientation[b], angular_correction_b);
                }

                any_correction = true;
            }
        }

        if (!any_correction) {
            break;
        }
    }
}

// Solves bilateral distance constraints between body pairs using a velocity-level
// XPBD impulse with Baumgarte position stabilisation.
//
// Each constraint enforces: |p_b − p_a| = rest_length
//   where p_a = position[body_a] + rotate(orientation[body_a], anchor_a)
//         p_b = position[body_b] + rotate(orientation[body_b], anchor_b)
//
// Formulation per iteration:
//   n            = (p_b − p_a) / |p_b − p_a|               (constraint axis)
//   v_n          = dot(v_b + ω_b×r_b − v_a − ω_a×r_a, n)  (relative anchor velocity)
//   bias         = (kConstraintBiasFactor / dt) * C_pos     (Baumgarte position correction)
//   alpha_tilde  = compliance / dt²                         (XPBD softened denominator)
//   k_eff        = 1 / (w_eff + alpha_tilde)                (effective impulse scale)
//   delta_lambda = -(v_n + bias) * k_eff                    (bilateral; no sign clamp)
//
// Impulse applied as:
//   v_a  -= n * delta_lambda * inv_mass_a
//   ω_a  -= I_a⁻¹(r_a × n) * delta_lambda
//   v_b  += n * delta_lambda * inv_mass_b
//   ω_b  += I_b⁻¹(r_b × n) * delta_lambda
//
// Positions are not modified here; positional drift is corrected purely through
// the Baumgarte bias term injected into the velocity target.
void solve_distance_constraints(std::span<Vec3> velocity, std::span<Vec3> angular_velocity,
                                std::span<const f32> inv_mass, std::span<const Vec3> inv_inertia_body,
                                std::span<const Quat> orientation, std::span<const Vec3> position,
                                std::span<const DistanceConstraint> constraints, const f32 dt,
                                std::span<const u8> asleep) {
    ZoneScopedN("Physics solve constraints");

    if (constraints.empty()) {
        return;
    }
#ifndef NDEBUG
    if (velocity.size() != angular_velocity.size() || velocity.size() != inv_mass.size() ||
        velocity.size() != inv_inertia_body.size() || velocity.size() != orientation.size() ||
        velocity.size() != position.size() || velocity.size() != asleep.size()) {
        log::error(physics,
                   "Constraint solver span size mismatch (vel={} ang_vel={} inv_mass={} inv_inertia={} "
                   "orientation={} position={} asleep={})",
                   velocity.size(), angular_velocity.size(), inv_mass.size(), inv_inertia_body.size(),
                   orientation.size(), position.size(), asleep.size());
        std::terminate();
    }
#endif
    if (dt <= detail::kDtEps) {
        return;
    }

    const f32 inv_dt = 1.0f / dt;
    const f32 inv_dt2 = inv_dt * inv_dt;

    // Precompute world-space geometry from current positions and orientations.
    // Positions and orientations are not modified during this function, so the
    // constraint axis, anchor offsets, and effective mass are invariant across all
    // velocity iterations.
    std::vector<detail::ConstraintGeometry> &geom = detail::constraint_geometry_cache();
    geom.clear();
    geom.reserve(constraints.size());

    for (const DistanceConstraint &c : constraints) {
        // Skip constraints where both participants are asleep.  One-asleep constraints
        // are processed so the awake body remains properly constrained until both sleep.
        if (asleep[c.body_a] != 0u && asleep[c.body_b] != 0u) {
            continue;
        }

        const Vec3 r_a = rotate(orientation[c.body_a], c.anchor_a);
        const Vec3 r_b = rotate(orientation[c.body_b], c.anchor_b);
        const Vec3 p_a = position[c.body_a] + r_a;
        const Vec3 p_b = position[c.body_b] + r_b;

        Vec3 n = p_b - p_a;
        const f32 dist_sq = n.length_sq();
        if (dist_sq < detail::kConstraintAxisEpsSq) {
            // Anchors are coincident: axis is undefined.  Skip this tick; other
            // forces will separate the bodies and restore a usable direction.
            continue;
        }
        const f32 dist = std::sqrt(dist_sq);
        n *= (1.0f / dist);

        // C_pos > 0: bodies are too far apart (constraint pulls them together).
        // C_pos < 0: bodies are too close   (constraint pushes them apart).
        const f32 C_pos = dist - c.rest_length;

        const Vec3 ra_cross_n = cross(r_a, n);
        const Vec3 rb_cross_n = cross(r_b, n);

        // Translational + rotational inverse mass along the constraint axis.
        const f32 w_a = inv_mass[c.body_a] +
                        detail::inv_inertia_term(inv_inertia_body[c.body_a], orientation[c.body_a], ra_cross_n);
        const f32 w_b = inv_mass[c.body_b] +
                        detail::inv_inertia_term(inv_inertia_body[c.body_b], orientation[c.body_b], rb_cross_n);

        // XPBD: alpha_tilde = compliance / dt² softens the constraint.
        // compliance = 0 gives a rigid bilateral rod.
        const f32 alpha_tilde = c.compliance * inv_dt2;
        const f32 denom = w_a + w_b + alpha_tilde;
        if (denom <= detail::kMassEps) {
            continue; // both bodies have infinite mass — constraint has no effect
        }

        geom.push_back(detail::ConstraintGeometry{
            .a = c.body_a,
            .b = c.body_b,
            .r_a = r_a,
            .r_b = r_b,
            .n = n,
            .ra_cross_n = ra_cross_n,
            .rb_cross_n = rb_cross_n,
            .C_pos = C_pos,
            .k_eff = 1.0f / denom,
        });
    }

    if (geom.empty()) {
        return;
    }

    // Iterative velocity correction: drive the relative anchor-point velocity toward
    // the Baumgarte position-correction target.
    for (u32 iter = 0; iter < detail::kConstraintIterations; ++iter) {
        for (const detail::ConstraintGeometry &g : geom) {
            const Vec3 v_a = velocity[g.a] + cross(angular_velocity[g.a], g.r_a);
            const Vec3 v_b = velocity[g.b] + cross(angular_velocity[g.b], g.r_b);
            const f32 v_n = dot(v_b - v_a, g.n);

            // Baumgarte bias drives v_n toward −(factor/dt)*C_pos to correct drift.
            const f32 bias = detail::kConstraintBiasFactor * inv_dt * g.C_pos;
            const f32 delta_lambda = -(v_n + bias) * g.k_eff;

            // Apply impulse along the constraint axis.
            // cross(r, n * delta_lambda) == ra_cross_n * delta_lambda (precomputed).
            const Vec3 impulse = g.n * delta_lambda;
            velocity[g.a] -= impulse * inv_mass[g.a];
            angular_velocity[g.a] -=
                detail::apply_inv_inertia(inv_inertia_body[g.a], orientation[g.a], g.ra_cross_n * delta_lambda);
            velocity[g.b] += impulse * inv_mass[g.b];
            angular_velocity[g.b] +=
                detail::apply_inv_inertia(inv_inertia_body[g.b], orientation[g.b], g.rb_cross_n * delta_lambda);
        }
    }
}

} // namespace javelin
