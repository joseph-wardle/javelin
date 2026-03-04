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

// World-space inverse inertia tensor cached per awake body for this tick:
// apply(v) = col0 * v.x + col1 * v.y + col2 * v.z.
// Caching avoids repeated rotate-to-body / rotate-to-world for every solver
// point impulse application and effective-mass query.
struct WorldInvInertia final {
    Vec3 col0{};
    Vec3 col1{};
    Vec3 col2{};
};

struct WorldInvInertiaCache final {
    std::vector<WorldInvInertia> tensors{};
    std::vector<u8> valid{};
};

[[nodiscard]] inline WorldInvInertiaCache &world_inv_inertia_cache() {
    static thread_local WorldInvInertiaCache cache{};
    return cache;
}

[[nodiscard]] inline bool has_body_b(const u32 body_b) noexcept { return body_b != kInvalidBody; }

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

// Solver-point data streams. Hot fields are stored as independent contiguous
// arrays to reduce per-point cache footprint inside the PGS iterations.
struct SolverPointStreams final {
    std::vector<ContactPoint *> contact_point{};
    std::vector<u32> body_a{};
    std::vector<u32> body_b{};
    std::vector<Vec3> normal{};
    std::vector<Vec3> tangent_u{};
    std::vector<Vec3> tangent_v{};
    std::vector<Vec3> r_a{};
    std::vector<Vec3> r_b{};
    std::vector<f32> normal_mass{};
    std::vector<f32> tangent_mass_u{};
    std::vector<f32> tangent_mass_v{};
    std::vector<f32> velocity_bias{};
    std::vector<f32> friction_coeff{};

    void clear_and_reserve(const usize capacity) {
        contact_point.clear();
        body_a.clear();
        body_b.clear();
        normal.clear();
        tangent_u.clear();
        tangent_v.clear();
        r_a.clear();
        r_b.clear();
        normal_mass.clear();
        tangent_mass_u.clear();
        tangent_mass_v.clear();
        velocity_bias.clear();
        friction_coeff.clear();

        contact_point.reserve(capacity);
        body_a.reserve(capacity);
        body_b.reserve(capacity);
        normal.reserve(capacity);
        tangent_u.reserve(capacity);
        tangent_v.reserve(capacity);
        r_a.reserve(capacity);
        r_b.reserve(capacity);
        normal_mass.reserve(capacity);
        tangent_mass_u.reserve(capacity);
        tangent_mass_v.reserve(capacity);
        velocity_bias.reserve(capacity);
        friction_coeff.reserve(capacity);
    }

    [[nodiscard]] usize push_back(ContactPoint *point_ptr, const u32 a, const u32 b, const Vec3 normal_axis,
                                  const Vec3 tangent_axis_u, const Vec3 tangent_axis_v, const Vec3 world_anchor_a,
                                  const Vec3 world_anchor_b, const f32 friction) {
        const usize index = contact_point.size();
        contact_point.push_back(point_ptr);
        body_a.push_back(a);
        body_b.push_back(b);
        normal.push_back(normal_axis);
        tangent_u.push_back(tangent_axis_u);
        tangent_v.push_back(tangent_axis_v);
        r_a.push_back(world_anchor_a);
        r_b.push_back(world_anchor_b);
        normal_mass.push_back(0.0f);
        tangent_mass_u.push_back(0.0f);
        tangent_mass_v.push_back(0.0f);
        velocity_bias.push_back(0.0f);
        friction_coeff.push_back(friction);
        return index;
    }

    [[nodiscard]] usize size() const noexcept { return contact_point.size(); }
};

[[nodiscard]] inline SolverPointStreams &solver_points_cache() {
    static thread_local SolverPointStreams cache{};
    return cache;
}

void build_world_inv_inertia_cache(std::span<const f32> inv_mass, std::span<const Vec3> inv_inertia_body,
                                   std::span<const Quat> orientation, std::span<const u8> asleep) {
    ZoneScopedN("Physics build world inv inertia cache");
    WorldInvInertiaCache &cache = world_inv_inertia_cache();
    const u32 body_count = static_cast<u32>(inv_mass.size());
    if (cache.tensors.size() < body_count) {
        cache.tensors.resize(body_count);
        cache.valid.resize(body_count);
    }

    constexpr Vec3 unit_x = Vec3{1.0f, 0.0f, 0.0f};
    constexpr Vec3 unit_y = Vec3{0.0f, 1.0f, 0.0f};
    constexpr Vec3 unit_z = Vec3{0.0f, 0.0f, 1.0f};

    for (u32 i = 0u; i < body_count; ++i) {
        if (inv_mass[i] == 0.0f || asleep[i] != 0u) {
            cache.valid[i] = 0u;
            cache.tensors[i] = WorldInvInertia{};
            continue;
        }

        cache.valid[i] = 1u;
        cache.tensors[i] = WorldInvInertia{
            .col0 = apply_inv_inertia(inv_inertia_body[i], orientation[i], unit_x),
            .col1 = apply_inv_inertia(inv_inertia_body[i], orientation[i], unit_y),
            .col2 = apply_inv_inertia(inv_inertia_body[i], orientation[i], unit_z),
        };
    }
}

[[nodiscard]] inline const WorldInvInertiaCache &world_inv_inertia_view() { return world_inv_inertia_cache(); }

[[nodiscard]] inline Vec3 apply_world_inv_inertia(const WorldInvInertiaCache &cache, const u32 body,
                                                  const Vec3 v_world) noexcept {
#ifndef NDEBUG
    if (cache.valid[body] == 0u) {
        log::error(physics, "World inverse inertia cache miss (body={})", body);
        std::terminate();
    }
#endif
    const WorldInvInertia &world_inv = cache.tensors[body];
    return world_inv.col0 * v_world.x + world_inv.col1 * v_world.y + world_inv.col2 * v_world.z;
}

[[nodiscard]] inline f32 inv_inertia_term_world(const WorldInvInertiaCache &cache, const u32 body,
                                                const Vec3 v_world) noexcept {
    const Vec3 scaled = apply_world_inv_inertia(cache, body, v_world);
    return dot(scaled, v_world);
}

[[nodiscard]] inline f32 axis_effective_mass_inverse(const SolverPointStreams &points, const usize index,
                                                     std::span<const f32> inv_mass,
                                                     const WorldInvInertiaCache &world_inv_inertia,
                                                     const Vec3 axis) noexcept {
    const u32 a = points.body_a[index];
    const u32 b = points.body_b[index];
    const f32 inv_mass_a = inv_mass[a];
    const f32 inv_mass_b = has_body_b(b) ? inv_mass[b] : 0.0f;
    const f32 inv_mass_sum = inv_mass_a + inv_mass_b;
    if (inv_mass_sum <= kMassEps) {
        return 0.0f;
    }

    const Vec3 ra_cross_axis = cross(points.r_a[index], axis);
    const f32 ang_term_a = inv_inertia_term_world(world_inv_inertia, a, ra_cross_axis);
    f32 ang_term_b = 0.0f;
    if (has_body_b(b)) {
        const Vec3 rb_cross_axis = cross(points.r_b[index], axis);
        ang_term_b = inv_inertia_term_world(world_inv_inertia, b, rb_cross_axis);
    }

    const f32 denom = inv_mass_sum + ang_term_a + ang_term_b;
    return (denom > kMassEps) ? (1.0f / denom) : 0.0f;
}

[[nodiscard]] inline Vec3 relative_velocity_at_point(const SolverPointStreams &points, const usize index,
                                                     std::span<const Vec3> velocity,
                                                     std::span<const Vec3> angular_velocity) noexcept {
    const u32 a = points.body_a[index];
    const u32 b = points.body_b[index];
    const Vec3 v_a = velocity[a] + cross(angular_velocity[a], points.r_a[index]);
    if (!has_body_b(b)) {
        return -v_a;
    }
    const Vec3 v_b = velocity[b] + cross(angular_velocity[b], points.r_b[index]);
    return v_b - v_a;
}

inline void apply_impulse_at_point(const SolverPointStreams &points, const usize index, const Vec3 impulse,
                                   std::span<Vec3> velocity, std::span<Vec3> angular_velocity,
                                   std::span<const f32> inv_mass,
                                   const WorldInvInertiaCache &world_inv_inertia) noexcept {
    const u32 a = points.body_a[index];
    const u32 b = points.body_b[index];

    velocity[a] -= impulse * inv_mass[a];
    angular_velocity[a] -= apply_world_inv_inertia(world_inv_inertia, a, cross(points.r_a[index], impulse));

    if (!has_body_b(b)) {
        return;
    }

    velocity[b] += impulse * inv_mass[b];
    angular_velocity[b] += apply_world_inv_inertia(world_inv_inertia, b, cross(points.r_b[index], impulse));
}

inline void warm_start_solver_point(const SolverPointStreams &points, const usize index, std::span<Vec3> velocity,
                                    std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                                    const WorldInvInertiaCache &world_inv_inertia) noexcept {
    ContactPoint &point = *points.contact_point[index];
    point.normal_impulse = std::max(point.normal_impulse, 0.0f);
    point.tangent_impulse = project_and_clamp_tangent_impulse(point.tangent_impulse, points.normal[index],
                                                              point.normal_impulse, points.friction_coeff[index]);
    if (!has_warm_start_impulse(point)) {
        return;
    }
    const Vec3 impulse = points.normal[index] * point.normal_impulse + point.tangent_impulse;
    apply_impulse_at_point(points, index, impulse, velocity, angular_velocity, inv_mass, world_inv_inertia);
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

struct ContactSolveConfig final {
    // Deterministic fixed-iteration mode remains the default.
    bool adaptive_iteration_cap{};
    // Upper bound; fixed mode runs exactly this many iterations.
    u32 max_iterations{16u};
    // Adaptive mode cannot early-out before this floor.
    u32 min_iterations_before_adapt{4u};
    // Early-out when the maximum per-point impulse delta² in an iteration
    // drops below this threshold².
    f32 adaptive_impulse_epsilon{1e-4f};
};

void solve_contact_velocities(std::span<Vec3> velocity, std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                              std::span<const Vec3> inv_inertia_body, std::span<const Quat> orientation,
                              std::span<ContactManifold> manifolds, const f32 dt,
                              std::span<const f32> manifold_restitution, std::span<const f32> manifold_friction,
                              std::span<const u8> asleep, const ContactSolveConfig &config);

void solve_contact_velocities(std::span<Vec3> velocity, std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                              std::span<const Vec3> inv_inertia_body, std::span<const Quat> orientation,
                              std::span<ContactManifold> manifolds, const f32 dt,
                              std::span<const f32> manifold_restitution, std::span<const f32> manifold_friction,
                              std::span<const u8> asleep) {
    solve_contact_velocities(velocity, angular_velocity, inv_mass, inv_inertia_body, orientation, manifolds, dt,
                             manifold_restitution, manifold_friction, asleep, ContactSolveConfig{});
}

void solve_contact_velocities(std::span<Vec3> velocity, std::span<Vec3> angular_velocity, std::span<const f32> inv_mass,
                              std::span<const Vec3> inv_inertia_body, std::span<const Quat> orientation,
                              std::span<ContactManifold> manifolds, const f32 dt,
                              std::span<const f32> manifold_restitution, std::span<const f32> manifold_friction,
                              std::span<const u8> asleep, const ContactSolveConfig &config) {
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
    if (manifold_restitution.size() != manifolds.size() || manifold_friction.size() != manifolds.size()) {
        log::error(physics, "Velocity solver manifold material span mismatch (manifolds={} restitution={} friction={})",
                   manifolds.size(), manifold_restitution.size(), manifold_friction.size());
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

    detail::build_world_inv_inertia_cache(inv_mass, inv_inertia_body, orientation, asleep);
    const detail::WorldInvInertiaCache &world_inv_inertia = detail::world_inv_inertia_view();

    detail::SolverPointStreams &solver_points = detail::solver_points_cache();
    solver_points.clear_and_reserve(point_count);

    // Pre-step: world anchors, tangent basis, effective masses, and velocity bias.
    // Skip manifolds where both participants are asleep: a body is only still
    // asleep if all its contacts are ground/static/other-sleeping (the island
    // wake pass already cleared asleep for any body touched by an awake dynamic
    // neighbour).
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
            const usize solver_point_index = solver_points.push_back(
                &point, manifold.a, manifold.b, normal, tangent_u, tangent_v,
                rotate(orientation[manifold.a], point.local_anchor_a),
                (manifold.b != kInvalidBody) ? rotate(orientation[manifold.b], point.local_anchor_b) : Vec3{},
                fric_coeff);

            solver_points.normal_mass[solver_point_index] = detail::axis_effective_mass_inverse(
                solver_points, solver_point_index, inv_mass, world_inv_inertia, normal);
            solver_points.tangent_mass_u[solver_point_index] = detail::axis_effective_mass_inverse(
                solver_points, solver_point_index, inv_mass, world_inv_inertia, tangent_u);
            solver_points.tangent_mass_v[solver_point_index] = detail::axis_effective_mass_inverse(
                solver_points, solver_point_index, inv_mass, world_inv_inertia, tangent_v);

            const f32 separation = point.separation + detail::kPenetrationSlop;
            const f32 penetration_bias =
                (inv_dt > 0.0f) ? (-detail::kPenetrationBiasFactor * inv_dt * std::min(separation, 0.0f)) : 0.0f;
            const f32 clamped_penetration_bias = std::min(penetration_bias, detail::kMaxPenetrationBias);

            const Vec3 relative_velocity =
                detail::relative_velocity_at_point(solver_points, solver_point_index, velocity, angular_velocity);
            const f32 normal_velocity = dot(relative_velocity, normal);
            const f32 restitution_velocity_bias = detail::restitution_bias(normal_velocity, rest_coeff);
            solver_points.velocity_bias[solver_point_index] =
                std::max(clamped_penetration_bias, restitution_velocity_bias);
        }
    }
    if (solver_points.size() == 0u) {
        return;
    }

    // Warm start: apply accumulated normal impulse plus projected/clamped world-space friction impulse.
    for (usize i = 0; i < solver_points.size(); ++i) {
        detail::warm_start_solver_point(solver_points, i, velocity, angular_velocity, inv_mass, world_inv_inertia);
    }

    // Iterative projected Gauss-Seidel.
    const u32 max_iterations = std::max(config.max_iterations, 1u);
    const u32 min_iterations = std::min(config.min_iterations_before_adapt, max_iterations);
    const bool adaptive_enabled = config.adaptive_iteration_cap && config.adaptive_impulse_epsilon > 0.0f;
    const f32 adaptive_impulse_epsilon_sq = config.adaptive_impulse_epsilon * config.adaptive_impulse_epsilon;
    u32 iterations_used = max_iterations;

    for (u32 iteration = 0; iteration < max_iterations; ++iteration) {
        f32 max_impulse_delta_sq = 0.0f;
        for (usize i = 0; i < solver_points.size(); ++i) {
            ContactPoint &point = *solver_points.contact_point[i];

            Vec3 relative_velocity = detail::relative_velocity_at_point(solver_points, i, velocity, angular_velocity);
            const f32 vn = dot(relative_velocity, solver_points.normal[i]);
            const f32 delta_normal = solver_points.normal_mass[i] * (solver_points.velocity_bias[i] - vn);
            const f32 old_normal_impulse = point.normal_impulse;
            point.normal_impulse = std::max(old_normal_impulse + delta_normal, 0.0f);
            const f32 applied_normal_impulse = point.normal_impulse - old_normal_impulse;
            if (std::fabs(applied_normal_impulse) > detail::kMassEps) {
                detail::apply_impulse_at_point(solver_points, i, solver_points.normal[i] * applied_normal_impulse,
                                               velocity, angular_velocity, inv_mass, world_inv_inertia);
            }

            relative_velocity = detail::relative_velocity_at_point(solver_points, i, velocity, angular_velocity);
            const f32 vt_u = dot(relative_velocity, solver_points.tangent_u[i]);
            const f32 vt_v = dot(relative_velocity, solver_points.tangent_v[i]);
            const Vec3 old_tangent_impulse = point.tangent_impulse;
            const f32 old_tangent_u = dot(old_tangent_impulse, solver_points.tangent_u[i]);
            const f32 old_tangent_v = dot(old_tangent_impulse, solver_points.tangent_v[i]);

            f32 new_tangent_u = old_tangent_u - solver_points.tangent_mass_u[i] * vt_u;
            f32 new_tangent_v = old_tangent_v - solver_points.tangent_mass_v[i] * vt_v;

            // Coulomb friction on the accumulated tangent impulse vector.
            const f32 max_friction = solver_points.friction_coeff[i] * point.normal_impulse;
            const f32 max_friction_sq = max_friction * max_friction;
            const f32 tangent_impulse_sq = new_tangent_u * new_tangent_u + new_tangent_v * new_tangent_v;
            if (tangent_impulse_sq > max_friction_sq && tangent_impulse_sq > detail::kMassEps * detail::kMassEps) {
                const f32 scale = max_friction / std::sqrt(tangent_impulse_sq);
                new_tangent_u *= scale;
                new_tangent_v *= scale;
            }

            const Vec3 new_tangent_impulse =
                solver_points.tangent_u[i] * new_tangent_u + solver_points.tangent_v[i] * new_tangent_v;
            point.tangent_impulse = new_tangent_impulse;

            const Vec3 applied_tangent_impulse = new_tangent_impulse - old_tangent_impulse;
            if (applied_tangent_impulse.length_sq() > detail::kMassEps * detail::kMassEps) {
                detail::apply_impulse_at_point(solver_points, i, applied_tangent_impulse, velocity, angular_velocity,
                                               inv_mass, world_inv_inertia);
            }

            if (adaptive_enabled) {
                const f32 normal_delta_sq = applied_normal_impulse * applied_normal_impulse;
                const f32 tangent_delta_sq = applied_tangent_impulse.length_sq();
                max_impulse_delta_sq = std::max(max_impulse_delta_sq, normal_delta_sq + tangent_delta_sq);
            }
        }

        if (adaptive_enabled && iteration + 1u >= min_iterations &&
            max_impulse_delta_sq <= adaptive_impulse_epsilon_sq) {
            iterations_used = iteration + 1u;
            break;
        }
    }
    TracyPlot("physics_contact_solver_iterations_used", static_cast<i64>(iterations_used));
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
    detail::build_world_inv_inertia_cache(inv_mass, inv_inertia_body, orientation, asleep);
    const detail::WorldInvInertiaCache &world_inv_inertia = detail::world_inv_inertia_view();

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
        const f32 w_a = inv_mass[c.body_a] + detail::inv_inertia_term_world(world_inv_inertia, c.body_a, ra_cross_n);
        const f32 w_b = inv_mass[c.body_b] + detail::inv_inertia_term_world(world_inv_inertia, c.body_b, rb_cross_n);

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
                detail::apply_world_inv_inertia(world_inv_inertia, g.a, g.ra_cross_n * delta_lambda);
            velocity[g.b] += impulse * inv_mass[g.b];
            angular_velocity[g.b] +=
                detail::apply_world_inv_inertia(world_inv_inertia, g.b, g.rb_cross_n * delta_lambda);
        }
    }
}

} // namespace javelin
