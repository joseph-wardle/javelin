export module javelin.physics.constraint_types;

import javelin.core.types;
import javelin.math;

export namespace javelin {

// A bilateral distance constraint between two body anchor points.
//
// Enforces: |p_b − p_a| = rest_length
//   where p_a = position[body_a] + rotate(orientation[body_a], anchor_a)
//         p_b = position[body_b] + rotate(orientation[body_b], anchor_b)
//
// The constraint is solved as an XPBD velocity-level impulse applied to both bodies
// each tick.  Compliance α controls how stiffness is traded against stability:
//
//   compliance = 0          — rigid rod; the distance error is fully corrected
//                             each tick within the solver iteration budget.
//   compliance = 1 / (k·dt²) — spring of stiffness k (N/m).  Larger values
//                              produce a softer, more elastic connection.
//
// Anchors are expressed in each body's local space so they remain fixed relative
// to the body regardless of rotation.  The solver transforms them to world space
// each tick using the body's current orientation.
//
// Both body_a and body_b must be valid scene body indices.  To anchor a constraint
// to a fixed world point (e.g. a pendulum pivot), add a static body at that point
// and use it as one of the endpoints.
struct DistanceConstraint final {
    // Indices into the scene body SoA arrays.
    u32 body_a{};
    u32 body_b{};
    // Attachment points in each body's local space (metres).
    Vec3 anchor_a{};
    Vec3 anchor_b{};
    // Target distance between the world-space anchor points (metres, ≥ 0).
    f32 rest_length{};
    // XPBD compliance α (m/N): 0 = rigid link, > 0 = spring.
    f32 compliance{};
};

} // namespace javelin
