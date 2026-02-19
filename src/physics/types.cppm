module;

export module javelin.physics.types;

import std;
import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {

struct BodyPair final {
    u32 a{};
    u32 b{};
};

inline constexpr u32 kMaxManifoldPoints = 4;
inline constexpr u32 kInvalidContactFeature = std::numeric_limits<u32>::max();

// Contact manifold invariants:
// - Normal always points from body a to body b.
// - Pair keys are canonicalized as (min(a, b), max(a, b)).
// - points[] is stored in deterministic order.
// - separation is signed along normal; penetration means separation < 0.
struct ContactPoint final {
    // Contact anchors in each body's local space.
    Vec3 local_anchor_a{};
    Vec3 local_anchor_b{};
    // Signed distance along manifold normal (negative means penetration).
    f32 separation{};
    // Warm-start impulse cache for sequential impulse solve.
    f32 normal_impulse{};
    f32 tangent_impulse{};
    // Stable identifier used to match points across frames.
    u32 feature_id{kInvalidContactFeature};
};

struct ContactManifold final {
    u32 a{};
    u32 b{};
    // Normal points from A to B.
    Vec3 normal{};
    // Number of valid entries in points[].
    u32 point_count{};
    std::array<ContactPoint, kMaxManifoldPoints> points{};
};

struct Contact final {
    u32 a{};
    u32 b{};
    // Normal points from A to B.
    Vec3 normal{};
    f32 penetration{};
    // Contact offsets in world space from each body's center.
    Vec3 r_a{};
    Vec3 r_b{};
};

inline constexpr u32 kInvalidBody = std::numeric_limits<u32>::max();

} // namespace javelin
