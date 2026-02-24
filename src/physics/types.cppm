module;

export module javelin.physics.types;

import std;
import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {

// Canonical representation for unordered body interactions.
// All pair-based maps/sets use this ordering to avoid duplicate keys.
struct BodyPair final {
    u32 a{};
    u32 b{};
};

// Normalizes an unordered pair into deterministic ascending id order.
[[nodiscard]] inline constexpr BodyPair canonical_body_pair(const u32 a, const u32 b) noexcept {
    if (a <= b) {
        return BodyPair{.a = a, .b = b};
    }
    return BodyPair{.a = b, .b = a};
}

// Packs a canonical body pair into one 64-bit key for hash tables.
[[nodiscard]] inline constexpr u64 body_pair_key(const u32 a, const u32 b) noexcept {
    const BodyPair pair = canonical_body_pair(a, b);
    return (static_cast<u64>(pair.a) << 32u) | static_cast<u64>(pair.b);
}

[[nodiscard]] inline constexpr u64 body_pair_key(const BodyPair pair) noexcept { return body_pair_key(pair.a, pair.b); }

inline constexpr u32 kMaxManifoldPoints = 4;
inline constexpr u32 kInvalidContactFeature = std::numeric_limits<u32>::max();

// Contact manifold invariants:
// - Normal always points from body a to body b.
// - Pair keys are canonicalized as (min(a, b), max(a, b)).
// - manifold_feature_id identifies manifold-level topology when available.
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
    // World-space accumulated friction impulse.
    Vec3 tangent_impulse{};
    // True when this point reuses cached state from a previous frame.
    bool persisted{};
    // Stable identifier used to match points across frames.
    u32 feature_id{kInvalidContactFeature};
};

struct ContactManifold final {
    u32 a{};
    u32 b{};
    // Stable manifold-level feature id (for example the selected SAT axis key).
    // This is intentionally separate from point feature ids.
    u32 manifold_feature_id{kInvalidContactFeature};
    // Normal points from A to B.
    Vec3 normal{};
    // Number of valid entries in points[].
    u32 point_count{};
    std::array<ContactPoint, kMaxManifoldPoints> points{};
};

[[nodiscard]] inline constexpr u32 ordered_float_key(const f32 value) noexcept {
    const u32 bits = std::bit_cast<u32>(value);
    return (bits & 0x80000000u) ? ~bits : (bits ^ 0x80000000u);
}

// Deterministic manifold point ordering for warm-start matching:
// 1) valid point feature id (when present on both points),
// 2) local anchors on body A then B,
// 3) separation,
// 4) stable index fallback.
inline void sort_manifold_points(ContactManifold &manifold) noexcept {
#ifndef NDEBUG
    if (manifold.point_count > kMaxManifoldPoints) {
        std::terminate();
    }
#endif
    if (manifold.point_count <= 1u) {
        return;
    }

    std::array<u32, kMaxManifoldPoints> order{};
    for (u32 i = 0; i < manifold.point_count; ++i) {
        order[i] = i;
    }

    auto point_less = [&](const u32 lhs_index, const u32 rhs_index) noexcept {
        const ContactPoint &lhs = manifold.points[lhs_index];
        const ContactPoint &rhs = manifold.points[rhs_index];
        const bool lhs_valid_feature = lhs.feature_id != kInvalidContactFeature;
        const bool rhs_valid_feature = rhs.feature_id != kInvalidContactFeature;

        // Primary key: feature id when valid on both points.
        if (lhs_valid_feature && rhs_valid_feature && lhs.feature_id != rhs.feature_id) {
            return lhs.feature_id < rhs.feature_id;
        }
        // Valid features sort before invalid ones.
        if (lhs_valid_feature != rhs_valid_feature) {
            return lhs_valid_feature;
        }

        // Geometric tie-breaker in local space.
        const u32 lhs_a_x = ordered_float_key(lhs.local_anchor_a.x);
        const u32 rhs_a_x = ordered_float_key(rhs.local_anchor_a.x);
        if (lhs_a_x != rhs_a_x) {
            return lhs_a_x < rhs_a_x;
        }
        const u32 lhs_a_y = ordered_float_key(lhs.local_anchor_a.y);
        const u32 rhs_a_y = ordered_float_key(rhs.local_anchor_a.y);
        if (lhs_a_y != rhs_a_y) {
            return lhs_a_y < rhs_a_y;
        }
        const u32 lhs_a_z = ordered_float_key(lhs.local_anchor_a.z);
        const u32 rhs_a_z = ordered_float_key(rhs.local_anchor_a.z);
        if (lhs_a_z != rhs_a_z) {
            return lhs_a_z < rhs_a_z;
        }

        const u32 lhs_b_x = ordered_float_key(lhs.local_anchor_b.x);
        const u32 rhs_b_x = ordered_float_key(rhs.local_anchor_b.x);
        if (lhs_b_x != rhs_b_x) {
            return lhs_b_x < rhs_b_x;
        }
        const u32 lhs_b_y = ordered_float_key(lhs.local_anchor_b.y);
        const u32 rhs_b_y = ordered_float_key(rhs.local_anchor_b.y);
        if (lhs_b_y != rhs_b_y) {
            return lhs_b_y < rhs_b_y;
        }
        const u32 lhs_b_z = ordered_float_key(lhs.local_anchor_b.z);
        const u32 rhs_b_z = ordered_float_key(rhs.local_anchor_b.z);
        if (lhs_b_z != rhs_b_z) {
            return lhs_b_z < rhs_b_z;
        }

        const u32 lhs_sep = ordered_float_key(lhs.separation);
        const u32 rhs_sep = ordered_float_key(rhs.separation);
        if (lhs_sep != rhs_sep) {
            return lhs_sep < rhs_sep;
        }

        // Last-resort deterministic fallback.
        if (lhs.feature_id != rhs.feature_id) {
            return lhs.feature_id < rhs.feature_id;
        }
        return lhs_index < rhs_index;
    };

    std::sort(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(manifold.point_count), point_less);

    std::array<ContactPoint, kMaxManifoldPoints> sorted_points{};
    for (u32 i = 0; i < manifold.point_count; ++i) {
        sorted_points[i] = manifold.points[order[i]];
    }
    for (u32 i = 0; i < manifold.point_count; ++i) {
        manifold.points[i] = sorted_points[i];
    }
}

// Re-orients manifold storage to canonical body order (a <= b).
// When swapping bodies, normal and local anchors are flipped accordingly.
inline void canonicalize_manifold_orientation(ContactManifold &manifold) noexcept {
    if (manifold.a <= manifold.b) {
        return;
    }

    std::swap(manifold.a, manifold.b);
    manifold.normal = -manifold.normal;

#ifndef NDEBUG
    if (manifold.point_count > kMaxManifoldPoints) {
        std::terminate();
    }
#endif
    for (u32 i = 0; i < manifold.point_count; ++i) {
        std::swap(manifold.points[i].local_anchor_a, manifold.points[i].local_anchor_b);
    }
}

inline constexpr u32 kInvalidBody = std::numeric_limits<u32>::max();

} // namespace javelin
