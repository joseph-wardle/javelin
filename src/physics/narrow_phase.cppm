module;

#include <tracy/Tracy.hpp>

export module javelin.physics.narrow_phase;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.types;
import javelin.scene.shapes;

namespace javelin::detail {
// Narrow phase contract:
// - inputs are current-frame world transforms and broad-phase candidate pairs.
// - outputs are contact manifolds with canonical pair orientation (a <= b).
// - manifold normal always points from body a to body b after canonicalization.
// - feature ids are stable, deterministic keys used for cross-frame cache matching.
inline constexpr Vec3 kGroundNormal{0.0f, 1.0f, 0.0f};
// Intentional simplification: world collision always includes an infinite ground plane at y=0.
// This keeps sample scenes compact and avoids a mandatory authored static "floor" body.
inline constexpr f32 kGroundOffset = 0.0f;
// Degenerate normal guard for center deltas (sphere-sphere and sphere-box fallback paths).
// 1e-6 m^2 corresponds to ~1 mm positional tolerance.
inline constexpr f32 kMinDistanceEpsSq = 1e-6f;
// SAT cross-axis validity epsilon; filters near-parallel edge-edge axes before normalization.
inline constexpr f32 kAxisEpsSq = 1e-8f;
// Absolute projection/separation epsilon for interval overlap decisions in SAT.
inline constexpr f32 kAxisAbsEps = 1e-6f;
// Feature id tag bits (payload encoding is shape-pair specific).
inline constexpr u32 kBoxFaceFeatureTag = 1u << 8u;
inline constexpr u32 kBoxGroundVertexFeatureTag = 1u << 9u;
inline constexpr u32 kBoxAxisFeatureTag = 1u << 10u;
inline constexpr u32 kBoxFaceFaceFeatureTag = 1u << 13u;
// Tie-break epsilon for manifold reduction and deterministic candidate ordering.
inline constexpr f32 kReductionEps = 1e-6f;
// Axis selection hysteresis (metres): keep prior axis when depth is close to reduce normal flip-flop.
inline constexpr f32 kAxisHysteresisAbs = 0.02f;
// Segment/line denominator epsilon for edge-edge closest-point math.
inline constexpr f32 kSegmentEps = 1e-8f;

[[nodiscard]] inline const SphereShape &sphere_shape(std::span<const ShapeKind> shape_kind,
                                                     std::span<const ShapeData> shapes,
                                                     std::span<const u32> shape_index, const u32 id) noexcept {
    const u32 shape_id = shape_index[id];
#ifndef NDEBUG
    if (shape_id >= shapes.size()) {
        log::error(physics, "Shape index out of range (id={} shape_id={})", id, shape_id);
        std::terminate();
    }
#endif
    const ShapeData &shape = shapes[shape_id];
#ifndef NDEBUG
    if (shape_kind[id] != ShapeKind::sphere || shape.kind != ShapeKind::sphere) {
        log::error(physics, "Narrow phase expects sphere shape (id={})", id);
        std::terminate();
    }
#endif
    return shape_sphere(shape);
}

[[nodiscard]] inline const BoxShape &box_shape(std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                                               std::span<const u32> shape_index, const u32 id) noexcept {
    const u32 shape_id = shape_index[id];
#ifndef NDEBUG
    if (shape_id >= shapes.size()) {
        log::error(physics, "Shape index out of range (id={} shape_id={})", id, shape_id);
        std::terminate();
    }
#endif
    const ShapeData &shape = shapes[shape_id];
#ifndef NDEBUG
    if (shape_kind[id] != ShapeKind::box || shape.kind != ShapeKind::box) {
        log::error(physics, "Narrow phase expects box shape (id={})", id);
        std::terminate();
    }
#endif
    return shape_box(shape);
}

[[nodiscard]] inline Vec3 box_support_point(const Vec3 center, const Vec3 a0, const Vec3 a1, const Vec3 a2,
                                            const Vec3 half_extents, const Vec3 dir) noexcept {
    const f32 s0 = (dot(dir, a0) >= 0.0f) ? half_extents.x : -half_extents.x;
    const f32 s1 = (dot(dir, a1) >= 0.0f) ? half_extents.y : -half_extents.y;
    const f32 s2 = (dot(dir, a2) >= 0.0f) ? half_extents.z : -half_extents.z;
    return center + a0 * s0 + a1 * s1 + a2 * s2;
}

[[nodiscard]] inline u32 dominant_axis_abs(const Vec3 v) noexcept {
    const f32 ax = std::fabs(v.x);
    const f32 ay = std::fabs(v.y);
    const f32 az = std::fabs(v.z);
    if (ax >= ay && ax >= az) {
        return 0u;
    }
    if (ay >= az) {
        return 1u;
    }
    return 2u;
}

[[nodiscard]] inline u32 box_face_feature_id(const u32 axis, const bool positive, const bool inside_fallback) noexcept {
    return kBoxFaceFeatureTag | axis | (positive ? (1u << 2u) : 0u) | (inside_fallback ? (1u << 3u) : 0u);
}

[[nodiscard]] inline u32 box_ground_vertex_feature_id(const u32 vertex_index) noexcept {
    return kBoxGroundVertexFeatureTag | vertex_index;
}

enum struct BoxAxisType : u8 { face_a = 0, face_b = 1, edge_edge = 2 };

struct BoxAxisKey final {
    BoxAxisType type{BoxAxisType::face_a};
    u8 i{};
    u8 j{};
};

[[nodiscard]] inline u32 box_axis_feature_id(const BoxAxisKey key) noexcept {
    return kBoxAxisFeatureTag | (static_cast<u32>(key.type) & 0x3u) | (static_cast<u32>(key.i & 0x3u) << 2u) |
           (static_cast<u32>(key.j & 0x3u) << 4u);
}

[[nodiscard]] inline u32 box_edge_edge_feature_id(const BoxAxisKey key, const u8 edge_a, const u8 edge_b) noexcept {
    return box_axis_feature_id(key) | ((static_cast<u32>(edge_a) & 0x3u) << 6u) |
           ((static_cast<u32>(edge_b) & 0x3u) << 8u);
}

[[nodiscard]] inline bool decode_box_axis_feature_id(const u32 feature_id, BoxAxisKey &out) noexcept {
    if ((feature_id & kBoxAxisFeatureTag) == 0u) {
        return false;
    }
    // Face-face point feature ids reuse lower payload bits and can set bit10;
    // they are not manifold axis ids.
    if ((feature_id & kBoxFaceFaceFeatureTag) != 0u) {
        return false;
    }
    const u32 type = feature_id & 0x3u;
    if (type > static_cast<u32>(BoxAxisType::edge_edge)) {
        return false;
    }
    out.type = static_cast<BoxAxisType>(type);
    out.i = static_cast<u8>((feature_id >> 2u) & 0x3u);
    out.j = static_cast<u8>((feature_id >> 4u) & 0x3u);
    return true;
}

[[nodiscard]] inline bool axis_key_less(const BoxAxisKey lhs, const BoxAxisKey rhs) noexcept {
    if (lhs.type != rhs.type) {
        return static_cast<u8>(lhs.type) < static_cast<u8>(rhs.type);
    }
    if (lhs.i != rhs.i) {
        return lhs.i < rhs.i;
    }
    return lhs.j < rhs.j;
}

[[nodiscard]] inline Vec3 world_offset_to_local_anchor(const Quat q, const Vec3 offset_world) noexcept {
    return rotate(inverse_unit(q), offset_world);
}

// Helper for one-point contacts. Used by sphere-sphere, sphere-box, edge-edge, and fallback paths.
// Caller provides world-space anchor offsets relative to each center of mass.
void push_single_point_manifold(std::span<const Quat> orientation, const u32 a, const u32 b, const Vec3 normal,
                                const f32 penetration, const u32 point_feature_id, const Vec3 r_a_world,
                                const Vec3 r_b_world, std::vector<ContactManifold> &manifolds,
                                const u32 manifold_feature_id = kInvalidContactFeature) {
    ContactManifold manifold{
        .a = a,
        .b = b,
        .manifold_feature_id = manifold_feature_id,
        .normal = normal,
        .point_count = 1u,
    };
    ContactPoint &point = manifold.points[0];
    point.local_anchor_a = world_offset_to_local_anchor(orientation[a], r_a_world);
    point.local_anchor_b = (b != kInvalidBody) ? world_offset_to_local_anchor(orientation[b], r_b_world) : Vec3{};
    point.separation = -penetration;
    point.feature_id = point_feature_id;

    canonicalize_manifold_orientation(manifold);
    manifolds.push_back(manifold);
}

void add_sphere_sphere_contact(std::span<const Vec3> position, std::span<const ShapeKind> shape_kind,
                               std::span<const Quat> orientation, std::span<const ShapeData> shapes,
                               std::span<const u32> shape_index, const u32 a, const u32 b,
                               std::vector<ContactManifold> &manifolds) {
    const SphereShape &sphere_a = sphere_shape(shape_kind, shapes, shape_index, a);
    const SphereShape &sphere_b = sphere_shape(shape_kind, shapes, shape_index, b);

    const Vec3 delta = position[b] - position[a];
    const f32 radius_sum = sphere_a.radius + sphere_b.radius;
    if (std::fabs(delta.x) > radius_sum || std::fabs(delta.y) > radius_sum || std::fabs(delta.z) > radius_sum) {
        return;
    }

    const f32 dist2 = delta.length_sq();
    const f32 radius_sum2 = radius_sum * radius_sum;
    if (dist2 >= radius_sum2) {
        return;
    }

    Vec3 normal = Vec3::unit_x();
    f32 dist = 0.0f;
    if (dist2 > kMinDistanceEpsSq) {
        dist = std::sqrt(dist2);
        normal = delta / dist;
    }
    const f32 penetration = radius_sum - dist;
    const Vec3 r_a = normal * sphere_a.radius;
    const Vec3 r_b = -normal * sphere_b.radius;
    // Sphere-sphere always uses one persistent point id.
    push_single_point_manifold(orientation, a, b, normal, penetration, 0u, r_a, r_b, manifolds);
}

void add_sphere_box_contact(std::span<const Vec3> position, std::span<const Quat> orientation,
                            std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                            std::span<const u32> shape_index, const u32 sphere_id, const u32 box_id,
                            const bool sphere_is_a, std::vector<ContactManifold> &manifolds) {
    const SphereShape &sphere = sphere_shape(shape_kind, shapes, shape_index, sphere_id);
    const BoxShape &box = box_shape(shape_kind, shapes, shape_index, box_id);

    const Vec3 sphere_center_world = position[sphere_id];
    const Vec3 box_center_world = position[box_id];
    const Mat3 box_rotation = to_mat3(orientation[box_id]);
    const Vec3 box_axis_x = box_rotation.col(0);
    const Vec3 box_axis_y = box_rotation.col(1);
    const Vec3 box_axis_z = box_rotation.col(2);

    const Vec3 sphere_center_from_box = sphere_center_world - box_center_world;
    const Vec3 sphere_center_box_local{
        dot(sphere_center_from_box, box_axis_x),
        dot(sphere_center_from_box, box_axis_y),
        dot(sphere_center_from_box, box_axis_z),
    };

    const Vec3 box_half_extents = box.half_extents;
    const Vec3 closest_point_local{
        std::clamp(sphere_center_box_local.x, -box_half_extents.x, box_half_extents.x),
        std::clamp(sphere_center_box_local.y, -box_half_extents.y, box_half_extents.y),
        std::clamp(sphere_center_box_local.z, -box_half_extents.z, box_half_extents.z),
    };

    const Vec3 offset_local = sphere_center_box_local - closest_point_local;
    const f32 distance_sq = offset_local.length_sq();
    const f32 radius2 = sphere.radius * sphere.radius;
    if (distance_sq > radius2) {
        return;
    }

    Vec3 normal_local = Vec3::unit_x();
    Vec3 contact_point_local = closest_point_local;
    f32 penetration = 0.0f;
    u32 feature_axis = 0u;
    bool feature_positive = true;
    bool inside_fallback = false;
    if (distance_sq > kMinDistanceEpsSq) {
        const f32 distance = std::sqrt(distance_sq);
        normal_local = offset_local / distance;
        penetration = sphere.radius - distance;
        feature_axis = dominant_axis_abs(normal_local);
        const f32 axis_value = (feature_axis == 0u)   ? normal_local.x
                               : (feature_axis == 1u) ? normal_local.y
                                                      : normal_local.z;
        feature_positive = axis_value >= 0.0f;
    } else {
        inside_fallback = true;
        const f32 distance_to_x_face = box_half_extents.x - std::fabs(sphere_center_box_local.x);
        const f32 distance_to_y_face = box_half_extents.y - std::fabs(sphere_center_box_local.y);
        const f32 distance_to_z_face = box_half_extents.z - std::fabs(sphere_center_box_local.z);
        if (distance_to_x_face <= distance_to_y_face && distance_to_x_face <= distance_to_z_face) {
            const f32 sign = (sphere_center_box_local.x >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{sign, 0.0f, 0.0f};
            contact_point_local = Vec3{sign * box_half_extents.x, sphere_center_box_local.y, sphere_center_box_local.z};
            penetration = sphere.radius + distance_to_x_face;
            feature_axis = 0u;
            feature_positive = sign > 0.0f;
        } else if (distance_to_y_face <= distance_to_z_face) {
            const f32 sign = (sphere_center_box_local.y >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{0.0f, sign, 0.0f};
            contact_point_local = Vec3{sphere_center_box_local.x, sign * box_half_extents.y, sphere_center_box_local.z};
            penetration = sphere.radius + distance_to_y_face;
            feature_axis = 1u;
            feature_positive = sign > 0.0f;
        } else {
            const f32 sign = (sphere_center_box_local.z >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{0.0f, 0.0f, sign};
            contact_point_local = Vec3{sphere_center_box_local.x, sphere_center_box_local.y, sign * box_half_extents.z};
            penetration = sphere.radius + distance_to_z_face;
            feature_axis = 2u;
            feature_positive = sign > 0.0f;
        }
    }
    const u32 feature_id = box_face_feature_id(feature_axis, feature_positive, inside_fallback);

    const Vec3 normal_world = box_axis_x * normal_local.x + box_axis_y * normal_local.y + box_axis_z * normal_local.z;
    const Vec3 contact_point_world = box_center_world + box_axis_x * contact_point_local.x +
                                     box_axis_y * contact_point_local.y + box_axis_z * contact_point_local.z;

    Vec3 manifold_normal = normal_world;
    Vec3 anchor_a_world_offset{};
    Vec3 anchor_b_world_offset{};
    if (sphere_is_a) {
        manifold_normal = -normal_world;
        anchor_a_world_offset = manifold_normal * sphere.radius;
        anchor_b_world_offset = contact_point_world - box_center_world;
    } else {
        anchor_a_world_offset = contact_point_world - box_center_world;
        anchor_b_world_offset = -manifold_normal * sphere.radius;
    }

    push_single_point_manifold(orientation, sphere_is_a ? sphere_id : box_id, sphere_is_a ? box_id : sphere_id,
                               manifold_normal, penetration, feature_id, anchor_a_world_offset, anchor_b_world_offset,
                               manifolds);
}

struct BoxAxisCandidate final {
    BoxAxisKey key{};
    Vec3 axis{};
    f32 depth{};
    bool valid{};
};

[[nodiscard]] inline bool better_axis_candidate(const BoxAxisCandidate &lhs, const BoxAxisCandidate &rhs) noexcept {
    if (!lhs.valid) {
        return false;
    }
    if (!rhs.valid) {
        return true;
    }
    const f32 diff = lhs.depth - rhs.depth;
    if (std::fabs(diff) > kReductionEps) {
        return lhs.depth < rhs.depth;
    }
    return axis_key_less(lhs.key, rhs.key);
}

[[nodiscard]] inline const BoxAxisCandidate *
find_axis_candidate(const std::array<BoxAxisCandidate, 3> &face_axes_a,
                    const std::array<BoxAxisCandidate, 3> &face_axes_b,
                    const std::array<std::array<BoxAxisCandidate, 3>, 3> &edge_axes, const BoxAxisKey key) noexcept {
    switch (key.type) {
    case BoxAxisType::face_a:
        return (key.i < 3u) ? &face_axes_a[key.i] : nullptr;
    case BoxAxisType::face_b:
        return (key.i < 3u) ? &face_axes_b[key.i] : nullptr;
    case BoxAxisType::edge_edge:
        return (key.i < 3u && key.j < 3u) ? &edge_axes[key.i][key.j] : nullptr;
    }
    return nullptr;
}

[[nodiscard]] inline std::optional<BoxAxisKey> previous_box_axis_key(const ContactManifold *manifold) noexcept {
    if (manifold == nullptr) {
        return std::nullopt;
    }
    BoxAxisKey key{};
    if (!decode_box_axis_feature_id(manifold->manifold_feature_id, key)) {
        return std::nullopt;
    }
    return key;
}

[[nodiscard]] inline Vec3 box_local_vertex(const Vec3 half_extents, const u32 vertex_index) noexcept {
    return Vec3{
        (vertex_index & 0x1u) ? half_extents.x : -half_extents.x,
        (vertex_index & 0x2u) ? half_extents.y : -half_extents.y,
        (vertex_index & 0x4u) ? half_extents.z : -half_extents.z,
    };
}

[[nodiscard]] inline std::array<u8, 4> box_face_vertex_indices(const u32 axis, const bool positive) noexcept {
    // Vertex index bits: x=bit0, y=bit1, z=bit2.
    static constexpr std::array<std::array<u8, 4>, 6> kFaceVerts{{
        {{0u, 4u, 6u, 2u}}, // -X
        {{1u, 3u, 7u, 5u}}, // +X
        {{0u, 1u, 5u, 4u}}, // -Y
        {{2u, 6u, 7u, 3u}}, // +Y
        {{0u, 2u, 3u, 1u}}, // -Z
        {{4u, 5u, 7u, 6u}}, // +Z
    }};
    return kFaceVerts[axis * 2u + (positive ? 1u : 0u)];
}

struct ClipVertex final {
    Vec3 point_world{};
    f32 u{};
    f32 v{};
    u8 id0{};
    u8 id1{};
};

[[nodiscard]] inline u32 clip_polygon_axis(std::span<const ClipVertex> input, const bool use_u, const f32 boundary,
                                           const bool keep_less_equal, std::span<ClipVertex> output) {
    const auto coord = [use_u](const ClipVertex &vertex) noexcept { return use_u ? vertex.u : vertex.v; };
    const auto inside = [boundary, keep_less_equal](const f32 value) noexcept {
        if (keep_less_equal) {
            return value <= boundary + kReductionEps;
        }
        return value >= boundary - kReductionEps;
    };

    const u32 count = static_cast<u32>(input.size());
    if (count == 0u) {
        return 0u;
    }

    u32 out_count = 0u;
    ClipVertex s = input[count - 1u];
    f32 cs = coord(s);
    bool s_inside = inside(cs);
    for (u32 i = 0; i < count; ++i) {
        const ClipVertex e = input[i];
        const f32 ce = coord(e);
        const bool e_inside = inside(ce);

        if (s_inside != e_inside) {
            const f32 denom = ce - cs;
            f32 t = (std::fabs(denom) > kReductionEps) ? (boundary - cs) / denom : 0.0f;
            t = std::clamp(t, 0.0f, 1.0f);

            ClipVertex intersection{};
            intersection.point_world = lerp(s.point_world, e.point_world, t);
            intersection.u = s.u + (e.u - s.u) * t;
            intersection.v = s.v + (e.v - s.v) * t;
            intersection.id0 = std::min(std::min(s.id0, s.id1), std::min(e.id0, e.id1));
            intersection.id1 = std::max(std::max(s.id0, s.id1), std::max(e.id0, e.id1));
#ifndef NDEBUG
            if (out_count >= output.size()) {
                log::error(physics, "Clip polygon overflow out_count={} capacity={}", out_count, output.size());
                std::terminate();
            }
#endif
            output[out_count++] = intersection;
        }

        if (e_inside) {
#ifndef NDEBUG
            if (out_count >= output.size()) {
                log::error(physics, "Clip polygon overflow out_count={} capacity={}", out_count, output.size());
                std::terminate();
            }
#endif
            output[out_count++] = e;
        }

        s = e;
        cs = ce;
        s_inside = e_inside;
    }
    return out_count;
}

[[nodiscard]] inline u32 box_face_face_feature_id(const u8 ref_axis, const bool ref_positive, const bool ref_is_a,
                                                  const u8 incident_axis, const bool incident_positive, const u8 id0,
                                                  const u8 id1, const u8 quantized_u, const u8 quantized_v) noexcept {
    const u8 a = std::min(id0, id1);
    const u8 b = std::max(id0, id1);
    return kBoxFaceFaceFeatureTag | (static_cast<u32>(ref_axis) & 0x3u) | (ref_positive ? (1u << 2u) : 0u) |
           (ref_is_a ? (1u << 3u) : 0u) | ((static_cast<u32>(incident_axis) & 0x3u) << 4u) |
           (incident_positive ? (1u << 6u) : 0u) | ((static_cast<u32>(a & 0x7u)) << 7u) |
           ((static_cast<u32>(b & 0x7u)) << 10u) | ((static_cast<u32>(quantized_u) & 0x3Fu) << 14u) |
           ((static_cast<u32>(quantized_v) & 0x3Fu) << 20u);
}

[[nodiscard]] inline u8 quantize_face_coordinate(const f32 value, const f32 extent) noexcept {
    if (extent <= kReductionEps) {
        return 0u;
    }
    const f32 normalized = std::clamp((value / extent) * 0.5f + 0.5f, 0.0f, 1.0f);
    const f32 scaled = normalized * 63.0f;
    return static_cast<u8>(std::lround(scaled));
}

struct BoxSupportEdge final {
    Vec3 p0_world{};
    Vec3 p1_world{};
    u8 edge_code{};
};

[[nodiscard]] inline BoxSupportEdge box_support_edge(const Vec3 center, const std::array<Vec3, 3> &axes,
                                                     const Vec3 half_extents, const u32 edge_axis,
                                                     const Vec3 support_dir) noexcept {
#ifndef NDEBUG
    if (edge_axis >= 3u) {
        log::error(physics, "Invalid edge axis index={}", edge_axis);
        std::terminate();
    }
#endif

    const u32 u_axis = (edge_axis + 1u) % 3u;
    const u32 v_axis = (edge_axis + 2u) % 3u;

    const bool u_positive = dot(support_dir, axes[u_axis]) >= 0.0f;
    const bool v_positive = dot(support_dir, axes[v_axis]) >= 0.0f;
    const f32 u_offset = u_positive ? half_extents[u_axis] : -half_extents[u_axis];
    const f32 v_offset = v_positive ? half_extents[v_axis] : -half_extents[v_axis];

    const Vec3 base = center + axes[u_axis] * u_offset + axes[v_axis] * v_offset;
    const Vec3 edge_extent = axes[edge_axis] * half_extents[edge_axis];
    return BoxSupportEdge{
        .p0_world = base - edge_extent,
        .p1_world = base + edge_extent,
        .edge_code = static_cast<u8>((u_positive ? 1u : 0u) | (v_positive ? 2u : 0u)),
    };
}

[[nodiscard]] inline std::pair<Vec3, Vec3> closest_points_on_segments(const Vec3 p0, const Vec3 p1, const Vec3 q0,
                                                                      const Vec3 q1) noexcept {
    const Vec3 d1 = p1 - p0;
    const Vec3 d2 = q1 - q0;
    const Vec3 r = p0 - q0;
    const f32 a = dot(d1, d1);
    const f32 e = dot(d2, d2);
    const f32 f = dot(d2, r);

    f32 s = 0.0f;
    f32 t = 0.0f;

    if (a <= kSegmentEps && e <= kSegmentEps) {
        return std::pair{p0, q0};
    }

    if (a <= kSegmentEps) {
        t = std::clamp(f / e, 0.0f, 1.0f);
    } else {
        const f32 c = dot(d1, r);
        if (e <= kSegmentEps) {
            s = std::clamp(-c / a, 0.0f, 1.0f);
        } else {
            const f32 b = dot(d1, d2);
            const f32 denom = a * e - b * b;
            if (std::fabs(denom) > kSegmentEps) {
                s = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }

            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    return std::pair{p0 + d1 * s, q0 + d2 * t};
}

void add_box_box_contact(std::span<const Vec3> position, std::span<const Quat> orientation,
                         std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                         std::span<const u32> shape_index, const u32 a, const u32 b,
                         const ContactManifold *previous_manifold, std::vector<ContactManifold> &manifolds) {
    const BoxShape &box_a = box_shape(shape_kind, shapes, shape_index, a);
    const BoxShape &box_b = box_shape(shape_kind, shapes, shape_index, b);

    const Vec3 center_a = position[a];
    const Vec3 center_b = position[b];
    const Mat3 rot_a = to_mat3(orientation[a]);
    const Mat3 rot_b = to_mat3(orientation[b]);
    const Vec3 a0 = rot_a.col(0);
    const Vec3 a1 = rot_a.col(1);
    const Vec3 a2 = rot_a.col(2);
    const Vec3 b0 = rot_b.col(0);
    const Vec3 b1 = rot_b.col(1);
    const Vec3 b2 = rot_b.col(2);

    const Vec3 he_a = box_a.half_extents;
    const Vec3 he_b = box_b.half_extents;

    // SAT frame preparation: pairwise axis dot products and center delta in A-space.
    f32 axis_dot_a_to_b[3][3];
    f32 axis_dot_a_to_b_abs[3][3];
    axis_dot_a_to_b[0][0] = dot(a0, b0);
    axis_dot_a_to_b[0][1] = dot(a0, b1);
    axis_dot_a_to_b[0][2] = dot(a0, b2);
    axis_dot_a_to_b[1][0] = dot(a1, b0);
    axis_dot_a_to_b[1][1] = dot(a1, b1);
    axis_dot_a_to_b[1][2] = dot(a1, b2);
    axis_dot_a_to_b[2][0] = dot(a2, b0);
    axis_dot_a_to_b[2][1] = dot(a2, b1);
    axis_dot_a_to_b[2][2] = dot(a2, b2);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            axis_dot_a_to_b_abs[i][j] = std::fabs(axis_dot_a_to_b[i][j]) + kAxisAbsEps;
        }
    }

    const Vec3 delta = center_b - center_a;
    const f32 delta_in_a[3]{dot(delta, a0), dot(delta, a1), dot(delta, a2)};

    std::array<BoxAxisCandidate, 3> face_axes_a{};
    std::array<BoxAxisCandidate, 3> face_axes_b{};
    std::array<std::array<BoxAxisCandidate, 3>, 3> edge_axes{};

    // Axes: A0, A1, A2
    for (int i = 0; i < 3; ++i) {
        const f32 ra = he_a[i];
        const f32 rb = he_b.x * axis_dot_a_to_b_abs[i][0] + he_b.y * axis_dot_a_to_b_abs[i][1] +
                       he_b.z * axis_dot_a_to_b_abs[i][2];
        const f32 dist = std::fabs(delta_in_a[i]);
        if (dist > ra + rb) {
            return;
        }
        face_axes_a[i] = BoxAxisCandidate{
            .key = BoxAxisKey{.type = BoxAxisType::face_a, .i = static_cast<u8>(i), .j = 0u},
            .axis = (i == 0)   ? a0
                    : (i == 1) ? a1
                               : a2,
            .depth = (ra + rb) - dist,
            .valid = true,
        };
    }

    // Axes: B0, B1, B2
    for (int j = 0; j < 3; ++j) {
        const f32 ra = he_a.x * axis_dot_a_to_b_abs[0][j] + he_a.y * axis_dot_a_to_b_abs[1][j] +
                       he_a.z * axis_dot_a_to_b_abs[2][j];
        const f32 rb = he_b[j];
        const f32 dist = std::fabs(delta_in_a[0] * axis_dot_a_to_b[0][j] + delta_in_a[1] * axis_dot_a_to_b[1][j] +
                                   delta_in_a[2] * axis_dot_a_to_b[2][j]);
        if (dist > ra + rb) {
            return;
        }
        face_axes_b[j] = BoxAxisCandidate{
            .key = BoxAxisKey{.type = BoxAxisType::face_b, .i = static_cast<u8>(j), .j = 0u},
            .axis = (j == 0)   ? b0
                    : (j == 1) ? b1
                               : b2,
            .depth = (ra + rb) - dist,
            .valid = true,
        };
    }

    // Axes: Ai x Bj
    for (int i = 0; i < 3; ++i) {
        const int i1 = (i + 1) % 3;
        const int i2 = (i + 2) % 3;
        for (int j = 0; j < 3; ++j) {
            const int j1 = (j + 1) % 3;
            const int j2 = (j + 2) % 3;

            const f32 ra = he_a[i1] * axis_dot_a_to_b_abs[i2][j] + he_a[i2] * axis_dot_a_to_b_abs[i1][j];
            const f32 rb = he_b[j1] * axis_dot_a_to_b_abs[i][j2] + he_b[j2] * axis_dot_a_to_b_abs[i][j1];
            const f32 dist =
                std::fabs(delta_in_a[i2] * axis_dot_a_to_b[i1][j] - delta_in_a[i1] * axis_dot_a_to_b[i2][j]);
            if (dist > ra + rb) {
                return;
            }

            Vec3 axis{};
            if (i == 0) {
                axis = (j == 0) ? cross(a0, b0) : (j == 1) ? cross(a0, b1) : cross(a0, b2);
            } else if (i == 1) {
                axis = (j == 0) ? cross(a1, b0) : (j == 1) ? cross(a1, b1) : cross(a1, b2);
            } else {
                axis = (j == 0) ? cross(a2, b0) : (j == 1) ? cross(a2, b1) : cross(a2, b2);
            }

            const f32 axis_len_sq = axis.length_sq();
            if (axis_len_sq <= kAxisEpsSq) {
                continue;
            }
            const f32 axis_len = std::sqrt(axis_len_sq);
            edge_axes[i][j] = BoxAxisCandidate{
                .key = BoxAxisKey{.type = BoxAxisType::edge_edge, .i = static_cast<u8>(i), .j = static_cast<u8>(j)},
                .axis = axis / axis_len,
                .depth = ((ra + rb) - dist) / axis_len,
                .valid = true,
            };
        }
    }

    BoxAxisCandidate best_axis{};
    // Select minimum-penetration separating axis with deterministic tie-breaks.
    for (const BoxAxisCandidate &candidate : face_axes_a) {
        if (better_axis_candidate(candidate, best_axis)) {
            best_axis = candidate;
        }
    }
    for (const BoxAxisCandidate &candidate : face_axes_b) {
        if (better_axis_candidate(candidate, best_axis)) {
            best_axis = candidate;
        }
    }
    for (const auto &row : edge_axes) {
        for (const BoxAxisCandidate &candidate : row) {
            if (better_axis_candidate(candidate, best_axis)) {
                best_axis = candidate;
            }
        }
    }

    if (const std::optional<BoxAxisKey> previous_axis_key = previous_box_axis_key(previous_manifold)) {
        // Axis hysteresis: prefer the previous valid axis when depths are similar.
        if (const BoxAxisCandidate *previous_axis =
                find_axis_candidate(face_axes_a, face_axes_b, edge_axes, *previous_axis_key);
            previous_axis != nullptr && previous_axis->valid &&
            previous_axis->depth <= best_axis.depth + kAxisHysteresisAbs) {
            best_axis = *previous_axis;
        }
    }

    Vec3 normal = best_axis.axis;
    if (dot(delta, normal) < 0.0f) {
        normal = -normal;
    }
    const u32 axis_feature_id = box_axis_feature_id(best_axis.key);

    const std::array<Vec3, 3> axes_a{a0, a1, a2};
    const std::array<Vec3, 3> axes_b{b0, b1, b2};

    if (best_axis.key.type == BoxAxisType::edge_edge) {
        // Edge-edge contact resolves to one support segment pair and one contact point.
        const BoxSupportEdge edge_a = box_support_edge(center_a, axes_a, he_a, best_axis.key.i, normal);
        const BoxSupportEdge edge_b = box_support_edge(center_b, axes_b, he_b, best_axis.key.j, -normal);
        const auto [point_a, point_b] =
            closest_points_on_segments(edge_a.p0_world, edge_a.p1_world, edge_b.p0_world, edge_b.p1_world);
        const u32 feature_id = box_edge_edge_feature_id(best_axis.key, edge_a.edge_code, edge_b.edge_code);
        push_single_point_manifold(orientation, a, b, normal, best_axis.depth, feature_id, point_a - center_a,
                                   point_b - center_b, manifolds, axis_feature_id);
        return;
    }

    const bool ref_is_a = best_axis.key.type == BoxAxisType::face_a;
    const std::array<Vec3, 3> &ref_axes = ref_is_a ? axes_a : axes_b;
    const std::array<Vec3, 3> &incident_axes = ref_is_a ? axes_b : axes_a;
    const Vec3 ref_center = ref_is_a ? center_a : center_b;
    const Vec3 incident_center = ref_is_a ? center_b : center_a;
    const Vec3 ref_he = ref_is_a ? he_a : he_b;
    const Vec3 incident_he = ref_is_a ? he_b : he_a;

    const u32 ref_axis = best_axis.key.i;
    const Vec3 ref_axis_vec = ref_axes[ref_axis];
    const Vec3 ref_to_incident_normal = ref_is_a ? normal : -normal;
    const bool ref_positive = dot(ref_axis_vec, ref_to_incident_normal) >= 0.0f;
    const Vec3 ref_face_center = ref_center + ref_axis_vec * (ref_positive ? ref_he[ref_axis] : -ref_he[ref_axis]);
    const u32 ref_u_axis = (ref_axis + 1u) % 3u;
    const u32 ref_v_axis = (ref_axis + 2u) % 3u;
    const Vec3 ref_u = ref_axes[ref_u_axis];
    const Vec3 ref_v = ref_axes[ref_v_axis];
    const f32 ref_u_extent = ref_he[ref_u_axis];
    const f32 ref_v_extent = ref_he[ref_v_axis];

    u8 incident_axis = 0u;
    bool incident_positive = false;
    f32 best_incident_dot = std::numeric_limits<f32>::max();
    for (u8 axis = 0u; axis < 3u; ++axis) {
        for (u8 sign_i = 0u; sign_i < 2u; ++sign_i) {
            const bool positive = sign_i != 0u;
            const Vec3 incident_normal = incident_axes[axis] * (positive ? 1.0f : -1.0f);
            const f32 dot_value = dot(incident_normal, ref_to_incident_normal);
            const bool better = dot_value < best_incident_dot - kReductionEps ||
                                (std::fabs(dot_value - best_incident_dot) <= kReductionEps &&
                                 (axis < incident_axis || (axis == incident_axis && positive < incident_positive)));
            if (better) {
                best_incident_dot = dot_value;
                incident_axis = axis;
                incident_positive = positive;
            }
        }
    }

    std::array<ClipVertex, 8> clip_a{};
    std::array<ClipVertex, 8> clip_b{};
    const std::array<u8, 4> incident_face_indices = box_face_vertex_indices(incident_axis, incident_positive);
    u32 clip_count = 4u;
    for (u32 i = 0; i < clip_count; ++i) {
        const u8 vertex_index = incident_face_indices[i];
        const Vec3 local_vertex = box_local_vertex(incident_he, vertex_index);
        const Vec3 point_world = incident_center + incident_axes[0] * local_vertex.x +
                                 incident_axes[1] * local_vertex.y + incident_axes[2] * local_vertex.z;
        const Vec3 relative = point_world - ref_face_center;
        clip_a[i] = ClipVertex{
            .point_world = point_world,
            .u = dot(relative, ref_u),
            .v = dot(relative, ref_v),
            .id0 = vertex_index,
            .id1 = vertex_index,
        };
    }

    clip_count =
        clip_polygon_axis(std::span<const ClipVertex>{clip_a.data(), clip_count}, true, ref_u_extent, true, clip_b);
    clip_count =
        clip_polygon_axis(std::span<const ClipVertex>{clip_b.data(), clip_count}, true, -ref_u_extent, false, clip_a);
    clip_count =
        clip_polygon_axis(std::span<const ClipVertex>{clip_a.data(), clip_count}, false, ref_v_extent, true, clip_b);
    clip_count =
        clip_polygon_axis(std::span<const ClipVertex>{clip_b.data(), clip_count}, false, -ref_v_extent, false, clip_a);

    // Face-face clipping produced candidate contact points in reference-face coordinates.
    struct FaceContactCandidate final {
        Vec3 r_a_world{};
        Vec3 r_b_world{};
        f32 separation{};
        f32 penetration{};
        f32 u{};
        f32 v{};
        u32 feature_id{};
    };

    std::array<FaceContactCandidate, 8> candidates{};
    u32 candidate_count = 0u;
    for (u32 i = 0; i < clip_count; ++i) {
        const ClipVertex &clip_vertex = clip_a[i];
        const f32 separation = ref_is_a ? dot(clip_vertex.point_world - ref_face_center, normal)
                                        : dot(ref_face_center - clip_vertex.point_world, normal);
        if (separation > kAxisAbsEps) {
            continue;
        }

        Vec3 point_a_world{};
        Vec3 point_b_world{};
        if (ref_is_a) {
            point_b_world = clip_vertex.point_world;
            point_a_world = point_b_world - normal * separation;
        } else {
            point_a_world = clip_vertex.point_world;
            point_b_world = point_a_world + normal * separation;
        }

#ifndef NDEBUG
        if (candidate_count >= candidates.size()) {
            log::error(physics, "Box-box face clipping overflow candidate_count={}", candidate_count);
            std::terminate();
        }
#endif
        candidates[candidate_count++] = FaceContactCandidate{
            .r_a_world = point_a_world - center_a,
            .r_b_world = point_b_world - center_b,
            .separation = separation,
            .penetration = std::max(-separation, 0.0f),
            .u = clip_vertex.u,
            .v = clip_vertex.v,
            .feature_id = box_face_face_feature_id(static_cast<u8>(ref_axis), ref_positive, ref_is_a, incident_axis,
                                                   incident_positive, clip_vertex.id0, clip_vertex.id1,
                                                   quantize_face_coordinate(clip_vertex.u, ref_u_extent),
                                                   quantize_face_coordinate(clip_vertex.v, ref_v_extent)),
        };
    }

    if (candidate_count > 1u) {
        // Merge duplicate feature ids and keep deepest representative for warm-start stability.
        u32 unique_count = 0u;
        for (u32 i = 0u; i < candidate_count; ++i) {
            const FaceContactCandidate candidate = candidates[i];
            bool merged = false;
            for (u32 unique_index = 0u; unique_index < unique_count; ++unique_index) {
                FaceContactCandidate &existing = candidates[unique_index];
                if (existing.feature_id != candidate.feature_id) {
                    continue;
                }
                if (candidate.penetration > existing.penetration + kReductionEps) {
                    existing = candidate;
                }
                merged = true;
                break;
            }
            if (!merged) {
                candidates[unique_count++] = candidate;
            }
        }
        candidate_count = unique_count;
    }

    if (candidate_count == 0u) {
        const Vec3 point_a = box_support_point(center_a, a0, a1, a2, he_a, normal);
        const Vec3 point_b = box_support_point(center_b, b0, b1, b2, he_b, -normal);
        push_single_point_manifold(orientation, a, b, normal, best_axis.depth, axis_feature_id, point_a - center_a,
                                   point_b - center_b, manifolds, axis_feature_id);
        return;
    }

    auto candidate_deeper = [&](const u32 lhs, const u32 rhs) {
        const f32 diff = candidates[lhs].penetration - candidates[rhs].penetration;
        if (std::fabs(diff) > kReductionEps) {
            return diff > 0.0f;
        }
        if (candidates[lhs].feature_id != candidates[rhs].feature_id) {
            return candidates[lhs].feature_id < candidates[rhs].feature_id;
        }
        return lhs < rhs;
    };

    std::array<u32, kMaxManifoldPoints> selected{};
    u32 selected_count = 0u;
    if (candidate_count <= kMaxManifoldPoints) {
        for (u32 i = 0; i < candidate_count; ++i) {
            selected[selected_count++] = i;
        }
    } else {
        // Reduction policy: keep deepest point and maximize spatial spread of the remaining three.
        u32 deepest = 0u;
        for (u32 i = 1u; i < candidate_count; ++i) {
            if (candidate_deeper(i, deepest)) {
                deepest = i;
            }
        }

        auto spread_score = [&](const std::array<u32, 4> pick) {
            f32 best_area2 = 0.0f;
            for (u32 i = 0; i < 4u; ++i) {
                for (u32 j = i + 1u; j < 4u; ++j) {
                    for (u32 k = j + 1u; k < 4u; ++k) {
                        const f32 ab_u = candidates[pick[j]].u - candidates[pick[i]].u;
                        const f32 ab_v = candidates[pick[j]].v - candidates[pick[i]].v;
                        const f32 ac_u = candidates[pick[k]].u - candidates[pick[i]].u;
                        const f32 ac_v = candidates[pick[k]].v - candidates[pick[i]].v;
                        const f32 area2 = std::fabs(ab_u * ac_v - ab_v * ac_u);
                        best_area2 = std::max(best_area2, area2);
                    }
                }
            }
            return best_area2;
        };

        std::array<u32, 7> remaining{};
        u32 remaining_count = 0u;
        for (u32 i = 0u; i < candidate_count; ++i) {
            if (i == deepest) {
                continue;
            }
            remaining[remaining_count++] = i;
        }

        std::array<u32, 3> best_extra{remaining[0], remaining[1], remaining[2]};
        f32 best_score = -1.0f;
        bool have_best = false;
        for (u32 i = 0u; i + 2u < remaining_count; ++i) {
            for (u32 j = i + 1u; j + 1u < remaining_count; ++j) {
                for (u32 k = j + 1u; k < remaining_count; ++k) {
                    const std::array<u32, 3> extra{remaining[i], remaining[j], remaining[k]};
                    const std::array<u32, 4> combo{deepest, extra[0], extra[1], extra[2]};
                    const f32 score = spread_score(combo);

                    std::array<u32, 3> ids{
                        candidates[extra[0]].feature_id,
                        candidates[extra[1]].feature_id,
                        candidates[extra[2]].feature_id,
                    };
                    std::sort(ids.begin(), ids.end());

                    std::array<u32, 3> best_ids{
                        candidates[best_extra[0]].feature_id,
                        candidates[best_extra[1]].feature_id,
                        candidates[best_extra[2]].feature_id,
                    };
                    std::sort(best_ids.begin(), best_ids.end());

                    const bool better = !have_best || score > best_score + kReductionEps ||
                                        (std::fabs(score - best_score) <= kReductionEps && ids < best_ids);
                    if (better) {
                        best_extra = extra;
                        best_score = score;
                        have_best = true;
                    }
                }
            }
        }

        selected[selected_count++] = deepest;
        selected[selected_count++] = best_extra[0];
        selected[selected_count++] = best_extra[1];
        selected[selected_count++] = best_extra[2];
    }

    std::sort(selected.begin(), selected.begin() + selected_count,
              [&](const u32 lhs, const u32 rhs) { return candidate_deeper(lhs, rhs); });

    ContactManifold manifold{
        .a = a,
        .b = b,
        .manifold_feature_id = axis_feature_id,
        .normal = normal,
        .point_count = selected_count,
    };
    for (u32 i = 0u; i < selected_count; ++i) {
        const FaceContactCandidate &candidate = candidates[selected[i]];
        ContactPoint point{};
        point.local_anchor_a = world_offset_to_local_anchor(orientation[a], candidate.r_a_world);
        point.local_anchor_b = world_offset_to_local_anchor(orientation[b], candidate.r_b_world);
        point.separation = candidate.separation;
        point.feature_id = candidate.feature_id;
        manifold.points[i] = point;
    }

    canonicalize_manifold_orientation(manifold);
    manifolds.push_back(manifold);
}

void add_sphere_ground_contact(std::span<const Vec3> position, std::span<const Quat> orientation,
                               std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                               std::span<const u32> shape_index, std::span<const f32> inv_mass, const u32 id,
                               std::vector<ContactManifold> &manifolds) {
    if (inv_mass[id] == 0.0f) {
        return;
    }
    const SphereShape &sphere = sphere_shape(shape_kind, shapes, shape_index, id);
    const f32 signed_distance = position[id].y - kGroundOffset;
    if (signed_distance >= sphere.radius) {
        return;
    }
    const f32 penetration = sphere.radius - signed_distance;
    const Vec3 normal = -kGroundNormal;
    const Vec3 r_a = normal * sphere.radius;
    push_single_point_manifold(orientation, id, kInvalidBody, normal, penetration, 0u, r_a, Vec3{}, manifolds);
}

void add_box_ground_contact(std::span<const Vec3> position, std::span<const Quat> orientation,
                            std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                            std::span<const u32> shape_index, std::span<const f32> inv_mass, const u32 id,
                            std::vector<ContactManifold> &manifolds) {
    if (inv_mass[id] == 0.0f) {
        return;
    }
    const BoxShape &box = box_shape(shape_kind, shapes, shape_index, id);
    const Vec3 center = position[id];
    const Mat3 rot = to_mat3(orientation[id]);
    const Vec3 a0 = rot.col(0);
    const Vec3 a1 = rot.col(1);
    const Vec3 a2 = rot.col(2);
    struct CandidatePoint final {
        Vec3 r_a_world{};
        f32 penetration{};
        u32 vertex_index{};
    };

    std::array<CandidatePoint, 8> candidates{};
    u32 candidate_count = 0;
    for (u32 vertex_index = 0; vertex_index < 8; ++vertex_index) {
        const f32 sx = (vertex_index & 0x1u) ? 1.0f : -1.0f;
        const f32 sy = (vertex_index & 0x2u) ? 1.0f : -1.0f;
        const f32 sz = (vertex_index & 0x4u) ? 1.0f : -1.0f;
        const Vec3 local_vertex{sx * box.half_extents.x, sy * box.half_extents.y, sz * box.half_extents.z};
        const Vec3 r_a_world = a0 * local_vertex.x + a1 * local_vertex.y + a2 * local_vertex.z;
        const Vec3 world_vertex = center + r_a_world;
        const f32 signed_distance = world_vertex.y - kGroundOffset;
        if (signed_distance >= 0.0f) {
            continue;
        }
        candidates[candidate_count++] = CandidatePoint{
            .r_a_world = r_a_world,
            .penetration = -signed_distance,
            .vertex_index = vertex_index,
        };
    }

    if (candidate_count == 0u) {
        return;
    }

    auto penetration_greater = [&](const u32 lhs, const u32 rhs) {
        const f32 diff = candidates[lhs].penetration - candidates[rhs].penetration;
        if (std::fabs(diff) > kReductionEps) {
            return diff > 0.0f;
        }
        return candidates[lhs].vertex_index < candidates[rhs].vertex_index;
    };

    u32 deepest = 0u;
    for (u32 i = 1; i < candidate_count; ++i) {
        if (penetration_greater(i, deepest)) {
            deepest = i;
        }
    }

    auto spread_score = [&](const std::array<u32, 4> selected) {
        f32 best_area2 = 0.0f;
        for (u32 i = 0; i < 4; ++i) {
            for (u32 j = i + 1; j < 4; ++j) {
                for (u32 k = j + 1; k < 4; ++k) {
                    const Vec3 &a = candidates[selected[i]].r_a_world;
                    const Vec3 &b = candidates[selected[j]].r_a_world;
                    const Vec3 &c = candidates[selected[k]].r_a_world;
                    const f32 abx = b.x - a.x;
                    const f32 abz = b.z - a.z;
                    const f32 acx = c.x - a.x;
                    const f32 acz = c.z - a.z;
                    const f32 area2 = std::fabs(abx * acz - abz * acx);
                    best_area2 = std::max(best_area2, area2);
                }
            }
        }
        return best_area2;
    };

    std::array<u32, kMaxManifoldPoints> selected{};
    u32 selected_count = 0u;
    selected[selected_count++] = deepest;

    if (candidate_count <= kMaxManifoldPoints) {
        for (u32 i = 0; i < candidate_count; ++i) {
            if (i == deepest) {
                continue;
            }
            selected[selected_count++] = i;
        }
    } else {
        std::array<u32, 7> remaining{};
        u32 remaining_count = 0u;
        for (u32 i = 0; i < candidate_count; ++i) {
            if (i == deepest) {
                continue;
            }
            remaining[remaining_count++] = i;
        }

        std::array<u32, 3> best_extra{remaining[0], remaining[1], remaining[2]};
        f32 best_score = -1.0f;
        bool have_best = false;

        for (u32 i = 0; i + 2 < remaining_count; ++i) {
            for (u32 j = i + 1; j + 1 < remaining_count; ++j) {
                for (u32 k = j + 1; k < remaining_count; ++k) {
                    const std::array<u32, 3> extra{remaining[i], remaining[j], remaining[k]};
                    const std::array<u32, 4> combo{deepest, extra[0], extra[1], extra[2]};
                    const f32 score = spread_score(combo);

                    bool better = false;
                    if (!have_best || score > best_score + kReductionEps) {
                        better = true;
                    } else if (std::fabs(score - best_score) <= kReductionEps) {
                        const std::array<u32, 3> best_ids{
                            candidates[best_extra[0]].vertex_index,
                            candidates[best_extra[1]].vertex_index,
                            candidates[best_extra[2]].vertex_index,
                        };
                        const std::array<u32, 3> ids{
                            candidates[extra[0]].vertex_index,
                            candidates[extra[1]].vertex_index,
                            candidates[extra[2]].vertex_index,
                        };
                        better = ids < best_ids;
                    }

                    if (better) {
                        best_extra = extra;
                        best_score = score;
                        have_best = true;
                    }
                }
            }
        }

        selected[selected_count++] = best_extra[0];
        selected[selected_count++] = best_extra[1];
        selected[selected_count++] = best_extra[2];
    }

    std::sort(selected.begin(), selected.begin() + selected_count,
              [&](const u32 lhs, const u32 rhs) { return penetration_greater(lhs, rhs); });

    ContactManifold manifold{
        .a = id,
        .b = kInvalidBody,
        .normal = -kGroundNormal,
        .point_count = selected_count,
    };
    for (u32 i = 0; i < selected_count; ++i) {
        const CandidatePoint &candidate = candidates[selected[i]];
        ContactPoint point{};
        point.local_anchor_a = world_offset_to_local_anchor(orientation[id], candidate.r_a_world);
        point.local_anchor_b = Vec3{};
        point.separation = -candidate.penetration;
        point.feature_id = box_ground_vertex_feature_id(candidate.vertex_index);
        manifold.points[i] = point;
    }

    canonicalize_manifold_orientation(manifold);
    manifolds.push_back(manifold);
}
} // namespace javelin::detail

export namespace javelin {

void narrow_phase_contacts(std::span<const Vec3> position, std::span<const Quat> orientation,
                           std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                           std::span<const u32> shape_index, std::span<const f32> inv_mass,
                           std::span<const BodyPair> pairs, std::span<const ContactManifold> previous_manifolds,
                           std::vector<ContactManifold> &manifolds) {
    ZoneScopedN("Physics narrow phase");
    manifolds.clear();
    manifolds.reserve(pairs.size() + position.size());

    // previous_manifolds are pair-sorted by the physics system.
    // Track one monotonic cursor to avoid per-pair hash lookups.
    u32 previous_cursor = 0u;
#ifndef NDEBUG
    for (u32 i = 1; i < previous_manifolds.size(); ++i) {
        const ContactManifold &prev = previous_manifolds[i - 1];
        const ContactManifold &curr = previous_manifolds[i];
        if (curr.a < prev.a || (curr.a == prev.a && curr.b < prev.b)) {
            log::error(physics, "Previous manifolds are not pair-sorted (index={} prev=({}, {}) curr=({}, {}))", i,
                       prev.a, prev.b, curr.a, curr.b);
            std::terminate();
        }
        if (curr.a == prev.a && curr.b == prev.b) {
            log::error(physics, "Duplicate previous manifold pair (index={} pair=({}, {}))", i, curr.a, curr.b);
            std::terminate();
        }
    }
#endif

    for (const BodyPair pair : pairs) {
        const u32 a = pair.a;
        const u32 b = pair.b;
        if (inv_mass[a] == 0.0f && inv_mass[b] == 0.0f) {
            continue;
        }
        const ShapeKind kind_a = shape_kind[a];
        const ShapeKind kind_b = shape_kind[b];
        if (kind_a == ShapeKind::sphere && kind_b == ShapeKind::sphere) {
            detail::add_sphere_sphere_contact(position, shape_kind, orientation, shapes, shape_index, a, b, manifolds);
        } else if (kind_a == ShapeKind::sphere && kind_b == ShapeKind::box) {
            detail::add_sphere_box_contact(position, orientation, shape_kind, shapes, shape_index, a, b, true,
                                           manifolds);
        } else if (kind_a == ShapeKind::box && kind_b == ShapeKind::sphere) {
            detail::add_sphere_box_contact(position, orientation, shape_kind, shapes, shape_index, b, a, false,
                                           manifolds);
        } else if (kind_a == ShapeKind::box && kind_b == ShapeKind::box) {
            const BodyPair canonical_pair = canonical_body_pair(a, b);
            const ContactManifold *previous_manifold = nullptr;
            while (previous_cursor < previous_manifolds.size()) {
                const ContactManifold &candidate = previous_manifolds[previous_cursor];
                if (candidate.a < canonical_pair.a ||
                    (candidate.a == canonical_pair.a && candidate.b < canonical_pair.b)) {
                    ++previous_cursor;
                    continue;
                }
                if (candidate.a == canonical_pair.a && candidate.b == canonical_pair.b) {
                    previous_manifold = &candidate;
                }
                break;
            }
#ifndef NDEBUG
            if (previous_manifold != nullptr && previous_cursor + 1u < previous_manifolds.size()) {
                const ContactManifold &next = previous_manifolds[previous_cursor + 1u];
                if (next.a == canonical_pair.a && next.b == canonical_pair.b) {
                    log::error(physics, "Duplicate previous manifold pair during narrow phase (a={} b={})",
                               canonical_pair.a, canonical_pair.b);
                    std::terminate();
                }
            }
#endif
            detail::add_box_box_contact(position, orientation, shape_kind, shapes, shape_index, canonical_pair.a,
                                        canonical_pair.b, previous_manifold, manifolds);
        }
    }

    const u32 count = static_cast<u32>(position.size());
    for (u32 i = 0; i < count; ++i) {
        if (shape_kind[i] == ShapeKind::sphere) {
            detail::add_sphere_ground_contact(position, orientation, shape_kind, shapes, shape_index, inv_mass, i,
                                              manifolds);
        } else if (shape_kind[i] == ShapeKind::box) {
            detail::add_box_ground_contact(position, orientation, shape_kind, shapes, shape_index, inv_mass, i,
                                           manifolds);
        }
    }
}

} // namespace javelin
