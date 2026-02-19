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
inline constexpr Vec3 kGroundNormal{0.0f, 1.0f, 0.0f};
inline constexpr f32 kGroundOffset = 0.0f;
inline constexpr f32 kMinDistanceEpsSq = 1e-6f;
inline constexpr f32 kAxisEpsSq = 1e-8f;
inline constexpr f32 kAxisAbsEps = 1e-6f;
inline constexpr u32 kBoxFaceFeatureTag = 1u << 8u;
inline constexpr u32 kBoxGroundVertexFeatureTag = 1u << 9u;
inline constexpr f32 kReductionEps = 1e-6f;

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

[[nodiscard]] inline Vec3 world_offset_to_local_anchor(const Quat q, const Vec3 offset_world) noexcept {
    return rotate(inverse_unit(q), offset_world);
}

void push_single_point_manifold(std::span<const Quat> orientation, const u32 a, const u32 b, const Vec3 normal,
                                const f32 penetration, const u32 feature_id, const Vec3 r_a_world, const Vec3 r_b_world,
                                std::vector<ContactManifold> &manifolds) {
    ContactManifold manifold{
        .a = a,
        .b = b,
        .normal = normal,
        .point_count = 1u,
    };
    ContactPoint &point = manifold.points[0];
    point.local_anchor_a = world_offset_to_local_anchor(orientation[a], r_a_world);
    point.local_anchor_b = (b != kInvalidBody) ? world_offset_to_local_anchor(orientation[b], r_b_world) : Vec3{};
    point.separation = -penetration;
    point.feature_id = feature_id;

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

    const Vec3 center_s = position[sphere_id];
    const Vec3 center_b = position[box_id];
    const Mat3 rot = to_mat3(orientation[box_id]);
    const Vec3 a0 = rot.col(0);
    const Vec3 a1 = rot.col(1);
    const Vec3 a2 = rot.col(2);

    const Vec3 delta = center_s - center_b;
    const Vec3 local{dot(delta, a0), dot(delta, a1), dot(delta, a2)};

    const Vec3 he = box.half_extents;
    const Vec3 clamped{
        std::clamp(local.x, -he.x, he.x),
        std::clamp(local.y, -he.y, he.y),
        std::clamp(local.z, -he.z, he.z),
    };

    const Vec3 diff = local - clamped;
    const f32 dist2 = diff.length_sq();
    const f32 radius2 = sphere.radius * sphere.radius;
    if (dist2 > radius2) {
        return;
    }

    Vec3 normal_local = Vec3::unit_x();
    Vec3 contact_local = clamped;
    f32 penetration = 0.0f;
    u32 feature_axis = 0u;
    bool feature_positive = true;
    bool inside_fallback = false;
    if (dist2 > kMinDistanceEpsSq) {
        const f32 dist = std::sqrt(dist2);
        normal_local = diff / dist;
        penetration = sphere.radius - dist;
        feature_axis = dominant_axis_abs(normal_local);
        const f32 axis_value = (feature_axis == 0u) ? normal_local.x : (feature_axis == 1u) ? normal_local.y : normal_local.z;
        feature_positive = axis_value >= 0.0f;
    } else {
        inside_fallback = true;
        const f32 dx = he.x - std::fabs(local.x);
        const f32 dy = he.y - std::fabs(local.y);
        const f32 dz = he.z - std::fabs(local.z);
        if (dx <= dy && dx <= dz) {
            const f32 sign = (local.x >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{sign, 0.0f, 0.0f};
            contact_local = Vec3{sign * he.x, local.y, local.z};
            penetration = sphere.radius + dx;
            feature_axis = 0u;
            feature_positive = sign > 0.0f;
        } else if (dy <= dz) {
            const f32 sign = (local.y >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{0.0f, sign, 0.0f};
            contact_local = Vec3{local.x, sign * he.y, local.z};
            penetration = sphere.radius + dy;
            feature_axis = 1u;
            feature_positive = sign > 0.0f;
        } else {
            const f32 sign = (local.z >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{0.0f, 0.0f, sign};
            contact_local = Vec3{local.x, local.y, sign * he.z};
            penetration = sphere.radius + dz;
            feature_axis = 2u;
            feature_positive = sign > 0.0f;
        }
    }
    const u32 feature_id = box_face_feature_id(feature_axis, feature_positive, inside_fallback);

    const Vec3 normal_world = a0 * normal_local.x + a1 * normal_local.y + a2 * normal_local.z;
    const Vec3 contact_world = center_b + a0 * contact_local.x + a1 * contact_local.y + a2 * contact_local.z;

    Vec3 normal = normal_world;
    Vec3 r_a{};
    Vec3 r_b{};
    if (sphere_is_a) {
        normal = -normal_world;
        r_a = normal * sphere.radius;
        r_b = contact_world - center_b;
    } else {
        r_a = contact_world - center_b;
        r_b = -normal * sphere.radius;
    }

    push_single_point_manifold(orientation, sphere_is_a ? sphere_id : box_id, sphere_is_a ? box_id : sphere_id, normal,
                               penetration, feature_id, r_a, r_b, manifolds);
}

void add_box_box_contact(std::span<const Vec3> position, std::span<const Quat> orientation,
                         std::span<const ShapeKind> shape_kind, std::span<const ShapeData> shapes,
                         std::span<const u32> shape_index, const u32 a, const u32 b,
                         std::vector<ContactManifold> &manifolds) {
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

    f32 R[3][3];
    f32 absR[3][3];
    R[0][0] = dot(a0, b0);
    R[0][1] = dot(a0, b1);
    R[0][2] = dot(a0, b2);
    R[1][0] = dot(a1, b0);
    R[1][1] = dot(a1, b1);
    R[1][2] = dot(a1, b2);
    R[2][0] = dot(a2, b0);
    R[2][1] = dot(a2, b1);
    R[2][2] = dot(a2, b2);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            absR[i][j] = std::fabs(R[i][j]) + kAxisAbsEps;
        }
    }

    const Vec3 delta = center_b - center_a;
    const f32 t[3]{dot(delta, a0), dot(delta, a1), dot(delta, a2)};

    f32 best_overlap = std::numeric_limits<f32>::max();
    Vec3 best_axis = Vec3::unit_x();

    auto update_axis = [&](const Vec3 axis, const f32 overlap, const f32 axis_len) {
        const f32 depth = overlap / axis_len;
        if (depth < best_overlap) {
            best_overlap = depth;
            best_axis = axis / axis_len;
        }
    };

    // Axes: A0, A1, A2
    for (int i = 0; i < 3; ++i) {
        const f32 ra = he_a[i];
        const f32 rb = he_b.x * absR[i][0] + he_b.y * absR[i][1] + he_b.z * absR[i][2];
        const f32 dist = std::fabs(t[i]);
        if (dist > ra + rb) {
            return;
        }
        update_axis((i == 0) ? a0 : (i == 1) ? a1 : a2, (ra + rb) - dist, 1.0f);
    }

    // Axes: B0, B1, B2
    for (int j = 0; j < 3; ++j) {
        const f32 ra = he_a.x * absR[0][j] + he_a.y * absR[1][j] + he_a.z * absR[2][j];
        const f32 rb = he_b[j];
        const f32 dist = std::fabs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]);
        if (dist > ra + rb) {
            return;
        }
        update_axis((j == 0) ? b0 : (j == 1) ? b1 : b2, (ra + rb) - dist, 1.0f);
    }

    // Axes: Ai x Bj
    for (int i = 0; i < 3; ++i) {
        const int i1 = (i + 1) % 3;
        const int i2 = (i + 2) % 3;
        for (int j = 0; j < 3; ++j) {
            const int j1 = (j + 1) % 3;
            const int j2 = (j + 2) % 3;

            const f32 ra = he_a[i1] * absR[i2][j] + he_a[i2] * absR[i1][j];
            const f32 rb = he_b[j1] * absR[i][j2] + he_b[j2] * absR[i][j1];
            const f32 dist = std::fabs(t[i2] * R[i1][j] - t[i1] * R[i2][j]);
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
            update_axis(axis, (ra + rb) - dist, axis_len);
        }
    }

    Vec3 normal = best_axis;
    if (dot(delta, normal) < 0.0f) {
        normal = -normal;
    }

    const Vec3 point_a = box_support_point(center_a, a0, a1, a2, he_a, normal);
    const Vec3 point_b = box_support_point(center_b, b0, b1, b2, he_b, -normal);

    push_single_point_manifold(orientation, a, b, normal, best_overlap, 0u, point_a - center_a, point_b - center_b,
                               manifolds);
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

    std::sort(selected.begin(), selected.begin() + selected_count, [&](const u32 lhs, const u32 rhs) {
        return penetration_greater(lhs, rhs);
    });

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
                           std::span<const BodyPair> pairs, std::vector<ContactManifold> &manifolds) {
    ZoneScopedN("Physics narrow phase");
    manifolds.clear();
    manifolds.reserve(pairs.size() + position.size());

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
            detail::add_box_box_contact(position, orientation, shape_kind, shapes, shape_index, a, b, manifolds);
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
