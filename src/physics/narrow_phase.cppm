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

[[nodiscard]] inline Vec3 world_offset_to_local_anchor(const Quat q, const Vec3 offset_world) noexcept {
    return rotate(inverse_unit(q), offset_world);
}

void push_single_point_manifold(std::span<const Quat> orientation, const u32 a, const u32 b, const Vec3 normal,
                                const f32 penetration, const Vec3 r_a_world, const Vec3 r_b_world,
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
    point.feature_id = 0u;

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
    push_single_point_manifold(orientation, a, b, normal, penetration, r_a, r_b, manifolds);
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
    if (dist2 > kMinDistanceEpsSq) {
        const f32 dist = std::sqrt(dist2);
        normal_local = diff / dist;
        penetration = sphere.radius - dist;
    } else {
        const f32 dx = he.x - std::fabs(local.x);
        const f32 dy = he.y - std::fabs(local.y);
        const f32 dz = he.z - std::fabs(local.z);
        if (dx <= dy && dx <= dz) {
            const f32 sign = (local.x >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{sign, 0.0f, 0.0f};
            contact_local = Vec3{sign * he.x, local.y, local.z};
            penetration = sphere.radius + dx;
        } else if (dy <= dz) {
            const f32 sign = (local.y >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{0.0f, sign, 0.0f};
            contact_local = Vec3{local.x, sign * he.y, local.z};
            penetration = sphere.radius + dy;
        } else {
            const f32 sign = (local.z >= 0.0f) ? 1.0f : -1.0f;
            normal_local = Vec3{0.0f, 0.0f, sign};
            contact_local = Vec3{local.x, local.y, sign * he.z};
            penetration = sphere.radius + dz;
        }
    }

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

    push_single_point_manifold(orientation, sphere_is_a ? sphere_id : box_id, sphere_is_a ? box_id : sphere_id,
                               normal, penetration, r_a, r_b, manifolds);
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

    push_single_point_manifold(orientation, a, b, normal, best_overlap, point_a - center_a, point_b - center_b,
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
    push_single_point_manifold(orientation, id, kInvalidBody, normal, penetration, r_a, Vec3{}, manifolds);
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

    const f32 radius = std::fabs(a0.y) * box.half_extents.x + std::fabs(a1.y) * box.half_extents.y +
                       std::fabs(a2.y) * box.half_extents.z;
    const f32 signed_distance = center.y - kGroundOffset;
    if (signed_distance >= radius) {
        return;
    }
    const f32 penetration = radius - signed_distance;
    const Vec3 normal = -kGroundNormal;
    const Vec3 contact = box_support_point(center, a0, a1, a2, box.half_extents, normal);
    push_single_point_manifold(orientation, id, kInvalidBody, normal, penetration, contact - center, Vec3{}, manifolds);
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
