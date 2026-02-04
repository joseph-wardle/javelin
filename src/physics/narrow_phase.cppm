module;

#include <tracy/Tracy.hpp>

export module javelin.physics.narrow_phase;

import std;
import javelin.core.logging;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.types;
import javelin.scene.shapes;

namespace javelin::detail {
inline constexpr Vec3 kGroundNormal{0.0f, 1.0f, 0.0f};
inline constexpr f32 kGroundOffset = 0.0f;
inline constexpr f32 kMinDistanceEpsSq = 1e-6f;

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

void add_sphere_sphere_contacts(std::span<const Vec3> position, std::span<const ShapeKind> shape_kind,
                                std::span<const ShapeData> shapes, std::span<const u32> shape_index,
                                std::span<const f32> inv_mass, std::span<const BodyPair> pairs,
                                std::vector<Contact> &contacts) {
    for (const BodyPair pair : pairs) {
        const u32 a = pair.a;
        const u32 b = pair.b;
        if (shape_kind[a] != ShapeKind::sphere || shape_kind[b] != ShapeKind::sphere) {
            continue;
        }

        const SphereShape &sphere_a = sphere_shape(shape_kind, shapes, shape_index, a);
        const SphereShape &sphere_b = sphere_shape(shape_kind, shapes, shape_index, b);

        const Vec3 delta = position[b] - position[a];
        const f32 radius_sum = sphere_a.radius + sphere_b.radius;
        if (std::fabs(delta.x) > radius_sum || std::fabs(delta.y) > radius_sum || std::fabs(delta.z) > radius_sum) {
            continue;
        }

        const f32 dist2 = delta.length_sq();
        const f32 radius_sum2 = radius_sum * radius_sum;
        if (dist2 >= radius_sum2) {
            continue;
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
        contacts.push_back(Contact{
            .a = a,
            .b = b,
            .normal = normal,
            .penetration = penetration,
            .r_a = r_a,
            .r_b = r_b,
        });
    }
}

void add_sphere_ground_contacts(std::span<const Vec3> position, std::span<const ShapeKind> shape_kind,
                                std::span<const ShapeData> shapes, std::span<const u32> shape_index,
                                std::span<const f32> inv_mass, std::vector<Contact> &contacts) {
    const u32 count = static_cast<u32>(position.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        if (shape_kind[i] != ShapeKind::sphere) {
            continue;
        }
        const SphereShape &sphere = sphere_shape(shape_kind, shapes, shape_index, i);
        const f32 signed_distance = position[i].y - kGroundOffset;
        if (signed_distance >= sphere.radius) {
            continue;
        }
        const f32 penetration = sphere.radius - signed_distance;
        const Vec3 normal = -kGroundNormal;
        const Vec3 r_a = normal * sphere.radius;
        contacts.push_back(Contact{
            .a = i,
            .b = kInvalidBody,
            .normal = normal,
            .penetration = penetration,
            .r_a = r_a,
            .r_b = Vec3{},
        });
    }
}
} // namespace javelin::detail

export namespace javelin {

void narrow_phase_contacts(std::span<const Vec3> position, std::span<const ShapeKind> shape_kind,
                           std::span<const ShapeData> shapes, std::span<const u32> shape_index,
                           std::span<const f32> inv_mass, std::span<const BodyPair> pairs,
                           std::vector<Contact> &contacts) {
    ZoneScopedN("Physics narrow phase");
    contacts.clear();
    contacts.reserve(pairs.size() + position.size());
    detail::add_sphere_sphere_contacts(position, shape_kind, shapes, shape_index, inv_mass, pairs, contacts);
    detail::add_sphere_ground_contacts(position, shape_kind, shapes, shape_index, inv_mass, contacts);
}

} // namespace javelin
