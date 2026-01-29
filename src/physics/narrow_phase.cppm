module;

#include <tracy/Tracy.hpp>

export module javelin.physics.narrow_phase;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.types;
import javelin.scene.shapes;

namespace javelin::detail {
inline constexpr Vec3 kGroundNormal{0.0f, 1.0f, 0.0f};
inline constexpr f32 kGroundOffset = 0.0f;
inline constexpr f32 kMinDistanceEpsSq = 1e-6f;

void add_sphere_sphere_contacts(std::span<const Vec3> position, std::span<const SphereShape> sphere,
                                std::span<const f32> inv_mass, std::span<const BodyPair> pairs,
                                std::vector<Contact> &contacts) {
    for (const BodyPair pair : pairs) {
        const u32 a = pair.a;
        const u32 b = pair.b;
        if (inv_mass[a] == 0.0f && inv_mass[b] == 0.0f) {
            continue;
        }

        const Vec3 delta = position[b] - position[a];
        const f32 dist2 = delta.length_sq();
        const f32 radius_sum = sphere[a].radius + sphere[b].radius;
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
        contacts.push_back(Contact{.a = a, .b = b, .normal = normal, .penetration = penetration});
    }
}

void add_sphere_ground_contacts(std::span<const Vec3> position, std::span<const SphereShape> sphere,
                                std::span<const f32> inv_mass, std::vector<Contact> &contacts) {
    const u32 count = static_cast<u32>(position.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        const f32 signed_distance = dot(kGroundNormal, position[i]) - kGroundOffset;
        if (signed_distance >= sphere[i].radius) {
            continue;
        }
        const f32 penetration = sphere[i].radius - signed_distance;
        contacts.push_back(Contact{.a = i, .b = kInvalidBody, .normal = -kGroundNormal, .penetration = penetration});
    }
}
} // namespace javelin::detail

export namespace javelin {

void narrow_phase_contacts(std::span<const Vec3> position, std::span<const SphereShape> sphere,
                           std::span<const f32> inv_mass, std::span<const BodyPair> pairs,
                           std::vector<Contact> &contacts) {
    ZoneScopedN("Physics narrow phase");
    contacts.clear();
    contacts.reserve(pairs.size() + position.size());
    detail::add_sphere_sphere_contacts(position, sphere, inv_mass, pairs, contacts);
    detail::add_sphere_ground_contacts(position, sphere, inv_mass, contacts);
}

} // namespace javelin
