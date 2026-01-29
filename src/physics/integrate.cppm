module;

#include <tracy/Tracy.hpp>

export module javelin.physics.integrate;

import std;
import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {

void accumulate_forces(std::span<Vec3> velocity, std::span<const f32> inv_mass, const f32 gravity,
                        const f32 dt) noexcept {
    ZoneScopedN("Physics accumulate forces");
    const u32 count = static_cast<u32>(velocity.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        velocity[i].y += gravity * dt;
    }
}

void integrate_predicted_positions(std::span<Vec3> position, std::span<const Vec3> velocity,
                                   std::span<const f32> inv_mass, const f32 dt) noexcept {
    ZoneScopedN("Physics integrate positions");
    const u32 count = static_cast<u32>(position.size());
    for (u32 i = 0; i < count; ++i) {
        if (inv_mass[i] == 0.0f) {
            continue;
        }
        position[i] += velocity[i] * dt;
    }
}

} // namespace javelin
