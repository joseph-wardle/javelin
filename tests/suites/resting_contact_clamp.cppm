export module javelin.tests.resting_contact_clamp;

import std;

import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.integrate;
import javelin.tests.assert;

namespace javelin::tests {

export void resting_contact_clamp_hysteresis() {
    std::array<Vec3, 2> velocity{
        Vec3{0.01f, 0.0f, 0.0f},
        Vec3{0.01f, 0.0f, 0.0f},
    };
    std::array<Vec3, 2> angular_velocity{
        Vec3{},
        Vec3{},
    };
    const std::array<f32, 2> inv_mass{
        1.0f,
        1.0f,
    };
    const std::array<u8, 2> in_contact{
        1u,
        0u,
    };
    const std::array<u8, 2> asleep{
        0u,
        0u,
    };
    std::array<u8, 2> rest_ticks{
        0u,
        0u,
    };

    constexpr f32 kLinearThresholdSq = 0.02f * 0.02f;
    constexpr f32 kAngularThresholdSq = 0.04f * 0.04f;
    constexpr u8 kRestTicksToClamp = 3u;

    clamp_resting_contact_velocities(velocity, angular_velocity, inv_mass, in_contact, asleep, rest_ticks,
                                     kLinearThresholdSq, kAngularThresholdSq, kRestTicksToClamp);
    require_near(velocity[0].x, 0.01f, 1e-6f, "resting_contact_clamp.tick1.velocity0");
    require_near(velocity[1].x, 0.01f, 1e-6f, "resting_contact_clamp.tick1.velocity1");
    require(rest_ticks[0] == 1u, "resting_contact_clamp.tick1.counter0");
    require(rest_ticks[1] == 0u, "resting_contact_clamp.tick1.counter1");

    clamp_resting_contact_velocities(velocity, angular_velocity, inv_mass, in_contact, asleep, rest_ticks,
                                     kLinearThresholdSq, kAngularThresholdSq, kRestTicksToClamp);
    require_near(velocity[0].x, 0.01f, 1e-6f, "resting_contact_clamp.tick2.velocity0");
    require(rest_ticks[0] == 2u, "resting_contact_clamp.tick2.counter0");

    clamp_resting_contact_velocities(velocity, angular_velocity, inv_mass, in_contact, asleep, rest_ticks,
                                     kLinearThresholdSq, kAngularThresholdSq, kRestTicksToClamp);
    require_near(velocity[0].x, 0.0f, 1e-6f, "resting_contact_clamp.tick3.velocity0");
    require_near(velocity[1].x, 0.01f, 1e-6f, "resting_contact_clamp.tick3.velocity1");
    require(rest_ticks[0] == 3u, "resting_contact_clamp.tick3.counter0");

    velocity[0].x = 0.03f;
    clamp_resting_contact_velocities(velocity, angular_velocity, inv_mass, in_contact, asleep, rest_ticks,
                                     kLinearThresholdSq, kAngularThresholdSq, kRestTicksToClamp);
    require_near(velocity[0].x, 0.03f, 1e-6f, "resting_contact_clamp.reset.velocity0");
    require(rest_ticks[0] == 0u, "resting_contact_clamp.reset.counter0");
}

} // namespace javelin::tests
