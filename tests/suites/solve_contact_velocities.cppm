export module javelin.tests.solve_contact_velocities;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.solve;
import javelin.physics.types;
import javelin.tests.assert;

namespace javelin::tests {

export void solve_contact_velocities_known_answers() {
    auto run_case = [](const f32 restitution, const f32 expected_a_x, const f32 expected_b_x,
                       const f32 expected_impulse, const std::string_view label) {
        std::array<Vec3, 2> velocity{
            Vec3{1.0f, 0.0f, 0.0f},
            Vec3{-1.0f, 0.0f, 0.0f},
        };
        std::array<Vec3, 2> angular_velocity{
            Vec3{},
            Vec3{},
        };
        const std::array<f32, 2> inv_mass{1.0f, 1.0f};
        const std::array<Vec3, 2> inv_inertia_body{
            Vec3{},
            Vec3{},
        };
        const std::array<Quat, 2> orientation{
            Quat::identity(),
            Quat::identity(),
        };
        std::array<ContactManifold, 1> manifolds{
            ContactManifold{
                .a = 0u,
                .b = 1u,
                .normal = Vec3{1.0f, 0.0f, 0.0f},
                .point_count = 1u,
            },
        };
        manifolds[0].points[0] = ContactPoint{
            .local_anchor_a = Vec3{},
            .local_anchor_b = Vec3{},
            .separation = 0.0f,
            .normal_impulse = 0.0f,
            .tangent_impulse = Vec3{},
            .persisted = false,
            .feature_id = 0u,
        };

        const std::array<f32, 1> manifold_restitution{restitution};
        const std::array<f32, 1> manifold_friction{0.0f};
        const std::array<u8, 2> asleep{0u, 0u};

        solve_contact_velocities(velocity, angular_velocity, inv_mass, inv_inertia_body, orientation, manifolds,
                                 1.0f / 60.0f, manifold_restitution, manifold_friction, asleep);

        require_near(velocity[0].x, expected_a_x, 1e-4f, std::format("{}.velocity_a_x", label));
        require_near(velocity[1].x, expected_b_x, 1e-4f, std::format("{}.velocity_b_x", label));
        require_near(manifolds[0].points[0].normal_impulse, expected_impulse, 1e-4f,
                     std::format("{}.normal_impulse", label));
    };

    run_case(0.0f, 0.0f, 0.0f, 1.0f, "solve_contact_velocities.inelastic_equal_mass");
    run_case(1.0f, -1.0f, 1.0f, 2.0f, "solve_contact_velocities.elastic_equal_mass");
}

} // namespace javelin::tests
