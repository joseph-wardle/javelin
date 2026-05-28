export module javelin.tests.narrow_phase_stage_persistence;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.narrow_phase;
import javelin.physics.types;
import javelin.scene.shapes;
import javelin.tests.assert;

namespace javelin::tests {

export void narrow_phase_stage_persistence() {
    const std::array<Vec3, 2> position{
        Vec3{0.0f, 2.0f, 0.0f},
        Vec3{1.5f, 2.0f, 0.0f},
    };
    const std::array<Quat, 2> orientation{
        Quat::identity(),
        Quat::identity(),
    };
    const std::array<ShapeKind, 2> shape_kind{
        ShapeKind::sphere,
        ShapeKind::sphere,
    };
    const std::array<u32, 2> shape_index{0u, 1u};
    const std::array<ShapeData, 2> shapes{
        ShapeData::make_sphere(SphereShape{.radius = 1.0f}),
        ShapeData::make_sphere(SphereShape{.radius = 1.0f}),
    };
    const std::array<f32, 2> inv_mass{1.0f, 1.0f};
    const std::array<BodyPair, 1> pairs{
        BodyPair{.a = 0u, .b = 1u},
    };
    const std::array<u32, 2> awake_dynamic_ids{0u, 1u};

    NarrowPhaseStage stage;
    stage.reserve(2u);

    stage.prepare();
    const auto stats_1 = stage.run(position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs,
                                   awake_dynamic_ids);
    require(stats_1.manifold_count == 1u, "narrow_phase_stage.frame1.manifold_count");
    require(stats_1.contact_point_count == 1u, "narrow_phase_stage.frame1.contact_point_count");
    require(stats_1.matched_point_count == 0u, "narrow_phase_stage.frame1.matched_point_count");

    {
        const std::span<ContactManifold> manifolds = stage.manifolds();
        require(manifolds.size() == 1u, "narrow_phase_stage.frame1.manifold_span_size");
        require(manifolds[0].point_count == 1u, "narrow_phase_stage.frame1.point_count");
        manifolds[0].points[0].normal_impulse = 0.5f;
        manifolds[0].points[0].tangent_impulse = Vec3{0.1f, 0.0f, 0.0f};
    }

    stage.prepare();
    const auto stats_2 = stage.run(position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs,
                                   awake_dynamic_ids);
    require(stats_2.manifold_count == 1u, "narrow_phase_stage.frame2.manifold_count");
    require(stats_2.matched_point_count == 1u, "narrow_phase_stage.frame2.matched_point_count");
    require(stats_2.dropped_point_count == 0u, "narrow_phase_stage.frame2.dropped_point_count");

    const std::span<const ContactManifold> manifolds_2 = stage.manifolds();
    require(manifolds_2.size() == 1u, "narrow_phase_stage.frame2.manifold_span_size");
    require(manifolds_2[0].point_count == 1u, "narrow_phase_stage.frame2.point_count");
    require(manifolds_2[0].points[0].persisted, "narrow_phase_stage.frame2.point_persisted");
    require_near(manifolds_2[0].points[0].normal_impulse, 0.5f, 1e-4f, "narrow_phase_stage.frame2.warm_start_normal_impulse");
    require_near(manifolds_2[0].points[0].tangent_impulse.x, 0.1f, 1e-4f, "narrow_phase_stage.frame2.warm_start_tangent_impulse_x");
}

} // namespace javelin::tests
