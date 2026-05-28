export module javelin.tests.box_box_sat;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.narrow_phase;
import javelin.physics.types;
import javelin.scene.shapes;
import javelin.tests.assert;

namespace javelin::tests {

export void box_box_sat() {
    const std::array<ShapeData, 2> shapes{
        ShapeData::make_box(BoxShape{.half_extents = Vec3{1.0f, 1.0f, 1.0f}}),
        ShapeData::make_box(BoxShape{.half_extents = Vec3{1.0f, 1.0f, 1.0f}}),
    };
    const std::array<ShapeKind, 2> shape_kind{
        ShapeKind::box,
        ShapeKind::box,
    };
    const std::array<u32, 2> shape_index{0u, 1u};
    const std::array<Quat, 2> orientation{
        Quat::identity(),
        Quat::identity(),
    };
    const std::array<f32, 2> inv_mass{1.0f, 1.0f};
    const std::array<BodyPair, 1> pairs{
        BodyPair{.a = 0u, .b = 1u},
    };
    const std::array<u32, 2> awake_dynamic_ids{0u, 1u};
    const std::vector<ContactManifold> previous_manifolds{};

    std::vector<ContactManifold> manifolds{};
    {
        const std::array<Vec3, 2> overlap_position{
            Vec3{0.0f, 3.0f, 0.0f},
            Vec3{1.5f, 3.0f, 0.0f},
        };

        narrow_phase_contacts(overlap_position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs,
                              previous_manifolds, awake_dynamic_ids, manifolds);

        require(manifolds.size() == 1u, "box_box_sat: expected one manifold for overlapping boxes");
        const ContactManifold &manifold = manifolds[0];
        require(manifold.a == 0u && manifold.b == 1u, "box_box_sat: expected canonical body order (0,1)");
        require(manifold.point_count >= 2u && manifold.point_count <= kMaxManifoldPoints,
                "box_box_sat: expected face manifold with 2-4 contact points");
        require(manifold.normal.x > 0.99f, "box_box_sat: expected SAT normal to point along +X");
        require(std::fabs(manifold.normal.y) <= 1e-4f && std::fabs(manifold.normal.z) <= 1e-4f,
                "box_box_sat: expected SAT normal to remain axis-aligned");
        for (u32 i = 0u; i < manifold.point_count; ++i) {
            require(manifold.points[i].separation <= -0.49f,
                    "box_box_sat: expected negative separation close to overlap "
                    "depth (0.5m)");
        }
    }

    {
        const std::array<Vec3, 2> separated_position{
            Vec3{0.0f, 3.0f, 0.0f},
            Vec3{2.2f, 3.0f, 0.0f},
        };
        narrow_phase_contacts(separated_position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs,
                              previous_manifolds, awake_dynamic_ids, manifolds);
        require(manifolds.empty(), "box_box_sat: expected no manifold when boxes are separated");
    }
}

} // namespace javelin::tests
