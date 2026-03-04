export module javelin.tests.sphere_sphere_contact;

import std;

import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.narrow_phase;
import javelin.physics.types;
import javelin.scene.shapes;
import javelin.tests.assert;

namespace javelin::tests {

export void sphere_sphere_contact() {
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

    std::vector<ContactManifold> manifolds{};
    const std::vector<ContactManifold> previous_manifolds{};

    narrow_phase_contacts(position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs, previous_manifolds,
                          awake_dynamic_ids, manifolds);

    require(manifolds.size() == 1u, "sphere_sphere_contact: expected exactly one manifold");
    const ContactManifold &manifold = manifolds[0];
    require(manifold.a == 0u && manifold.b == 1u, "sphere_sphere_contact: expected canonical body order (0,1)");
    require(manifold.point_count == 1u, "sphere_sphere_contact: expected one contact point");
    require_near(manifold.normal.x, 1.0f, 1e-4f, "sphere_sphere_contact.normal.x");
    require_near(manifold.normal.y, 0.0f, 1e-4f, "sphere_sphere_contact.normal.y");
    require_near(manifold.normal.z, 0.0f, 1e-4f, "sphere_sphere_contact.normal.z");
    require_near(manifold.points[0].separation, -0.5f, 1e-4f, "sphere_sphere_contact.separation");
    require_near(manifold.points[0].local_anchor_a.x, 1.0f, 1e-4f, "sphere_sphere_contact.anchor_a.x");
    require_near(manifold.points[0].local_anchor_b.x, -1.0f, 1e-4f, "sphere_sphere_contact.anchor_b.x");
}

} // namespace javelin::tests
