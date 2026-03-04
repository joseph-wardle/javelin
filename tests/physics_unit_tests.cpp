import std;

import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.physics.narrow_phase;
import javelin.physics.solve;
import javelin.physics.types;
import javelin.scene.shapes;

namespace {
using namespace javelin;

struct TestFailure final : std::runtime_error {
    using std::runtime_error::runtime_error;
};

[[nodiscard]] bool nearly_equal(const f32 lhs, const f32 rhs, const f32 eps = 1e-4f) noexcept {
    return std::fabs(lhs - rhs) <= eps;
}

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw TestFailure{std::string{message}};
    }
}

void require_near(const f32 actual, const f32 expected, const f32 eps, const std::string_view label) {
    if (!nearly_equal(actual, expected, eps)) {
        throw TestFailure{std::format("{} mismatch: expected {:.6f}, got {:.6f}", label, expected, actual)};
    }
}

void sphere_sphere_contact() {
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

    std::vector<ContactManifold> manifolds{};
    const std::vector<ContactManifold> previous_manifolds{};

    narrow_phase_contacts(position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs, previous_manifolds,
                          manifolds);

    require(manifolds.size() == 1u, "sphere_sphere_contact: expected exactly one manifold");
    const ContactManifold &m = manifolds[0];
    require(m.a == 0u && m.b == 1u, "sphere_sphere_contact: expected canonical body order (0,1)");
    require(m.point_count == 1u, "sphere_sphere_contact: expected one contact point");
    require_near(m.normal.x, 1.0f, 1e-4f, "sphere_sphere_contact.normal.x");
    require_near(m.normal.y, 0.0f, 1e-4f, "sphere_sphere_contact.normal.y");
    require_near(m.normal.z, 0.0f, 1e-4f, "sphere_sphere_contact.normal.z");
    require_near(m.points[0].separation, -0.5f, 1e-4f, "sphere_sphere_contact.separation");
    require_near(m.points[0].local_anchor_a.x, 1.0f, 1e-4f, "sphere_sphere_contact.anchor_a.x");
    require_near(m.points[0].local_anchor_b.x, -1.0f, 1e-4f, "sphere_sphere_contact.anchor_b.x");
}

void box_box_sat() {
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
    const std::vector<ContactManifold> previous_manifolds{};

    std::vector<ContactManifold> manifolds{};
    {
        const std::array<Vec3, 2> overlap_position{
            Vec3{0.0f, 3.0f, 0.0f},
            Vec3{1.5f, 3.0f, 0.0f},
        };

        narrow_phase_contacts(overlap_position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs,
                              previous_manifolds, manifolds);

        require(manifolds.size() == 1u, "box_box_sat: expected one manifold for overlapping boxes");
        const ContactManifold &m = manifolds[0];
        require(m.a == 0u && m.b == 1u, "box_box_sat: expected canonical body order (0,1)");
        require(m.point_count >= 2u && m.point_count <= kMaxManifoldPoints,
                "box_box_sat: expected face manifold with 2-4 contact points");
        require(m.normal.x > 0.99f, "box_box_sat: expected SAT normal to point along +X");
        require(std::fabs(m.normal.y) <= 1e-4f && std::fabs(m.normal.z) <= 1e-4f,
                "box_box_sat: expected SAT normal to remain axis-aligned");
        for (u32 i = 0u; i < m.point_count; ++i) {
            require(m.points[i].separation <= -0.49f,
                    "box_box_sat: expected negative separation close to overlap depth (0.5m)");
        }
    }

    {
        const std::array<Vec3, 2> separated_position{
            Vec3{0.0f, 3.0f, 0.0f},
            Vec3{2.2f, 3.0f, 0.0f},
        };
        narrow_phase_contacts(separated_position, orientation, shape_kind, shapes, shape_index, inv_mass, pairs,
                              previous_manifolds, manifolds);
        require(manifolds.empty(), "box_box_sat: expected no manifold when boxes are separated");
    }
}

void solve_contact_velocities_known_answers() {
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

using TestFn = void (*)();
struct TestCase final {
    std::string_view name;
    TestFn fn;
};

[[nodiscard]] constexpr std::array<TestCase, 3> tests() noexcept {
    return std::array<TestCase, 3>{
        TestCase{.name = "sphere_sphere_contact", .fn = sphere_sphere_contact},
        TestCase{.name = "box_box_sat", .fn = box_box_sat},
        TestCase{.name = "solve_contact_velocities", .fn = solve_contact_velocities_known_answers},
    };
}

[[nodiscard]] bool run_one(const TestCase &test_case) {
    try {
        test_case.fn();
        std::cout << "[PASS] " << test_case.name << '\n';
        return true;
    } catch (const TestFailure &failure) {
        std::cerr << "[FAIL] " << test_case.name << ": " << failure.what() << '\n';
        return false;
    } catch (const std::exception &exception) {
        std::cerr << "[FAIL] " << test_case.name << ": unexpected exception: " << exception.what() << '\n';
        return false;
    } catch (...) {
        std::cerr << "[FAIL] " << test_case.name << ": unknown exception\n";
        return false;
    }
}

} // namespace

int main(int argc, char **argv) {
    const auto all_tests = tests();

    if (argc == 2) {
        const std::string_view requested{argv[1]};
        const auto it = std::find_if(all_tests.begin(), all_tests.end(),
                                     [&](const TestCase &test_case) { return test_case.name == requested; });
        if (it == all_tests.end()) {
            std::cerr << "Unknown test: " << requested << '\n';
            std::cerr << "Available tests:\n";
            for (const TestCase &test_case : all_tests) {
                std::cerr << "  - " << test_case.name << '\n';
            }
            return 2;
        }
        return run_one(*it) ? 0 : 1;
    }

    bool ok = true;
    for (const TestCase &test_case : all_tests) {
        ok = run_one(test_case) && ok;
    }
    return ok ? 0 : 1;
}
