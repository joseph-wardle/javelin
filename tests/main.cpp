import std;

import javelin.tests.box_box_sat;
import javelin.tests.runner;
import javelin.tests.solve_contact_velocities;
import javelin.tests.sphere_sphere_contact;

using namespace javelin::tests;

int main(int argc, char **argv) {
    constexpr std::array<TestCase, 3> kTestCases{
        TestCase{.name = "sphere_sphere_contact", .fn = sphere_sphere_contact},
        TestCase{.name = "box_box_sat", .fn = box_box_sat},
        TestCase{.name = "solve_contact_velocities", .fn = solve_contact_velocities_known_answers},
    };

    return run_cli(kTestCases, argc, argv);
}
