import std;

import javelin.tests.box_box_sat;
import javelin.tests.island_manager_wake;
import javelin.tests.narrow_phase_stage_persistence;
import javelin.tests.resting_contact_clamp;
import javelin.tests.runner;
import javelin.tests.solve_contact_velocities;
import javelin.tests.sphere_sphere_contact;

using namespace javelin::tests;

int main(int argc, char **argv) {
    constexpr std::array<TestCase, 6> kTestCases{
        TestCase{.name = "sphere_sphere_contact", .fn = sphere_sphere_contact},
        TestCase{.name = "box_box_sat", .fn = box_box_sat},
        TestCase{.name = "solve_contact_velocities", .fn = solve_contact_velocities_known_answers},
        TestCase{.name = "resting_contact_clamp", .fn = resting_contact_clamp_hysteresis},
        TestCase{.name = "narrow_phase_stage_persistence", .fn = narrow_phase_stage_persistence},
        TestCase{.name = "island_manager_chain_and_wake", .fn = island_manager_chain_and_wake},
    };

    return run_cli(kTestCases, argc, argv);
}
