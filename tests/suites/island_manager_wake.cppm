export module javelin.tests.island_manager_wake;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.body_graph;
import javelin.physics.constraint_types;
import javelin.physics.types;
import javelin.tests.assert;

namespace javelin::tests {

namespace {
[[nodiscard]] ContactManifold make_contact(const u32 a, const u32 b) noexcept {
    ContactManifold manifold{.a = a, .b = b, .point_count = 1u};
    manifold.points[0].feature_id = 0u;
    return manifold;
}
} // namespace

export void island_manager_chain_and_wake() {
    constexpr u32 kBodyCount = 4u;
    BodyGraph islands;
    islands.reserve(kBodyCount);
    islands.clear(kBodyCount);

    const std::array<f32, kBodyCount> inv_mass{1.0f, 1.0f, 1.0f, 1.0f};
    const std::span<const DistanceConstraint> no_constraints{};

    // Chain manifolds (0-1) + (1-2) must collapse bodies 0/1/2 into a single 3-body island.
    const std::array<ContactManifold, 2> chain{make_contact(0u, 1u), make_contact(1u, 2u)};
    const std::span<const ContactManifold> chain_span{chain};
    const std::array<u32, 3> dynamic_chain_ids{0u, 1u, 2u};
    islands.build_activity_masks(kBodyCount, chain_span, no_constraints, inv_mass);
    const u32 max_island_size =
        islands.build_dynamic_islands(kBodyCount, inv_mass, chain_span, no_constraints, dynamic_chain_ids);
    require(max_island_size == 3u, "island_manager.chain.expected_island_size_3");

    const std::span<const u8> contact_mask = islands.contact_activity_mask(kBodyCount);
    require(contact_mask[0] == 1u, "island_manager.chain.body0_contact_active");
    require(contact_mask[1] == 1u, "island_manager.chain.body1_contact_active");
    require(contact_mask[2] == 1u, "island_manager.chain.body2_contact_active");
    require(contact_mask[3] == 0u, "island_manager.chain.body3_no_contact");

    // Now isolate bodies 1, 2 as a separate ready-to-sleep island so we can put them
    // to sleep without dragging body 0 along.  Body 0 stays awake for the wake test.
    const std::array<ContactManifold, 1> pair_12{make_contact(1u, 2u)};
    const std::span<const ContactManifold> pair_12_span{pair_12};
    const std::array<u32, 2> dynamic_pair_ids{1u, 2u};
    islands.build_activity_masks(kBodyCount, pair_12_span, no_constraints, inv_mass);
    static_cast<void>(
        islands.build_dynamic_islands(kBodyCount, inv_mass, pair_12_span, no_constraints, dynamic_pair_ids));

    std::array<u32, kBodyCount> sleep_timer{0u, 60u, 60u, 0u};
    std::array<u8, kBodyCount> asleep{0u, 0u, 0u, 0u};
    const auto sleep_stats = islands.put_settled_to_sleep(sleep_timer, asleep);
    require(sleep_stats.slept_island_count == 1u, "island_manager.sleep.expected_one_island_slept");
    require(sleep_stats.slept_body_count == 2u, "island_manager.sleep.expected_two_bodies_slept");
    require(asleep[0] == 0u, "island_manager.sleep.body0_still_awake");
    require(asleep[1] == 1u, "island_manager.sleep.body1_asleep");
    require(asleep[2] == 1u, "island_manager.sleep.body2_asleep");
    require(asleep[3] == 0u, "island_manager.sleep.body3_unchanged");

    // Active edge between awake body 0 and sleeping body 1 must wake the entire {1,2} sleep island.
    const std::array<ContactManifold, 1> pair_01{make_contact(0u, 1u)};
    const std::span<const ContactManifold> pair_01_span{pair_01};
    const auto wake_stats = islands.wake_with_active_edges(pair_01_span, no_constraints, inv_mass,
                                                           std::span<u8>{asleep},
                                                           std::span<u32>{sleep_timer});
    require(wake_stats.woken_island_count == 1u, "island_manager.wake.expected_one_island_woken");
    require(wake_stats.woken_body_count == 2u, "island_manager.wake.expected_two_bodies_woken");
    require(asleep[1] == 0u, "island_manager.wake.body1_awake");
    require(asleep[2] == 0u, "island_manager.wake.body2_awake");
    require(sleep_timer[1] == 0u, "island_manager.wake.body1_timer_reset");
    require(sleep_timer[2] == 0u, "island_manager.wake.body2_timer_reset");
}

} // namespace javelin::tests
