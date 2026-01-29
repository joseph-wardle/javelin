module;

#include <tracy/Tracy.hpp>

export module javelin.physics.broad_phase;

import std;
import javelin.core.types;
import javelin.physics.types;

export namespace javelin {

void broad_phase_sphere_pairs(const u32 count, std::vector<BodyPair> &pairs) {
    ZoneScopedN("Physics broad phase");
    pairs.clear();
    if (count < 2) {
        return;
    }
    const usize pair_count = (static_cast<usize>(count) * static_cast<usize>(count - 1)) / 2;
    pairs.reserve(pair_count);
    for (u32 i = 0; i < count; ++i) {
        for (u32 j = i + 1; j < count; ++j) {
            pairs.push_back(BodyPair{.a = i, .b = j});
        }
    }
}

} // namespace javelin
