module;

export module javelin.physics.types;

import std;
import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {

struct BodyPair final {
    u32 a{};
    u32 b{};
};

struct Contact final {
    u32 a{};
    u32 b{};
    // Normal points from A to B.
    Vec3 normal{};
    f32 penetration{};
    // Contact offsets in world space from each body's center.
    Vec3 r_a{};
    Vec3 r_b{};
};

inline constexpr u32 kInvalidBody = std::numeric_limits<u32>::max();

} // namespace javelin
