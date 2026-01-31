module;

export module javelin.physics.bounds;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;

export namespace javelin {

[[nodiscard]] constexpr Aabb bounds_sphere(const Vec3 position, const f32 radius) noexcept {
    const Vec3 r{radius};
    return Aabb{position - r, position + r};
}

[[nodiscard]] constexpr Aabb inflate(const Aabb a, const f32 margin) noexcept { return expand(a, Vec3{margin}); }

[[nodiscard]] constexpr Aabb sweep(const Aabb a, const Vec3 velocity, const f32 dt) noexcept {
    const Vec3 delta = velocity * dt;
    const Aabb moved{a.min + delta, a.max + delta};
    return merge(a, moved);
}

} // namespace javelin
