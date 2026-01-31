module;

export module javelin.physics.aabb;

import std;
import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {

struct Aabb final {
    Vec3 min{};
    Vec3 max{};

    constexpr Aabb() noexcept = default;
    constexpr Aabb(const Vec3 min_, const Vec3 max_) noexcept : min{min_}, max{max_} {}

    [[nodiscard]] static constexpr Aabb from_sphere(const Vec3 position, const f32 radius) noexcept {
        const Vec3 r{radius};
        return Aabb{position - r, position + r};
    }

    [[nodiscard]] static constexpr Aabb sweep(const Aabb a, const Vec3 velocity, const f32 dt) noexcept {
        const Vec3 delta = velocity * dt;
        const Aabb moved{a.min + delta, a.max + delta};
        return merge(a, moved);
    }
};

[[nodiscard]] constexpr Vec3 center(const Aabb a) noexcept { return (a.min + a.max) * 0.5f; }
[[nodiscard]] constexpr Vec3 size(const Aabb a) noexcept { return a.max - a.min; }
[[nodiscard]] constexpr Vec3 extents(const Aabb a) noexcept { return size(a) * 0.5f; }

[[nodiscard]] constexpr Aabb merge(const Aabb a, const Aabb b) noexcept {
    return Aabb{min(a.min, b.min), max(a.max, b.max)};
}

[[nodiscard]] constexpr bool overlaps(const Aabb a, const Aabb b) noexcept {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) && (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
           (a.min.z <= b.max.z && a.max.z >= b.min.z);
}

[[nodiscard]] constexpr bool contains_point(const Aabb a, const Vec3 p) noexcept {
    return (p.x >= a.min.x && p.x <= a.max.x) && (p.y >= a.min.y && p.y <= a.max.y) &&
           (p.z >= a.min.z && p.z <= a.max.z);
}

[[nodiscard]] constexpr bool contains_aabb(const Aabb a, const Aabb b) noexcept {
    return (b.min.x >= a.min.x && b.max.x <= a.max.x) && (b.min.y >= a.min.y && b.max.y <= a.max.y) &&
           (b.min.z >= a.min.z && b.max.z <= a.max.z);
}

[[nodiscard]] constexpr Aabb inflate(const Aabb a, const Vec3 padding) noexcept {
    return Aabb{a.min - padding, a.max + padding};
}

[[nodiscard]] constexpr f32 surface_area(const Aabb a) noexcept {
    const Vec3 s = size(a);
    return 2.0f * (s.x * s.y + s.y * s.z + s.z * s.x);
}

} // namespace javelin
