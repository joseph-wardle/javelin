export module javelin.scene.shapes;

import std;

import javelin.core.types;
import javelin.math;

export namespace javelin {
enum struct ShapeKind : u8 { sphere, box /*, capsule...*/ };

struct SphereShape final {
    f32 radius{0.5f};
};

struct BoxShape final {
    Vec3 half_extents{0.5f};
};

struct ShapeData final {
    ShapeKind kind{ShapeKind::sphere};
    // Intentional: not UB — kind is a separate field, not a union member.
    union {
        SphereShape sphere;
        BoxShape box;
    };

    constexpr ShapeData() noexcept : kind{ShapeKind::sphere}, sphere{} {}

    [[nodiscard]] static constexpr ShapeData make_sphere(const SphereShape s) noexcept {
        ShapeData out{};
        out.kind = ShapeKind::sphere;
        out.sphere = s;
        return out;
    }

    [[nodiscard]] static constexpr ShapeData make_box(const BoxShape b) noexcept {
        ShapeData out{};
        out.kind = ShapeKind::box;
        out.box = b;
        return out;
    }
};

[[nodiscard]] inline const SphereShape &shape_sphere(const ShapeData &shape) noexcept { return shape.sphere; }
[[nodiscard]] inline const BoxShape &shape_box(const ShapeData &shape) noexcept { return shape.box; }
} // namespace javelin
