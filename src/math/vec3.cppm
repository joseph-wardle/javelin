export module javelin.math.vec3;

import std;

import javelin.core.types;

export namespace javelin {

struct Vec3 final {
    f32 x{};
    f32 y{};
    f32 z{};

    constexpr Vec3() noexcept = default;
    constexpr Vec3(const f32 x_, const f32 y_, const f32 z_) noexcept : x{x_}, y{y_}, z{z_} {}
    explicit constexpr Vec3(const f32 s) noexcept : x{s}, y{s}, z{s} {}

    [[nodiscard]] constexpr f32 *data() noexcept { return &x; }
    [[nodiscard]] constexpr const f32 *data() const noexcept { return &x; }

    [[nodiscard]] constexpr f32 &operator[](const usize i) noexcept { return data()[i]; }
    [[nodiscard]] constexpr f32 operator[](const usize i) const noexcept { return data()[i]; }

    constexpr Vec3 &operator+=(const Vec3 rhs) noexcept {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    constexpr Vec3 &operator-=(const Vec3 rhs) noexcept {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    constexpr Vec3 &operator*=(const f32 s) noexcept {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    constexpr Vec3 &operator/=(const f32 s) noexcept {
        const f32 inv = 1.0f / s;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    [[nodiscard]] constexpr Vec3 operator-() const noexcept { return Vec3{-x, -y, -z}; }

    [[nodiscard]] constexpr f32 length_sq() const noexcept { return x * x + y * y + z * z; }

    [[nodiscard]] f32 length() const noexcept { return std::sqrt(length_sq()); }

    bool try_normalize(const f32 eps = 1e-8f) noexcept {
        const f32 len2 = length_sq();
        if (const f32 eps2 = eps * eps; len2 <= eps2) {
            return false;
        }
        const f32 inv_len = 1.0f / std::sqrt(len2);
        x *= inv_len;
        y *= inv_len;
        z *= inv_len;
        return true;
    }

    [[nodiscard]] Vec3 normalized_or_zero(const f32 eps = 1e-8f) const noexcept {
        Vec3 v = *this;
        if (!v.try_normalize(eps)) {
            return Vec3{};
        }
        return v;
    }

    [[nodiscard]] bool is_finite() const noexcept { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }

    [[nodiscard]] static constexpr Vec3 zero() noexcept { return Vec3{0.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec3 one() noexcept { return Vec3{1.0f, 1.0f, 1.0f}; }
    [[nodiscard]] static constexpr Vec3 unit_x() noexcept { return Vec3{1.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec3 unit_y() noexcept { return Vec3{0.0f, 1.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec3 unit_z() noexcept { return Vec3{0.0f, 0.0f, 1.0f}; }
};

[[nodiscard]] constexpr Vec3 operator+(Vec3 a, const Vec3 b) noexcept { return a += b; }
[[nodiscard]] constexpr Vec3 operator-(Vec3 a, const Vec3 b) noexcept { return a -= b; }

[[nodiscard]] constexpr Vec3 operator*(Vec3 v, const f32 s) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec3 operator*(const f32 s, Vec3 v) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec3 operator/(Vec3 v, const f32 s) noexcept { return v /= s; }

[[nodiscard]] constexpr f32 dot(const Vec3 a, const Vec3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }

[[nodiscard]] constexpr Vec3 cross(const Vec3 a, const Vec3 b) noexcept {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

[[nodiscard]] constexpr Vec3 hadamard(const Vec3 a, const Vec3 b) noexcept {
    return Vec3{a.x * b.x, a.y * b.y, a.z * b.z};
}

[[nodiscard]] constexpr f32 distance_sq(const Vec3 a, const Vec3 b) noexcept {
    const Vec3 d = a - b;
    return d.length_sq();
}

[[nodiscard]] f32 distance(const Vec3 a, const Vec3 b) noexcept { return std::sqrt(distance_sq(a, b)); }

[[nodiscard]] constexpr Vec3 lerp(const Vec3 a, const Vec3 b, const f32 t) noexcept { return a + (b - a) * t; }

[[nodiscard]] constexpr Vec3 min(const Vec3 a, const Vec3 b) noexcept {
    return Vec3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
[[nodiscard]] constexpr Vec3 max(const Vec3 a, const Vec3 b) noexcept {
    return Vec3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
[[nodiscard]] constexpr Vec3 clamp(const Vec3 v, const Vec3 lo, const Vec3 hi) noexcept {
    return Vec3{
        std::clamp(v.x, lo.x, hi.x),
        std::clamp(v.y, lo.y, hi.y),
        std::clamp(v.z, lo.z, hi.z),
    };
}

[[nodiscard]] inline bool approx_equal(const Vec3 a, const Vec3 b, const f32 eps = 1e-5f) noexcept {
    auto close = [eps](const f32 u, const f32 v) noexcept {
        const f32 diff = std::fabs(u - v);
        const f32 scale = std::max(1.0f, std::max(std::fabs(u), std::fabs(v)));
        return diff <= eps * scale;
    };
    return close(a.x, b.x) && close(a.y, b.y) && close(a.z, b.z);
}

} // namespace javelin
