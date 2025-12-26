export module core.math.vec3;

import std;

export namespace javelin::math {

struct Vec3 final {
    float x{};
    float y{};
    float z{};

    constexpr Vec3() noexcept = default;
    constexpr Vec3(const float x_, const float y_, const float z_) noexcept : x{x_}, y{y_}, z{z_} {}
    explicit constexpr Vec3(const float s) noexcept : x{s}, y{s}, z{s} {}

    [[nodiscard]] constexpr float* data() noexcept { return &x; }
    [[nodiscard]] constexpr const float* data() const noexcept { return &x; }

    constexpr Vec3& operator+=(const Vec3 rhs) noexcept { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    constexpr Vec3& operator-=(const Vec3 rhs) noexcept { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    constexpr Vec3& operator*=(const float s) noexcept  { x *= s; y *= s; z *= s; return *this; }
    constexpr Vec3& operator/=(const float s) noexcept  { const float inv = 1.0f / s; x *= inv; y *= inv; z *= inv; return *this; }

    [[nodiscard]] constexpr Vec3 operator-() const noexcept { return Vec3{-x, -y, -z}; }

    [[nodiscard]] constexpr float length_sq() const noexcept { return x * x + y * y + z * z; }

    [[nodiscard]] float length() const noexcept { return std::sqrt(length_sq()); }

    bool try_normalize(const float eps = 1e-8f) noexcept {
        const float len2 = length_sq();
        if (const float eps2 = eps * eps; len2 <= eps2) {
            return false;
        }
        const float inv_len = 1.0f / std::sqrt(len2);
        x *= inv_len;
        y *= inv_len;
        z *= inv_len;
        return true;
    }

    [[nodiscard]] Vec3 normalized_or_zero(const float eps = 1e-8f) const noexcept {
        Vec3 v = *this;
        if (!v.try_normalize(eps)) {
            return Vec3{};
        }
        return v;
    }

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    // Handy constants (kept explicit and discoverable)
    [[nodiscard]] static constexpr Vec3 zero()   noexcept { return Vec3{0.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec3 one()    noexcept { return Vec3{1.0f, 1.0f, 1.0f}; }
    [[nodiscard]] static constexpr Vec3 unit_x() noexcept { return Vec3{1.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec3 unit_y() noexcept { return Vec3{0.0f, 1.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec3 unit_z() noexcept { return Vec3{0.0f, 0.0f, 1.0f}; }
};

[[nodiscard]] constexpr Vec3 operator+(Vec3 a, const Vec3 b) noexcept { return a += b; }
[[nodiscard]] constexpr Vec3 operator-(Vec3 a, const Vec3 b) noexcept { return a -= b; }

[[nodiscard]] constexpr Vec3 operator*(Vec3 v, const float s) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec3 operator*(const float s, Vec3 v) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec3 operator/(Vec3 v, const float s) noexcept { return v /= s; }

// ---- core vector ops ----
[[nodiscard]] constexpr float dot(const Vec3 a, const Vec3 b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

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

[[nodiscard]] constexpr float distance_sq(const Vec3 a, const Vec3 b) noexcept {
    const Vec3 d = a - b;
    return d.length_sq();
}

[[nodiscard]] float distance(const Vec3 a, const Vec3 b) noexcept {
    return std::sqrt(distance_sq(a, b));
}

[[nodiscard]] constexpr Vec3 lerp(const Vec3 a, const Vec3 b, const float t) noexcept {
    return a + (b - a) * t;
}

[[nodiscard]] constexpr Vec3 min(const Vec3 a, const Vec3 b) noexcept {
    return Vec3{ std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) };
}
[[nodiscard]] constexpr Vec3 max(const Vec3 a, const Vec3 b) noexcept {
    return Vec3{ std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) };
}
[[nodiscard]] constexpr Vec3 clamp(const Vec3 v, const Vec3 lo, const Vec3 hi) noexcept {
    return Vec3{
        std::clamp(v.x, lo.x, hi.x),
        std::clamp(v.y, lo.y, hi.y),
        std::clamp(v.z, lo.z, hi.z),
    };
}

// Float comparisons: don’t overload == with “approx”; keep it explicit.
[[nodiscard]] inline bool approx_equal(const Vec3 a, const Vec3 b, const float eps = 1e-5f) noexcept {
    auto close = [eps](const float u, const float v) noexcept {
        const float diff = std::fabs(u - v);
        const float scale = std::max(1.0f, std::max(std::fabs(u), std::fabs(v)));
        return diff <= eps * scale;
    };
    return close(a.x, b.x) && close(a.y, b.y) && close(a.z, b.z);
}

} // namespace javelin::math
