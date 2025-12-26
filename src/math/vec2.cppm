export module core.math.vec2;

import std;

export namespace javelin::math {

struct Vec2 final {
    float x{};
    float y{};

    constexpr Vec2() noexcept = default;
    constexpr Vec2(const float x_, const float y_) noexcept : x{x_}, y{y_} {}
    explicit constexpr Vec2(float s) noexcept : x{s}, y{s} {}

    [[nodiscard]] constexpr float* data() noexcept { return &x; }
    [[nodiscard]] constexpr const float* data() const noexcept { return &x; }

    constexpr Vec2& operator+=(const Vec2 rhs) noexcept { x += rhs.x; y += rhs.y; return *this; }
    constexpr Vec2& operator-=(const Vec2 rhs) noexcept { x -= rhs.x; y -= rhs.y; return *this; }
    constexpr Vec2& operator*=(const float s)  noexcept { x *= s; y *= s; return *this; }
    constexpr Vec2& operator/=(const float s)  noexcept { const float inv = 1.0f / s; x *= inv; y *= inv; return *this; }

    [[nodiscard]] constexpr Vec2 operator-() const noexcept { return Vec2{-x, -y}; }

    [[nodiscard]] constexpr float length_sq() const noexcept { return x * x + y * y; }

    [[nodiscard]] float length() const noexcept { return std::sqrt(length_sq()); }

    bool try_normalize(const float eps = 1e-8f) noexcept {
        const float len2 = length_sq();
        if (const float eps2 = eps * eps; len2 <= eps2) {
            return false;
        }
        const float inv_len = 1.0f / std::sqrt(len2);
        x *= inv_len;
        y *= inv_len;
        return true;
    }

    [[nodiscard]] Vec2 normalized_or_zero(const float eps = 1e-8f) const noexcept {
        Vec2 v = *this;
        if (!v.try_normalize(eps)) {
            return Vec2{};
        }
        return v;
    }

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(x) && std::isfinite(y);
    }

    [[nodiscard]] static constexpr Vec2 zero()   noexcept { return Vec2{0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec2 one()    noexcept { return Vec2{1.0f, 1.0f}; }
    [[nodiscard]] static constexpr Vec2 unit_x() noexcept { return Vec2{1.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec2 unit_y() noexcept { return Vec2{0.0f, 1.0f}; }
};


[[nodiscard]] constexpr Vec2 operator+(Vec2 a, const Vec2 b) noexcept { return a += b; }
[[nodiscard]] constexpr Vec2 operator-(Vec2 a, const Vec2 b) noexcept { return a -= b; }

[[nodiscard]] constexpr Vec2 operator*(Vec2 v, const float s) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec2 operator*(const float s, Vec2 v) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec2 operator/(Vec2 v, const float s) noexcept { return v /= s; }

[[nodiscard]] constexpr float dot(const Vec2 a, const Vec2 b) noexcept {
    return a.x * b.x + a.y * b.y;
}

[[nodiscard]] constexpr float cross(const Vec2 a, const Vec2 b) noexcept {
    return a.x * b.y - a.y * b.x;
}

[[nodiscard]] constexpr Vec2 hadamard(const Vec2 a, const Vec2 b) noexcept {
    return Vec2{a.x * b.x, a.y * b.y};
}

[[nodiscard]] constexpr float distance_sq(const Vec2 a, const Vec2 b) noexcept {
    const Vec2 d = a - b;
    return d.length_sq();
}

[[nodiscard]] float distance(const Vec2 a, const Vec2 b) noexcept {
    return std::sqrt(distance_sq(a, b));
}

[[nodiscard]] constexpr Vec2 lerp(const Vec2 a, const Vec2 b, const float t) noexcept {
    return a + (b - a) * t;
}

[[nodiscard]] constexpr Vec2 min(const Vec2 a, const Vec2 b) noexcept {
    return Vec2{ std::min(a.x, b.x), std::min(a.y, b.y) };
}
[[nodiscard]] constexpr Vec2 max(const Vec2 a, const Vec2 b) noexcept {
    return Vec2{ std::max(a.x, b.x), std::max(a.y, b.y) };
}
[[nodiscard]] constexpr Vec2 clamp(const Vec2 v, const Vec2 lo, const Vec2 hi) noexcept {
    return Vec2{
        std::clamp(v.x, lo.x, hi.x),
        std::clamp(v.y, lo.y, hi.y),
    };
}

[[nodiscard]] inline bool approx_equal(const Vec2 a, const Vec2 b, float eps = 1e-5f) noexcept {
    auto close = [eps](const float u, const float v) noexcept {
        const float diff = std::fabs(u - v);
        const float scale = std::max(1.0f, std::max(std::fabs(u), std::fabs(v)));
        return diff <= eps * scale;
    };
    return close(a.x, b.x) && close(a.y, b.y);
}

} // namespace javelin::math
