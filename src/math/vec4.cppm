export module core.math.vec4;

import std;

export namespace javelin::math {

struct Vec4 final {
    float x{};
    float y{};
    float z{};
    float w{};

    constexpr Vec4() noexcept = default;
    constexpr Vec4(const float x_, const float y_, const float z_, const float w_) noexcept : x{x_}, y{y_}, z{z_}, w{w_} {}

    explicit constexpr Vec4(const float s) noexcept : x{s}, y{s}, z{s}, w{s} {}

    [[nodiscard]] constexpr float* data() noexcept { return &x; }
    [[nodiscard]] constexpr const float* data() const noexcept { return &x; }

    constexpr Vec4& operator+=(const Vec4 rhs) noexcept { x += rhs.x; y += rhs.y; z += rhs.z; w += rhs.w; return *this; }
    constexpr Vec4& operator-=(const Vec4 rhs) noexcept { x -= rhs.x; y -= rhs.y; z -= rhs.z; w -= rhs.w; return *this; }
    constexpr Vec4& operator*=(const float s) noexcept  { x *= s; y *= s; z *= s; w *= s; return *this; }
    constexpr Vec4& operator/=(const float s) noexcept  { const float inv = 1.0f / s; x *= inv; y *= inv; z *= inv; w *= inv; return *this; }

    [[nodiscard]] constexpr Vec4 operator-() const noexcept { return Vec4{-x, -y, -z, -w}; }

    [[nodiscard]] constexpr float length_sq() const noexcept { return x * x + y * y + z * z + w * w; }

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
        w *= inv_len;
        return true;
    }

    [[nodiscard]] Vec4 normalized_or_zero(const float eps = 1e-8f) const noexcept {
        Vec4 v = *this;
        if (!v.try_normalize(eps)) {
            return Vec4{};
        }
        return v;
    }

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
    }

    [[nodiscard]] static constexpr Vec4 zero()   noexcept { return Vec4{0.0f, 0.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec4 one()    noexcept { return Vec4{1.0f, 1.0f, 1.0f, 1.0f}; }
    [[nodiscard]] static constexpr Vec4 unit_x() noexcept { return Vec4{1.0f, 0.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec4 unit_y() noexcept { return Vec4{0.0f, 1.0f, 0.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec4 unit_z() noexcept { return Vec4{0.0f, 0.0f, 1.0f, 0.0f}; }
    [[nodiscard]] static constexpr Vec4 unit_w() noexcept { return Vec4{0.0f, 0.0f, 0.0f, 1.0f}; }
};

[[nodiscard]] constexpr Vec4 operator+(Vec4 a, const Vec4 b) noexcept { return a += b; }
[[nodiscard]] constexpr Vec4 operator-(Vec4 a, const Vec4 b) noexcept { return a -= b; }

[[nodiscard]] constexpr Vec4 operator*(Vec4 v, const float s) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec4 operator*(const float s, Vec4 v) noexcept { return v *= s; }
[[nodiscard]] constexpr Vec4 operator/(Vec4 v, const float s) noexcept { return v /= s; }

[[nodiscard]] constexpr float dot(const Vec4 a, const Vec4 b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

[[nodiscard]] constexpr Vec4 hadamard(const Vec4 a, const Vec4 b) noexcept {
    return Vec4{a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
}

[[nodiscard]] constexpr float distance_sq(const Vec4 a, const Vec4 b) noexcept {
    const Vec4 d = a - b;
    return d.length_sq();
}

[[nodiscard]] float distance(const Vec4 a, const Vec4 b) noexcept {
    return std::sqrt(distance_sq(a, b));
}

[[nodiscard]] constexpr Vec4 lerp(const Vec4 a, const Vec4 b, const float t) noexcept {
    return a + (b - a) * t;
}

[[nodiscard]] constexpr Vec4 min(const Vec4 a, const Vec4 b) noexcept {
    return Vec4{
        std::min(a.x, b.x),
        std::min(a.y, b.y),
        std::min(a.z, b.z),
        std::min(a.w, b.w),
    };
}
[[nodiscard]] constexpr Vec4 max(const Vec4 a, const Vec4 b) noexcept {
    return Vec4{
        std::max(a.x, b.x),
        std::max(a.y, b.y),
        std::max(a.z, b.z),
        std::max(a.w, b.w),
    };
}
[[nodiscard]] constexpr Vec4 clamp(const Vec4 v, const Vec4 lo, const Vec4 hi) noexcept {
    return Vec4{
        std::clamp(v.x, lo.x, hi.x),
        std::clamp(v.y, lo.y, hi.y),
        std::clamp(v.z, lo.z, hi.z),
        std::clamp(v.w, lo.w, hi.w),
    };
}

[[nodiscard]] inline bool approx_equal(const Vec4 a, const Vec4 b, const float eps = 1e-5f) noexcept {
    auto close = [eps](const float u, const float v) noexcept {
        const float diff = std::fabs(u - v);
        const float scale = std::max(1.0f, std::max(std::fabs(u), std::fabs(v)));
        return diff <= eps * scale;
    };
    return close(a.x, b.x) && close(a.y, b.y) && close(a.z, b.z) && close(a.w, b.w);
}

} // namespace javelin::math
