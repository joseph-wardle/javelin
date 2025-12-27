export module core.math.quat;

import std;

import javelin.core.types;
export import javelin.math.vec3;
export import javelin.math.mat3;

export namespace javelin::math {

struct Quat final {
    f32 x{};
    f32 y{};
    f32 z{};
    f32 w{1.0f};

    constexpr Quat() noexcept = default;
    constexpr Quat(const f32 x_, const f32 y_, const f32 z_, const f32 w_) noexcept : x{x_}, y{y_}, z{z_}, w{w_} {}

    [[nodiscard]] constexpr f32* data() noexcept { return &x; }
    [[nodiscard]] constexpr const f32* data() const noexcept { return &x; }

    [[nodiscard]] constexpr f32& operator[](const usize i) noexcept { return data()[i]; }
    [[nodiscard]] constexpr f32  operator[](const usize i) const noexcept { return data()[i]; }

    [[nodiscard]] bool is_finite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
    }

    [[nodiscard]] static constexpr Quat identity() noexcept { return Quat{0.0f, 0.0f, 0.0f, 1.0f}; }

    [[nodiscard]] constexpr f32 length_sq() const noexcept { return x*x + y*y + z*z + w*w; }
    [[nodiscard]] f32 length() const noexcept { return std::sqrt(length_sq()); }

    bool try_normalize(const f32 eps = 1e-8f) noexcept {
        const f32 len2 = length_sq();
        if (const f32 eps2 = eps * eps; len2 <= eps2) {
            return false;
        }
        const f32 inv_len = 1.0f / std::sqrt(len2);
        x *= inv_len; y *= inv_len; z *= inv_len; w *= inv_len;
        return true;
    }

    [[nodiscard]] Quat normalized_or_identity(const f32 eps = 1e-8f) const noexcept {
        Quat q = *this;
        if (!q.try_normalize(eps)) {
            return identity();
        }
        return q;
    }
};

[[nodiscard]] constexpr Quat operator+(const Quat a, const Quat b) noexcept { return Quat{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w}; }
[[nodiscard]] constexpr Quat operator-(const Quat a, const Quat b) noexcept { return Quat{a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w}; }
[[nodiscard]] constexpr Quat operator*(const Quat q, const f32 s) noexcept { return Quat{q.x * s, q.y * s, q.z * s, q.w * s}; }
[[nodiscard]] constexpr Quat operator*(const f32 s, const Quat q) noexcept { return q * s; }

[[nodiscard]] constexpr f32 dot(const Quat a, const Quat b) noexcept { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }

[[nodiscard]] constexpr Quat conjugate(const Quat q) noexcept { return Quat{-q.x, -q.y, -q.z, q.w}; }

[[nodiscard]] inline std::optional<Quat> try_inverse(const Quat q, const f32 eps = 1e-8f) noexcept {
    const f32 len2 = q.length_sq();
    if (len2 <= eps * eps) {
        return std::nullopt;
    }
    return conjugate(q) * (1.0f / len2);
}

[[nodiscard]] constexpr Quat inverse_unit(const Quat q) noexcept {
    return conjugate(q);
}

// Quaternion multiply (composition).
// With column-vector convention and v' = q * v * q^-1,
// (b * a) means "apply a, then apply b".
[[nodiscard]] constexpr Quat operator*(const Quat a, const Quat b) noexcept {
    return Quat{
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    };
}

[[nodiscard]] inline Quat from_axis_angle(Vec3 axis, const f32 radians, const f32 eps = 1e-8f) noexcept {
    // Normalize axis safely; if degenerate, return identity.
    if (!axis.try_normalize(eps)) {
        return Quat::identity();
    }
    const f32 half = 0.5f * radians;
    const f32 s = std::sin(half);
    const f32 c = std::cos(half);
    return Quat{axis.x * s, axis.y * s, axis.z * s, c};
}

// Shortest rotation taking 'from' to 'to'. Inputs need not be normalized.
[[nodiscard]] inline Quat from_to(Vec3 from, Vec3 to, const f32 eps = 1e-8f) noexcept {
    if (!from.try_normalize(eps) || !to.try_normalize(eps)) {
        return Quat::identity();
    }

    const f32 d = dot(from, to);
    if (d >= 1.0f - 1e-6f) {
        return Quat::identity();
    }
    if (d <= -1.0f + 1e-6f) {
        Vec3 axis = cross(from, Vec3::unit_x());
        if (!axis.try_normalize(eps)) {
            axis = cross(from, Vec3::unit_y());
            axis.try_normalize(eps);
        }
        return from_axis_angle(axis, 3.14159265358979323846f);
    }

    const Vec3 c = cross(from, to);
    const f32 s = std::sqrt((1.0f + d) * 2.0f);
    const f32 inv_s = 1.0f / s;

    const Quat q{c.x * inv_s, c.y * inv_s, c.z * inv_s, 0.5f * s};
    return q.normalized_or_identity(eps);
}

[[nodiscard]] inline Mat3 to_mat3(Quat q) noexcept {
    q = q.normalized_or_identity();

    const f32 xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    const f32 xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    const f32 wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

    // Column-major basis
    const Vec3 c0{ 1.0f - 2.0f*(yy + zz),         2.0f*(xy + wz),         2.0f*(xz - wy)};
    const Vec3 c1{        2.0f*(xy - wz),  1.0f - 2.0f*(xx + zz),         2.0f*(yz + wx)};
    const Vec3 c2{        2.0f*(xz + wy),         2.0f*(yz - wx),  1.0f - 2.0f*(xx + yy)};
    return Mat3::from_columns(c0, c1, c2);
}

[[nodiscard]] inline Quat from_mat3(Mat3 m, const f32 eps = 1e-8f) noexcept {
    const f32 trace = m(0,0) + m(1,1) + m(2,2);

    Quat q{};
    if (trace > 0.0f) {
        const f32 s = std::sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (m(2,1) - m(1,2)) / s;
        q.y = (m(0,2) - m(2,0)) / s;
        q.z = (m(1,0) - m(0,1)) / s;
    } else if (m(0,0) > m(1,1) && m(0,0) > m(2,2)) {
        const f32 s = std::sqrt(1.0f + m(0,0) - m(1,1) - m(2,2)) * 2.0f;
        q.w = (m(2,1) - m(1,2)) / s;
        q.x = 0.25f * s;
        q.y = (m(0,1) + m(1,0)) / s;
        q.z = (m(0,2) + m(2,0)) / s;
    } else if (m(1,1) > m(2,2)) {
        const f32 s = std::sqrt(1.0f + m(1,1) - m(0,0) - m(2,2)) * 2.0f;
        q.w = (m(0,2) - m(2,0)) / s;
        q.x = (m(0,1) + m(1,0)) / s;
        q.y = 0.25f * s;
        q.z = (m(1,2) + m(2,1)) / s;
    } else {
        const f32 s = std::sqrt(1.0f + m(2,2) - m(0,0) - m(1,1)) * 2.0f;
        q.w = (m(1,0) - m(0,1)) / s;
        q.x = (m(0,2) + m(2,0)) / s;
        q.y = (m(1,2) + m(2,1)) / s;
        q.z = 0.25f * s;
    }

    return q.normalized_or_identity(eps);
}

[[nodiscard]] inline Vec3 rotate(Quat q, const Vec3 v) noexcept {
    // Assumes q is (approximately) unit; normalize defensively.
    q = q.normalized_or_identity();

    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = 2.0f * cross(u, v);
    return v + q.w * t + cross(u, t);
}

[[nodiscard]] inline Quat nlerp(const Quat a, Quat b, const f32 t) noexcept {
    if (dot(a, b) < 0.0f) {
        b = b * -1.0f;
    }
    return (a * (1.0f - t) + b * t).normalized_or_identity();
}

[[nodiscard]] inline Quat slerp(const Quat a, Quat b, const f32 t) noexcept {
    f32 d = dot(a, b);
    if (d < 0.0f) {
        b = b * -1.0f;
        d = -d;
    }

    // If very close, fall back to nlerp to avoid precision trouble.
    if (d > 0.9995f) {
        return nlerp(a, b, t);
    }

    const f32 theta = std::acos(std::clamp(d, -1.0f, 1.0f));
    const f32 s = std::sin(theta);
    const f32 w0 = std::sin((1.0f - t) * theta) / s;
    const f32 w1 = std::sin(t * theta) / s;

    return (a * w0 + b * w1).normalized_or_identity();
}

} // namespace core::math
