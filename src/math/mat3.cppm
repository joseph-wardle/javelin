export module javelin.math.mat3;

import std;

import javelin.core.types;
export import javelin.math.vec3;

export namespace javelin::math {

struct Mat3 final {
    // Column-major: columns are basis vectors.
    // Memory order (data): [c0.x c0.y c0.z  c1.x c1.y c1.z  c2.x c2.y c2.z]
    Vec3 c0{1.0f, 0.0f, 0.0f};
    Vec3 c1{0.0f, 1.0f, 0.0f};
    Vec3 c2{0.0f, 0.0f, 1.0f};

    constexpr Mat3() noexcept = default;
    constexpr Mat3(const Vec3 c0_, const Vec3 c1_, const Vec3 c2_) noexcept : c0{c0_}, c1{c1_}, c2{c2_} {}

    [[nodiscard]] static constexpr Mat3 identity() noexcept { return Mat3{}; }

    [[nodiscard]] static constexpr Mat3 diagonal(const f32 d0, const f32 d1, const f32 d2) noexcept {
        return Mat3{ Vec3{d0, 0.0f, 0.0f}, Vec3{0.0f, d1, 0.0f}, Vec3{0.0f, 0.0f, d2} };
    }

    [[nodiscard]] static constexpr Mat3 from_columns(const Vec3 c0_, const Vec3 c1_, const Vec3 c2_) noexcept {
        return Mat3{c0_, c1_, c2_};
    }

    [[nodiscard]] static constexpr Mat3 from_rows(const Vec3 r0, const Vec3 r1, const Vec3 r2) noexcept {
        return Mat3{
            Vec3{r0.x, r1.x, r2.x},
            Vec3{r0.y, r1.y, r2.y},
            Vec3{r0.z, r1.z, r2.z},
        };
    }

    [[nodiscard]] constexpr f32* data() noexcept { return &c0.x; }
    [[nodiscard]] constexpr const f32* data() const noexcept { return &c0.x; }

    [[nodiscard]] constexpr f32& operator()(const usize row, const usize col) noexcept {
        return (&c0)[col][row];
    }
    [[nodiscard]] constexpr f32 operator()(const usize row, const usize col) const noexcept {
        return (&c0)[col][row];
    }

    [[nodiscard]] constexpr Vec3 col(const usize i) const noexcept {
        return (&c0)[i];
    }

    [[nodiscard]] constexpr Vec3 row(const usize i) const noexcept {
        return Vec3{(*this)(i, 0), (*this)(i, 1), (*this)(i, 2)};
    }

    [[nodiscard]] bool is_finite() const noexcept {
        return c0.is_finite() && c1.is_finite() && c2.is_finite();
    }

    [[nodiscard]] static Mat3 rotation_x(const f32 radians) noexcept {
        const f32 s = std::sin(radians);
        const f32 c = std::cos(radians);
        return from_rows(
            Vec3{1.0f, 0.0f, 0.0f},
            Vec3{0.0f,    c,   -s},
            Vec3{0.0f,    s,    c}
        );
    }

    [[nodiscard]] static Mat3 rotation_y(const f32 radians) noexcept {
        const f32 s = std::sin(radians);
        const f32 c = std::cos(radians);
        return from_rows(
            Vec3{   c, 0.0f,    s},
            Vec3{0.0f, 1.0f, 0.0f},
            Vec3{  -s, 0.0f,    c}
        );
    }

    [[nodiscard]] static Mat3 rotation_z(const f32 radians) noexcept {
        const f32 s = std::sin(radians);
        const f32 c = std::cos(radians);
        return from_rows(
            Vec3{   c,  -s,  0.0f},
            Vec3{   s,   c,  0.0f},
            Vec3{0.0f, 0.0f, 1.0f}
        );
    }
};

[[nodiscard]] constexpr Mat3 operator*(const Mat3 &m, const f32 s) noexcept {
    return Mat3{m.c0 * s, m.c1 * s, m.c2 * s};
}
[[nodiscard]] constexpr Mat3 operator*(const f32 s, const Mat3 &m) noexcept { return m * s; }

[[nodiscard]] constexpr Vec3 operator*(const Mat3 &m, const Vec3 v) noexcept {
    return m.c0 * v.x + m.c1 * v.y + m.c2 * v.z;
}

[[nodiscard]] constexpr Mat3 operator*(const Mat3 &a, const Mat3 &b) noexcept {
    return Mat3::from_columns(a * b.c0, a * b.c1, a * b.c2);
}

[[nodiscard]] constexpr f32 determinant(const Mat3 &m) noexcept {
    return dot(m.c0, cross(m.c1, m.c2));
}

[[nodiscard]] constexpr Mat3 transpose(const Mat3 &m) noexcept {
    return Mat3::from_rows(
        Vec3{m.c0.x, m.c1.x, m.c2.x},
        Vec3{m.c0.y, m.c1.y, m.c2.y},
        Vec3{m.c0.z, m.c1.z, m.c2.z}
    );
}

[[nodiscard]] inline std::optional<Mat3> try_inverse(const Mat3 &m, const f32 eps = 1e-8f) noexcept {
    // For M = [a b c] (columns), rows of adjugate are:
    // r0 = cross(b,c), r1 = cross(c,a), r2 = cross(a,b)
    const Vec3 r0 = cross(m.c1, m.c2);
    const Vec3 r1 = cross(m.c2, m.c0);
    const Vec3 r2 = cross(m.c0, m.c1);

    const f32 det = dot(m.c0, r0);
    if (std::fabs(det) <= eps) {
        return std::nullopt;
    }

    const f32 inv_det = 1.0f / det;
    return Mat3::from_rows(r0 * inv_det, r1 * inv_det, r2 * inv_det);
}

[[nodiscard]] inline Mat3 inverse_or_identity(const Mat3 &m, const f32 eps = 1e-8f) noexcept {
    if (const auto inv = try_inverse(m, eps)) {
        return *inv;
    }
    return Mat3::identity();
}

// Normal matrix for transforming normals with non-uniform scale: (M^-1)^T
[[nodiscard]] inline std::optional<Mat3> try_normal_matrix(const Mat3 &m, const f32 eps = 1e-8f) noexcept {
    const auto inv = try_inverse(m, eps);
    if (!inv) {
        return std::nullopt;
    }
    return transpose(*inv);
}

} // namespace javelin::math
