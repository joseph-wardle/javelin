export module javelin.math.mat4;

import std;

import javelin.core.types;
export import javelin.math.vec3;
export import javelin.math.vec4;
export import javelin.math.mat3;

export namespace javelin {

struct Mat4 final {
    // Column-major: columns are basis + translation.
    // data(): [cols[0].x cols[0].y cols[0].z cols[0].w  cols[1].x ...  cols[3].w]
    Vec4 cols[4]{
        Vec4{1.0f, 0.0f, 0.0f, 0.0f},
        Vec4{0.0f, 1.0f, 0.0f, 0.0f},
        Vec4{0.0f, 0.0f, 1.0f, 0.0f},
        Vec4{0.0f, 0.0f, 0.0f, 1.0f},
    };

    constexpr Mat4() noexcept = default;
    constexpr Mat4(const Vec4 c0_, const Vec4 c1_, const Vec4 c2_, const Vec4 c3_) noexcept
        : cols{c0_, c1_, c2_, c3_} {}

    [[nodiscard]] static constexpr Mat4 identity() noexcept { return Mat4{}; }

    [[nodiscard]] static constexpr Mat4 from_columns(const Vec4 c0_, const Vec4 c1_, const Vec4 c2_,
                                                     const Vec4 c3_) noexcept {
        return Mat4{c0_, c1_, c2_, c3_};
    }

    [[nodiscard]] static constexpr Mat4 from_rows(const Vec4 r0, const Vec4 r1, const Vec4 r2, const Vec4 r3) noexcept {
        return Mat4{
            Vec4{r0.x, r1.x, r2.x, r3.x},
            Vec4{r0.y, r1.y, r2.y, r3.y},
            Vec4{r0.z, r1.z, r2.z, r3.z},
            Vec4{r0.w, r1.w, r2.w, r3.w},
        };
    }

    [[nodiscard]] constexpr f32 *data() noexcept { return cols[0].data(); }
    [[nodiscard]] constexpr const f32 *data() const noexcept { return cols[0].data(); }

    [[nodiscard]] constexpr f32 &operator()(const usize row, const usize col) noexcept {
        switch (col) {
        case 0:
            return cols[0][row];
        case 1:
            return cols[1][row];
        case 2:
            return cols[2][row];
        default:
            return cols[3][row];
        }
    }
    [[nodiscard]] constexpr f32 operator()(const usize row, const usize col) const noexcept {
        switch (col) {
        case 0:
            return cols[0][row];
        case 1:
            return cols[1][row];
        case 2:
            return cols[2][row];
        default:
            return cols[3][row];
        }
    }

    [[nodiscard]] constexpr Vec4 col(const usize i) const noexcept {
        switch (i) {
        case 0:
            return cols[0];
        case 1:
            return cols[1];
        case 2:
            return cols[2];
        default:
            return cols[3];
        }
    }
    [[nodiscard]] constexpr Vec4 row(const usize i) const noexcept {
        return Vec4{(*this)(i, 0), (*this)(i, 1), (*this)(i, 2), (*this)(i, 3)};
    }

    [[nodiscard]] bool is_finite() const noexcept {
        return cols[0].is_finite() && cols[1].is_finite() && cols[2].is_finite() && cols[3].is_finite();
    }

    [[nodiscard]] static constexpr Mat4 translation(const Vec3 t) noexcept {
        Mat4 m{};
        m.cols[3] = Vec4{t.x, t.y, t.z, 1.0f};
        return m;
    }

    [[nodiscard]] static constexpr Mat4 scale(const Vec3 s) noexcept {
        return Mat4{
            Vec4{s.x, 0.0f, 0.0f, 0.0f},
            Vec4{0.0f, s.y, 0.0f, 0.0f},
            Vec4{0.0f, 0.0f, s.z, 0.0f},
            Vec4{0.0f, 0.0f, 0.0f, 1.0f},
        };
    }

    [[nodiscard]] static constexpr Mat4 from_affine(const Mat3 &linear, const Vec3 t) noexcept {
        const Vec3 c0 = linear.col(0);
        const Vec3 c1 = linear.col(1);
        const Vec3 c2 = linear.col(2);
        return Mat4{
            Vec4{c0.x, c0.y, c0.z, 0.0f},
            Vec4{c1.x, c1.y, c1.z, 0.0f},
            Vec4{c2.x, c2.y, c2.z, 0.0f},
            Vec4{t.x, t.y, t.z, 1.0f},
        };
    }

    [[nodiscard]] static Mat4 rotation_x(const f32 radians) noexcept {
        const f32 s = std::sin(radians);
        const f32 c = std::cos(radians);
        return from_rows(Vec4{1.0f, 0.0f, 0.0f, 0.0f}, Vec4{0.0f, c, -s, 0.0f}, Vec4{0.0f, s, c, 0.0f},
                         Vec4{0.0f, 0.0f, 0.0f, 1.0f});
    }

    [[nodiscard]] static Mat4 rotation_y(const f32 radians) noexcept {
        const f32 s = std::sin(radians);
        const f32 c = std::cos(radians);
        return from_rows(Vec4{c, 0.0f, s, 0.0f}, Vec4{0.0f, 1.0f, 0.0f, 0.0f}, Vec4{-s, 0.0f, c, 0.0f},
                         Vec4{0.0f, 0.0f, 0.0f, 1.0f});
    }

    [[nodiscard]] static Mat4 rotation_z(const f32 radians) noexcept {
        const f32 s = std::sin(radians);
        const f32 c = std::cos(radians);
        return from_rows(Vec4{c, -s, 0.0f, 0.0f}, Vec4{s, c, 0.0f, 0.0f}, Vec4{0.0f, 0.0f, 1.0f, 0.0f},
                         Vec4{0.0f, 0.0f, 0.0f, 1.0f});
    }
};

[[nodiscard]] constexpr Vec4 operator*(const Mat4 &m, const Vec4 v) noexcept {
    return m.cols[0] * v.x + m.cols[1] * v.y + m.cols[2] * v.z + m.cols[3] * v.w;
}

[[nodiscard]] constexpr Mat4 operator*(const Mat4 &a, const Mat4 &b) noexcept {
    return Mat4::from_columns(a * b.cols[0], a * b.cols[1], a * b.cols[2], a * b.cols[3]);
}

[[nodiscard]] inline Mat4 transpose(const Mat4 &m) noexcept {
    return Mat4::from_rows(Vec4{m.cols[0].x, m.cols[1].x, m.cols[2].x, m.cols[3].x},
                           Vec4{m.cols[0].y, m.cols[1].y, m.cols[2].y, m.cols[3].y},
                           Vec4{m.cols[0].z, m.cols[1].z, m.cols[2].z, m.cols[3].z},
                           Vec4{m.cols[0].w, m.cols[1].w, m.cols[2].w, m.cols[3].w});
}

[[nodiscard]] inline Vec3 transform_vector(const Mat4 &m, const Vec3 v) noexcept {
    const Vec4 r = m * Vec4{v.x, v.y, v.z, 0.0f};
    return Vec3{r.x, r.y, r.z};
}

[[nodiscard]] inline Vec3 transform_point_affine(const Mat4 &m, const Vec3 p) noexcept {
    const Vec4 r = m * Vec4{p.x, p.y, p.z, 1.0f};
    return Vec3{r.x, r.y, r.z};
}

[[nodiscard]] inline std::optional<Vec3> try_project_point(const Mat4 &m, const Vec3 p,
                                                           const f32 eps = 1e-8f) noexcept {
    const Vec4 r = m * Vec4{p.x, p.y, p.z, 1.0f};
    if (std::fabs(r.w) <= eps) {
        return std::nullopt;
    }
    const f32 inv_w = 1.0f / r.w;
    return Vec3{r.x * inv_w, r.y * inv_w, r.z * inv_w};
}

namespace detail {
[[nodiscard]] inline std::optional<std::pair<Mat4, f32>> try_inverse_and_det(Mat4 m, const f32 eps) noexcept {
    f32 a[4][8]{};

    for (usize r = 0; r < 4; ++r) {
        for (usize c = 0; c < 4; ++c) {
            a[r][c] = m(r, c);
        }
        a[r][4 + r] = 1.0f;
    }

    f32 det = 1.0f;
    i32 det_sign = 1;

    for (usize col = 0; col < 4; ++col) {
        usize pivot = col;
        f32 best = std::fabs(a[col][col]);
        for (usize r = col + 1; r < 4; ++r) {
            if (const f32 v = std::fabs(a[r][col]); v > best) {
                best = v;
                pivot = r;
            }
        }
        if (best <= eps) {
            return std::nullopt;
        }
        if (pivot != col) {
            for (usize c = 0; c < 8; ++c) {
                std::swap(a[col][c], a[pivot][c]);
            }
            det_sign = -det_sign;
        }

        const f32 piv = a[col][col];
        det *= piv;

        const f32 inv_piv = 1.0f / piv;
        for (usize c = 0; c < 8; ++c) {
            a[col][c] *= inv_piv;
        }

        for (usize r = 0; r < 4; ++r) {
            if (r == col) {
                continue;
            }
            const f32 f = a[r][col];
            if (f == 0.0f) {
                continue;
            }
            for (usize c = 0; c < 8; ++c) {
                a[r][c] -= f * a[col][c];
            }
        }
    }

    det *= static_cast<f32>(det_sign);

    Mat4 inv{};
    for (usize r = 0; r < 4; ++r) {
        for (usize c = 0; c < 4; ++c) {
            inv(r, c) = a[r][4 + c];
        }
    }
    return std::pair{inv, det};
}
} // namespace detail

[[nodiscard]] inline std::optional<Mat4> try_inverse(const Mat4 &m, const f32 eps = 1e-8f) noexcept {
    if (auto out = detail::try_inverse_and_det(m, eps)) {
        return out->first;
    }
    return std::nullopt;
}

[[nodiscard]] inline Mat4 inverse_or_identity(const Mat4 &m, const f32 eps = 1e-8f) noexcept {
    if (const auto inv = try_inverse(m, eps)) {
        return *inv;
    }
    return Mat4::identity();
}

[[nodiscard]] inline std::optional<f32> try_determinant(const Mat4 &m, const f32 eps = 1e-8f) noexcept {
    if (auto out = detail::try_inverse_and_det(m, eps)) {
        return out->second;
    }
    return std::nullopt;
}

// Normal matrix for the upper-left 3x3 linear part: (M^-1)^T, then take 3x3.
[[nodiscard]] inline std::optional<Mat3> try_normal_matrix(const Mat4 &m, const f32 eps = 1e-8f) noexcept {
    const auto inv = try_inverse(m, eps);
    if (!inv) {
        return std::nullopt;
    }
    const Mat4 n = transpose(*inv);
    return Mat3::from_columns(Vec3{n.cols[0].x, n.cols[0].y, n.cols[0].z}, Vec3{n.cols[1].x, n.cols[1].y, n.cols[1].z},
                              Vec3{n.cols[2].x, n.cols[2].y, n.cols[2].z});
}

} // namespace javelin
