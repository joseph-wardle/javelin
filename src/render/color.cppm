export module javelin.render.color;

import javelin.math.mat3;
import javelin.math.vec3;

export namespace javelin {

[[nodiscard]] constexpr Vec3 linear_srgb_to_acescg(const Vec3 c) noexcept {
    // Linear sRGB (Rec.709, D65) -> ACEScg (AP1, D60) conversion.
    constexpr Mat3 m = Mat3::from_rows(
        Vec3{0.613097f, 0.339523f, 0.047380f},
        Vec3{0.070194f, 0.916355f, 0.013451f},
        Vec3{0.020615f, 0.109569f, 0.869816f});
    return m * c;
}

} // namespace javelin
