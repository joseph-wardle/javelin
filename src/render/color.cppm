export module javelin.render.color;

import std;

import javelin.math.mat3;
import javelin.math.vec3;

export namespace javelin {

[[nodiscard]] constexpr float srgb_to_linear(const float x) noexcept {
    // IEC 61966-2-1
    if (x <= 0.04045f)
        return x / 12.92f;
    return std::powf((x + 0.055f) / 1.055f, 2.4f);
}

[[nodiscard]] constexpr Vec3 srgb_to_linear(const Vec3 c) noexcept {
    return Vec3{
        srgb_to_linear(c.x),
        srgb_to_linear(c.y),
        srgb_to_linear(c.z),
    };
}

[[nodiscard]] constexpr Vec3 linear_srgb_to_acescg(const Vec3 c_lin_srgb) noexcept {
    // Linear sRGB / Rec.709 (D65) -> ACEScg (AP1, D60)
    // Commonly referred to as sRGB_2_AP1. :contentReference[oaicite:3]{index=3}
    constexpr Mat3 m = Mat3::from_rows(Vec3{0.613097f, 0.339523f, 0.047380f}, Vec3{0.070194f, 0.916355f, 0.013451f},
                                       Vec3{0.020615f, 0.109569f, 0.869816f});
    return m * c_lin_srgb;
}

[[nodiscard]] constexpr Vec3 srgb_to_acescg(const Vec3 c_srgb_encoded_0_1) noexcept {
    return linear_srgb_to_acescg(srgb_to_linear(c_srgb_encoded_0_1));
}

} // namespace javelin
