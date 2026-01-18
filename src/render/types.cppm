export module javelin.render.types;

import javelin.core.types;

export namespace javelin {

struct Extent2D final {
    i32 width{};
    i32 height{};

    [[nodiscard]] constexpr bool is_valid() const noexcept { return width > 0 && height > 0; }
};

using TextureHandle = u32; // GLuint
using FboHandle = u32;     // GLuint
using RboHandle = u32;     // GLuint

} // namespace javelin
