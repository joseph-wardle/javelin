export module javelin.scene.entity;

import std;

import javelin.core.types;

export namespace javelin {

struct EntityId final {
    u32 index{};
    u32 generation{};
};

struct MaterialId final {
    u32 value{};
};
struct MeshId final {
    u32 value{};
};

// Authored motion intent for a body.  Dynamic bodies get inv_mass / inv_inertia
// derived from shape + material density; static bodies have inv_mass == 0 and
// are skipped by the integrator.  Stored in Scene as part of the round-trip
// authored record (not derived from simulation state).
enum struct BodyMotion : u8 {
    dynamic_body,
    static_body,
};

[[nodiscard]] constexpr std::string_view to_string(const BodyMotion motion) noexcept {
    switch (motion) {
    case BodyMotion::dynamic_body:
        return "dynamic";
    case BodyMotion::static_body:
        return "static";
    }
    return "unknown";
}

[[nodiscard]] constexpr std::optional<BodyMotion> parse_body_motion(const std::string_view token) noexcept {
    if (token == "dynamic") {
        return BodyMotion::dynamic_body;
    }
    if (token == "static") {
        return BodyMotion::static_body;
    }
    return std::nullopt;
}

} // namespace javelin
