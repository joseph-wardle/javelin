export module javelin.scene.scene_file;

import std;

import javelin.core.types;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.scene.entity;
import javelin.scene.shapes;

export namespace javelin {

inline constexpr u32 kSceneFileVersion = 1u;
inline constexpr std::string_view kSceneFileUnitsMeters = "m";

/*
Scene file text schema (v1, .jvscene):
- One record per line, key=value pairs, '#' comments.
- Core records:
  - scene version=<u32> units=m
  - shape id=<id> kind=sphere r=<f32>
  - shape id=<id> kind=box hx=<f32> hy=<f32> hz=<f32>
  - body id=<id> shape=<shape_id> motion=<dynamic|static> material=<u32> mesh=<u32>
         px=<f32> py=<f32> pz=<f32>
         ox=<f32> oy=<f32> oz=<f32> ow=<f32>
         vx=<f32> vy=<f32> vz=<f32>
         wx=<f32> wy=<f32> wz=<f32>
- The text format is intentionally grep-friendly and can be streamed line-by-line.
*/

enum struct SceneFileBodyMotion : u8 {
    dynamic_body,
    static_body,
};

[[nodiscard]] constexpr std::string_view to_string(const ShapeKind kind) noexcept {
    switch (kind) {
    case ShapeKind::sphere:
        return "sphere";
    case ShapeKind::box:
        return "box";
    }
    return "unknown";
}

[[nodiscard]] constexpr std::string_view to_string(const SceneFileBodyMotion motion) noexcept {
    switch (motion) {
    case SceneFileBodyMotion::dynamic_body:
        return "dynamic";
    case SceneFileBodyMotion::static_body:
        return "static";
    }
    return "unknown";
}

[[nodiscard]] constexpr std::optional<ShapeKind> parse_shape_kind(const std::string_view token) noexcept {
    if (token == "sphere") {
        return ShapeKind::sphere;
    }
    if (token == "box") {
        return ShapeKind::box;
    }
    return std::nullopt;
}

[[nodiscard]] constexpr std::optional<SceneFileBodyMotion> parse_scene_file_body_motion(
    const std::string_view token) noexcept {
    if (token == "dynamic") {
        return SceneFileBodyMotion::dynamic_body;
    }
    if (token == "static") {
        return SceneFileBodyMotion::static_body;
    }
    return std::nullopt;
}

struct SceneFileError final {
    std::filesystem::path path{};
    u32 line{0};
    std::string message{};
};

[[nodiscard]] inline std::string format_scene_file_error(const SceneFileError &error) {
    if (!error.path.empty() && error.line > 0u) {
        return std::format("{}:{}: {}", error.path.string(), error.line, error.message);
    }
    if (!error.path.empty()) {
        return std::format("{}: {}", error.path.string(), error.message);
    }
    return error.message;
}

struct SceneFileShape final {
    std::string id{};
    ShapeData shape{};
};

struct SceneFileBody final {
    std::string id{};
    std::string shape_id{};
    SceneFileBodyMotion motion{SceneFileBodyMotion::dynamic_body};
    MaterialId material{0u};
    MeshId mesh{0u};

    Vec3 position{};
    Quat orientation{Quat::identity()};
    Vec3 velocity{};
    Vec3 angular_velocity{};
};

struct SceneFileLoadOptions final {
    bool validate_after_parse{true};
};

struct SceneFileSaveOptions final {
    bool validate_before_write{true};
};

struct SceneFile final {
    u32 version{kSceneFileVersion};
    std::string units{std::string{kSceneFileUnitsMeters}};
    std::vector<SceneFileShape> shapes{};
    std::vector<SceneFileBody> bodies{};

    void clear() {
        version = kSceneFileVersion;
        units.assign(kSceneFileUnitsMeters);
        shapes.clear();
        bodies.clear();
    }

    void reserve(const u32 shape_count, const u32 body_count) {
        shapes.reserve(shape_count);
        bodies.reserve(body_count);
    }

    [[nodiscard]] bool empty() const noexcept { return shapes.empty() && bodies.empty(); }

    [[nodiscard]] std::expected<void, SceneFileError>
    validate(const std::filesystem::path &source_path = std::filesystem::path{}) const {
        auto error = [&source_path](const std::string message) -> std::expected<void, SceneFileError> {
            return std::unexpected(SceneFileError{.path = source_path, .line = 0u, .message = message});
        };

        if (version != kSceneFileVersion) {
            return error(std::format("Unsupported scene file version {} (expected {})", version, kSceneFileVersion));
        }
        if (units != kSceneFileUnitsMeters) {
            return error(std::format("Unsupported scene file units '{}' (expected '{}')", units, kSceneFileUnitsMeters));
        }

        std::unordered_set<std::string_view> shape_ids{};
        shape_ids.reserve(shapes.size() * 2u + 1u);
        for (u32 i = 0; i < shapes.size(); ++i) {
            const SceneFileShape &shape = shapes[i];
            if (shape.id.empty()) {
                return error(std::format("shape[{}] has empty id", i));
            }
            if (!shape_ids.insert(shape.id).second) {
                return error(std::format("Duplicate shape id '{}'", shape.id));
            }

            switch (shape.shape.kind) {
            case ShapeKind::sphere: {
                const f32 radius = shape_sphere(shape.shape).radius;
                if (!std::isfinite(radius) || radius <= 0.0f) {
                    return error(std::format("shape '{}' has invalid sphere radius {}", shape.id, radius));
                }
            } break;
            case ShapeKind::box: {
                const Vec3 half_extents = shape_box(shape.shape).half_extents;
                if (!half_extents.is_finite() || half_extents.x <= 0.0f || half_extents.y <= 0.0f ||
                    half_extents.z <= 0.0f) {
                    return error(std::format("shape '{}' has invalid box half extents [{}, {}, {}]", shape.id,
                                             half_extents.x, half_extents.y, half_extents.z));
                }
            } break;
            }
        }

        std::unordered_set<std::string_view> body_ids{};
        body_ids.reserve(bodies.size() * 2u + 1u);
        for (u32 i = 0; i < bodies.size(); ++i) {
            const SceneFileBody &body = bodies[i];
            if (body.id.empty()) {
                return error(std::format("body[{}] has empty id", i));
            }
            if (!body_ids.insert(body.id).second) {
                return error(std::format("Duplicate body id '{}'", body.id));
            }
            if (body.shape_id.empty()) {
                return error(std::format("body '{}' has empty shape id", body.id));
            }
            if (!shape_ids.contains(body.shape_id)) {
                return error(std::format("body '{}' references unknown shape '{}'", body.id, body.shape_id));
            }
            if (!body.position.is_finite()) {
                return error(std::format("body '{}' has non-finite position", body.id));
            }
            if (!body.orientation.is_finite() || body.orientation.length_sq() <= 1e-8f) {
                return error(std::format("body '{}' has invalid orientation", body.id));
            }
            if (!body.velocity.is_finite()) {
                return error(std::format("body '{}' has non-finite linear velocity", body.id));
            }
            if (!body.angular_velocity.is_finite()) {
                return error(std::format("body '{}' has non-finite angular velocity", body.id));
            }
        }

        return {};
    }

    [[nodiscard]] static std::expected<SceneFile, SceneFileError>
    load(const std::filesystem::path &path, const SceneFileLoadOptions & /*options*/ = {}) {
        return std::unexpected(SceneFileError{
            .path = path,
            .line = 0u,
            .message = "SceneFile::load is not implemented yet",
        });
    }

    [[nodiscard]] std::expected<void, SceneFileError> save(
        const std::filesystem::path &path, const SceneFileSaveOptions & /*options*/ = {}) const {
        return std::unexpected(SceneFileError{
            .path = path,
            .line = 0u,
            .message = "SceneFile::save is not implemented yet",
        });
    }
};

} // namespace javelin
