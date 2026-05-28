module;

#include <tracy/Tracy.hpp>

export module javelin.scene.scene_file;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.constraint_types;
import javelin.scene.entity;
import javelin.scene.physics_materials;
import javelin.scene.shapes;

export namespace javelin {

inline constexpr u32 kSceneFileVersion = 1u;
inline constexpr std::string_view kSceneFileUnitsMeters = "m";

/*
Scene file text schema (v1, .jvscene):
- One record per line, key=value pairs, '#' comments.
- Core records:
  - scene version=<u32> units=m
  - physics_material id=<u32> restitution=<f32> friction=<f32> [density=<f32>]
  - shape id=<id> kind=sphere r=<f32>
  - shape id=<id> kind=box hx=<f32> hy=<f32> hz=<f32>
  - body id=<id> shape=<shape_id> motion=<dynamic|static> material=<u32>
         mesh=<u32> px=<f32> py=<f32> pz=<f32> [ox=<f32> oy=<f32> oz=<f32> ow=<f32>]
         [vx=<f32> vy=<f32> vz=<f32>]
         [wx=<f32> wy=<f32> wz=<f32>]
  - constraint id=<id> kind=distance body_a=<body_id> body_b=<body_id>
              ax=<f32> ay=<f32> az=<f32> bx=<f32> by=<f32> bz=<f32>
              rest=<f32> compliance=<f32>
    Enforces |p_b - p_a| = rest between world-space anchor points.
    ax/ay/az: anchor on body_a in its local space (metres).
    bx/by/bz: anchor on body_b in its local space (metres).
    rest:       target distance (metres, >= 0).
    compliance: XPBD alpha (m/N); 0 = rigid link.
- Defaults:
  - physics_material: material id=0 always exists as kDefaultPhysicsMaterial; only
    non-default materials need explicit records.
  - body.motion=dynamic, body.material=0, body.mesh=0
  - body.orientation=identity
  - body.velocity=(0,0,0), body.angular_velocity=(0,0,0)
  - body position (px/py/pz) is required.
- physics_material.id and body.material share the same numeric namespace: a body
  with material=2 uses the physics_material record with id=2.
- The format is intentionally grep-friendly and parses line-by-line with no
  external dependencies.
- Validation is strict by design: programmatic construction and text parsing are
  held to the same contract.
*/

[[nodiscard]] constexpr std::string_view to_string(const ShapeKind kind) noexcept {
    switch (kind) {
    case ShapeKind::sphere:
        return "sphere";
    case ShapeKind::box:
        return "box";
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
    BodyMotion motion{BodyMotion::dynamic_body};
    MaterialId material{0u};
    MeshId mesh{0u};

    Vec3 position{};
    Quat orientation{Quat::identity()};
    Vec3 velocity{};
    Vec3 angular_velocity{};
};

// A physics_material record as authored in the scene file.
// id is the MaterialId.value shared with body records' material=<u32> field.
// density (kg/m³, default=1.0) scales the mass computed from shape volume at load time.
// It is a load-time property only: changing density at runtime has no effect until the
// scene is reloaded. It is intentionally not part of PhysicsMaterial so runtime simulation
// updates cannot accidentally reset an authored density.
inline constexpr f32 kDefaultMaterialDensity = 1.0f;
struct SceneFilePhysicsMaterial final {
    u32 id{};
    PhysicsMaterial material{};
    f32 density{kDefaultMaterialDensity};
};

// A distance constraint record as authored in the scene file.
// body_a_id and body_b_id are resolved to numeric scene body indices at load time.
// Anchors are in each body's local space; the solver rotates them to world space each tick.
struct SceneFileConstraint final {
    std::string id{};
    std::string body_a_id{};
    std::string body_b_id{};
    Vec3 anchor_a{};   // attachment point in body A local space (metres)
    Vec3 anchor_b{};   // attachment point in body B local space (metres)
    f32 rest_length{}; // target distance (metres, >= 0)
    f32 compliance{};  // XPBD alpha (m/N); 0 = rigid link
};

struct SceneFileLoadOptions final {
    bool validate_after_parse{true};
};

struct SceneFileSaveOptions final {
    bool validate_before_write{true};
};

namespace detail {
inline constexpr std::string_view kUtf8Bom = "\xEF\xBB\xBF";

[[nodiscard]] constexpr bool ascii_space(const char c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
}

[[nodiscard]] constexpr std::string_view trim_left(std::string_view text) noexcept {
    while (!text.empty() && ascii_space(text.front())) {
        text.remove_prefix(1);
    }
    return text;
}

[[nodiscard]] constexpr std::string_view trim_right(std::string_view text) noexcept {
    while (!text.empty() && ascii_space(text.back())) {
        text.remove_suffix(1);
    }
    return text;
}

[[nodiscard]] constexpr std::string_view trim(std::string_view text) noexcept { return trim_right(trim_left(text)); }

[[nodiscard]] constexpr std::string_view strip_comment(std::string_view text) noexcept {
    const usize comment_pos = text.find('#');
    if (comment_pos == std::string_view::npos) {
        return text;
    }
    return text.substr(0u, comment_pos);
}

struct TokenCursor final {
    std::string_view rest{};

    [[nodiscard]] constexpr std::optional<std::string_view> next() noexcept {
        rest = trim_left(rest);
        if (rest.empty()) {
            return std::nullopt;
        }

        usize end = 0u;
        while (end < rest.size() && !ascii_space(rest[end])) {
            ++end;
        }

        const std::string_view token = rest.substr(0u, end);
        rest = (end < rest.size()) ? rest.substr(end) : std::string_view{};
        return token;
    }
};

struct KeyValueToken final {
    std::string_view key{};
    std::string_view value{};
};

[[nodiscard]] constexpr std::optional<KeyValueToken> split_key_value_token(const std::string_view token) noexcept {
    const usize eq = token.find('=');
    if (eq == std::string_view::npos || eq == 0u || eq + 1u >= token.size()) {
        return std::nullopt;
    }
    return KeyValueToken{
        .key = token.substr(0u, eq),
        .value = token.substr(eq + 1u),
    };
}

template <class Number> [[nodiscard]] std::optional<Number> parse_number(const std::string_view token) noexcept {
    Number out{};
    const char *const begin = token.data();
    const char *const end = begin + token.size();
    const auto [ptr, ec] = std::from_chars(begin, end, out);
    if (ec != std::errc{} || ptr != end) {
        return std::nullopt;
    }
    return out;
}

[[nodiscard]] constexpr bool is_identifier_char(const char c) noexcept {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-' ||
           c == '.' || c == '/' || c == ':';
}

[[nodiscard]] constexpr bool is_valid_identifier(const std::string_view id) noexcept {
    if (id.empty()) {
        return false;
    }
    for (const char c : id) {
        if (!is_identifier_char(c)) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] constexpr f32 canonicalize_signed_zero(const f32 value) noexcept {
    return (value == 0.0f) ? 0.0f : value;
}

[[nodiscard]] inline Quat canonicalize_quaternion_for_text(Quat q) noexcept {
    if (!q.try_normalize()) {
        return Quat::identity();
    }

    // q and -q represent the same orientation. Canonicalize sign for stable
    // diffs.
    const bool flip =
        (q.w < 0.0f) || (q.w == 0.0f && (q.z < 0.0f || (q.z == 0.0f && (q.y < 0.0f || (q.y == 0.0f && q.x < 0.0f)))));
    if (flip) {
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
    }
    return q;
}

template <class T>
[[nodiscard]] inline std::vector<usize> sorted_indices_by_id(const std::vector<T> &items) {
    std::vector<usize> indices(items.size());
    std::iota(indices.begin(), indices.end(), static_cast<usize>(0));
    std::stable_sort(indices.begin(), indices.end(),
                     [&items](const usize a, const usize b) { return items[a].id < items[b].id; });
    return indices;
}

inline void append_token(std::string &line, const std::string_view token) {
    if (!line.empty()) {
        line.push_back(' ');
    }
    line.append(token);
}

inline void append_key_value(std::string &line, const std::string_view key, const std::string_view value) {
    if (!line.empty()) {
        line.push_back(' ');
    }
    line.append(key);
    line.push_back('=');
    line.append(value);
}

inline void append_key_value_u32(std::string &line, const std::string_view key, const u32 value) {
    std::array<char, 16> buffer{};
    const auto [ptr, ec] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    if (ec != std::errc{}) {
        append_key_value(line, key, std::to_string(value));
        return;
    }
    append_key_value(line, key, std::string_view{buffer.data(), static_cast<usize>(ptr - buffer.data())});
}

inline void append_key_value_f32(std::string &line, const std::string_view key, const f32 value) {
    const f32 canon = canonicalize_signed_zero(value);
    std::array<char, 64> buffer{};
    const auto [ptr, ec] = std::to_chars(buffer.data(), buffer.data() + buffer.size(), canon,
                                         std::chars_format::general, std::numeric_limits<f32>::max_digits10);
    if (ec != std::errc{}) {
        append_key_value(line, key, std::format("{:.9g}", canon));
        return;
    }
    append_key_value(line, key, std::string_view{buffer.data(), static_cast<usize>(ptr - buffer.data())});
}

// Builder-style parser for `key=value` records.
//
// Construction walks the cursor once, populating a flat (key,value) list and
// detecting duplicates / malformed tokens. The first failure is stashed in
// `error_` and all subsequent extractor calls become no-ops; `finish()`
// returns either the stashed error or "Unknown <record_kind> key '<k>'" for
// any leftover keys the caller did not extract.
//
// Each take_* helper consumes the key on success; unconsumed keys at finish()
// time are surfaced as unknown-key errors.
class KeyValueParser final {
  public:
    KeyValueParser(const std::string_view payload, const std::string_view record_kind) : record_kind_{record_kind} {
        TokenCursor cursor{payload};
        while (const auto token_opt = cursor.next()) {
            const std::string_view token = *token_opt;
            const auto kv = split_key_value_token(token);
            if (!kv) {
                set_error_(std::format("Expected key=value token, got '{}'", token));
                return;
            }
            for (const Entry &existing : entries_) {
                if (existing.key == kv->key) {
                    set_error_(std::format("Duplicate key '{}'", kv->key));
                    return;
                }
            }
            entries_.push_back({.key = kv->key, .value = kv->value, .consumed = false});
        }
    }

    // True if construction or a take_* call already recorded an error.
    [[nodiscard]] bool failed() const noexcept { return error_.has_value(); }

    // Surface an error from caller-side validation (e.g., bad enum value).
    void fail(std::string message) { set_error_(std::move(message)); }

    // Consume and return the raw value if present; nullopt otherwise. Used for
    // enum-like fields where the caller wants to dispatch on the string.
    [[nodiscard]] std::optional<std::string_view> take_raw(const std::string_view key) noexcept {
        if (failed()) {
            return std::nullopt;
        }
        Entry *e = find_(key);
        if (e == nullptr) {
            return std::nullopt;
        }
        e->consumed = true;
        return e->value;
    }

    // Returns true if the key was present in the input (even if already consumed).
    [[nodiscard]] bool seen(const std::string_view key) const noexcept {
        for (const Entry &e : entries_) {
            if (e.key == key) {
                return true;
            }
        }
        return false;
    }

    // After all known extractions, finalize. Returns the first error encountered,
    // or "Unknown <record_kind> key '<k>'" for the first leftover key.
    [[nodiscard]] std::expected<void, std::string> finish() {
        if (error_) {
            return std::unexpected(std::move(*error_));
        }
        for (const Entry &e : entries_) {
            if (!e.consumed) {
                return std::unexpected(std::format("Unknown {} key '{}'", record_kind_, e.key));
            }
        }
        return {};
    }

  private:
    struct Entry final {
        std::string_view key;
        std::string_view value;
        bool consumed;
    };

    std::string_view record_kind_;
    std::vector<Entry> entries_;
    std::optional<std::string> error_;

    void set_error_(std::string message) {
        if (!error_) {
            error_ = std::move(message);
        }
    }

    [[nodiscard]] Entry *find_(const std::string_view key) noexcept {
        for (Entry &e : entries_) {
            if (e.key == key) {
                return &e;
            }
        }
        return nullptr;
    }

    template <class Num>
    friend bool take_optional_number(KeyValueParser &p, std::string_view key, Num &out, bool *present);
    template <class Num>
    friend bool take_required_number(KeyValueParser &p, std::string_view key, Num &out);
    friend bool take_required_identifier(KeyValueParser &p, std::string_view key, std::string_view what,
                                         std::string &out);
};

// Extract a numeric value for the given key. If the key is missing, leaves
// `out` untouched and (if non-null) sets *present=false. Records an error if
// the value fails to parse.
template <class Num>
inline bool take_optional_number(KeyValueParser &p, const std::string_view key, Num &out) {
    return take_optional_number(p, key, out, static_cast<bool *>(nullptr));
}

template <class Num>
inline bool take_optional_number(KeyValueParser &p, const std::string_view key, Num &out, bool *present) {
    if (present) {
        *present = false;
    }
    if (p.failed()) {
        return false;
    }
    KeyValueParser::Entry *e = p.find_(key);
    if (e == nullptr) {
        return false;
    }
    e->consumed = true;
    const auto parsed = parse_number<Num>(e->value);
    if (!parsed) {
        p.set_error_(std::format("Invalid {} value '{}'", key, e->value));
        return false;
    }
    out = *parsed;
    if (present) {
        *present = true;
    }
    return true;
}

template <class Num>
inline bool take_required_number(KeyValueParser &p, const std::string_view key, Num &out) {
    if (p.failed()) {
        return false;
    }
    KeyValueParser::Entry *e = p.find_(key);
    if (e == nullptr) {
        p.set_error_(std::format("Missing required key '{}'", key));
        return false;
    }
    e->consumed = true;
    const auto parsed = parse_number<Num>(e->value);
    if (!parsed) {
        p.set_error_(std::format("Invalid {} value '{}'", key, e->value));
        return false;
    }
    out = *parsed;
    return true;
}

inline bool take_required_identifier(KeyValueParser &p, const std::string_view key, const std::string_view what,
                                     std::string &out) {
    if (p.failed()) {
        return false;
    }
    KeyValueParser::Entry *e = p.find_(key);
    if (e == nullptr) {
        p.set_error_(std::format("Missing required key '{}'", key));
        return false;
    }
    e->consumed = true;
    if (!is_valid_identifier(e->value)) {
        p.set_error_(std::format("Invalid {} '{}'", what, e->value));
        return false;
    }
    out.assign(e->value);
    return true;
}

struct ParsedSceneHeader final {
    u32 version{};
    std::string units{};
};

[[nodiscard]] inline std::expected<ParsedSceneHeader, std::string>
parse_scene_header_record(const std::string_view payload) {
    KeyValueParser p{payload, "scene"};
    ParsedSceneHeader out{};

    take_required_number<u32>(p, "version", out.version);

    if (const auto units_value = p.take_raw("units")) {
        if (!is_valid_identifier(*units_value)) {
            p.fail(std::format("Invalid units value '{}'", *units_value));
        } else {
            out.units.assign(*units_value);
        }
    } else if (!p.failed()) {
        p.fail("Missing required key 'units'");
    }

    if (auto e = p.finish(); !e) {
        return std::unexpected(std::move(e.error()));
    }
    return out;
}

[[nodiscard]] inline std::expected<SceneFileShape, std::string> parse_shape_record(const std::string_view payload) {
    KeyValueParser p{payload, "shape"};
    SceneFileShape out{};

    take_required_identifier(p, "id", "shape id", out.id);

    ShapeKind kind = ShapeKind::sphere;
    bool has_kind = false;
    if (const auto kind_value = p.take_raw("kind")) {
        const auto parsed = parse_shape_kind(*kind_value);
        if (!parsed) {
            p.fail(std::format("Invalid shape kind '{}' (expected sphere|box)", *kind_value));
        } else {
            kind = *parsed;
            has_kind = true;
        }
    } else if (!p.failed()) {
        p.fail("Missing required key 'kind'");
    }

    bool has_r = false, has_hx = false, has_hy = false, has_hz = false;
    f32 r = 0.0f, hx = 0.0f, hy = 0.0f, hz = 0.0f;
    take_optional_number<f32>(p, "r", r, &has_r);
    take_optional_number<f32>(p, "hx", hx, &has_hx);
    take_optional_number<f32>(p, "hy", hy, &has_hy);
    take_optional_number<f32>(p, "hz", hz, &has_hz);

    if (auto e = p.finish(); !e) {
        return std::unexpected(std::move(e.error()));
    }
    if (!has_kind) {
        return std::unexpected("Missing required key 'kind'");
    }

    switch (kind) {
    case ShapeKind::sphere:
        if (!has_r) {
            return std::unexpected("Sphere shape missing required key 'r'");
        }
        if (has_hx || has_hy || has_hz) {
            return std::unexpected("Sphere shape does not allow hx/hy/hz");
        }
        if (!std::isfinite(r) || r <= 0.0f) {
            return std::unexpected(std::format("Invalid sphere radius {}", r));
        }
        out.shape = ShapeData::make_sphere(SphereShape{r});
        break;
    case ShapeKind::box:
        if (!has_hx || !has_hy || !has_hz) {
            return std::unexpected("Box shape missing required key(s) hx/hy/hz");
        }
        if (has_r) {
            return std::unexpected("Box shape does not allow key 'r'");
        }
        if (!std::isfinite(hx) || !std::isfinite(hy) || !std::isfinite(hz) || hx <= 0.0f || hy <= 0.0f || hz <= 0.0f) {
            return std::unexpected(std::format("Invalid box half extents [{}, {}, {}]", hx, hy, hz));
        }
        out.shape = ShapeData::make_box(BoxShape{Vec3{hx, hy, hz}});
        break;
    }

    return out;
}

[[nodiscard]] inline std::expected<SceneFileBody, std::string> parse_body_record(const std::string_view payload) {
    KeyValueParser p{payload, "body"};
    SceneFileBody out{};

    take_required_identifier(p, "id", "body id", out.id);
    take_required_identifier(p, "shape", "body shape id", out.shape_id);

    if (const auto motion_value = p.take_raw("motion")) {
        const auto parsed = parse_body_motion(*motion_value);
        if (!parsed) {
            p.fail(std::format("Invalid body motion '{}' (expected dynamic|static)", *motion_value));
        } else {
            out.motion = *parsed;
        }
    }

    take_optional_number<u32>(p, "material", out.material.value);
    take_optional_number<u32>(p, "mesh", out.mesh.value);

    bool has_px = false, has_py = false, has_pz = false;
    f32 px = 0.0f, py = 0.0f, pz = 0.0f;
    take_optional_number<f32>(p, "px", px, &has_px);
    take_optional_number<f32>(p, "py", py, &has_py);
    take_optional_number<f32>(p, "pz", pz, &has_pz);

    bool has_ox = false, has_oy = false, has_oz = false, has_ow = false;
    f32 ox = 0.0f, oy = 0.0f, oz = 0.0f, ow = 1.0f;
    take_optional_number<f32>(p, "ox", ox, &has_ox);
    take_optional_number<f32>(p, "oy", oy, &has_oy);
    take_optional_number<f32>(p, "oz", oz, &has_oz);
    take_optional_number<f32>(p, "ow", ow, &has_ow);

    bool has_vx = false, has_vy = false, has_vz = false;
    f32 vx = 0.0f, vy = 0.0f, vz = 0.0f;
    take_optional_number<f32>(p, "vx", vx, &has_vx);
    take_optional_number<f32>(p, "vy", vy, &has_vy);
    take_optional_number<f32>(p, "vz", vz, &has_vz);

    bool has_wx = false, has_wy = false, has_wz = false;
    f32 wx = 0.0f, wy = 0.0f, wz = 0.0f;
    take_optional_number<f32>(p, "wx", wx, &has_wx);
    take_optional_number<f32>(p, "wy", wy, &has_wy);
    take_optional_number<f32>(p, "wz", wz, &has_wz);

    if (auto e = p.finish(); !e) {
        return std::unexpected(std::move(e.error()));
    }

    if (!has_px || !has_py || !has_pz) {
        return std::unexpected("Body missing required key(s) px/py/pz");
    }
    out.position = Vec3{px, py, pz};

    const bool any_orientation = has_ox || has_oy || has_oz || has_ow;
    const bool full_orientation = has_ox && has_oy && has_oz && has_ow;
    if (any_orientation && !full_orientation) {
        return std::unexpected("Body orientation must specify all of ox/oy/oz/ow when present");
    }
    if (full_orientation) {
        out.orientation = Quat{ox, oy, oz, ow};
        if (!out.orientation.try_normalize()) {
            return std::unexpected("Body orientation is degenerate");
        }
    }

    const bool any_velocity = has_vx || has_vy || has_vz;
    const bool full_velocity = has_vx && has_vy && has_vz;
    if (any_velocity && !full_velocity) {
        return std::unexpected("Body velocity must specify all of vx/vy/vz when present");
    }
    if (full_velocity) {
        out.velocity = Vec3{vx, vy, vz};
    }

    const bool any_angular = has_wx || has_wy || has_wz;
    const bool full_angular = has_wx && has_wy && has_wz;
    if (any_angular && !full_angular) {
        return std::unexpected("Body angular velocity must specify all of wx/wy/wz when present");
    }
    if (full_angular) {
        out.angular_velocity = Vec3{wx, wy, wz};
    }

    if (!out.position.is_finite()) {
        return std::unexpected("Body position contains non-finite values");
    }
    if (!out.orientation.is_finite()) {
        return std::unexpected("Body orientation contains non-finite values");
    }
    if (!out.velocity.is_finite()) {
        return std::unexpected("Body velocity contains non-finite values");
    }
    if (!out.angular_velocity.is_finite()) {
        return std::unexpected("Body angular velocity contains non-finite values");
    }

    return out;
}

[[nodiscard]] inline std::expected<SceneFilePhysicsMaterial, std::string>
parse_physics_material_record(const std::string_view payload) {
    KeyValueParser p{payload, "physics_material"};
    SceneFilePhysicsMaterial out{};

    take_required_number<u32>(p, "id", out.id);

    bool has_restitution = false;
    take_optional_number<f32>(p, "restitution", out.material.restitution, &has_restitution);
    if (has_restitution &&
        (!std::isfinite(out.material.restitution) || out.material.restitution < 0.0f ||
         out.material.restitution > 1.0f)) {
        p.fail(std::format("Invalid restitution {} (expected [0, 1])", out.material.restitution));
    }

    bool has_friction = false;
    take_optional_number<f32>(p, "friction", out.material.friction, &has_friction);
    if (has_friction && (!std::isfinite(out.material.friction) || out.material.friction < 0.0f)) {
        p.fail(std::format("Invalid friction {} (expected >= 0)", out.material.friction));
    }

    bool has_density = false;
    take_optional_number<f32>(p, "density", out.density, &has_density);
    if (has_density && (!std::isfinite(out.density) || out.density <= 0.0f)) {
        p.fail(std::format("Invalid density {} (expected finite > 0)", out.density));
    }

    if (auto e = p.finish(); !e) {
        return std::unexpected(std::move(e.error()));
    }
    if (!has_restitution) {
        return std::unexpected("Missing required key 'restitution'");
    }
    if (!has_friction) {
        return std::unexpected("Missing required key 'friction'");
    }
    return out;
}

[[nodiscard]] inline std::expected<SceneFileConstraint, std::string>
parse_constraint_record(const std::string_view payload) {
    KeyValueParser p{payload, "constraint"};
    SceneFileConstraint out{};

    take_required_identifier(p, "id", "constraint id", out.id);

    if (const auto kind_value = p.take_raw("kind")) {
        if (*kind_value != "distance") {
            p.fail(std::format("Unknown constraint kind '{}' (expected 'distance')", *kind_value));
        }
    } else if (!p.failed()) {
        p.fail("Missing required key 'kind'");
    }

    take_required_identifier(p, "body_a", "body_a id", out.body_a_id);
    take_required_identifier(p, "body_b", "body_b id", out.body_b_id);

    bool has_ax = false, has_ay = false, has_az = false;
    bool has_bx = false, has_by = false, has_bz = false;
    f32 ax = 0.0f, ay = 0.0f, az = 0.0f;
    f32 bx = 0.0f, by = 0.0f, bz = 0.0f;
    take_optional_number<f32>(p, "ax", ax, &has_ax);
    take_optional_number<f32>(p, "ay", ay, &has_ay);
    take_optional_number<f32>(p, "az", az, &has_az);
    take_optional_number<f32>(p, "bx", bx, &has_bx);
    take_optional_number<f32>(p, "by", by, &has_by);
    take_optional_number<f32>(p, "bz", bz, &has_bz);

    bool has_rest = false;
    take_optional_number<f32>(p, "rest", out.rest_length, &has_rest);
    if (has_rest && (!std::isfinite(out.rest_length) || out.rest_length < 0.0f)) {
        p.fail(std::format("Invalid rest_length {} (expected finite >= 0)", out.rest_length));
    }

    bool has_compliance = false;
    take_optional_number<f32>(p, "compliance", out.compliance, &has_compliance);
    if (has_compliance && (!std::isfinite(out.compliance) || out.compliance < 0.0f)) {
        p.fail(std::format("Invalid compliance {} (expected finite >= 0)", out.compliance));
    }

    if (auto e = p.finish(); !e) {
        return std::unexpected(std::move(e.error()));
    }
    if (!has_ax || !has_ay || !has_az) {
        return std::unexpected("Constraint missing required anchor A fields (ax/ay/az)");
    }
    if (!has_bx || !has_by || !has_bz) {
        return std::unexpected("Constraint missing required anchor B fields (bx/by/bz)");
    }
    if (!has_rest) {
        return std::unexpected("Missing required key 'rest'");
    }
    if (!has_compliance) {
        return std::unexpected("Missing required key 'compliance'");
    }

    out.anchor_a = Vec3{ax, ay, az};
    out.anchor_b = Vec3{bx, by, bz};
    return out;
}

} // namespace detail

struct SceneFile final {
    u32 version{kSceneFileVersion};
    std::string units{std::string{kSceneFileUnitsMeters}};
    std::vector<SceneFilePhysicsMaterial> physics_materials{};
    std::vector<SceneFileShape> shapes{};
    std::vector<SceneFileBody> bodies{};
    std::vector<SceneFileConstraint> constraints{};

    void clear() {
        version = kSceneFileVersion;
        units.assign(kSceneFileUnitsMeters);
        physics_materials.clear();
        shapes.clear();
        bodies.clear();
        constraints.clear();
    }

    void reserve(const u32 shape_count, const u32 body_count, const u32 physics_material_count = 0u,
                 const u32 constraint_count = 0u) {
        physics_materials.reserve(physics_material_count);
        shapes.reserve(shape_count);
        bodies.reserve(body_count);
        constraints.reserve(constraint_count);
    }

    [[nodiscard]] bool empty() const noexcept {
        return physics_materials.empty() && shapes.empty() && bodies.empty() && constraints.empty();
    }

    [[nodiscard]] std::expected<void, SceneFileError>
    validate(const std::filesystem::path &source_path = std::filesystem::path{}) const {
        auto error = [&source_path](const std::string message) -> std::expected<void, SceneFileError> {
            return std::unexpected(SceneFileError{.path = source_path, .line = 0u, .message = message});
        };

        if (version != kSceneFileVersion) {
            return error(std::format("Unsupported scene file version {} (expected {})", version, kSceneFileVersion));
        }
        if (units != kSceneFileUnitsMeters) {
            return error(
                std::format("Unsupported scene file units '{}' (expected '{}')", units, kSceneFileUnitsMeters));
        }

        std::unordered_set<u32> physics_material_ids{};
        physics_material_ids.reserve(physics_materials.size() * 2u + 1u);
        for (u32 i = 0; i < physics_materials.size(); ++i) {
            const SceneFilePhysicsMaterial &mat = physics_materials[i];
            if (!physics_material_ids.insert(mat.id).second) {
                return error(std::format("Duplicate physics_material id {}", mat.id));
            }
            if (!std::isfinite(mat.material.restitution) || mat.material.restitution < 0.0f ||
                mat.material.restitution > 1.0f) {
                return error(std::format("physics_material id={} has invalid restitution {} (expected [0, 1])", mat.id,
                                         mat.material.restitution));
            }
            if (!std::isfinite(mat.material.friction) || mat.material.friction < 0.0f) {
                return error(std::format("physics_material id={} has invalid friction {} (expected >= 0)", mat.id,
                                         mat.material.friction));
            }
            if (!std::isfinite(mat.density) || mat.density <= 0.0f) {
                return error(std::format("physics_material id={} has invalid density {} (expected finite > 0)", mat.id,
                                         mat.density));
            }
        }

        std::unordered_set<std::string_view> shape_ids{};
        shape_ids.reserve(shapes.size() * 2u + 1u);
        for (u32 i = 0; i < shapes.size(); ++i) {
            const SceneFileShape &shape = shapes[i];
            if (shape.id.empty()) {
                return error(std::format("shape[{}] has empty id", i));
            }
            if (!detail::is_valid_identifier(shape.id)) {
                return error(std::format("shape[{}] has invalid id '{}'", i, shape.id));
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
            if (!detail::is_valid_identifier(body.id)) {
                return error(std::format("body[{}] has invalid id '{}'", i, body.id));
            }
            if (!body_ids.insert(body.id).second) {
                return error(std::format("Duplicate body id '{}'", body.id));
            }
            if (body.shape_id.empty()) {
                return error(std::format("body '{}' has empty shape id", body.id));
            }
            if (!detail::is_valid_identifier(body.shape_id)) {
                return error(std::format("body '{}' has invalid shape id '{}'", body.id, body.shape_id));
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

        std::unordered_set<std::string_view> constraint_ids{};
        constraint_ids.reserve(constraints.size() * 2u + 1u);
        for (u32 i = 0; i < constraints.size(); ++i) {
            const SceneFileConstraint &c = constraints[i];
            if (c.id.empty()) {
                return error(std::format("constraint[{}] has empty id", i));
            }
            if (!detail::is_valid_identifier(c.id)) {
                return error(std::format("constraint[{}] has invalid id '{}'", i, c.id));
            }
            if (!constraint_ids.insert(c.id).second) {
                return error(std::format("Duplicate constraint id '{}'", c.id));
            }
            if (c.body_a_id.empty() || !detail::is_valid_identifier(c.body_a_id)) {
                return error(std::format("constraint '{}' has invalid body_a id '{}'", c.id, c.body_a_id));
            }
            if (!body_ids.contains(c.body_a_id)) {
                return error(std::format("constraint '{}' references unknown body_a '{}'", c.id, c.body_a_id));
            }
            if (c.body_b_id.empty() || !detail::is_valid_identifier(c.body_b_id)) {
                return error(std::format("constraint '{}' has invalid body_b id '{}'", c.id, c.body_b_id));
            }
            if (!body_ids.contains(c.body_b_id)) {
                return error(std::format("constraint '{}' references unknown body_b '{}'", c.id, c.body_b_id));
            }
            if (!c.anchor_a.is_finite()) {
                return error(std::format("constraint '{}' has non-finite anchor_a", c.id));
            }
            if (!c.anchor_b.is_finite()) {
                return error(std::format("constraint '{}' has non-finite anchor_b", c.id));
            }
            if (!std::isfinite(c.rest_length) || c.rest_length < 0.0f) {
                return error(std::format("constraint '{}' has invalid rest_length {} (expected finite >= 0)", c.id,
                                         c.rest_length));
            }
            if (!std::isfinite(c.compliance) || c.compliance < 0.0f) {
                return error(std::format("constraint '{}' has invalid compliance {} (expected finite >= 0)", c.id,
                                         c.compliance));
            }
        }

        return {};
    }

    [[nodiscard]] static std::expected<SceneFile, SceneFileError> load(const std::filesystem::path &path,
                                                                       const SceneFileLoadOptions &options = {}) {
        ZoneScopedN("SceneFile load");

        auto error = [&path](const u32 line, std::string message) -> std::expected<SceneFile, SceneFileError> {
            return std::unexpected(SceneFileError{
                .path = path,
                .line = line,
                .message = std::move(message),
            });
        };

        std::ifstream file{path, std::ios::binary};
        if (!file.is_open()) {
            return error(0u, "Unable to open scene file for reading");
        }

        SceneFile out{};
        out.clear();

        u32 scene_line = 0u;
        u32 line_no = 0u;
        std::unordered_map<u32, u32> physics_material_line_by_id{};
        std::unordered_map<std::string, u32> shape_line_by_id{};
        std::unordered_map<std::string, u32> body_line_by_id{};
        std::vector<u32> body_lines{};
        std::unordered_map<std::string, u32> constraint_line_by_id{};
        std::vector<u32> constraint_lines{};
        std::string line{};
        line.reserve(256);

        while (std::getline(file, line)) {
            ++line_no;
            std::string_view line_view{line};
            if (!line_view.empty() && line_view.back() == '\r') {
                line_view.remove_suffix(1);
            }
            if (line_no == 1u && line_view.starts_with(detail::kUtf8Bom)) {
                line_view.remove_prefix(detail::kUtf8Bom.size());
            }
            line_view = detail::trim(detail::strip_comment(line_view));
            if (line_view.empty()) {
                continue;
            }

            detail::TokenCursor cursor{line_view};
            const auto record_token = cursor.next();
            if (!record_token) {
                continue;
            }

            const std::string_view record = *record_token;
            const std::string_view payload = cursor.rest;
            if (record == "scene") {
                if (scene_line != 0u) {
                    return error(line_no,
                                 std::format("Duplicate scene record (first declared on line {})", scene_line));
                }
                auto parsed = detail::parse_scene_header_record(payload);
                if (!parsed) {
                    return error(line_no, std::move(parsed.error()));
                }
                scene_line = line_no;
                out.version = parsed->version;
                out.units = std::move(parsed->units);
                continue;
            }

            if (record == "physics_material") {
                auto parsed = detail::parse_physics_material_record(payload);
                if (!parsed) {
                    return error(line_no, std::move(parsed.error()));
                }
                auto [it, inserted] = physics_material_line_by_id.emplace(parsed->id, line_no);
                if (!inserted) {
                    return error(line_no, std::format("Duplicate physics_material id {} (first declared on line {})",
                                                      parsed->id, it->second));
                }
                out.physics_materials.push_back(std::move(*parsed));
                continue;
            }

            if (record == "shape") {
                auto parsed = detail::parse_shape_record(payload);
                if (!parsed) {
                    return error(line_no, std::move(parsed.error()));
                }
                auto [it, inserted] = shape_line_by_id.emplace(parsed->id, line_no);
                if (!inserted) {
                    return error(line_no, std::format("Duplicate shape id '{}' (first declared on line {})", parsed->id,
                                                      it->second));
                }
                out.shapes.push_back(std::move(*parsed));
                continue;
            }

            if (record == "body") {
                auto parsed = detail::parse_body_record(payload);
                if (!parsed) {
                    return error(line_no, std::move(parsed.error()));
                }
                auto [it, inserted] = body_line_by_id.emplace(parsed->id, line_no);
                if (!inserted) {
                    return error(line_no, std::format("Duplicate body id '{}' (first declared on line {})", parsed->id,
                                                      it->second));
                }
                body_lines.push_back(line_no);
                out.bodies.push_back(std::move(*parsed));
                continue;
            }

            if (record == "constraint") {
                auto parsed = detail::parse_constraint_record(payload);
                if (!parsed) {
                    return error(line_no, std::move(parsed.error()));
                }
                auto [it, inserted] = constraint_line_by_id.emplace(parsed->id, line_no);
                if (!inserted) {
                    return error(line_no, std::format("Duplicate constraint id '{}' (first declared on line {})",
                                                      parsed->id, it->second));
                }
                constraint_lines.push_back(line_no);
                out.constraints.push_back(std::move(*parsed));
                continue;
            }

            return error(line_no, std::format("Unknown record '{}'", record));
        }

        if (file.bad()) {
            return error(line_no, "I/O error while reading scene file");
        }
        if (scene_line == 0u) {
            return error(0u, "Missing required 'scene' record");
        }

        for (usize i = 0; i < out.bodies.size(); ++i) {
            const SceneFileBody &body = out.bodies[i];
            if (!shape_line_by_id.contains(body.shape_id)) {
                return error(body_lines[i],
                             std::format("body '{}' references unknown shape '{}'", body.id, body.shape_id));
            }
        }

        for (usize i = 0; i < out.constraints.size(); ++i) {
            const SceneFileConstraint &c = out.constraints[i];
            if (!body_line_by_id.contains(c.body_a_id)) {
                return error(constraint_lines[i],
                             std::format("constraint '{}' references unknown body_a '{}'", c.id, c.body_a_id));
            }
            if (!body_line_by_id.contains(c.body_b_id)) {
                return error(constraint_lines[i],
                             std::format("constraint '{}' references unknown body_b '{}'", c.id, c.body_b_id));
            }
        }

        if (options.validate_after_parse) {
            auto valid = out.validate(path);
            if (!valid) {
                return std::unexpected(valid.error());
            }
        }

        TracyPlot("scenefile_lines", static_cast<i64>(line_no));
        TracyPlot("scenefile_physics_materials", static_cast<i64>(out.physics_materials.size()));
        TracyPlot("scenefile_shapes", static_cast<i64>(out.shapes.size()));
        TracyPlot("scenefile_bodies", static_cast<i64>(out.bodies.size()));
        TracyPlot("scenefile_constraints", static_cast<i64>(out.constraints.size()));
        return out;
    }

    [[nodiscard]] std::expected<void, SceneFileError> save(const std::filesystem::path &path,
                                                           const SceneFileSaveOptions &options = {}) const {
        ZoneScopedN("SceneFile save");

        auto error = [&path](std::string message) -> std::expected<void, SceneFileError> {
            return std::unexpected(SceneFileError{
                .path = path,
                .line = 0u,
                .message = std::move(message),
            });
        };

        if (options.validate_before_write) {
            auto valid = validate(path);
            if (!valid) {
                return std::unexpected(valid.error());
            }
        }

        if (const std::filesystem::path parent = path.parent_path(); !parent.empty()) {
            std::error_code ec{};
            std::filesystem::create_directories(parent, ec);
            if (ec) {
                return error(std::format("Unable to create parent directory '{}': {}", parent.string(), ec.message()));
            }
        }

        std::string out_text{};
        out_text.reserve(256 + physics_materials.size() * 64 + shapes.size() * 64 + bodies.size() * 220 +
                         constraints.size() * 160);
        out_text.append("# javelin scene file (.jvscene)\n");
        out_text.append("# schema=v1 units=m one-record-per-line key=value\n");

        std::string line{};
        line.reserve(256);

        line.clear();
        detail::append_token(line, "scene");
        detail::append_key_value_u32(line, "version", version);
        detail::append_key_value(line, "units", units);
        out_text.append(line);
        out_text.push_back('\n');

        // physics_materials sorted by id for stable, diffable output.
        if (!physics_materials.empty()) {
            std::vector<usize> sorted_mat_indices(physics_materials.size());
            std::iota(sorted_mat_indices.begin(), sorted_mat_indices.end(), static_cast<usize>(0));
            std::sort(sorted_mat_indices.begin(), sorted_mat_indices.end(),
                      [&](const usize a, const usize b) { return physics_materials[a].id < physics_materials[b].id; });

            out_text.push_back('\n');
            out_text.append("# physics materials\n");
            for (const usize idx : sorted_mat_indices) {
                const SceneFilePhysicsMaterial &mat = physics_materials[idx];
                line.clear();
                detail::append_token(line, "physics_material");
                detail::append_key_value_u32(line, "id", mat.id);
                detail::append_key_value_f32(line, "restitution", mat.material.restitution);
                detail::append_key_value_f32(line, "friction", mat.material.friction);
                if (mat.density != kDefaultMaterialDensity) {
                    detail::append_key_value_f32(line, "density", mat.density);
                }
                out_text.append(line);
                out_text.push_back('\n');
            }
        }

        const std::vector<usize> sorted_shape_indices = detail::sorted_indices_by_id(shapes);
        if (!sorted_shape_indices.empty()) {
            out_text.push_back('\n');
            out_text.append("# shapes\n");
            for (const usize idx : sorted_shape_indices) {
                const SceneFileShape &shape = shapes[idx];
                line.clear();
                detail::append_token(line, "shape");
                detail::append_key_value(line, "id", shape.id);
                detail::append_key_value(line, "kind", to_string(shape.shape.kind));
                switch (shape.shape.kind) {
                case ShapeKind::sphere:
                    detail::append_key_value_f32(line, "r", shape_sphere(shape.shape).radius);
                    break;
                case ShapeKind::box: {
                    const Vec3 half_extents = shape_box(shape.shape).half_extents;
                    detail::append_key_value_f32(line, "hx", half_extents.x);
                    detail::append_key_value_f32(line, "hy", half_extents.y);
                    detail::append_key_value_f32(line, "hz", half_extents.z);
                } break;
                }
                out_text.append(line);
                out_text.push_back('\n');
            }
        }

        const std::vector<usize> sorted_body_indices = detail::sorted_indices_by_id(bodies);
        if (!sorted_body_indices.empty()) {
            out_text.push_back('\n');
            out_text.append("# bodies\n");
            for (const usize idx : sorted_body_indices) {
                const SceneFileBody &body = bodies[idx];
                line.clear();
                detail::append_token(line, "body");
                detail::append_key_value(line, "id", body.id);
                detail::append_key_value(line, "shape", body.shape_id);
                detail::append_key_value(line, "motion", to_string(body.motion));
                detail::append_key_value_u32(line, "material", body.material.value);
                detail::append_key_value_u32(line, "mesh", body.mesh.value);

                detail::append_key_value_f32(line, "px", body.position.x);
                detail::append_key_value_f32(line, "py", body.position.y);
                detail::append_key_value_f32(line, "pz", body.position.z);

                const Quat orientation = detail::canonicalize_quaternion_for_text(body.orientation);
                detail::append_key_value_f32(line, "ox", orientation.x);
                detail::append_key_value_f32(line, "oy", orientation.y);
                detail::append_key_value_f32(line, "oz", orientation.z);
                detail::append_key_value_f32(line, "ow", orientation.w);

                detail::append_key_value_f32(line, "vx", body.velocity.x);
                detail::append_key_value_f32(line, "vy", body.velocity.y);
                detail::append_key_value_f32(line, "vz", body.velocity.z);
                detail::append_key_value_f32(line, "wx", body.angular_velocity.x);
                detail::append_key_value_f32(line, "wy", body.angular_velocity.y);
                detail::append_key_value_f32(line, "wz", body.angular_velocity.z);

                out_text.append(line);
                out_text.push_back('\n');
            }
        }

        const std::vector<usize> sorted_constraint_indices = detail::sorted_indices_by_id(constraints);
        if (!sorted_constraint_indices.empty()) {
            out_text.push_back('\n');
            out_text.append("# constraints\n");
            for (const usize idx : sorted_constraint_indices) {
                const SceneFileConstraint &c = constraints[idx];
                line.clear();
                detail::append_token(line, "constraint");
                detail::append_key_value(line, "id", c.id);
                detail::append_key_value(line, "kind", "distance");
                detail::append_key_value(line, "body_a", c.body_a_id);
                detail::append_key_value(line, "body_b", c.body_b_id);
                detail::append_key_value_f32(line, "ax", c.anchor_a.x);
                detail::append_key_value_f32(line, "ay", c.anchor_a.y);
                detail::append_key_value_f32(line, "az", c.anchor_a.z);
                detail::append_key_value_f32(line, "bx", c.anchor_b.x);
                detail::append_key_value_f32(line, "by", c.anchor_b.y);
                detail::append_key_value_f32(line, "bz", c.anchor_b.z);
                detail::append_key_value_f32(line, "rest", c.rest_length);
                detail::append_key_value_f32(line, "compliance", c.compliance);
                out_text.append(line);
                out_text.push_back('\n');
            }
        }

        std::ofstream file{path, std::ios::binary | std::ios::trunc};
        if (!file.is_open()) {
            return error("Unable to open scene file for writing");
        }
        file.write(out_text.data(), static_cast<std::streamsize>(out_text.size()));
        if (!file) {
            return error("I/O error while writing scene file");
        }
        file.flush();
        if (!file) {
            return error("I/O error while finalizing scene file write");
        }

        TracyPlot("scenefile_write_bytes", static_cast<i64>(out_text.size()));
        TracyPlot("scenefile_write_physics_materials", static_cast<i64>(physics_materials.size()));
        TracyPlot("scenefile_write_shapes", static_cast<i64>(shapes.size()));
        TracyPlot("scenefile_write_bodies", static_cast<i64>(bodies.size()));
        TracyPlot("scenefile_write_constraints", static_cast<i64>(constraints.size()));
        return {};
    }
};

} // namespace javelin
