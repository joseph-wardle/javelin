export module javelin.scene.shapes;

import std;

import javelin.core.types;

export namespace javelin {
enum struct ShapeKind : u8 { sphere /*, box, capsule...*/ };

struct SphereShape final {
    f32 radius{0.5f};
};
} // namespace javelin
