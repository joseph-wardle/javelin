export module javelin.scene.bodies;

import std;

import javelin.core.types;
import javelin.math;
import javelin.physics.constraint_types;
import javelin.scene.entity;
import javelin.scene.pose_channel;
import javelin.scene.shapes;

export namespace javelin {

namespace detail {

// Shared SoA accessor for the Scene body arrays. Mutability of the simulation
// state spans (position/velocity/orientation/angular_velocity, sleep state,
// poses ref) is selected by the template parameter:
//   Bodies     = writable, used by the physics tick pipeline to step the sim.
//   BodiesRead = read-only, used by render / UI / camera framing.
// Authored data (identity, shapes, materials, mass, constraints) is always
// exposed as std::span<const T> regardless of mutability.
template <bool Writable>
struct BodiesT final {
  private:
    template <class T> using S = std::conditional_t<Writable, std::span<T>, std::span<const T>>;
    template <class T> using R = std::conditional_t<Writable, T &, const T &>;

  public:
    u32 count{};

    // identity (read)
    std::span<const u32> generation;
    std::span<const u8> alive;

    // authored / static (read)
    std::span<const ShapeKind> shape_kind;
    std::span<const u32> shape_index;       // per-body index into the shape pool
    std::span<const ShapeData> shapes;      // shape pool, indexed by shape_index[body]
    std::span<const MaterialId> material;   // per-body index into the physics material pool
    std::span<const f32> material_restitution; // physics material pool
    std::span<const f32> material_friction;    // physics material pool
    std::span<const MeshId> mesh;
    std::span<const f32> inv_mass;
    std::span<const Vec3> inv_inertia;

    // simulation state (read/write when Writable, otherwise read-only)
    S<Vec3> position;
    S<Vec3> velocity;
    S<Quat> orientation;
    S<Vec3> angular_velocity;

    // sleep state (read/write when Writable, otherwise read-only)
    // sleep_timer: consecutive ticks this body's speed has been below the sleep threshold.
    // asleep:      0 = awake, 1 = asleep; set once sleep_timer reaches the threshold.
    S<u32> sleep_timer;
    S<u8> asleep;

    // constraints (read)
    // Authored distance constraints between body pairs; solver reads these each tick.
    std::span<const DistanceConstraint> constraints;

    // pose channel (read/write when Writable, otherwise read-only)
    R<PoseChannel> poses;
};

} // namespace detail

using Bodies = detail::BodiesT<true>;
using BodiesRead = detail::BodiesT<false>;

} // namespace javelin
