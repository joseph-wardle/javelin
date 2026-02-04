export module javelin.scene;

import std;

import javelin.core.logging;
import javelin.core.types;
import javelin.math.constants;
import javelin.math.quat;
import javelin.math.vec3;
import javelin.scene.entity;
import javelin.scene.physics_view;
import javelin.scene.pose_channel;
import javelin.scene.render_view;
import javelin.scene.shapes;

export namespace javelin {
namespace detail {
[[nodiscard]] constexpr u32 hash_u32(u32 x) noexcept {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

[[nodiscard]] constexpr f32 hash_to_unit(u32 x) noexcept {
    const u32 h = hash_u32(x);
    return static_cast<f32>(h & 0x00FFFFFFu) / static_cast<f32>(0x01000000u);
}

struct SpawnSettings final {
    u32 count{};
    f32 pile_radius{};
    f32 radius_min{};
    f32 radius_max{};
    f32 height_min{};
    f32 height_max{};
};

inline constexpr SpawnSettings kSpawnSettings{
    .count = 512,
    .pile_radius = 2.5f,
    .radius_min = 0.25f,
    .radius_max = 0.6f,
    .height_min = 6.0f,
    .height_max = 300.0f,
};

[[nodiscard]] constexpr u32 spawn_count() noexcept { return kSpawnSettings.count; }

[[nodiscard]] inline f32 spawn_radius(const u32 idx) noexcept {
    const u32 seed = idx * 747796405u + 2891336453u;
    const f32 rand_radius = hash_to_unit(seed);
    return kSpawnSettings.radius_min + rand_radius * (kSpawnSettings.radius_max - kSpawnSettings.radius_min);
}

[[nodiscard]] inline Vec3 spawn_position(const u32 idx) noexcept {
    const u32 seed = idx * 747796405u + 2891336453u;
    const f32 rand_height = hash_to_unit(seed ^ 0x9e3779b9u);
    const f32 rand_r = hash_to_unit(seed ^ 0x85ebca6bu);
    const f32 rand_angle = hash_to_unit(seed ^ 0xc2b2ae35u);

    const f32 height =
        kSpawnSettings.height_min + rand_height * (kSpawnSettings.height_max - kSpawnSettings.height_min);
    const f32 radius = std::sqrt(rand_r) * kSpawnSettings.pile_radius;
    const f32 angle = rand_angle * TWO_PI;
    const f32 px = std::cos(angle) * radius;
    const f32 pz = std::sin(angle) * radius;

    return Vec3{px, height, pz};
}

[[nodiscard]] inline f32 spawn_inv_mass(const f32 radius) noexcept {
    const f32 mass = radius * radius * radius;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}

[[nodiscard]] inline Vec3 sphere_inv_inertia(const f32 radius, const f32 inv_mass) noexcept {
    if (inv_mass <= 0.0f) {
        return Vec3{};
    }
    const f32 r2 = radius * radius;
    if (r2 <= 1e-6f) {
        return Vec3{};
    }
    const f32 inv_inertia = 2.5f * inv_mass / r2;
    return Vec3{inv_inertia};
}

[[nodiscard]] inline Vec3 box_inv_inertia(const Vec3 half_extents, const f32 inv_mass) noexcept {
    if (inv_mass <= 0.0f) {
        return Vec3{};
    }
    const f32 hx2 = half_extents.x * half_extents.x;
    const f32 hy2 = half_extents.y * half_extents.y;
    const f32 hz2 = half_extents.z * half_extents.z;
    const f32 denom_x = hy2 + hz2;
    const f32 denom_y = hx2 + hz2;
    const f32 denom_z = hx2 + hy2;
    const f32 inv_ix = (denom_x > 1e-6f) ? (3.0f * inv_mass / denom_x) : 0.0f;
    const f32 inv_iy = (denom_y > 1e-6f) ? (3.0f * inv_mass / denom_y) : 0.0f;
    const f32 inv_iz = (denom_z > 1e-6f) ? (3.0f * inv_mass / denom_z) : 0.0f;
    return Vec3{inv_ix, inv_iy, inv_iz};
}

[[nodiscard]] inline Vec3 shape_inv_inertia(const ShapeKind kind, const ShapeData &shape,
                                            const f32 inv_mass) noexcept {
#ifndef NDEBUG
    if (kind != shape.kind) {
        log::error(scene, "Shape kind mismatch during inertia compute");
        std::terminate();
    }
#endif
    switch (kind) {
    case ShapeKind::sphere:
        return sphere_inv_inertia(shape_sphere(shape).radius, inv_mass);
    case ShapeKind::box:
        return box_inv_inertia(shape_box(shape).half_extents, inv_mass);
    }
    return Vec3{};
}
} // namespace detail

struct Scene final {
    void reserve(u32 capacity) {
        capacity_ = capacity;

        // identity
        generation_.resize(capacity_);
        alive_.resize(capacity_, 0u);

        // authored/static
        shape_kind_.resize(capacity_, ShapeKind::sphere);
        shape_index_.resize(capacity_);
        shapes_.reserve(capacity_);
        material_.resize(capacity_);
        mesh_.resize(capacity_);
        inv_mass_.resize(capacity_, 1.0f);
        inv_inertia_.resize(capacity_);

        // simulation
        position_.resize(capacity_);
        velocity_.resize(capacity_);
        orientation_.resize(capacity_);
        angular_velocity_.resize(capacity_);

        poses_.reserve(capacity_);
    }

    [[nodiscard]] PhysicsView physics_view() noexcept {
        return PhysicsView{
            .count = count_,
            .generation = std::span<const u32>{generation_.data(), count_},
            .alive = std::span<const u8>{alive_.data(), count_},
            .shape_kind = std::span<const ShapeKind>{shape_kind_.data(), count_},
            .shape_index = std::span<const u32>{shape_index_.data(), count_},
            .shapes = std::span<const ShapeData>{shapes_.data(), shapes_.size()},
            .material = std::span<const MaterialId>{material_.data(), count_},
            .mesh = std::span<const MeshId>{mesh_.data(), count_},
            .inv_mass = std::span<const f32>{inv_mass_.data(), count_},
            .inv_inertia = std::span<const Vec3>{inv_inertia_.data(), count_},
            .position = std::span<Vec3>{position_.data(), count_},
            .velocity = std::span<Vec3>{velocity_.data(), count_},
            .orientation = std::span<Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<Vec3>{angular_velocity_.data(), count_},
            .poses = poses_,
        };
    }

    [[nodiscard]] RenderView render_view() const noexcept {
        return RenderView{
            .generation = std::span<const u32>{generation_.data(), count_},
            .alive = std::span<const u8>{alive_.data(), count_},
            .shape_kind = std::span<const ShapeKind>{shape_kind_.data(), count_},
            .shape_index = std::span<const u32>{shape_index_.data(), count_},
            .shapes = std::span<const ShapeData>{shapes_.data(), shapes_.size()},
            .material = std::span<const MaterialId>{material_.data(), count_},
            .mesh = std::span<const MeshId>{mesh_.data(), count_},
            .inv_mass = std::span<const f32>{inv_mass_.data(), count_},
            .inv_inertia = std::span<const Vec3>{inv_inertia_.data(), count_},
            .position = std::span<const Vec3>{position_.data(), count_},
            .velocity = std::span<const Vec3>{velocity_.data(), count_},
            .orientation = std::span<const Quat>{orientation_.data(), count_},
            .angular_velocity = std::span<const Vec3>{angular_velocity_.data(), count_},
            .poses = poses_,
        };
    }

    // Render reads this each frame for interpolation.
    [[nodiscard]] PoseSnapshot pose_snapshot() const noexcept { return poses_.snapshot(); }

    // Physics calls this after stepping to publish.
    void publish_poses_from_sim() noexcept {
        auto out = poses_.write_pose(count_);
        for (u32 i = 0; i < count_; ++i) {
            out.positions[i] = position_[i];
            out.orientations[i] = orientation_[i];
        }
        poses_.publish();
    }

    void reset_simulation() noexcept {
        for (u32 i = 0; i < count_; ++i) {
            position_[i] = detail::spawn_position(i);
            velocity_[i] = Vec3{};
            orientation_[i] = Quat::identity();
            angular_velocity_[i] = Vec3{};
        }
    }

    static Scene load_scene_from_disk(std::filesystem::path scene_path) {
        log::info(scene, "Loading scene from disk: {}", scene_path.string());

        // TEMP: procedural sphere cloud until real scene data/asset loading is in place.
        Scene out{};
        constexpr u32 kSphereCount = detail::spawn_count();

        out.reserve(kSphereCount);
        out.count_ = kSphereCount;
        out.shapes_.clear();
        out.shapes_.reserve(out.count_);

        for (u32 idx = 0; idx < out.count_; ++idx) {
            const f32 radius = detail::spawn_radius(idx);
            const f32 inv_mass = detail::spawn_inv_mass(radius);
            const Vec3 position = detail::spawn_position(idx);

            out.alive_[idx] = 1u;
            out.generation_[idx] = 1;
            out.shape_kind_[idx] = ShapeKind::sphere;
            out.shape_index_[idx] = static_cast<u32>(out.shapes_.size());
            out.shapes_.push_back(ShapeData::make_sphere(SphereShape{radius}));
            out.material_[idx] = MaterialId{0};
            out.mesh_[idx] = MeshId{0};
            out.inv_mass_[idx] = inv_mass;
            out.inv_inertia_[idx] = detail::shape_inv_inertia(out.shape_kind_[idx],
                                                              out.shapes_[out.shape_index_[idx]], inv_mass);
            out.position_[idx] = position;
            out.velocity_[idx] = Vec3{};
            out.orientation_[idx] = Quat::identity();
            out.angular_velocity_[idx] = Vec3{};
        }

        out.publish_poses_from_sim();
        log::info(scene, "Loaded {} spheres (pile)", out.count_);
        log::info(scene, "Test scene params: radius=[{}..{}], height=[{}..{}], pile_radius={}",
                  detail::kSpawnSettings.radius_min, detail::kSpawnSettings.radius_max,
                  detail::kSpawnSettings.height_min, detail::kSpawnSettings.height_max,
                  detail::kSpawnSettings.pile_radius);
        return out;
    }

  private:
    u32 capacity_{0};
    u32 count_{0};

    // identity (kept for future spawn/despawn; can be minimal in v1)
    std::vector<u32> generation_{};
    std::vector<u8> alive_{};

    // authored/static
    std::vector<ShapeKind> shape_kind_{};
    std::vector<u32> shape_index_{};
    std::vector<ShapeData> shapes_{};
    std::vector<MaterialId> material_{};
    std::vector<MeshId> mesh_{};
    std::vector<f32> inv_mass_{};
    std::vector<Vec3> inv_inertia_{};

    // simulation state (physics-owned)
    std::vector<Vec3> position_{};
    std::vector<Vec3> velocity_{};
    std::vector<Quat> orientation_{};
    std::vector<Vec3> angular_velocity_{};

    // presentation channel (physics publishes, render reads)
    PoseChannel poses_{};
};
} // namespace javelin
