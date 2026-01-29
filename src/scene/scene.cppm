export module javelin.scene;

import std;

import javelin.core.logging;
import javelin.core.types;
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
    u32 grid_dim{};
    f32 spacing{};
    f32 jitter{};
    f32 radius_min{};
    f32 radius_max{};
    f32 height_min{};
    f32 height_max{};
};

inline constexpr SpawnSettings kSpawnSettings{
    .grid_dim = 10,
    .spacing = 1.0f,
    .jitter = 0.35f,
    .radius_min = 0.25f,
    .radius_max = 0.6f,
    .height_min = 6.0f,
    .height_max = 14.0f,
};

[[nodiscard]] constexpr u32 spawn_count() noexcept { return kSpawnSettings.grid_dim * kSpawnSettings.grid_dim; }

[[nodiscard]] inline f32 spawn_radius(const u32 idx) noexcept {
    const u32 seed = idx * 747796405u + 2891336453u;
    const f32 rand_radius = hash_to_unit(seed);
    return kSpawnSettings.radius_min + rand_radius * (kSpawnSettings.radius_max - kSpawnSettings.radius_min);
}

[[nodiscard]] inline Vec3 spawn_position(const u32 idx) noexcept {
    const u32 seed = idx * 747796405u + 2891336453u;
    const f32 rand_height = hash_to_unit(seed ^ 0x9e3779b9u);
    const f32 rand_x = hash_to_unit(seed ^ 0x85ebca6bu);
    const f32 rand_z = hash_to_unit(seed ^ 0xc2b2ae35u);

    const u32 grid_dim = kSpawnSettings.grid_dim;
    const f32 spacing = kSpawnSettings.spacing;
    const f32 half_span = 0.5f * static_cast<f32>(grid_dim - 1) * spacing;
    const u32 x = idx % grid_dim;
    const u32 z = idx / grid_dim;

    const f32 height =
        kSpawnSettings.height_min + rand_height * (kSpawnSettings.height_max - kSpawnSettings.height_min);
    const f32 jitter_x = (rand_x * 2.0f - 1.0f) * kSpawnSettings.jitter;
    const f32 jitter_z = (rand_z * 2.0f - 1.0f) * kSpawnSettings.jitter;

    const f32 px = static_cast<f32>(x) * spacing - half_span + jitter_x;
    const f32 pz = static_cast<f32>(z) * spacing - half_span + jitter_z;

    return Vec3{px, height, pz};
}

[[nodiscard]] inline f32 spawn_inv_mass(const f32 radius) noexcept {
    const f32 mass = radius * radius * radius;
    return (mass > 1e-6f) ? (1.0f / mass) : 0.0f;
}
} // namespace detail

struct Scene final {
    void reserve(u32 capacity) {
        capacity_ = capacity;

        // identity
        generation_.resize(capacity_);
        alive_.resize(capacity_, false);

        // authored/static
        shape_kind_.resize(capacity_, ShapeKind::sphere);
        sphere_.resize(capacity_);
        material_.resize(capacity_);
        mesh_.resize(capacity_);
        inv_mass_.resize(capacity_, 1.0f);

        // simulation
        position_.resize(capacity_);
        velocity_.resize(capacity_);

        poses_.reserve(capacity_);
    }

    [[nodiscard]] PhysicsView physics_view() noexcept {
        return PhysicsView{
            .count = count_,
            .sphere = std::span<const SphereShape>{sphere_.data(), count_},
            .inv_mass = std::span<const f32>{inv_mass_.data(), count_},
            .position = std::span<Vec3>{position_.data(), count_},
            .velocity = std::span<Vec3>{velocity_.data(), count_},
            .poses = poses_,
        };
    }

    [[nodiscard]] RenderView render_view() const noexcept {
        return RenderView{
            .shape_kind = std::span<const ShapeKind>{shape_kind_.data(), count_},
            .sphere = std::span<const SphereShape>{sphere_.data(), count_},
            .material = std::span<const MaterialId>{material_.data(), count_},
            .mesh = std::span<const MeshId>{mesh_.data(), count_},
            .poses = poses_,
        };
    }

    // Render reads this each frame for interpolation.
    [[nodiscard]] PoseSnapshot pose_snapshot() const noexcept { return poses_.snapshot(); }

    // Physics calls this after stepping to publish.
    void publish_poses_from_sim() noexcept {
        auto out = poses_.write_positions(count_);
        for (u32 i = 0; i < count_; ++i) {
            out[i] = position_[i];
        }
        poses_.publish();
    }

    void reset_simulation() noexcept {
        for (u32 i = 0; i < count_; ++i) {
            position_[i] = detail::spawn_position(i);
            velocity_[i] = Vec3{};
        }
    }

    static Scene load_scene_from_disk(std::filesystem::path scene_path) {
        log::info(scene, "Loading scene from disk: {}", scene_path.string());

        // TEMP: procedural sphere cloud until real scene data/asset loading is in place.
        Scene out{};
        constexpr u32 kSphereCount = detail::spawn_count();

        out.reserve(kSphereCount);
        out.count_ = kSphereCount;

        for (u32 idx = 0; idx < out.count_; ++idx) {
            const f32 radius = detail::spawn_radius(idx);
            const f32 inv_mass = detail::spawn_inv_mass(radius);
            const Vec3 position = detail::spawn_position(idx);

            out.alive_[idx] = true;
            out.generation_[idx] = 1;
            out.shape_kind_[idx] = ShapeKind::sphere;
            out.sphere_[idx] = SphereShape{radius};
            out.material_[idx] = MaterialId{0};
            out.mesh_[idx] = MeshId{0};
            out.inv_mass_[idx] = inv_mass;
            out.position_[idx] = position;
            out.velocity_[idx] = Vec3{};
        }

        out.publish_poses_from_sim();
        log::info(scene, "Loaded {} spheres ({}x{} jittered grid)", out.count_, detail::kSpawnSettings.grid_dim,
                  detail::kSpawnSettings.grid_dim);
        log::info(scene, "Test scene params: radius=[{}..{}], height=[{}..{}], spacing={}, jitter={}",
                  detail::kSpawnSettings.radius_min, detail::kSpawnSettings.radius_max,
                  detail::kSpawnSettings.height_min, detail::kSpawnSettings.height_max,
                  detail::kSpawnSettings.spacing, detail::kSpawnSettings.jitter);
        return out;
    }

  private:
    u32 capacity_{0};
    u32 count_{0};

    // identity (kept for future spawn/despawn; can be minimal in v1)
    std::vector<u32> generation_{};
    std::vector<bool> alive_{};

    // authored/static
    std::vector<ShapeKind> shape_kind_{};
    std::vector<SphereShape> sphere_{};
    std::vector<MaterialId> material_{};
    std::vector<MeshId> mesh_{};
    std::vector<f32> inv_mass_{};

    // simulation state (physics-owned)
    std::vector<Vec3> position_{};
    std::vector<Vec3> velocity_{};

    // presentation channel (physics publishes, render reads)
    PoseChannel poses_{};
};
} // namespace javelin
