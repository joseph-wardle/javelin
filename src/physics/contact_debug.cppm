export module javelin.physics.contact_debug;

import std;

import javelin.core.time;
import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {
namespace detail {
[[nodiscard]] inline u64 now_ns() noexcept {
    const auto now = SteadyClock::now().time_since_epoch();
    return static_cast<u64>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}
} // namespace detail

// Per-contact debug attributes in world space.
// All spans in a snapshot are indexed in lockstep [0, count).
struct ContactDebugSnapshot final {
    std::span<const Vec3> points{};
    std::span<const Vec3> normals{};
    std::span<const f32> separations{};
    std::span<const f32> normal_impulses{};
    std::span<const u8> persisted{};
    u32 count{};
    // Monotonic completed-simulation step id associated with this snapshot.
    u64 step_id{};
    u64 time_ns{};
};

struct ContactDebugWrite final {
    std::span<Vec3> points{};
    std::span<Vec3> normals{};
    std::span<f32> separations{};
    std::span<f32> normal_impulses{};
    std::span<u8> persisted{};
    u32 count{};
};

// Lock-free physics->render contact snapshot channel.
//
// Ownership model:
// - Physics thread is the only writer (write_contacts/publish/publish_empty).
// - Render thread is the reader (snapshot).
// - Triple buffering prevents the writer from touching the current published
//   buffer in the immediate next publish.
//
// Capacity model:
// - Each buffer grows independently when it is the active write buffer.
// - No allocations occur once peak per-buffer size has stabilized.
struct ContactDebugChannel final {
    void reserve(const u32 capacity) {
        resize_buffer_(0u, capacity);
        resize_buffer_(1u, capacity);
        resize_buffer_(2u, capacity);
    }

    [[nodiscard]] ContactDebugWrite write_contacts(const u32 count) noexcept {
        ensure_write_capacity_(count);
        return ContactDebugWrite{
            .points = std::span<Vec3>{points_[write_idx_].data(), count},
            .normals = std::span<Vec3>{normals_[write_idx_].data(), count},
            .separations = std::span<f32>{separations_[write_idx_].data(), count},
            .normal_impulses = std::span<f32>{normal_impulses_[write_idx_].data(), count},
            .persisted = std::span<u8>{persisted_[write_idx_].data(), count},
            .count = count,
        };
    }

    void publish(const u32 count, const u64 step_id) noexcept {
        counts_[write_idx_].store(count, std::memory_order_relaxed);
        step_ids_[write_idx_].store(step_id, std::memory_order_relaxed);
        times_ns_[write_idx_].store(detail::now_ns(), std::memory_order_relaxed);

        curr_idx_ = write_idx_;
        write_idx_ = (write_idx_ + 1u) % 3u;

        curr_published_.store(curr_idx_, std::memory_order_release);
    }

    void publish_empty(const u64 step_id) noexcept { publish(0u, step_id); }

    [[nodiscard]] ContactDebugSnapshot snapshot() const noexcept {
        const u32 curr = curr_published_.load(std::memory_order_acquire);
        const u32 count = counts_[curr].load(std::memory_order_acquire);
        const u64 step_id = step_ids_[curr].load(std::memory_order_acquire);
        const u64 time_ns = times_ns_[curr].load(std::memory_order_acquire);
        return ContactDebugSnapshot{
            .points = std::span<const Vec3>{points_[curr].data(), count},
            .normals = std::span<const Vec3>{normals_[curr].data(), count},
            .separations = std::span<const f32>{separations_[curr].data(), count},
            .normal_impulses = std::span<const f32>{normal_impulses_[curr].data(), count},
            .persisted = std::span<const u8>{persisted_[curr].data(), count},
            .count = count,
            .step_id = step_id,
            .time_ns = time_ns,
        };
    }

  private:
    void ensure_write_capacity_(const u32 count) {
        if (points_[write_idx_].size() >= count) {
            return;
        }
        resize_buffer_(write_idx_, count);
    }

    void resize_buffer_(const u32 buffer_index, const u32 count) {
        points_[buffer_index].resize(count);
        normals_[buffer_index].resize(count);
        separations_[buffer_index].resize(count);
        normal_impulses_[buffer_index].resize(count);
        persisted_[buffer_index].resize(count);
    }

  private:
    std::array<std::vector<Vec3>, 3> points_{};
    std::array<std::vector<Vec3>, 3> normals_{};
    std::array<std::vector<f32>, 3> separations_{};
    std::array<std::vector<f32>, 3> normal_impulses_{};
    std::array<std::vector<u8>, 3> persisted_{};

    u32 write_idx_{2};
    u32 curr_idx_{1};

    std::atomic<u32> curr_published_{1};
    std::array<std::atomic<u32>, 3> counts_{};
    std::array<std::atomic<u64>, 3> step_ids_{};
    std::array<std::atomic<u64>, 3> times_ns_{};
};

} // namespace javelin
