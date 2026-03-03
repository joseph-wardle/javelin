export module javelin.physics.aabb_debug;

import std;

import javelin.core.time;
import javelin.core.types;
import javelin.physics.aabb;

export namespace javelin {

// Per-body AABB debug data in world space.
// Indexed in lockstep with the physics body array: aabbs[i] belongs to body i.
struct AabbDebugSnapshot final {
    std::span<const Aabb> aabbs{};
    u32 count{};
    // Monotonic completed-simulation step id associated with this snapshot.
    u64 step_id{};
    u64 time_ns{};
};

struct AabbDebugWrite final {
    std::span<Aabb> aabbs{};
    u32 count{};
};

// Lock-free physics->render AABB snapshot channel.
//
// Ownership model:
// - Physics thread is the only writer (write_aabbs/publish/publish_empty).
// - Render thread is the reader (snapshot).
// - Triple buffering prevents the writer from touching the current published
//   buffer in the immediate next publish.
//
// Capacity model:
// - Each buffer grows independently when it is the active write buffer.
// - No allocations occur once peak per-buffer size has stabilized (= body count).
struct AabbDebugChannel final {
    void reserve(const u32 capacity) {
        resize_buffer_(0u, capacity);
        resize_buffer_(1u, capacity);
        resize_buffer_(2u, capacity);
    }

    [[nodiscard]] AabbDebugWrite write_aabbs(const u32 count) noexcept {
        ensure_write_capacity_(count);
        return AabbDebugWrite{
            .aabbs = std::span<Aabb>{aabbs_[write_idx_].data(), count},
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

    [[nodiscard]] AabbDebugSnapshot snapshot() const noexcept {
        const u32 curr = curr_published_.load(std::memory_order_acquire);
        const u32 count = counts_[curr].load(std::memory_order_acquire);
        const u64 step_id = step_ids_[curr].load(std::memory_order_acquire);
        const u64 time_ns = times_ns_[curr].load(std::memory_order_acquire);
        return AabbDebugSnapshot{
            .aabbs = std::span<const Aabb>{aabbs_[curr].data(), count},
            .count = count,
            .step_id = step_id,
            .time_ns = time_ns,
        };
    }

  private:
    void ensure_write_capacity_(const u32 count) {
        if (aabbs_[write_idx_].size() >= count) {
            return;
        }
        resize_buffer_(write_idx_, count);
    }

    void resize_buffer_(const u32 buffer_index, const u32 count) { aabbs_[buffer_index].resize(count); }

  private:
    std::array<std::vector<Aabb>, 3> aabbs_{};

    u32 write_idx_{2};
    u32 curr_idx_{1};

    std::atomic<u32> curr_published_{1};
    std::array<std::atomic<u32>, 3> counts_{};
    std::array<std::atomic<u64>, 3> step_ids_{};
    std::array<std::atomic<u64>, 3> times_ns_{};
};

} // namespace javelin
