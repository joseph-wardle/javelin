export module javelin.scene.pose_channel;

import std;

import javelin.core.types;
import javelin.math.vec3;

export namespace javelin {
struct PoseSnapshot final {
    std::span<const Vec3> prev_positions;
    std::span<const Vec3> curr_positions;
    u32 count{};
};

struct PoseChannel final {
    PoseChannel() = default;
    PoseChannel(PoseChannel &&other) noexcept { *this = std::move(other); }

    PoseChannel &operator=(PoseChannel &&other) noexcept {
        if (this == &other)
            return *this;

        pos_ = std::move(other.pos_);
        capacity_ = other.capacity_;
        count_ = other.count_;
        write_idx_ = other.write_idx_;
        prev_idx_ = other.prev_idx_;
        curr_idx_ = other.curr_idx_;

        prev_published_.store(other.prev_published_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        curr_published_.store(other.curr_published_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        published_count_.store(other.published_count_.load(std::memory_order_relaxed), std::memory_order_relaxed);

        other.capacity_ = 0;
        other.count_ = 0;
        other.write_idx_ = 2;
        other.prev_idx_ = 0;
        other.curr_idx_ = 1;
        other.prev_published_.store(0, std::memory_order_relaxed);
        other.curr_published_.store(1, std::memory_order_relaxed);
        other.published_count_.store(0, std::memory_order_relaxed);

        return *this;
    }

    void reserve(u32 capacity) {
        for (auto &b : pos_)
            b.resize(capacity);
        capacity_ = capacity;
    }

    // Physics thread writes the back buffer.
    [[nodiscard]] std::span<Vec3> write_positions(u32 count) noexcept {
        count_ = count;
        return std::span<Vec3>{pos_[write_idx_].data(), count};
    }

    // Physics thread: publish what it just wrote.
    void publish() noexcept {
        // rotate (prev <- curr, curr <- write)
        prev_idx_ = curr_idx_;
        curr_idx_ = write_idx_;
        write_idx_ = (write_idx_ + 1u) % 3u;

        // release: make writes visible before indices update
        prev_published_.store(prev_idx_, std::memory_order_release);
        curr_published_.store(curr_idx_, std::memory_order_release);
        published_count_.store(count_, std::memory_order_release);
    }

    // Render thread: acquire snapshot spans.
    [[nodiscard]] PoseSnapshot snapshot() const noexcept {
        const u32 prev = prev_published_.load(std::memory_order_acquire);
        const u32 curr = curr_published_.load(std::memory_order_acquire);
        const u32 n = published_count_.load(std::memory_order_acquire);

        return PoseSnapshot{.prev_positions = std::span<const Vec3>{pos_[prev].data(), n},
                            .curr_positions = std::span<const Vec3>{pos_[curr].data(), n},
                            .count = n};
    }

  private:
    std::array<std::vector<Vec3>, 3> pos_{};
    u32 capacity_{0};
    u32 count_{0};

    u32 write_idx_{2};
    u32 prev_idx_{0};
    u32 curr_idx_{1};

    std::atomic<u32> prev_published_{0};
    std::atomic<u32> curr_published_{1};
    std::atomic<u32> published_count_{0};
};
} // namespace javelin
