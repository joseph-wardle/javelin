module;

#include <tracy/Tracy.hpp>

export module javelin.physics.bvh_static;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;

export namespace javelin {

struct StaticBvh final {
    struct Node final {
        Aabb bounds{};
        u32 left{std::numeric_limits<u32>::max()};
        u32 right{std::numeric_limits<u32>::max()};
        u32 body_id{std::numeric_limits<u32>::max()};
        bool is_leaf{};
    };

    void clear() noexcept {
        nodes_.clear();
        root_ = kInvalidNode;
    }

    void reserve(const u32 body_count) { nodes_.reserve(static_cast<usize>(body_count) * 2u); }

    void build(std::span<const u32> body_ids, std::span<const Aabb> bounds) {
        ZoneScopedN("Physics build static BVH");
        build_ids_.assign(body_ids.begin(), body_ids.end());
        build_from_ids_(bounds);
    }

    void build(std::span<const Aabb> bounds) {
        ZoneScopedN("Physics build static BVH");
        build_ids_.resize(bounds.size());
        std::iota(build_ids_.begin(), build_ids_.end(), 0u);
        build_from_ids_(bounds);
    }

    [[nodiscard]] bool empty() const noexcept { return root_ == kInvalidNode; }
    [[nodiscard]] u32 root() const noexcept { return root_; }
    [[nodiscard]] std::span<const Node> nodes() const noexcept { return nodes_; }

    // Appends overlapping body ids to out (caller-owned).
    void query(const Aabb aabb, std::vector<u32> &out) const {
        ZoneScopedN("Physics query static BVH");
        if (root_ == kInvalidNode) {
            return;
        }

        stack_.clear();
        stack_.push_back(root_);

        while (!stack_.empty()) {
            const u32 node_index = stack_.back();
            stack_.pop_back();

            const Node &node = nodes_[node_index];
            if (!overlaps(node.bounds, aabb)) {
                continue;
            }

            if (node.is_leaf) {
                out.push_back(node.body_id);
                continue;
            }

            if (node.left != kInvalidNode) {
                stack_.push_back(node.left);
            }
            if (node.right != kInvalidNode) {
                stack_.push_back(node.right);
            }
        }
    }

  private:
    static constexpr u32 kInvalidNode = std::numeric_limits<u32>::max();

    void build_from_ids_(std::span<const Aabb> bounds) {
        nodes_.clear();
        root_ = kInvalidNode;
        if (build_ids_.empty()) {
            return;
        }
        nodes_.reserve(build_ids_.size() * 2u);
        stack_.reserve(build_ids_.size() * 2u);
        root_ = build_range_(std::span<u32>{build_ids_.data(), build_ids_.size()}, bounds);
    }

    [[nodiscard]] static u32 choose_split_axis_(const Aabb bounds) noexcept {
        const Vec3 s = size(bounds);
        if (s.x >= s.y && s.x >= s.z) {
            return 0;
        }
        if (s.y >= s.z) {
            return 1;
        }
        return 2;
    }

    u32 build_range_(std::span<u32> ids, std::span<const Aabb> bounds) {
        Aabb node_bounds = bounds[ids[0]];
        for (usize i = 1; i < ids.size(); ++i) {
            node_bounds = merge(node_bounds, bounds[ids[i]]);
        }

        if (ids.size() == 1) {
            const u32 node_index = static_cast<u32>(nodes_.size());
            nodes_.push_back(Node{
                .bounds = node_bounds,
                .left = kInvalidNode,
                .right = kInvalidNode,
                .body_id = ids[0],
                .is_leaf = true,
            });
            return node_index;
        }

        const u32 axis = choose_split_axis_(node_bounds);
        const usize mid_offset = ids.size() / 2;
        auto mid = ids.begin() + static_cast<isize>(mid_offset);
        std::nth_element(ids.begin(), mid, ids.end(), [bounds, axis](const u32 lhs, const u32 rhs) {
            return center(bounds[lhs])[axis] < center(bounds[rhs])[axis];
        });

        const u32 left = build_range_(std::span<u32>{ids.data(), mid_offset}, bounds);
        const u32 right = build_range_(std::span<u32>{ids.data() + mid_offset, ids.size() - mid_offset}, bounds);

        const u32 node_index = static_cast<u32>(nodes_.size());
        nodes_.push_back(Node{
            .bounds = node_bounds,
            .left = left,
            .right = right,
            .body_id = kInvalidNode,
            .is_leaf = false,
        });
        return node_index;
    }

    std::vector<Node> nodes_{};
    u32 root_{kInvalidNode};
    std::vector<u32> build_ids_{};
    mutable std::vector<u32> stack_{};
};

} // namespace javelin
