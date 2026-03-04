module;

#include <tracy/Tracy.hpp>

export module javelin.physics.bvh_static;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;

export namespace javelin {

// Static BVH contract:
// - stores broad-phase bounds for static bodies.
// - rebuilt as a batch from sorted/unsorted body id input.
// - query is allocation-free once stack/out are provided by caller.
struct StaticBvh final {
    struct Node final {
        Aabb bounds{};
        u32 left{std::numeric_limits<u32>::max()};
        u32 right{std::numeric_limits<u32>::max()};
        u32 body_id{std::numeric_limits<u32>::max()};
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

    // Calls on_hit(body_id) for each leaf whose bounds overlap aabb.
    // Stack policy:
    // - fixed-size local stack for the common case (no allocations),
    // - caller scratch vector only when traversal depth exceeds local capacity.
    template <typename OnHit>
    void query_each_overlap(const Aabb aabb, OnHit &&on_hit, std::vector<u32> &overflow_stack) const {
        ZoneScopedN("Physics query static BVH");
        if (root_ == kInvalidNode) {
            return;
        }

        std::array<u32, kQueryStackInlineCapacity> local_stack{};
        u32 local_count = 0u;
        overflow_stack.clear();
        local_stack[local_count++] = root_;

        auto push_node = [&](const u32 node_index) {
            if (local_count < local_stack.size()) {
                local_stack[local_count++] = node_index;
                return;
            }
            overflow_stack.push_back(node_index);
        };

        auto pop_node = [&]() {
            if (!overflow_stack.empty()) {
                const u32 node_index = overflow_stack.back();
                overflow_stack.pop_back();
                return node_index;
            }
            --local_count;
            return local_stack[local_count];
        };

        while (local_count > 0u || !overflow_stack.empty()) {
            const u32 node_index = pop_node();

            const Node &node = nodes_[node_index];
            if (!overlaps(node.bounds, aabb)) {
                continue;
            }

            if (is_leaf_(node)) {
                on_hit(node.body_id);
                continue;
            }

            if (node.left != kInvalidNode) {
                push_node(node.left);
            }
            if (node.right != kInvalidNode) {
                push_node(node.right);
            }
        }
    }

    // Compatibility wrapper: appends overlapping body ids to out.
    void query(const Aabb aabb, std::vector<u32> &out, std::vector<u32> &stack) const {
        query_each_overlap(aabb, [&](const u32 body_id) { out.push_back(body_id); }, stack);
    }

  private:
    static constexpr u32 kInvalidNode = std::numeric_limits<u32>::max();
    static constexpr usize kQueryStackInlineCapacity = 128u;
    [[nodiscard]] static constexpr bool is_leaf_(const Node &node) noexcept { return node.left == kInvalidNode; }

    void build_from_ids_(std::span<const Aabb> bounds) {
        nodes_.clear();
        root_ = kInvalidNode;
        if (build_ids_.empty()) {
            return;
        }
        // Full binary tree upper bound for N leaves is ~2N nodes.
        nodes_.reserve(build_ids_.size() * 2u);
        root_ = build_range_(std::span<u32>{build_ids_.data(), build_ids_.size()}, bounds);
    }

    [[nodiscard]] static u32 choose_split_axis_(const Aabb node_bounds) noexcept {
        const Vec3 axis_span = size(node_bounds);
        if (axis_span.x >= axis_span.y && axis_span.x >= axis_span.z) {
            return 0;
        }
        if (axis_span.y >= axis_span.z) {
            return 1;
        }
        return 2;
    }

    u32 build_range_(std::span<u32> ids, std::span<const Aabb> bounds) {
        // Bottom-up bound for the current id subset.
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
            });
            return node_index;
        }

        // Median split by center coordinate along longest AABB axis.
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
        });
        return node_index;
    }

    std::vector<Node> nodes_{};
    u32 root_{kInvalidNode};
    std::vector<u32> build_ids_{};
};

} // namespace javelin
