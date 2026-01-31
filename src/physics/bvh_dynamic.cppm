module;

#include <tracy/Tracy.hpp>

export module javelin.physics.bvh_dynamic;

import std;
import javelin.core.types;
import javelin.math.vec3;
import javelin.physics.aabb;

export namespace javelin {

struct DynamicBvh final {
    struct Node final {
        Aabb bounds{};
        u32 parent{std::numeric_limits<u32>::max()};
        u32 left{std::numeric_limits<u32>::max()};
        u32 right{std::numeric_limits<u32>::max()};
        u32 body_id{std::numeric_limits<u32>::max()};
        u32 next{std::numeric_limits<u32>::max()};
    };

    void clear() noexcept {
        nodes_.clear();
        body_to_node_.clear();
        free_list_ = kInvalidNode;
        root_ = kInvalidNode;
    }

    void reserve(const u32 body_capacity) {
        nodes_.reserve(static_cast<usize>(body_capacity) * 2u);
        body_to_node_.reserve(body_capacity);
    }

    void insert(const u32 body_id, const Aabb bounds) {
        ZoneScopedN("Physics dynamic BVH insert");
        ensure_body_capacity_(body_id);
        if (body_to_node_[body_id] != kInvalidNode) {
            remove(body_id);
        }

        const u32 leaf = allocate_node_();
        nodes_[leaf].bounds = fatten_(bounds);
        nodes_[leaf].body_id = body_id;
        nodes_[leaf].left = kInvalidNode;
        nodes_[leaf].right = kInvalidNode;
        nodes_[leaf].parent = kInvalidNode;

        insert_leaf_(leaf);
        body_to_node_[body_id] = leaf;
    }

    void remove(const u32 body_id) {
        ZoneScopedN("Physics dynamic BVH remove");
        if (body_id >= body_to_node_.size()) {
            return;
        }
        const u32 leaf = body_to_node_[body_id];
        if (leaf == kInvalidNode) {
            return;
        }

        remove_leaf_(leaf);
        free_node_(leaf);
        body_to_node_[body_id] = kInvalidNode;
    }

    // Returns true if the tree structure changed.
    bool update(const u32 body_id, const Aabb bounds) {
        ZoneScopedN("Physics dynamic BVH update");
        ensure_body_capacity_(body_id);
        const u32 leaf = body_to_node_[body_id];
        if (leaf == kInvalidNode) {
            insert(body_id, bounds);
            return true;
        }

        const Aabb fat = fatten_(bounds);
        if (contains_aabb(nodes_[leaf].bounds, fat)) {
            return false;
        }

        remove_leaf_(leaf);
        nodes_[leaf].bounds = fat;
        insert_leaf_(leaf);
        return true;
    }

    // Appends overlapping body ids to out (caller-owned). stack is scratch storage.
    void query(const Aabb aabb, std::vector<u32> &out, std::vector<u32> &stack) const {
        ZoneScopedN("Physics query dynamic BVH");
        if (root_ == kInvalidNode) {
            return;
        }

        stack.clear();
        stack.push_back(root_);

        while (!stack.empty()) {
            const u32 node_index = stack.back();
            stack.pop_back();

            const Node &node = nodes_[node_index];
            if (!overlaps(node.bounds, aabb)) {
                continue;
            }

            if (is_leaf_(node)) {
                out.push_back(node.body_id);
                continue;
            }

            if (node.left != kInvalidNode) {
                stack.push_back(node.left);
            }
            if (node.right != kInvalidNode) {
                stack.push_back(node.right);
            }
        }
    }

    [[nodiscard]] bool empty() const noexcept { return root_ == kInvalidNode; }
    [[nodiscard]] std::span<const Node> nodes() const noexcept { return nodes_; }

  private:
    static constexpr u32 kInvalidNode = std::numeric_limits<u32>::max();
    static constexpr f32 kFatMargin = 0.1f;

    [[nodiscard]] static Aabb fatten_(const Aabb aabb) noexcept { return inflate(aabb, Vec3{kFatMargin}); }
    [[nodiscard]] static constexpr bool is_leaf_(const Node &node) noexcept { return node.left == kInvalidNode; }

    void ensure_body_capacity_(const u32 body_id) {
        if (body_id < body_to_node_.size()) {
            return;
        }
        body_to_node_.resize(static_cast<usize>(body_id) + 1u, kInvalidNode);
    }

    u32 allocate_node_() {
        if (free_list_ != kInvalidNode) {
            const u32 node = free_list_;
            free_list_ = nodes_[node].next;
            nodes_[node].next = kInvalidNode;
            return node;
        }

        const u32 node = static_cast<u32>(nodes_.size());
        nodes_.push_back(Node{});
        return node;
    }

    void free_node_(const u32 node) {
        nodes_[node].next = free_list_;
        nodes_[node].parent = kInvalidNode;
        nodes_[node].left = kInvalidNode;
        nodes_[node].right = kInvalidNode;
        nodes_[node].body_id = kInvalidNode;
        free_list_ = node;
    }

    u32 find_best_sibling_(const Aabb leaf_bounds) const {
        u32 index = root_;
        while (!is_leaf_(nodes_[index])) {
            const u32 left = nodes_[index].left;
            const u32 right = nodes_[index].right;
            const f32 cost_left = surface_area(merge(nodes_[left].bounds, leaf_bounds));
            const f32 cost_right = surface_area(merge(nodes_[right].bounds, leaf_bounds));
            index = (cost_left < cost_right) ? left : right;
        }
        return index;
    }

    void insert_leaf_(const u32 leaf) {
        if (root_ == kInvalidNode) {
            root_ = leaf;
            nodes_[leaf].parent = kInvalidNode;
            return;
        }

        const Aabb leaf_bounds = nodes_[leaf].bounds;
        const u32 sibling = find_best_sibling_(leaf_bounds);
        const u32 old_parent = nodes_[sibling].parent;

        const u32 parent = allocate_node_();
        nodes_[parent].parent = old_parent;
        nodes_[parent].left = sibling;
        nodes_[parent].right = leaf;
        nodes_[parent].body_id = kInvalidNode;
        nodes_[parent].bounds = merge(nodes_[sibling].bounds, leaf_bounds);

        nodes_[sibling].parent = parent;
        nodes_[leaf].parent = parent;

        if (old_parent == kInvalidNode) {
            root_ = parent;
        } else {
            if (nodes_[old_parent].left == sibling) {
                nodes_[old_parent].left = parent;
            } else {
                nodes_[old_parent].right = parent;
            }
        }

        fix_upwards_(parent);
    }

    void remove_leaf_(const u32 leaf) {
        if (leaf == root_) {
            root_ = kInvalidNode;
            nodes_[leaf].parent = kInvalidNode;
            return;
        }

        const u32 parent = nodes_[leaf].parent;
        const u32 grand_parent = nodes_[parent].parent;
        const u32 sibling = (nodes_[parent].left == leaf) ? nodes_[parent].right : nodes_[parent].left;

        if (grand_parent != kInvalidNode) {
            if (nodes_[grand_parent].left == parent) {
                nodes_[grand_parent].left = sibling;
            } else {
                nodes_[grand_parent].right = sibling;
            }
            nodes_[sibling].parent = grand_parent;
            free_node_(parent);
            fix_upwards_(grand_parent);
        } else {
            root_ = sibling;
            nodes_[sibling].parent = kInvalidNode;
            free_node_(parent);
        }

        nodes_[leaf].parent = kInvalidNode;
    }

    void fix_upwards_(u32 node) {
        while (node != kInvalidNode) {
            Node &n = nodes_[node];
            n.bounds = merge(nodes_[n.left].bounds, nodes_[n.right].bounds);
            node = n.parent;
        }
    }

    std::vector<Node> nodes_{};
    std::vector<u32> body_to_node_{};
    u32 free_list_{kInvalidNode};
    u32 root_{kInvalidNode};
};

} // namespace javelin
