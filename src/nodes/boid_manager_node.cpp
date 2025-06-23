#include "boid_manager_node.hpp"

#include "boid_node.hpp"

#include <bit>

namespace godot {

auto BoidManagerNode::insert(Node2D* node, int boid_type) -> void {
    ERR_FAIL_COND_MSG(!node, "insert a null node in BoidManagerNode");
    if (boid_type < 0 || boid_type >= gDataTypePrefix.size()) {
        boid_type = 0;
        for (int i = 0; i < gDataTypePrefix.size(); ++i) {
            if (node->get_name().contains(gDataTypePrefix[i])) {
                boid_type = i;
                break;
            }
        }
    }
    if (auto* boid = Node::cast_to<BoidSprite2D>(node); boid) {
        boid->updateManager(this);
        boid->connect("position_changed", Callable(this, "_on_position_changed"));
    }
    auto ret         = mGrid.add(node, node->get_position());
    mBoidNodes[node] = std::make_pair(ret, boid_type);
    mBoidClusters[boid_type].insert(node);
    mBoidDataHash[boid_type] = node->get_name().hash();
}

auto BoidManagerNode::remove(Node2D* node) -> void {
    if (auto item = mBoidNodes.find(node); item != mBoidNodes.end()) {
        if (auto* boid = Node::cast_to<BoidSprite2D>(node); node) {
            boid->updateManager(nullptr);
            boid->disconnect("position_changed", Callable(this, "_on_position_changed"));
        }
        mGrid.remove(item->second.first);
        mBoidClusters[item->second.second].erase(node);
        mBoidDataHash[item->second.second] = (uint64_t)rand() * rand() * rand();
        mBoidNodes.erase(item);
    }
}

auto BoidManagerNode::on_child_entered_tree(Node2D* node) -> void { insert(node, -1); }

auto BoidManagerNode::on_child_exiting_tree(Node2D* node) -> void { remove(node); }

auto BoidManagerNode::_on_position_changed(Object* node, Vector2 position) -> void {
    if (auto item = mBoidNodes.find(node); item != mBoidNodes.end()) {
        if (mGrid.get_cell(position) != item->second.first.cell) {
            mGrid.remove(item->second.first);
            item->second.first                 = mGrid.add(Node2D::cast_to<Node2D>(node), position);
            mBoidDataHash[item->second.second] = (uint64_t)rand() * rand() * rand();
        }
    }
}

auto BoidManagerNode::_find_closest_node(Node2D* self, DataType type) -> Node2D* {
    if (self == nullptr) {
        return nullptr;
    }
    int min_dist = std::numeric_limits<int>::max();
    Node2D* ret  = nullptr;
    auto pos     = self->get_global_position();
    for (auto& node : mBoidClusters[std::countr_zero((uint32_t)type)]) {
        if (auto dist = static_cast<Node2D*>(node)->get_global_position().distance_to(pos); dist < min_dist) {
            min_dist = dist;
            ret      = static_cast<Node2D*>(node);
            if (min_dist == 0) {
                return static_cast<Node2D*>(node);
            }
        }
    }
    return ret;
}

auto BoidManagerNode::_find_closest_nodes(Node2D* self, DataType type, int distance) -> Array {
    if (self == nullptr) {
        return Array{};
    }
    Array ret;
    auto pos = self->get_global_position();
    for (auto& node : mBoidClusters[std::countr_zero((uint32_t)type)]) {
        if (node != self && static_cast<Node2D*>(node)->get_global_position().distance_to(pos) <= distance) {
            ret.append(static_cast<Node2D*>(node));
        }
    }
    return ret;
}

auto BoidManagerNode::data_hash(DataType type) -> uint64_t {
    if (type == 0) {
        return 0;
    }
    return mBoidDataHash[std::countr_zero((uint32_t)type)];
}

// 获取指定节点需要寻找的目标位置
auto BoidManagerNode::get_seek_target(Node2D* self) -> Node2D* { return _find_closest_node(self, Seek); }

// 获取指定节点需要到达的目标位置
auto BoidManagerNode::get_arrive_target(Node2D* self) -> Node2D* { return _find_closest_node(self, Arrive); }

// 获取指定节点需要追逐的目标位置
auto BoidManagerNode::get_pursuit_target(Node2D* self, int distance) -> Node2D* {
    auto item = _find_closest_node(self, DataType::Pursuit);
    if (item != nullptr && item->get_global_position().distance_to(self->get_global_position()) <= distance) {
        return item;
    }
    return nullptr;
}

// 获取指定节点附近的捕食者
auto BoidManagerNode::get_nearby_predator(Node2D* self, int distance) -> Array {
    return _find_closest_nodes(self, DataType::Predator, distance);
}

// 获取指定节点需要避让的目标位置
auto BoidManagerNode::get_avoid_target(Node2D* self, int distance) -> Node2D* {
    auto item = _find_closest_node(self, DataType::Avoid);
    if (item != nullptr && item->get_global_position().distance_to(self->get_global_position()) <= distance) {
        return item;
    }
    return nullptr;
}

// 获取指定节点附近的障碍物
auto BoidManagerNode::get_nearby_obstacles(Node2D* self, int distance) -> Array {
    return _find_closest_nodes(self, DataType::Obstacle, distance);
}

auto BoidManagerNode::get_evade_target(Node2D* self, int distance) -> Node2D* {
    auto item = _find_closest_node(self, DataType::Evade);
    if (item != nullptr && item->get_global_position().distance_to(self->get_global_position()) <= distance) {
        return item;
    }
    return nullptr;
}

// 获取需要跟随的路径
auto BoidManagerNode::get_following_path(Node2D* self) -> Array {
    return _find_closest_nodes(self, DataType::Following, 10000);
}

// 获取附近同集群的节点
auto BoidManagerNode::get_nearby_same_cluster_nodes(Node2D* self, int distance) -> Array {
    return _find_closest_nodes(self, DataType::Cluster, distance);
}

void BoidManagerNode::_ready() {
    auto children = get_children();
    for (int i = 0; i < children.size(); i++) {
        if (children[i].get_type() == Variant::Type::OBJECT) {
            auto node = Node::cast_to<Node2D>(children[i].operator Object*());
            insert(node, -1);
        }
    }

    Error err = this->connect("child_entered_tree", Callable(this, "on_child_entered_tree"));
    ERR_FAIL_COND_MSG(err != OK, "Failed to connect to child_entered_tree signal: " + String::num(err));
    err = this->connect("child_exiting_tree", Callable(this, "on_child_exiting_tree"));
    ERR_FAIL_COND_MSG(err != OK, "Failed to connect to child_exiting_tree signal: " + String::num(err));
}

auto BoidManagerNode::_bind_methods() -> void {
    ClassDB::bind_method(D_METHOD("on_child_entered_tree", "child"), &BoidManagerNode::on_child_entered_tree);
    ClassDB::bind_method(D_METHOD("on_child_exiting_tree", "child"), &BoidManagerNode::on_child_exiting_tree);
    ClassDB::bind_method(D_METHOD("_on_position_changed", "node", "new_position"),
                         &BoidManagerNode::_on_position_changed);
}
} // namespace godot
