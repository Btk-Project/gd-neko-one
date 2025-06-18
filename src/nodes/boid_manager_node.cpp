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
        boid->setAcceleration(Vector2{0, 1});
        mBoidNodes[node]         = boid_type;
        mBoidDataHash[boid_type] = node->get_name().hash();
        boid->updateManager(this);
        boid->connect("position_changed", Callable(this, "_on_position_changed"));
    }
}

auto BoidManagerNode::remove(Node2D* node) -> void {
    if (auto item = mBoidNodes.find(node); item != mBoidNodes.end()) {
        if (auto* boid = Node::cast_to<BoidSprite2D>(node); node) {
            boid->updateManager(nullptr);
            boid->disconnect("position_changed", Callable(this, "_on_position_changed"));
        }
        mBoidDataHash[item->second] = (uint64_t)rand() * rand() * rand();
        mBoidNodes.erase(item);
    }
}

auto BoidManagerNode::on_child_entered_tree(Node2D* node) -> void { insert(node, -1); }

auto BoidManagerNode::on_child_exiting_tree(Node2D* node) -> void { remove(node); }

auto BoidManagerNode::_on_position_changed(Object* node, Vector2 position) -> void {
    if (auto item = mBoidNodes.find(Node::cast_to<Node2D>(node)); item != mBoidNodes.end()) {
        print_line("position changed: " + String(Variant(item->second)));
        mBoidDataHash[item->second] = (uint64_t)rand() * rand() * rand();
    }
}

auto BoidManagerNode::data_hash(DataType type) -> uint64_t {
    if (type == 0) {
        return 0;
    }
    return mBoidDataHash[std::countr_zero((uint32_t)type)];
}

// 获取指定节点需要寻找的目标位置
auto BoidManagerNode::get_seek_target(Node2D* self) -> Node2D* {
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)Seek) && node != self) {
            // TODO：找到最近的目标返回？
            return node;
        }
    }
    return nullptr;
}

// 获取指定节点需要到达的目标位置
auto BoidManagerNode::get_arrive_target(Node2D* self) -> Node2D* {
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Arrive) && node != self) {
            // TODO：找到最近的目标返回？
            return node;
        }
    }
    return nullptr;
}

// 获取指定节点需要追逐的目标位置
auto BoidManagerNode::get_pursuit_target(Node2D* self, int distance) -> Node2D* {
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Pursuit) && node != self &&
            node->get_position().distance_to(self->get_position()) < distance) {
            // TODO：找到最近的目标返回？
            return node;
        }
    }
    return nullptr;
}

// 获取指定节点附近的捕食者
auto BoidManagerNode::get_nearby_predator(Node2D* self, int distance) -> Array {
    Array result;
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Predator) && node != self &&
            (node->get_position() - self->get_position()).length() < distance) {
            // TODO：找到最近的目标返回？
            result.append(node);
        }
    }
    return result;
}

// 获取指定节点需要避让的目标位置
auto BoidManagerNode::get_avoid_target(Node2D* self, int distance) -> Node2D* {
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Avoid) && node != self &&
            (node->get_position() - self->get_position()).length() < distance) {
            // TODO：找到最近的目标返回？
            return node;
        }
    }
    return nullptr;
}

// 获取指定节点附近的障碍物
auto BoidManagerNode::get_nearby_obstacles(Node2D* self, int distance) -> Array {
    Array result;
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Obstacle) && node != self &&
            node->get_position().distance_to(self->get_position()) < distance) {
            // TODO：找到最近的目标返回？
            result.append(node->get_position());
        }
    }
    return result;
}

auto BoidManagerNode::get_evade_target(Node2D* self, int distance) -> Node2D* {
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Evade) && node != self &&
            node->get_position().distance_to(self->get_position()) < distance) {
            // TODO：找到最近的目标返回？
            return node;
        }
    }
    return nullptr;
}

// 获取需要跟随的路径
auto BoidManagerNode::get_following_path(Node2D* self) -> Array {
    Array result;
    for (auto& [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Following) && node != self) {
            // TODO：找到最近的目标返回？
            result.append(node->get_position());
        }
    }

    return result;
}

// 获取附近同集群的节点
auto BoidManagerNode::get_nearby_same_cluster_nodes(Node2D* self, int distance) -> Array {
    Array result;
    for (auto [node, type] : mBoidNodes) {
        if (type == std::countr_zero((uint32_t)DataType::Cluster) && node != self &&
            node->get_position().distance_to(self->get_position()) < distance) {
            result.append(node);
        }
    }
    return result;
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
}
} // namespace godot
