#pragma once

#include <godot_cpp/classes/node2d.hpp>

#include "../algorithm/uniform_grid.hpp"

namespace godot {
enum class BoidType : uint32_t {
    None      = 0,
    Cluster   = 1 << 0, // 集群
    Seek      = 1 << 1, // 寻找目标
    Arrive    = 1 << 2, // 到达目标
    Flee      = 1 << 3, // 逃离目標
    Avoid     = 1 << 4, // 避让目标
    Pursuit   = 1 << 5, // 追逐目标
    Evade     = 1 << 6, // 躲避目标
    Obstacle  = 1 << 7, // 障碍物
    Predator  = 1 << 8, // 捕食者
    Following = 1 << 9, // 跟随
};
class BoidManagerNode : public Node {
    GDCLASS(BoidManagerNode, Node)
    friend class Node;

public:
    constexpr static std::array gBoidTypePrefix = {
        "cluster", "seek", "arrive", "flee", "avoid", "pursuit", "evade", "obstacle", "predator", "following",
    };

public:
    BoidManagerNode()  = default;
    ~BoidManagerNode() = default;

    auto on_child_entered_tree(Node2D* node) -> void;
    auto on_child_exiting_tree(Node2D* node) -> void;
    auto insert(Node2D* node, BoidType boid_type) -> void;
    auto remove(Node2D* node) -> void;
    auto data_hash(BoidType type) -> uint64_t;

    // 获取指定节点需要寻找的目标位置
    auto get_seek_target(Node2D* self) -> Node2D*;

    // 获取指定节点需要到达的目标位置
    auto get_arrive_target(Node2D* self) -> Node2D*;

    // 获取指定节点需要追逐的目标位置
    auto get_pursuit_target(Node2D* self, int distance) -> Node2D*;

    // 获取需要躲避的目标位置
    auto get_evade_target(Node2D* self, int distance) -> Node2D*;

    // 获取指定节点需要避让的目标位置
    auto get_avoid_target(Node2D* self, int distance) -> Node2D*;

    // 获取指定节点附近的障碍物
    auto get_nearby_obstacles(Node2D* self, int distance) -> Array;

    // 获取指定节点附近的捕食者
    auto get_nearby_predator(Node2D* self, int distance) -> Array;

    // 获取指定节点附近的跟随者
    auto get_following_path(Node2D* self) -> Array;

    // 获取附近同集群的节点
    auto get_nearby_same_cluster_nodes(Node2D* self, int distance) -> Array;

protected:
    void _ready() override;
    static auto _bind_methods() -> void;
    auto _on_position_changed(Object* node, Vector2 position) -> void;
    auto _find_closest_node(Node2D* self, BoidType type) -> Node2D*;
    auto _find_closest_nodes(Node2D* self, BoidType type, int distance) -> Array;

private:
    std::array<uint64_t, 33> mBoidDataHash = {0};
    std::unordered_map<void*, std::pair<UniformGrid::UniformGridNode, int>> mBoidNodes;
    std::array<std::unordered_set<void*>, 33> mBoidClusters;
    UniformGrid mGrid;
};
} // namespace godot