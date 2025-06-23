#pragma once

#include <godot_cpp/classes/node2d.hpp>

#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace godot {
class UniformGrid {
public:
    struct UniformGridNode {
        Node2D* node;
        Point2i cell;
        bool operator==(const UniformGridNode& other) const { return (void*)node == (void*)other.node; }
        bool operator==(const Node2D* other) const { return (void*)node == (void*)other; }
    };
    struct NodeHash {
        size_t operator()(const UniformGridNode& node) const { return (size_t)node.node; }
    };
    template <typename U>
    inline static void hash_combine(std::size_t& seed, U const& v) {
        seed ^= std::hash<U>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    struct PointHash {
        size_t operator()(const Point2i& p) const {
            size_t seed = 0;
            hash_combine(seed, p.x);
            hash_combine(seed, p.y);
            return seed;
        }
    };

public:
    UniformGrid() = default;
    UniformGrid(int cellWidth, int cellHeight) : mCellWidth(cellWidth), mCellHeight(cellHeight) {
        if (cellWidth <= 0 || cellHeight <= 0) {
            mCellHeight = 100;
            mCellWidth  = 100;
            ERR_FAIL_MSG("Cell width and height must be greater than 0");
        }
    }
    auto add(Node2D* node, const Point2& pos) -> UniformGridNode;
    void remove(Node2D* node);
    void remove(UniformGridNode node);
    void clear();
    auto get_nodes_in_cell(const Point2i& cell) const -> std::vector<Node2D*>;
    auto get_cell(const Point2& pos) const -> Point2i;
    auto find_nodes_around(const Point2& pos, float radius) const -> std::vector<Node2D*>;
    auto find_nodes_around(const Point2& pos, float radius, std::function<bool(Node2D*)> predicate) const
        -> std::vector<Node2D*>;

private:
    int mCellWidth  = 100;
    int mCellHeight = 100;
    std::unordered_map<Point2i, std::unordered_set<UniformGridNode, NodeHash>, PointHash> mGrid;
};

} // namespace godot