#include "uniform_grid.hpp"

namespace godot {
auto UniformGrid::add(Node2D* node, const Point2& pos) -> UniformGridNode {
    auto ret = UniformGridNode{node, get_cell(pos)};
    mGrid[ret.cell].insert(ret);
    return ret;
}
void UniformGrid::remove(Node2D* node) {
    for (auto& [cell, nodes] : mGrid) {
        nodes.erase(UniformGridNode{node});
    }
}
void UniformGrid::remove(UniformGridNode node) { mGrid[node.cell].erase(node); }
void UniformGrid::clear() { mGrid.clear(); }
auto UniformGrid::get_nodes_in_cell(const Point2i& cell) const -> std::vector<Node2D*> {
    std::vector<Node2D*> ret;
    auto it = mGrid.find(cell);
    if (it == mGrid.end()) {
        return ret;
    }
    for (auto& node : it->second) {
        ret.push_back(node.node);
    }
    return ret;
}
auto UniformGrid::get_cell(const Point2& pos) const -> Point2i {
    return Point2i(pos.x / mCellWidth, pos.y / mCellHeight);
}
auto UniformGrid::find_nodes_around(const Point2& pos, float radius) const -> std::vector<Node2D*> {
    std::vector<Node2D*> ret;
    auto cell         = get_cell(pos);
    auto radius_cells = Point2i((radius + mCellWidth - 1) / mCellWidth, (radius + mCellHeight - 1) / mCellHeight);
    for (int x = -radius_cells.x; x <= radius_cells.x; x++) {
        for (int y = -radius_cells.y; y <= radius_cells.y; y++) {
            auto cell1 = Point2i(cell.x + x, cell.y + y);
            auto nodes = get_nodes_in_cell(cell1);
            ret.insert(ret.end(), nodes.begin(), nodes.end());
        }
    }
    return ret;
}

auto UniformGrid::find_nodes_around(const Point2& pos, float radius, std::function<bool(Node2D*)> predicate) const
    -> std::vector<Node2D*> {
    std::vector<Node2D*> ret;
    auto cell         = get_cell(pos);
    auto radius_cells = Point2i((radius + mCellWidth - 1) / mCellWidth, (radius + mCellHeight - 1) / mCellHeight);
    for (int x = -radius_cells.x; x <= radius_cells.x; x++) {
        for (int y = -radius_cells.y; y <= radius_cells.y; y++) {
            auto cell1 = Point2i(cell.x + x, cell.y + y);
            auto nodes = get_nodes_in_cell(cell1);
            for (auto node : nodes) {
                if (predicate(node)) {
                    ret.push_back(node);
                }
            }
        }
    }
    return ret;
}

} // namespace godot