#include <gtest/gtest.h>

#include "algorithm/uniform_grid.hpp"

TEST(uniform_grid, test_uniform_grid) {
    godot::UniformGrid grid(10, 10);
    std::array<godot::Node2D*, 1000> nodes;
    std::vector<godot::UniformGrid::UniformGridNode> unNodes;
    for (int i = 0; i < 1000; ++i) {
        unNodes.push_back(grid.add(nodes[i], godot::Vector2(rand() % 100, rand() % 100)));
    }
    auto nodeIn11 = grid.find_nodes_around({10, 10}, 10);
    EXPECT_GT(nodeIn11.size(), 10);
    while (unNodes.size() > 10) {
        grid.remove(unNodes.back());
        unNodes.pop_back();
    }
    for (auto& node : unNodes) {
        grid.remove(node);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}