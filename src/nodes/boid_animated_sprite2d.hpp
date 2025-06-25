#pragma once

#include <godot_cpp/classes/animated_sprite2d.hpp>

#include <bit>

#include "boid_cluster_node.hpp"

namespace godot {
class BoidManagerNode;

class BoidAnimatedSprite2D : public AnimatedSprite2D {
    // NOLINTBEGIN
    GDCLASS(BoidAnimatedSprite2D, AnimatedSprite2D)
    // NOLINTEND
    friend class Node;

public:
    BoidAnimatedSprite2D();
    ~BoidAnimatedSprite2D();
    auto mass() const -> float { return mMass; }
    void setMass(float mass) { mMass = mass; }
    void setVelocity(Vector2 velocity) { mVelocity = velocity; }
    auto velocity() const -> Vector2 { return mVelocity; }
    auto boidType() -> uint32_t {
        if (mType == BoidType::None) {
            return 0;
        } else {
            return std::countr_zero((uint32_t)mType) + 1;
        }
    }
    void setBoidType(uint32_t type) {
        if (type == 0) {
            mType = BoidType::None;
        } else {
            mType = (BoidType)(1 << (type - 1));
        }
    }

    // system
    auto updateManager(BoidManagerNode* manager) -> void { mBoidManager = manager; }
    auto updateBoidCluster(BoidClusterNode* cluster) -> void { mCluster = cluster; }
    void _draw() override;

protected:
    void _enter_tree() override;
    void _exit_tree() override;
    void _process(double delta) override;
    void _physics_process(double delta) override;
    static void _bind_methods();

private:
    Vector2 mForward  = {1, 0};         // boid 的方向
    Vector2 mVelocity = {0, 0};         // boid 的速度
    float mMass       = 1.0;            // boid 的质量
    BoidType mType    = BoidType::None; // boid 的类型

    BoidClusterNode* mCluster     = nullptr; // 引用BoidClusterNode
    BoidManagerNode* mBoidManager = nullptr; // 引用BoidManagerNode
    float mTimeEmit               = 0.0;
};
} // namespace godot