#pragma once

#include <godot_cpp/classes/rigid_body2d.hpp>

#include <bit>

#include "boid_cluster_node.hpp"

namespace godot {

class BoidRigidBody2D : public RigidBody2D {
    GDCLASS(BoidRigidBody2D, RigidBody2D)
    friend class Node;

public:
    BoidRigidBody2D();
    ~BoidRigidBody2D();

    auto mass() const -> float { return get_mass(); }
    void setMass(float mass) { set_mass(mass); }
    auto velocity() const -> Vector2 { return get_linear_velocity(); }
    void setVelocity(Vector2 velocity) { set_linear_velocity(velocity); }
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
    BoidType mType = BoidType::None; // boid 的类型

    BoidClusterNode* mCluster     = nullptr; // 引用BoidClusterNode
    BoidManagerNode* mBoidManager = nullptr; // 引用BoidManagerNode
    float mTimeEmit               = 0.0;
};
} // namespace godot