#include "boid_sprite2d.hpp"

#include <godot_cpp/classes/shape2d.hpp>

namespace godot {
BoidSprite2D::BoidSprite2D() {}

BoidSprite2D::~BoidSprite2D() {}

void BoidSprite2D::_process(double delta) {
    mTimeEmit += delta;
    if (int(mTimeEmit * 100) % 10 == 0) {
        queue_redraw();
    }
    if (mTimeEmit > 1.0) {
        emit_signal("position_changed", this, get_position());
        mTimeEmit = 0.0;
    }

    // auto steering_force = _avoid_obstacles({Rect2{0, 0, 1001, 1001}}) * 5;
    // mAcceleration += steering_force.limit_length(mMaxForce) / mMass;
}
void BoidSprite2D::_enter_tree() {
    auto parent = get_parent();
    if (parent != nullptr) {
        mBoidManager = Object::cast_to<BoidManagerNode>(parent);
        mCluster     = Object::cast_to<BoidClusterNode>(parent);
    }
}

void BoidSprite2D::_exit_tree() {
    mBoidManager = nullptr;
    mCluster     = nullptr;
}

void BoidSprite2D::_draw() {
    if (mCluster != nullptr && mCluster->isDebugDraw() && mBoidManager != nullptr) {
        draw_dashed_line({0, 0}, {10, 0}, Color("#FF00FF"));
        if (mCluster->flocking()) {
            auto neighbors = mBoidManager->get_nearby_same_cluster_nodes(
                this,
                std::max({mCluster->cohesionRadius(), mCluster->separationRadius(), mCluster->alignmentRadius()}));
            for (int i = 0; i < neighbors.size(); i++) {
                auto node = Node::cast_to<Node2D>(neighbors[i]);
                if (node != nullptr) {
                    draw_dashed_line({0, 0}, to_local(node->get_position()), Color("#00FF00"));
                }
            }
        }
        if (mCluster->seekTarget()) {
            auto seek = mBoidManager->get_seek_target(this);
            if (seek != nullptr) {
                draw_line({0, 0}, to_local(seek->get_position()), Color("#228B22"));
            }
        }
        if (mCluster->arriveTarget()) {
            auto arrive = mBoidManager->get_arrive_target(this);
            if (arrive != nullptr) {
                draw_line({0, 0}, to_local(arrive->get_position()), Color("#6B8E23"));
            }
        }
        if (mCluster->pursuitTarget()) {
            auto pursuit = mBoidManager->get_pursuit_target(this, 10000);
            if (pursuit != nullptr) {
                draw_line({0, 0}, to_local(pursuit->get_position()), Color("#FFD700"));
            }
        }
        if (mCluster->evadeTarget()) {
            auto evade = mBoidManager->get_evade_target(this, mCluster->evadeRadius());
            if (evade != nullptr) {
                draw_line({0, 0}, to_local(evade->get_position()), Color("#FF8000"));
            }
        }
        if (mCluster->wander()) {
            draw_arc({0, 0}, mCluster->wanderRadius(),
                     mCluster->wanderAngle() - Math::deg_to_rad(mCluster->wanderAngleChangeLimit()),
                     mCluster->wanderAngle() + Math::deg_to_rad(mCluster->wanderAngleChangeLimit()), 10,
                     Color("#FF00FF"));
        }
        // for (auto& obstacle : mObstacleRects) {
        //     draw_dashed_line({0, 0}, to_local(obstacle.get_center()), Color("#FF0000"));
        // }
        // for (auto& obstacle : mObstacleCircles) {
        //     draw_dashed_line({0, 0}, to_local(obstacle.center), Color("#FF0000"));
        // }
        if (mCluster->fleeFromPredator()) {
            auto flee = mBoidManager->get_nearby_predator(this, mCluster->fleeFromPredatorDistance());
            for (int i = 0; i < flee.size(); i++) {
                auto node = Node::cast_to<Node2D>(flee[i].operator Object*());
                if (node != nullptr) {
                    draw_line({0, 0}, to_local(node->get_position()), Color("#B0171F"));
                }
            }
        }
    }
    return Sprite2D::_draw();
}

void BoidSprite2D::_physics_process(double delta) {
    if (mCluster != nullptr) {
        auto force        = mCluster->applyRules(this);
        auto acceleration = force / mass();
        mVelocity += acceleration * (delta * mCluster->gameTimeFrameRate());
        mVelocity = mVelocity.limit_length(mCluster->maxSpeed());
        set_position(get_position() + mVelocity * (delta * mCluster->gameTimeFrameRate()));
    }
    if (!mVelocity.is_zero_approx()) {
        mForward = mVelocity.normalized();
        set_rotation(Vector2(0, 1).angle_to(mForward));
    }
}

void BoidSprite2D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("updateManager", "manager"), &BoidSprite2D::updateManager);
    ClassDB::bind_method(D_METHOD("updateBoidCluster", "cluster"), &BoidSprite2D::updateBoidCluster);
    ADD_SIGNAL(MethodInfo("position_changed", PropertyInfo(Variant::OBJECT, "node"),
                          PropertyInfo(Variant::VECTOR2, "new_pos")));

#define PROPERTY_BINDER(type, name, getter, setter)                                                                    \
    ClassDB::bind_method(D_METHOD(#getter), &BoidSprite2D::name);                                                      \
    ClassDB::bind_method(D_METHOD(#setter, #name), &BoidSprite2D::setter);

    PROPERTY_BINDER(VECTOR2, mass, mass, setMass);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "0.0, 100.0, 0.1,or_greater"), "setMass",
                 "mass");

    PROPERTY_BINDER(VECTOR2, velocity, velocity, setVelocity)
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "velocity"), "setVelocity", "velocity");

    PROPERTY_BINDER(INT, boidType, boidType, setBoidType);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "boidType", PROPERTY_HINT_ENUM,
                              "None,Cluster,Seek,Arrive,Flee,Avoid,Pursuit,Evade,Obstacle,Predator,Following"),
                 "setBoidType", "boidType");

#undef PROPERTY_BINDER
}
} // namespace godot