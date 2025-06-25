#include "boid_cluster_node.hpp"

namespace godot {
auto BoidClusterNode::updateManager(BoidManagerNode* manager) -> void {
    mBoidManager = manager;
    if (mBoidManager != nullptr) {
        auto childs = get_children();
        for (int i = 0; i < childs.size(); i++) {
            if (childs[i].get_type() != Variant::OBJECT) {
                continue;
            }
            if (auto item = Node::cast_to<Node2D>(childs[i].operator Object*()); item != nullptr) {
                connect("manager_changed", Callable(item, "updateManager"));
                mBoidManager->insert(item, BoidType::Cluster);
            }
        }
    }
    emit_signal("manager_changed", this, mBoidManager);
}

auto BoidClusterNode::on_child_entered_tree(Node2D* node) -> void {
    if (node == nullptr) {
        return;
    }
    connect("manager_changed", Callable(node, "updateManager"));
    node->call("updateManager", mBoidManager);
    if (mBoidManager != nullptr) {
        mBoidManager->insert(node, BoidType::Cluster);
    }
    node->call("updateBoidCluster", this);
}

auto BoidClusterNode::on_child_exiting_tree(Node2D* node) -> void {
    if (node == nullptr) {
        return;
    }
    disconnect("manager_changed", Callable(node, "updateManager"));
    node->call("updateManager", nullptr);
    node->call("updateBoidCluster", nullptr);
    if (mBoidManager != nullptr) {
        mBoidManager->remove(node);
    }
}

void BoidClusterNode::_enter_tree() {
    auto parent = get_parent();
    if (parent != nullptr) {
        mBoidManager = Object::cast_to<BoidManagerNode>(parent);
    }
}

void BoidClusterNode::_exit_tree() {
    if (mBoidManager != nullptr) {
        auto children = get_children();
        for (int i = 0; i < children.size(); i++) {
            if (children[i].get_type() == Variant::Type::OBJECT) {
                auto node = Node::cast_to<Node2D>(children[i].operator Object*());
                mBoidManager->remove(node);
            }
        }
    }
    mBoidManager = nullptr;
    emit_signal("manager_changed", this, nullptr);
}

void BoidClusterNode::_ready() {
    auto children = get_children();
    for (int i = 0; i < children.size(); i++) {
        if (children[i].get_type() == Variant::Type::OBJECT) {
            auto node = Node::cast_to<Node2D>(children[i].operator Object*());
            on_child_entered_tree(node);
        }
    }

    Error err = this->connect("child_entered_tree", Callable(this, "on_child_entered_tree"));
    ERR_FAIL_COND_MSG(err != OK, "Failed to connect to child_entered_tree signal: " + String::num(err));
    err = this->connect("child_exiting_tree", Callable(this, "on_child_exiting_tree"));
    ERR_FAIL_COND_MSG(err != OK, "Failed to connect to child_exiting_tree signal: " + String::num(err));
}

void BoidClusterNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("on_child_entered_tree", "child"), &BoidClusterNode::on_child_entered_tree);
    ClassDB::bind_method(D_METHOD("on_child_exiting_tree", "child"), &BoidClusterNode::on_child_exiting_tree);
    ClassDB::bind_method(D_METHOD("updateManager", "manager"), &BoidClusterNode::updateManager);
    ADD_SIGNAL(MethodInfo("manager_changed", PropertyInfo(Variant::OBJECT, "manager")));

#define PROPERTY_BINDER(type, name, getter, setter)                                                                    \
    ClassDB::bind_method(D_METHOD(#getter), &BoidClusterNode::name);                                                   \
    ClassDB::bind_method(D_METHOD(#setter, #name), &BoidClusterNode::setter);

    ADD_GROUP("base", "base_");

    PROPERTY_BINDER(FLOAT, maxSpeed, maxSpeed, setMaxSpeed)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_maxSpeed", PROPERTY_HINT_RANGE, "0.0, 1000.0, 1"), "setMaxSpeed",
                 "maxSpeed");

    PROPERTY_BINDER(FLOAT, maxForce, maxForce, setMaxForce)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_maxForce", PROPERTY_HINT_RANGE, "0.0, 1000.0, 1"), "setMaxForce",
                 "maxForce");

    PROPERTY_BINDER(BOOL, flocking, flocking, setFlocking)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "base_flocking"), "setFlocking", "flocking");
    PROPERTY_BINDER(FLOAT, separationRadius, separationRadius, setSeparationRadius)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_separationRadius", PROPERTY_HINT_RANGE, "0.0, 1000.0, 0.01"),
                 "setSeparationRadius", "separationRadius");

    PROPERTY_BINDER(FLOAT, separationWeight, separationWeight, setSeparationWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_separationWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setSeparationWeight", "separationWeight");

    PROPERTY_BINDER(FLOAT, alignmentRadius, alignmentRadius, setAlignmentRadius)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_alignmentRadius", PROPERTY_HINT_RANGE, "0.0, 1000.0, 0.01"),
                 "setAlignmentRadius", "alignmentRadius");

    PROPERTY_BINDER(FLOAT, alignmentWeight, alignmentWeight, setAlignmentWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_alignmentWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setAlignmentWeight", "alignmentWeight");

    PROPERTY_BINDER(FLOAT, cohesionRadius, cohesionRadius, setCohesionRadius)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_cohesionRadius", PROPERTY_HINT_RANGE, "0.0, 1000.0, 0.01"),
                 "setCohesionRadius", "cohesionRadius");

    PROPERTY_BINDER(FLOAT, cohesionWeight, cohesionWeight, setCohesionWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "base_cohesionWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setCohesionWeight", "cohesionWeight");

    ADD_GROUP("advance", "advance_");
    // ----------------- seek target -----------------
    PROPERTY_BINDER(BOOL, seekTarget, seekTarget, setSeekTarget)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_seekTarget"), "setSeekTarget", "seekTarget");
    PROPERTY_BINDER(FLOAT, seekTargetWeight, seekTargetWeight, setSeekTargetWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_seekTargetWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setSeekTargetWeight", "seekTargetWeight");

    // ----------------- arrive -----------------
    PROPERTY_BINDER(BOOL, arriveTarget, arriveTarget, setArriveTarget);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_arriveTarget"), "setArriveTarget", "arriveTarget");
    PROPERTY_BINDER(FLOAT, arriveWeight, arriveWeight, setArriveWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_arriveWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setArriveWeight", "arriveWeight");
    PROPERTY_BINDER(FLOAT, arriveSlowingRadius, arriveSlowingRadius, setArriveSlowingRadius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_arriveSlowingRadius", PROPERTY_HINT_RANGE, "0.0, 1000.0, 1"),
                 "setArriveSlowingRadius", "arriveSlowingRadius");

    // ----------------- pursuit -----------------
    PROPERTY_BINDER(BOOL, pursuitTarget, pursuitTarget, setPursuitTarget);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_pursuitTarget"), "setPursuitTarget", "pursuitTarget");
    PROPERTY_BINDER(FLOAT, pursuitWeight, pursuitWeight, setPursuitWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_pursuitWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setPursuitWeight", "pursuitWeight");

    // ----------------- evade -----------------
    PROPERTY_BINDER(BOOL, evadeTarget, evadeTarget, setEvadeTarget);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_evadeTarget"), "setEvadeTarget", "evadeTarget");
    PROPERTY_BINDER(FLOAT, evadeWeight, evadeWeight, setEvadeWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_evadeWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setEvadeWeight", "evadeWeight");
    PROPERTY_BINDER(FLOAT, evadeRadius, evadeRadius, setEvadeRadius)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_evadeRadius", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setEvadeRadius", "evadeRadius");

    // ----------------- wander -----------------
    PROPERTY_BINDER(BOOL, wander, wander, setWander)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_wander"), "setWander", "wander");
    PROPERTY_BINDER(FLOAT, wanderWeight, wanderWeight, setWanderWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_wanderWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setWanderWeight", "wanderWeight");
    PROPERTY_BINDER(FLOAT, wanderRadius, wanderRadius, setWanderRadius)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_wanderRadius", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setWanderRadius", "wanderRadius");
    PROPERTY_BINDER(FLOAT, wanderDistance, wanderDistance, setWanderDistance);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_wanderDistance", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setWanderDistance", "wanderDistance");
    PROPERTY_BINDER(FLOAT, wanderAngleChangeLimit, wanderAngleChangeLimit, setWanderAngleChangeLimit)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_wanderAngleChangeLimit", PROPERTY_HINT_RANGE, "0.0, 180.0, 1"),
                 "setWanderAngleChangeLimit", "wanderAngleChangeLimit");

    // ----------------- obstruct avoidance -----------------
    PROPERTY_BINDER(BOOL, avoidObstacles, avoidObstacles, setAvoidObstacles)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_avoidObstacles"), "setAvoidObstacles", "avoidObstacles");
    PROPERTY_BINDER(FLOAT, avoidObstaclesWeight, avoidObstaclesWeight, setAvoidObstaclesWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_avoidObstaclesWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setAvoidObstaclesWeight", "avoidObstaclesWeight");
    PROPERTY_BINDER(FLOAT, avoidObstaclesDistance, avoidObstaclesDistance, setAvoidObstaclesDistance)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_avoidObstaclesDistance", PROPERTY_HINT_RANGE, "0.0, 1000.0, 1"),
                 "setAvoidObstaclesDistance", "avoidObstaclesDistance");

    // ----------------- flee from predator -----------------
    PROPERTY_BINDER(BOOL, fleeFromPredator, fleeFromPredator, setFleeFromPredator)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_fleeFromPredator"), "setFleeFromPredator", "fleeFromPredator");
    PROPERTY_BINDER(FLOAT, fleeFromPredatorWeight, fleeFromPredatorWeight, setFleeFromPredatorWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_fleeFromPredatorWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setFleeFromPredatorWeight", "fleeFromPredatorWeight");
    PROPERTY_BINDER(FLOAT, fleeFromPredatorDistance, fleeFromPredatorDistance, setFleeFromPredatorDistance)
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "advance_fleeFromPredatorDistance", PROPERTY_HINT_RANGE, "0.0, 1000.0, 1"),
        "setFleeFromPredatorDistance", "fleeFromPredatorDistance");

    // ----------------- following path -----------------
    PROPERTY_BINDER(BOOL, followingPath, followingPath, setFollowingPath)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "advance_followingPath"), "setFollowingPath", "followingPath");
    PROPERTY_BINDER(FLOAT, followingPathWeight, followingPathWeight, setFollowingPathWeight)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "advance_followingPathWeight", PROPERTY_HINT_RANGE, "0.0, 10.0, 1"),
                 "setFollowingPathWeight", "followingPathWeight");

    ADD_GROUP("system", "system_");
    PROPERTY_BINDER(FLOAT, gameTimeFrameRate, gameTimeFrameRate, setGameTimeFrameRate)
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "system_gameTimeFrameRate", PROPERTY_HINT_RANGE, "0.0, 1000.0, 1"),
                 "setGameTimeFrameRate", "gameTimeFrameRate");
    PROPERTY_BINDER(BOOL, isDebugDraw, isDebugDraw, setDebugDraw)
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "system_isDebugDraw"), "setDebugDraw", "isDebugDraw");

#undef PROPERTY_BINDER
}

} // namespace godot