#include "boid_node.hpp"

#include <godot_cpp/classes/shape2d.hpp>
#include <random>

#include "boid_manager_node.hpp"

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
        mTimeEmit          = 0.0;
        mPursuitTargetHash = 0;
        mEvadeTargetHash   = 0;
        mPredatorHash = 0;
    }

    // auto steering_force = _avoid_obstacles({Rect2{0, 0, 1001, 1001}}) * 5;
    // mAcceleration += steering_force.limit_length(mMaxForce) / mMass;
}

void BoidSprite2D::_draw() {
#if 1 // DEBUG
    auto position = get_position();
    draw_dashed_line({0, 0}, {10, 0}, Color("#FF00FF"));
    for (auto& boid : mNeighborBoids) {
        draw_dashed_line({0, 0}, to_local(boid->get_position()), Color("#00FF00"));
    }
    if (mSeekTargetV != nullptr) {
        draw_line({0, 0}, to_local(mSeekTargetV->get_position()), Color("#228B22"));
    }
    if (mArriveTargetV != nullptr) {
        draw_line({0, 0}, to_local(mArriveTargetV->get_position()), Color("#6B8E23"));
    }
    if (mPursuitTargetV != nullptr) {
        draw_line({0, 0}, to_local(mPursuitTargetV->get_position()), Color("#FFD700"));
    }
    if (mEvadeTargetV != nullptr) {
        draw_line({0, 0}, to_local(mEvadeTargetV->get_position()), Color("#FF8000"));
    }
    if (_check_behavior_flag(Wander)) {
        draw_arc({0, 0}, mWanderRadius, mWanderAngle - Math::deg_to_rad(mWanderAngleChangeLimit),
                 mWanderAngle + Math::deg_to_rad(mWanderAngleChangeLimit), 10, Color("#FF00FF"));
    }
    for (auto& obstacle : mObstacleRects) {
        draw_dashed_line({0, 0}, to_local(obstacle.get_center()), Color("#FF0000"));
    }
    for (auto& obstacle : mObstacleCircles) {
        draw_dashed_line({0, 0}, to_local(obstacle.center), Color("#FF0000"));
    }
    for (auto& predator : mPredatorV) {
        draw_line({0, 0}, to_local(predator->get_position()), Color("#B0171F"));
    }
#endif
    return Sprite2D::_draw();
}

void BoidSprite2D::_physics_process(double delta) {
    _apply_rules();
    mVelocity += mAcceleration * (delta * mGameTimeFrameRate);
    mVelocity = mVelocity.limit_length(mMaxSpeed);
    set_position(get_position() + mVelocity * (delta * mGameTimeFrameRate));
    mAcceleration = Vector2(0, 0);
    if (!mVelocity.is_zero_approx()) {
        mForward = mVelocity.normalized();
        set_rotation(Vector2(0, 1).angle_to(mForward));
    }
}

auto BoidSprite2D::_apply_rules() -> void {
    /// 基础应用力
    if (!mBoidManager) {
        return;
    }
    auto steering_force = Vector2(0, 0);
    if (_check_behavior_flag(Flocking)) {
        if (mBoidManager->data_hash(BoidManagerNode::Cluster) != mClusterHash) {
            // TODO: 这里的类型转换会涉及字符串比较
            mClusterHash   = mBoidManager->data_hash(BoidManagerNode::Cluster);
            auto neighbors = mBoidManager->get_nearby_same_cluster_nodes(
                this, std::max({mSeparationRadius, mAlignmentRadius, mCohesionRadius}));
            mNeighborBoids.clear();
            for (int i = 0; i < neighbors.size(); ++i) {
                if (auto* neighbor = Node::cast_to<BoidSprite2D>(neighbors[i].operator Object*()); neighbor) {
                    mNeighborBoids.push_back(neighbor);
                }
            }
        }
        steering_force = _flocking(mNeighborBoids); // TODO: 查找所有邻居
    }
    /// ------------- 高级应用力 ----------------
    if (_check_behavior_flag(Seek)) {
        if (mBoidManager->data_hash(BoidManagerNode::Seek) != mSeekTargetHash) {
            mSeekTargetHash = mBoidManager->data_hash(BoidManagerNode::Seek);
            mSeekTargetV    = mBoidManager->get_seek_target(this);
        }
        if (mSeekTargetV != nullptr) {
            steering_force += _seek_target(mSeekTargetV->get_position()) * mSeekTargetWeight;
        }
    }
    if (_check_behavior_flag(Arrive)) {
        if (mBoidManager->data_hash(BoidManagerNode::Arrive) != mArriveTargetHash) {
            mArriveTargetHash = mBoidManager->data_hash(BoidManagerNode::Arrive);
            mArriveTargetV    = mBoidManager->get_arrive_target(this);
        }
        if (mArriveTargetV != nullptr) {
            steering_force += _arrive(mArriveTargetV->get_position()) * mArriveWeight;
        }
    }
    if (_check_behavior_flag(Pursuit)) {
        if (mBoidManager->data_hash(BoidManagerNode::Pursuit) != mPursuitTargetHash) {
            mPursuitTargetHash = mBoidManager->data_hash(BoidManagerNode::Pursuit);
            mPursuitTargetV    = Node::cast_to<BoidSprite2D>(mBoidManager->get_pursuit_target(this, 10000));
        }
        if (mPursuitTargetV != nullptr) {
            steering_force += _pursuit(*mPursuitTargetV) * mPursuitWeight;
        }
    }

    if (_check_behavior_flag(Evade)) {
        if (mBoidManager->data_hash(BoidManagerNode::Evade) != mEvadeTargetHash) {
            mEvadeTargetHash = mBoidManager->data_hash(BoidManagerNode::Evade);
            mEvadeTargetV    = Node::cast_to<BoidSprite2D>(mBoidManager->get_evade_target(this, mEvadeRadius));
        }
        if (mEvadeTargetV != nullptr) {
            steering_force += _evade(*mEvadeTargetV) * mEvadeWeight;
        }
    }

    if (_check_behavior_flag(Wander)) {
        steering_force += _wander() * mWanderWeight;
    }

    if (_check_behavior_flag(Obstacle)) {
        if (mBoidManager->data_hash(BoidManagerNode::Obstacle) != mObstaclesHash) {
            mObstaclesHash = mBoidManager->data_hash(BoidManagerNode::Obstacle);
            auto obstacles = mBoidManager->get_nearby_obstacles(this, mAvoidObstaclesDistance);
            mObstacleCircles.clear();
            mObstacleRects.clear();
            for (int i = 0; i < obstacles.size(); ++i) {
                if (obstacles[i].get_type() == Variant::Type::RECT2) {
                    mObstacleRects.push_back(obstacles[i]);
                } else if (obstacles[i].get_type() == Variant::Type::VECTOR2) {
                    mObstacleCircles.push_back(math::Circle(obstacles[i], 10));
                }
            }
        }
        if (!mObstacleCircles.empty()) {
            steering_force += _avoid_obstacles(mObstacleCircles) * mAvoidObstaclesWeight;
        }
        if (!mObstacleRects.empty()) {
            steering_force += _avoid_obstacles(mObstacleRects) * mAvoidObstaclesWeight;
        }
    }

    if (_check_behavior_flag(Predator)) {
        if (mBoidManager->data_hash(BoidManagerNode::Predator) != mPredatorHash) {
            mPredatorHash  = mBoidManager->data_hash(BoidManagerNode::Predator);
            auto predators = mBoidManager->get_nearby_predator(this, mFleeFromPredatorDistance);
            mPredatorV.clear();
            for (int i = 0; i < predators.size(); ++i) {
                if (predators[i].get_type() == Variant::Type::OBJECT) {
                    auto* predator = Object::cast_to<Node2D>(predators[i]);
                    if (predator) {
                        mPredatorV.push_back(predator);
                    }
                }
            }
        }
        steering_force += _leave_from_predator(mPredatorV) * mFleeFromPredatorWeight;
    }

    if (_check_behavior_flag(Following)) {
        if (mBoidManager->data_hash(BoidManagerNode::Following) != mFollowingHash) {
            mFollowingHash = mBoidManager->data_hash(BoidManagerNode::Following);
            auto following = mBoidManager->get_following_path(this);
            mPathPoints.clear();
            mPathIndex = 0; // 初始化为0
            for (int i = 0; i < following.size(); ++i) {
                if (following[i].get_type() == Variant::Type::VECTOR2) { // 如果路径点是 Vector2
                    mPathPoints.push_back(following[i]);
                } else if (following[i].get_type() == Variant::Type::OBJECT) { // 如果路径点是 Node2D
                    auto* follow = Object::cast_to<Node2D>(following[i]);
                    if (follow) {
                        mPathPoints.push_back(follow->get_position());
                    }
                }
            }
        }

        if (!mPathPoints.empty()) {
            // 首次设置路径或路径更新后，找到最近的路径点作为起始
            if (mPathIndex == 0) { // 仅当未开始或路径被重置时查找最近点
                float min_dist_path = 1000000.0f;
                for (int i = 0; i < mPathPoints.size(); ++i) {
                    float dist_to_point = get_position().distance_to(mPathPoints[i]);
                    if (dist_to_point < min_dist_path) {
                        min_dist_path = dist_to_point;
                        mPathIndex    = i;
                    }
                }
            }

            // 如果当前目标点已达到，前进到下一个点
            // 使用一个比 `10` 更合适的阈值，例如 `mArriveSlowingRadius / 2`
            if (mPathIndex < mPathPoints.size()) {
                float dist_to_current_waypoint = get_position().distance_to(mPathPoints[mPathIndex]);
                if (dist_to_current_waypoint < 10.0f) { // 到达当前 waypoint 的阈值
                    mPathIndex++;
                }
            }

            // 应用力到当前或下一个 waypoint
            if (mPathIndex < mPathPoints.size()) {
                steering_force += _path_following(mPathPoints[mPathIndex]) * mFollowingPathWeight;
            } else {
                // 已到达路径终点，这里可以处理循环或停止
                mPathIndex = 0;             // 示例：循环路径
                if (!mPathPoints.empty()) { // 循环后继续寻找第一个点
                    steering_force += _path_following(mPathPoints[mPathIndex]) * mFollowingPathWeight;
                }
            }
        }
    }
    mAcceleration += steering_force.limit_length(mMaxForce) / mMass;
}

auto BoidSprite2D::_seek_target(const Vector2& target) -> Vector2 {
    if (target.is_equal_approx(this->get_position())) {
        return {0, 0};
    }
    return ((target - this->get_position()).normalized() * mMaxSpeed - velocity());
}

auto BoidSprite2D::_leave_target(const Vector2& target) -> Vector2 { return _seek_target(target) * -1; }

auto BoidSprite2D::_wander() -> Vector2 {
    auto randomEngine = std::mt19937(std::random_device{}());
    std::uniform_real_distribution<float> distribution(-mWanderAngleChangeLimit, mWanderAngleChangeLimit);
    // 1. 更新漫游角度 (随机小幅变化)
    mWanderAngle += Math::deg_to_rad(distribution(randomEngine));
    // 2. 计算漫游圆心 (在角色前方)
    auto circle_center = get_position() + velocity().normalized() * mWanderDistance;
    // 3. 计算圆上目标点 (根据漫游角度)
    auto displacement_on_circle = Vector2::from_angle(mWanderAngle) * mWanderRadius;
    auto target_weight          = circle_center + displacement_on_circle;
    return _seek_target(target_weight);
}

auto BoidSprite2D::_avoid_obstacles(const std::vector<math::Circle>& obstacles) -> Vector2 {
    auto steering_avoidance         = Vector2(0, 0);
    auto min_distance               = mAvoidObstaclesDistance;
    auto closest_intersection_point = Vector2(0, 0);
    auto closest_obstacle           = math::Circle({0, 0}, 0);
    bool found_intersection         = false;

    for (const auto& obstacle : obstacles) {
        auto rojected_position  = get_position() + velocity().normalized() * mAvoidObstaclesDistance;
        auto intersection_point = obstacle.get_intersection_point({get_position(), rojected_position});
        if (intersection_point.has_value()) {
            auto distance_to_obstacle = intersection_point->distance_to(get_position());
            if (distance_to_obstacle < min_distance) {
                min_distance               = distance_to_obstacle;
                found_intersection         = true;
                closest_obstacle           = obstacle;
                closest_intersection_point = *intersection_point;
            }
        }
    }

    if (found_intersection) {
        auto avoid_direction = (closest_intersection_point - closest_obstacle.center).normalized();
        auto force_magnitude =
            Math::max(0.0, mMaxForce * (1.0 - (min_distance / mAvoidObstaclesDistance))); // 距离越短，权重越大
        steering_avoidance = (avoid_direction * force_magnitude);
    }
    return steering_avoidance;
}

auto BoidSprite2D::_avoid_obstacles(const std::vector<Rect2>& obstacles) -> Vector2 {
    auto steering_avoidance          = Vector2(0, 0);
    auto min_distance                = mAvoidObstaclesDistance;
    auto closest_intersection_normal = Vector2(0, 0);
    bool found_intersection          = false;

    for (const auto& obstacle : obstacles) {
        auto rojected_position = get_position() + velocity().normalized() * mAvoidObstaclesDistance;
        Point2 intersection_point;
        Vector2 intersection_normal;
        if (obstacle.intersects_segment(get_position(), rojected_position, &intersection_point, &intersection_normal)) {
            auto distance_to_obstacle = intersection_point.distance_to(get_position());
            if (distance_to_obstacle < min_distance) {
                min_distance                = distance_to_obstacle;
                found_intersection          = true;
                closest_intersection_normal = intersection_normal;
            }
        }
    }

    if (found_intersection) {
        auto force_magnitude =
            Math::max(0.0, mMaxForce * (1.0 - (min_distance / mAvoidObstaclesDistance))); // 距离越短，权重越大
        steering_avoidance = (closest_intersection_normal.normalized() * force_magnitude);
    }
    return steering_avoidance;
}

auto BoidSprite2D::_arrive(const Vector2& target) -> Vector2 {
    auto to_target = target - this->get_position();
    auto distance  = to_target.length();
    if (distance < mArriveSlowingRadius) {
        auto target_speed = mMaxSpeed * (distance / mArriveSlowingRadius);
        return (to_target.normalized() * target_speed - velocity());
    } else {
        return _seek_target(target);
    }
}

auto BoidSprite2D::_path_following(const Vector2& next_waypoint) -> Vector2 { return _arrive(next_waypoint); }

void BoidSprite2D::_bind_methods() {

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

    PROPERTY_BINDER(VECTOR2, acceleration, acceleration, setAcceleration);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "acceleration"), "setAcceleration", "acceleration");

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

#undef PROPERTY_BINDER
}
} // namespace godot