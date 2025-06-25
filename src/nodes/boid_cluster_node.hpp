#pragma once

#include <godot_cpp/classes/node2d.hpp>

#include <random>

#include "../algorithm/linear_algebra.hpp"
#include "boid_manager_node.hpp"

namespace godot {
enum class BoidBehavior : uint32_t {
    Flocking  = 1 << 0, // 群聚
    Seek      = 1 << 1, // 寻找目标
    Arrive    = 1 << 2, // 到达目标
    Pursuit   = 1 << 3, // 追逐目标
    Evade     = 1 << 4, // 逃离目标
    Wander    = 1 << 5, // 徘徊
    Obstacle  = 1 << 6, // 避障
    Predator  = 1 << 7, // 逃避捕食
    Flee      = 1 << 8, // 逃离
    Following = 1 << 9, // 路径跟随
};

class BoidClusterNode : public Node {
    GDCLASS(BoidClusterNode, Node)
    friend class Node;

public:
    void setMaxSpeed(float speed) { mMaxSpeed = speed; }
    auto maxSpeed() const -> float { return mMaxSpeed; }
    void setMaxForce(float force) { mMaxForce = force; }
    auto maxForce() const -> float { return mMaxForce; }

    auto flocking() const -> bool { return _check_behavior_flag(BoidBehavior::Flocking); }
    void setFlocking(bool flocking) { _set_behavior_flag(BoidBehavior::Flocking, flocking); }
    auto separationRadius() const -> float { return mSeparationRadius; }
    void setSeparationRadius(float radius) { mSeparationRadius = radius; }
    auto separationWeight() const -> float { return mSeparationWeight; }
    void setSeparationWeight(float weight) { mSeparationWeight = weight; }
    auto alignmentRadius() const -> float { return mAlignmentRadius; }
    void setAlignmentRadius(float distance) { mAlignmentRadius = distance; }
    auto alignmentWeight() const -> float { return mAlignmentWeight; }
    void setAlignmentWeight(float weight) { mAlignmentWeight = weight; }
    auto cohesionRadius() const -> float { return mCohesionRadius; }
    void setCohesionRadius(float distance) { mCohesionRadius = distance; }
    auto cohesionWeight() const -> float { return mCohesionWeight; }
    void setCohesionWeight(float weight) { mCohesionWeight = weight; }

    // advanced rules
    auto seekTarget() const -> bool { return _check_behavior_flag(BoidBehavior::Seek); }
    void setSeekTarget(bool seek) { _set_behavior_flag(BoidBehavior::Seek, seek); }
    auto seekTargetWeight() const -> float { return mSeekTargetWeight; }
    void setSeekTargetWeight(float weight) { mSeekTargetWeight = weight; }

    auto arriveTarget() const -> bool { return _check_behavior_flag(BoidBehavior::Arrive); }
    void setArriveTarget(bool arrive) { _set_behavior_flag(BoidBehavior::Arrive, arrive); }
    auto arriveWeight() const -> float { return mArriveWeight; }
    void setArriveWeight(float weight) { mArriveWeight = weight; }
    auto arriveSlowingRadius() const -> float { return mArriveSlowingRadius; }
    void setArriveSlowingRadius(float radius) { mArriveSlowingRadius = radius; }

    auto pursuitTarget() const -> bool { return _check_behavior_flag(BoidBehavior::Pursuit); }
    void setPursuitTarget(bool pursuit) { _set_behavior_flag(BoidBehavior::Pursuit, pursuit); }
    auto pursuitWeight() const -> float { return mPursuitWeight; }
    void setPursuitWeight(float weight) { mPursuitWeight = weight; }

    auto evadeTarget() const -> bool { return _check_behavior_flag(BoidBehavior::Evade); }
    void setEvadeTarget(bool evade) { _set_behavior_flag(BoidBehavior::Evade, evade); }
    auto evadeWeight() const -> float { return mEvadeWeight; }
    void setEvadeWeight(float weight) { mEvadeWeight = weight; }
    auto evadeRadius() const -> float { return mEvadeRadius; }
    void setEvadeRadius(float radius) { mEvadeRadius = radius; }

    auto wander() const -> bool { return _check_behavior_flag(BoidBehavior::Wander); }
    void setWander(bool wander) { _set_behavior_flag(BoidBehavior::Wander, wander); }
    auto wanderWeight() const -> float { return mWanderWeight; }
    void setWanderWeight(float weight) { mWanderWeight = weight; }
    auto wanderAngle() const -> float { return mWanderAngle; }
    void setWanderAngle(float angle) { mWanderAngle = angle; }
    auto wanderDistance() const -> float { return mWanderDistance; }
    void setWanderDistance(float distance) { mWanderDistance = distance; }
    auto wanderRadius() const -> float { return mWanderRadius; }
    void setWanderRadius(float radius) { mWanderRadius = radius; }
    auto wanderAngleChangeLimit() const -> float { return mWanderAngleChangeLimit; }
    void setWanderAngleChangeLimit(float limit) { mWanderAngleChangeLimit = limit; }

    auto avoidObstacles() const -> bool { return _check_behavior_flag(BoidBehavior::Obstacle); }
    void setAvoidObstacles(bool avoid) { _set_behavior_flag(BoidBehavior::Obstacle, avoid); }
    auto avoidObstaclesWeight() const -> float { return mAvoidObstaclesWeight; }
    void setAvoidObstaclesWeight(float weight) { mAvoidObstaclesWeight = weight; }
    auto avoidObstaclesDistance() const -> float { return mAvoidObstaclesDistance; }
    void setAvoidObstaclesDistance(float distance) { mAvoidObstaclesDistance = distance; }

    auto fleeFromPredator() const -> bool { return _check_behavior_flag(BoidBehavior::Predator); }
    void setFleeFromPredator(bool flee) { _set_behavior_flag(BoidBehavior::Predator, flee); }
    auto fleeFromPredatorWeight() const -> float { return mFleeFromPredatorWeight; }
    void setFleeFromPredatorWeight(float weight) { mFleeFromPredatorWeight = weight; }
    auto fleeFromPredatorDistance() const -> float { return mFleeFromPredatorDistance; }
    void setFleeFromPredatorDistance(float distance) { mFleeFromPredatorDistance = distance; }

    auto followingPath() const -> bool { return _check_behavior_flag(BoidBehavior::Following); }
    void setFollowingPath(bool follow) { _set_behavior_flag(BoidBehavior::Following, follow); }
    auto followingPathWeight() const -> float { return mFollowingPathWeight; }
    void setFollowingPathWeight(float weight) { mFollowingPathWeight = weight; }

    // system
    auto gameTimeFrameRate() const -> float { return mGameTimeFrameRate; }
    void setGameTimeFrameRate(float frameRate) { mGameTimeFrameRate = frameRate; }
    auto isDebugDraw() const -> bool { return mIsDebugDraw; }
    void setDebugDraw(bool debugDraw) { mIsDebugDraw = debugDraw; }

    auto updateManager(BoidManagerNode* manager) -> void;

    auto on_child_entered_tree(Node2D* node) -> void;
    auto on_child_exiting_tree(Node2D* node) -> void;

    template <typename T>
    Vector2 applyRules(T* self);

    /**
     * @brief 角色直线地向一个目标点移动。
     * 计算从角色位置指向目标位置的向量，将其标准化并乘以最大速度得到“期望速度”，然后计算达到这个期望速度所需的力。
     * @param target
     * @return Vector2
     */
    template <typename T>
    auto seekTargetRule(T* self, const Vector2& target) -> Vector2;

    /**
     * @brief 角色直线地远离一个目标点移动。
     * 与寻路相反，计算从目标位置指向角色位置的向量，标准化并乘以最大速度，然后计算达到这个期望速度所需的力。
     * @param target
     * @return Vector2
     */
    template <typename T>
    auto leaveTargetRule(T* self, const Vector2& target) -> Vector2;

    /**
     * @brief 群体行为 (Flocking)
     * 1. 分离 (Separation): 避免与附近的群体成员发生碰撞，保持最小距离。
     * 2. 对齐 (Alignment): 调整自己的速度和方向，以匹配附近群体成员的平均速度和方向。
     * 3. 聚合 (Cohesion): 朝向附近群体成员的平均位置（群体中心）移动，使群体保持在一起。
     * @par 分离 (Separation)
     * 遍历所有附近的群体成员，对于每个成员，计算一个远离它的力，该力的大小与距离成反比。将所有这些力叠加。
     * @par 对齐 (Alignment)
     * 找到所有附近群体成员的平均速度，然后计算一个力，使自己的速度趋向这个平均速度。
     * @par 聚合 (Cohesion)
     * 找到所有附近群体成员的中心点（平均位置），然后对该中心点执行“寻路”行为。
     * @tparam U
     * @param neighbors
     * @return Vector2
     */
    template <typename T>
    auto flockingRule(T* self, const std::vector<Node2D*>& neighbors) -> Vector2;

    /**
     * @brief 逃离行为
     * 遍历所有附近的捕食者，对于每个捕食者，计算一个远离它的力，该力的大小与距离成反比。将所有这些力叠加。
     * @tparam U
     * @param predators
     * @return Vector2
     */
    template <typename T>
    auto leaveFromPredatorRule(T* self, const std::vector<Node2D*>& predators) -> Vector2;

    /**
     * @brief 角色向目标点移动，但在接近目标时会减速，最终平滑地停在目标点。
     * 结合了寻路，但根据与目标的距离来调整期望速度的幅度。距离越近，期望速度越小。
     * @param target
     * @return Vector2
     */
    template <typename T>
    auto arriveRule(T* self, const Vector2& target) -> Vector2;

    /**
     * @brief 角色追逐一个移动的目标，通过预测目标的未来位置来更有效地拦截。
     * 根据目标的速度和当前距离，预测目标在未来某个时间点的可能位置，然后对那个预测位置执行“寻路”或“到达”行为。
     * @param target
     * @return Vector2
     */
    template <typename T>
    auto pursuitRule(T* self, Node2D* target) -> Vector2;

    /**
     * @brief 角色躲避一个移动的追逐者，通过预测追逐者的未来位置来提前逃离。
     * 与追逐相反，预测追逐者未来可能到达的位置，然后从那个预测位置执行“逃离”行为。
     * @param target
     * @return Vector2
     */
    template <typename T>
    auto evadeRule(T* self, Node2D* target) -> Vector2;

    /**
     * @brief 角色进行一种不设特定目标的随机、持续的运动，看起来像是在四处游荡。
     * 通常在一个角色前方的一个假想圆上选择一个随机点作为目标，然后寻路到那个点。随着时间推移，随机点在圆上小幅移动，造成连续的漫游效果。
     * @param target
     * @return Vector2
     */
    template <typename T>
    auto wanderRule(T* self) -> Vector2;

    /**
     * @brief 角色在移动时检测并避开前方的障碍物，防止碰撞。
     * 通常通过在角色前方发射“探测线”（或使用包围体投影）来检测潜在碰撞。如果检测到碰撞，则计算一个推离障碍物的力，力的大小与碰撞的紧迫程度相关。
     * @param obstacles 圆形障碍物
     * @return Vector2
     */
    template <typename T>
    auto avoidObstaclesRule(T* self, const std::vector<math::Circle>& obstacles) -> Vector2;
    template <typename T>
    auto avoidObstaclesRule(T* self, const std::vector<Rect2>& obstacles) -> Vector2; // 矩形障碍物

    /**
     * @brief 角色沿着预定义的路径（通常是waypoint序列）移动。
     * 找到角色在路径上的当前点，然后寻找路径上该点前方的一个目标点（下一个waypoint或投影点），并对该点执行“寻路”或“到达”行为。
     * @param path 一个函数，返回给定位置时路径上的下一个目标点
     * @return Vector2
     */
    template <typename T>
    auto pathFollowingRule(T* self, const Vector2& next_waypoint) -> Vector2;

protected:
    void _ready() override;
    void _enter_tree() override;
    void _exit_tree() override;
    static void _bind_methods(); // 绑定方法

    auto _check_behavior_flag(BoidBehavior flag) const -> bool { return mEnableFlags & (uint32_t)flag; }
    auto _set_behavior_flag(BoidBehavior flag, bool value = true) -> void {
        if (value) {
            mEnableFlags |= (uint32_t)flag;
        } else {
            mEnableFlags &= ~(uint32_t)flag;
        }
    }

private:
    float mMaxSpeed       = 5.0;                              // boid 的最大速度
    float mMaxForce       = 0.1;                              // boid 的最大加速度
    uint32_t mEnableFlags = (uint32_t)BoidBehavior::Flocking; // 行为使能标志

    // Flocking : 群体行为
    float mSeparationWeight = 1.5;   // 分离规则权重
    float mSeparationRadius = 40.0;  // 感知半径
    float mAlignmentWeight  = 1.0;   // 对齐规则权重
    float mAlignmentRadius  = 50.0;  // 感知半径
    float mCohesionWeight   = 1.0;   // 聚集规则权重
    float mCohesionRadius   = 100.0; // 感知半径

    // ------------- advanced rules ----------------
    // seek: 寻找目标
    float mSeekTargetWeight  = 1.0; // 寻找目标规则权重
    Node2D* mSeekTargetV     = nullptr;
    uint64_t mSeekTargetHash = 0;

    // arrive: 抵达
    float mArriveWeight        = 1.0;  // 抵达目标规则权重
    float mArriveSlowingRadius = 30.0; // 抵达目标减速半径
    Node2D* mArriveTargetV     = nullptr;
    uint64_t mArriveTargetHash = 0;

    // pursuit: 追逐
    float mPursuitWeight = 1.0; // 追逐规则权重

    // evade: 躲避
    float mEvadeWeight = 1.0;  // 躲避规则权重
    float mEvadeRadius = 50.0; // 躲避距离

    // wander: 漫步
    float mWanderWeight           = 1.0;  // 漫游规则权重
    float mWanderAngle            = 0.0;  // 漫步角度
    float mWanderDistance         = 10.0; // 漫步圆心距离
    float mWanderRadius           = 10.0; // 漫步圆半径
    float mWanderAngleChangeLimit = 10.0; // 漫步角度变化限制

    // obstacle avoidance: 避障
    float mAvoidObstaclesWeight   = 2.0;  // 避免障碍物规则权重
    float mAvoidObstaclesDistance = 20.0; // 避免障碍物的距离
    std::vector<math::Circle> mObstacleCircles;
    std::vector<Rect2> mObstacleRects;
    uint64_t mObstaclesHash = 0;

    // flee from predator: 逃离捕食者
    float mFleeFromPredatorWeight   = 3.0;   // 逃离捕食者规则权重
    float mFleeFromPredatorDistance = 150.0; // 捕食者逃离距离

    // following path : 跟随路径
    float mFollowingPathWeight = 1.0; // 跟随路径规则权重
    std::vector<Vector2> mPathPoints;
    uint64_t mFollowingHash = 0;

    // system
    float mGameTimeFrameRate = 30.0;  // 游戏速度, 1s 等价的游戏时间
    bool mIsDebugDraw        = false; // 是否开启调试绘制

    BoidManagerNode* mBoidManager = nullptr; // 引用BoidManagerNode
};

template <typename T>
Vector2 BoidClusterNode::applyRules(T* self) {
    /// 基础应用力
    if (!mBoidManager) {
        return Vector2(0, 0);
    }
    auto steering_force = Vector2(0, 0);
    if (_check_behavior_flag(BoidBehavior::Flocking)) {
        std::vector<Node2D*> neighborBoids;
        auto neighbors = mBoidManager->get_nearby_same_cluster_nodes(
            self, std::max({mSeparationRadius, mAlignmentRadius, mCohesionRadius}));
        neighborBoids.clear();
        for (int i = 0; i < neighbors.size(); ++i) {
            if (auto* neighbor = Node::cast_to<Node2D>(neighbors[i].operator Object*()); neighbor) {
                neighborBoids.push_back(neighbor);
            }
        }
        steering_force = flockingRule(self, neighborBoids); // TODO: 查找所有邻居
    }
    /// ------------- 高级应用力 ----------------
    if (_check_behavior_flag(BoidBehavior::Seek)) {
        if (mBoidManager->data_hash(BoidType::Seek) != mSeekTargetHash) {
            mSeekTargetHash = mBoidManager->data_hash(BoidType::Seek);
            mSeekTargetV    = mBoidManager->get_seek_target(self);
        }
        if (mSeekTargetV != nullptr) {
            steering_force += seekTargetRule(self, mSeekTargetV->get_position()) * mSeekTargetWeight;
        }
    }
    if (_check_behavior_flag(BoidBehavior::Arrive)) {
        if (mBoidManager->data_hash(BoidType::Arrive) != mArriveTargetHash) {
            mArriveTargetHash = mBoidManager->data_hash(BoidType::Arrive);
            mArriveTargetV    = mBoidManager->get_arrive_target(self);
        }
        if (mArriveTargetV != nullptr) {
            steering_force += arriveRule(self, mArriveTargetV->get_position()) * mArriveWeight;
        }
    }
    if (_check_behavior_flag(BoidBehavior::Pursuit)) {
        auto pursuitTargetV = Node::cast_to<Node2D>(mBoidManager->get_pursuit_target(self, 10000));
        if (pursuitTargetV != nullptr) {
            steering_force += pursuitRule(self, pursuitTargetV) * mPursuitWeight;
        }
    }

    if (_check_behavior_flag(BoidBehavior::Evade)) {
        auto evadeTargetV = Node::cast_to<Node2D>(mBoidManager->get_evade_target(self, mEvadeRadius));
        if (evadeTargetV != nullptr) {
            steering_force += evadeRule(self, evadeTargetV) * mEvadeWeight;
        }
    }

    if (_check_behavior_flag(BoidBehavior::Wander)) {
        steering_force += wanderRule(self) * mWanderWeight;
    }

    if (_check_behavior_flag(BoidBehavior::Obstacle)) {
        if (mBoidManager->data_hash(BoidType::Obstacle) != mObstaclesHash) {
            mObstaclesHash = mBoidManager->data_hash(BoidType::Obstacle);
            auto obstacles = mBoidManager->get_nearby_obstacles(self, mAvoidObstaclesDistance);
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
            steering_force += avoidObstaclesRule(self, mObstacleCircles) * mAvoidObstaclesWeight;
        }
        if (!mObstacleRects.empty()) {
            steering_force += avoidObstaclesRule(self, mObstacleRects) * mAvoidObstaclesWeight;
        }
    }

    if (_check_behavior_flag(BoidBehavior::Predator)) {
        std::vector<Node2D*> predators_v;
        auto predators = mBoidManager->get_nearby_predator(self, mFleeFromPredatorDistance);
        for (int i = 0; i < predators.size(); ++i) {
            if (predators[i].get_type() == Variant::Type::OBJECT) {
                auto* predator = Object::cast_to<Node2D>(predators[i]);
                if (predator) {
                    predators_v.push_back(predator);
                }
            }
        }
        steering_force += leaveFromPredatorRule(self, predators_v) * mFleeFromPredatorWeight;
    }

    if (_check_behavior_flag(BoidBehavior::Following)) {
        if (mBoidManager->data_hash(BoidType::Following) != mFollowingHash) {
            mFollowingHash = mBoidManager->data_hash(BoidType::Following);
            auto following = mBoidManager->get_following_path(self);
            mPathPoints.clear();
            for (int i = following.size(); i > 0; --i) {
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
            int idx             = 0;
            float min_dist_path = 1000000.0f;
            for (int i = (int)mPathPoints.size() - 1; i > 0; --i) {
                float dist_to_point = self->get_position().distance_to(mPathPoints[i]);
                if (dist_to_point < min_dist_path) {
                    min_dist_path = dist_to_point;
                    idx           = i;
                } else {
                    break;
                }
            }

            // 如果当前目标点已达到，前进到下一个点
            // 使用一个比 `10` 更合适的阈值，例如 `mArriveSlowingRadius / 2`
            if (idx < mPathPoints.size()) {
                float dist_to_current_waypoint = self->get_position().distance_to(mPathPoints[idx]);
                if (dist_to_current_waypoint < 10.0f) { // 到达当前 waypoint 的阈值
                    idx++;
                } else if (idx < mPathPoints.size() - 1) {
                    auto cur_to_self = self->get_position() - mPathPoints[idx];
                    auto cur_to_next = mPathPoints[idx + 1] - mPathPoints[idx];
                    if (cur_to_self.dot(cur_to_next) > 0) {
                        idx++;
                    }
                }
            }

            // 应用力到当前或下一个 waypoint
            if (idx < mPathPoints.size() - 1) {
                steering_force += seekTargetRule(self, mPathPoints[idx]) * mFollowingPathWeight;
            } else if (idx == mPathPoints.size() - 1) {
                steering_force += arriveRule(self, mPathPoints[idx]) * mFollowingPathWeight;
            }
        }
    }
    return steering_force.limit_length(mMaxForce);
}

template <typename T>
auto BoidClusterNode::seekTargetRule(T* self, const Vector2& target) -> Vector2 {
    if (target.is_equal_approx(self->get_position())) {
        return {0, 0};
    }
    return ((target - self->get_position()).normalized() * mMaxSpeed - self->velocity());
}

template <typename T>
auto BoidClusterNode::leaveTargetRule(T* self, const Vector2& target) -> Vector2 {
    return seekTargetRule(self, target) * -1;
}

template <typename T>
auto BoidClusterNode::flockingRule(T* self, const std::vector<Node2D*>& neighbors) -> Vector2 {
    auto separation    = Vector2(0, 0);
    auto alignment     = Vector2(0, 0);
    auto cohesion      = Vector2(0, 0);
    auto sepTotal      = 0;
    auto alignTotal    = 0;
    auto cohesionTotal = 0;

    for (auto* other : neighbors) {
        auto distance = std::fabs(self->get_position().distance_to(other->get_position()));
        if (distance > 0) {
            if (distance < mSeparationRadius) {
                Vector2 dic = (self->get_position() - other->get_position());
                if (dic.is_zero_approx()) {
                    dic = Vector2(1, 0); // 避免除以零
                }
                separation += dic.normalized() * (mMaxSpeed * mSeparationRadius / (mSeparationRadius - distance));
                sepTotal++;
            }
            if (distance < mAlignmentRadius) {
                auto other_velocity = other->call("velocity");
                alignment +=
                    other_velocity.get_type() == Variant::VECTOR2 ? other_velocity.operator Vector2() : Vector2(0, 0);
                alignTotal++;
            }
            if (distance < mCohesionRadius) {
                cohesion += other->get_position();
                cohesionTotal++;
            }
        }
    }
    if (sepTotal > 0) {
        separation /= static_cast<float>(sepTotal);
    }
    if (alignTotal > 0) {
        alignment /= static_cast<float>(alignTotal);
        alignment = (alignment - self->velocity());
    }
    if (cohesionTotal > 0) {
        cohesion /= static_cast<float>(cohesionTotal);
        cohesion = seekTargetRule(self, cohesion);
    }

    return (separation * mSeparationWeight) + (alignment - self->velocity()) * mAlignmentWeight +
           cohesion * mCohesionWeight;
}

template <typename T>
auto BoidClusterNode::leaveFromPredatorRule(T* self, const std::vector<Node2D*>& predators) -> Vector2 {
    auto steering_predator = Vector2(0, 0);
    for (const auto& predator : predators) {
        auto distance = predator->get_position().distance_to(self->get_position());
        if (distance < mFleeFromPredatorDistance) {
            auto force = leaveTargetRule(self, predator->get_position());
            steering_predator += force;
        }
    }
    return steering_predator;
}

template <typename T>
auto BoidClusterNode::arriveRule(T* self, const Vector2& target) -> Vector2 {
    auto to_target = target - self->get_position();
    auto distance  = to_target.length();
    if (distance < mArriveSlowingRadius) {
        auto target_speed = mMaxSpeed * (distance / mArriveSlowingRadius);
        return (to_target.normalized() * target_speed - self->velocity());
    } else {
        return seekTargetRule(self, target);
    }
}

template <typename T>
auto BoidClusterNode::pursuitRule(T* self, Node2D* target) -> Vector2 {
    auto to_target         = target->get_position() - self->get_position();
    auto target_velocity_v = target->call("velocity");
    auto target_velocity =
        target_velocity_v.get_type() == Variant::VECTOR2 ? target_velocity_v.operator Vector2() : Vector2(0, 0);
    auto relative_heading  = self->velocity().dot(target_velocity);
    double prediction_time = 0;
    if (relative_heading < -0.95f && to_target.length() > 0) {
        prediction_time = to_target.length() / mMaxSpeed;
    } else {
        prediction_time = to_target.length() / (target_velocity.length() + 0.001); // 避免除以零
    }

    auto predicted_target_position = target->get_position() + target_velocity * prediction_time;
    return seekTargetRule(self, predicted_target_position);
}

template <typename T>
auto BoidClusterNode::evadeRule(T* self, Node2D* target) -> Vector2 {
    auto to_target = target->get_position() - self->get_position();
    auto distance  = to_target.length();
    if (distance > mEvadeRadius) {
        return {0, 0};
    }
    auto target_velocity_v = target->call("velocity");
    auto target_velocity =
        target_velocity_v.get_type() == Variant::VECTOR2 ? target_velocity_v.operator Vector2() : Vector2(0, 0);
    auto prediction_time            = distance / (target_velocity.length() + 0.001); // 避免除以零
    auto predicted_pursuer_position = target->get_position() + target_velocity * prediction_time;
    return leaveTargetRule(self, predicted_pursuer_position);
}

template <typename T>
auto BoidClusterNode::wanderRule(T* self) -> Vector2 {
    auto randomEngine = std::mt19937(std::random_device{}());
    std::uniform_real_distribution<float> distribution(-mWanderAngleChangeLimit, mWanderAngleChangeLimit);
    // 1. 更新漫游角度 (随机小幅变化)
    mWanderAngle += Math::deg_to_rad(distribution(randomEngine));
    // 2. 计算漫游圆心 (在角色前方)
    auto circle_center = self->get_position() + self->velocity().normalized() * mWanderDistance;
    // 3. 计算圆上目标点 (根据漫游角度)
    auto displacement_on_circle = Vector2::from_angle(mWanderAngle) * mWanderRadius;
    auto target_weight          = circle_center + displacement_on_circle;
    return seekTargetRule(self, target_weight);
}

template <typename T>
auto BoidClusterNode::avoidObstaclesRule(T* self, const std::vector<math::Circle>& obstacles) -> Vector2 {
    auto steering_avoidance         = Vector2(0, 0);
    auto min_distance               = mAvoidObstaclesDistance;
    auto closest_intersection_point = Vector2(0, 0);
    auto closest_obstacle           = math::Circle({0, 0}, 0);
    bool found_intersection         = false;

    for (const auto& obstacle : obstacles) {
        auto rojected_position  = self->get_position() + self->velocity().normalized() * mAvoidObstaclesDistance;
        auto intersection_point = obstacle.get_intersection_point({self->get_position(), rojected_position});
        if (intersection_point.has_value()) {
            auto distance_to_obstacle = intersection_point->distance_to(self->get_position());
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

template <typename T>
auto BoidClusterNode::avoidObstaclesRule(T* self, const std::vector<Rect2>& obstacles) -> Vector2 {
    auto steering_avoidance          = Vector2(0, 0);
    auto min_distance                = mAvoidObstaclesDistance;
    auto closest_intersection_normal = Vector2(0, 0);
    bool found_intersection          = false;

    for (const auto& obstacle : obstacles) {
        auto rojected_position = self->get_position() + self->velocity().normalized() * mAvoidObstaclesDistance;
        Point2 intersection_point;
        Vector2 intersection_normal;
        if (obstacle.intersects_segment(self->get_position(), rojected_position, &intersection_point,
                                        &intersection_normal)) {
            auto distance_to_obstacle = intersection_point.distance_to(self->get_position());
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

} // namespace godot