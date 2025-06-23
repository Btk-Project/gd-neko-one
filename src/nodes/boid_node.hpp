#pragma once

#include <godot_cpp/classes/sprite2d.hpp>

#include <functional>
#include <vector>

#include "../algorithm/linear_algebra.hpp"

namespace godot {
class BoidManagerNode;

class BoidSprite2D : public Sprite2D {
    // NOLINTBEGIN
    GDCLASS(BoidSprite2D, Sprite2D)
    // NOLINTEND
    friend class Node;

    enum BehaviorFlag : uint32_t {
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

public:
    BoidSprite2D();
    ~BoidSprite2D();
    auto mass() const -> float { return mMass; }
    void setMass(float mass) { mMass = mass; }
    void setVelocity(Vector2 velocity) { mVelocity = velocity; }
    auto velocity() const -> Vector2 { return mVelocity; }
    void setAcceleration(Vector2 acceleration) { mAcceleration = acceleration; }
    auto acceleration() const -> Vector2 { return mAcceleration; }
    void setMaxSpeed(float speed) { mMaxSpeed = speed; }
    auto maxSpeed() const -> float { return mMaxSpeed; }
    void setMaxForce(float force) { mMaxForce = force; }
    auto maxForce() const -> float { return mMaxForce; }

    auto flocking() const -> bool { return _check_behavior_flag(Flocking); }
    void setFlocking(bool flocking) { _set_behavior_flag(Flocking, flocking); }
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
    auto seekTarget() const -> bool { return _check_behavior_flag(Seek); }
    void setSeekTarget(bool seek) { _set_behavior_flag(Seek, seek); }
    auto seekTargetWeight() const -> float { return mSeekTargetWeight; }
    void setSeekTargetWeight(float weight) { mSeekTargetWeight = weight; }

    auto arriveTarget() const -> bool { return _check_behavior_flag(Arrive); }
    void setArriveTarget(bool arrive) { _set_behavior_flag(Arrive, arrive); }
    auto arriveWeight() const -> float { return mArriveWeight; }
    void setArriveWeight(float weight) { mArriveWeight = weight; }
    auto arriveSlowingRadius() const -> float { return mArriveSlowingRadius; }
    void setArriveSlowingRadius(float radius) { mArriveSlowingRadius = radius; }

    auto pursuitTarget() const -> bool { return _check_behavior_flag(Pursuit); }
    void setPursuitTarget(bool pursuit) { _set_behavior_flag(Pursuit, pursuit); }
    auto pursuitWeight() const -> float { return mPursuitWeight; }
    void setPursuitWeight(float weight) { mPursuitWeight = weight; }

    auto evadeTarget() const -> bool { return _check_behavior_flag(Evade); }
    void setEvadeTarget(bool evade) { _set_behavior_flag(Evade, evade); }
    auto evadeWeight() const -> float { return mEvadeWeight; }
    void setEvadeWeight(float weight) { mEvadeWeight = weight; }
    auto evadeRadius() const -> float { return mEvadeRadius; }
    void setEvadeRadius(float radius) { mEvadeRadius = radius; }

    auto wander() const -> bool { return _check_behavior_flag(Wander); }
    void setWander(bool wander) { _set_behavior_flag(Wander, wander); }
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

    auto avoidObstacles() const -> bool { return _check_behavior_flag(Obstacle); }
    void setAvoidObstacles(bool avoid) { _set_behavior_flag(Obstacle, avoid); }
    auto avoidObstaclesWeight() const -> float { return mAvoidObstaclesWeight; }
    void setAvoidObstaclesWeight(float weight) { mAvoidObstaclesWeight = weight; }
    auto avoidObstaclesDistance() const -> float { return mAvoidObstaclesDistance; }
    void setAvoidObstaclesDistance(float distance) { mAvoidObstaclesDistance = distance; }

    auto fleeFromPredator() const -> bool { return _check_behavior_flag(Predator); }
    void setFleeFromPredator(bool flee) { _set_behavior_flag(Predator, flee); }
    auto fleeFromPredatorWeight() const -> float { return mFleeFromPredatorWeight; }
    void setFleeFromPredatorWeight(float weight) { mFleeFromPredatorWeight = weight; }
    auto fleeFromPredatorDistance() const -> float { return mFleeFromPredatorDistance; }
    void setFleeFromPredatorDistance(float distance) { mFleeFromPredatorDistance = distance; }

    auto followingPath() const -> bool { return _check_behavior_flag(Following); }
    void setFollowingPath(bool follow) { _set_behavior_flag(Following, follow); }
    auto followingPathWeight() const -> float { return mFollowingPathWeight; }
    void setFollowingPathWeight(float weight) { mFollowingPathWeight = weight; }

    // system
    auto gameTimeFrameRate() const -> float { return mGameTimeFrameRate; }
    void setGameTimeFrameRate(float frameRate) { mGameTimeFrameRate = frameRate; }
    auto updateManager(BoidManagerNode* manager) -> void { mBoidManager = manager; }
    void _draw() override;

protected:
    void _process(double delta) override;
    void _physics_process(double delta) override;
    static void _bind_methods();

    void _apply_rules();

    /**
     * @brief 角色直线地向一个目标点移动。
     * 计算从角色位置指向目标位置的向量，将其标准化并乘以最大速度得到“期望速度”，然后计算达到这个期望速度所需的力。
     * @param target
     * @return Vector2
     */
    auto _seek_target(const Vector2& target) -> Vector2;

    /**
     * @brief 角色直线地远离一个目标点移动。
     * 与寻路相反，计算从目标位置指向角色位置的向量，标准化并乘以最大速度，然后计算达到这个期望速度所需的力。
     * @param target
     * @return Vector2
     */
    auto _leave_target(const Vector2& target) -> Vector2;

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
    template <typename U>
    auto _flocking(const std::vector<U*>& neighbors) -> Vector2;

    /**
     * @brief 逃离行为
     * 遍历所有附近的捕食者，对于每个捕食者，计算一个远离它的力，该力的大小与距离成反比。将所有这些力叠加。
     * @tparam U
     * @param predators
     * @return Vector2
     */
    template <typename U>
    auto _leave_from_predator(const std::vector<U*>& predators) -> Vector2;

    /**
     * @brief 角色向目标点移动，但在接近目标时会减速，最终平滑地停在目标点。
     * 结合了寻路，但根据与目标的距离来调整期望速度的幅度。距离越近，期望速度越小。
     * @param target
     * @return Vector2
     */
    auto _arrive(const Vector2& target) -> Vector2;

    /**
     * @brief 角色追逐一个移动的目标，通过预测目标的未来位置来更有效地拦截。
     * 根据目标的速度和当前距离，预测目标在未来某个时间点的可能位置，然后对那个预测位置执行“寻路”或“到达”行为。
     * @param target
     * @return Vector2
     */
    template <typename U>
    auto _pursuit(const U& target) -> Vector2;

    /**
     * @brief 角色躲避一个移动的追逐者，通过预测追逐者的未来位置来提前逃离。
     * 与追逐相反，预测追逐者未来可能到达的位置，然后从那个预测位置执行“逃离”行为。
     * @param target
     * @return Vector2
     */
    template <typename U>
    auto _evade(const U& target) -> Vector2;

    /**
     * @brief 角色进行一种不设特定目标的随机、持续的运动，看起来像是在四处游荡。
     * 通常在一个角色前方的一个假想圆上选择一个随机点作为目标，然后寻路到那个点。随着时间推移，随机点在圆上小幅移动，造成连续的漫游效果。
     * @param target
     * @return Vector2
     */
    auto _wander() -> Vector2;

    /**
     * @brief 角色在移动时检测并避开前方的障碍物，防止碰撞。
     * 通常通过在角色前方发射“探测线”（或使用包围体投影）来检测潜在碰撞。如果检测到碰撞，则计算一个推离障碍物的力，力的大小与碰撞的紧迫程度相关。
     * @param obstacles 圆形障碍物
     * @return Vector2
     */
    auto _avoid_obstacles(const std::vector<math::Circle>& obstacles) -> Vector2;
    auto _avoid_obstacles(const std::vector<Rect2>& obstacles) -> Vector2; // 矩形障碍物

    /**
     * @brief 角色沿着预定义的路径（通常是waypoint序列）移动。
     * 找到角色在路径上的当前点，然后寻找路径上该点前方的一个目标点（下一个waypoint或投影点），并对该点执行“寻路”或“到达”行为。
     * @param path 一个函数，返回给定位置时路径上的下一个目标点
     * @return Vector2
     */
    auto _path_following(const Vector2& next_waypoint) -> Vector2;

    auto _check_behavior_flag(BehaviorFlag flag) const -> bool { return mEnableFlags & (uint32_t)flag; }
    auto _set_behavior_flag(BehaviorFlag flag, bool value = true) -> void {
        if (value) {
            mEnableFlags |= (uint32_t)flag;
        } else {
            mEnableFlags &= ~(uint32_t)flag;
        }
    }

private:
    double mGameTimeFrameRate = 30.0;     // 游戏速度, 1s 等价的游戏时间
    Vector2 mForward          = {1, 0};   // boid 的方向
    Vector2 mVelocity         = {0, 0};   // boid 的速度
    Vector2 mAcceleration     = {0, 0};   // boid 的加速度
    float mMaxSpeed           = 5.0;      // boid 的最大速度
    float mMaxForce           = 0.1;      // boid 的最大加速度
    float mMass               = 1.0;      // boid 的质量
    uint32_t mEnableFlags     = Flocking; // 行为使能标志

    // Flocking : 群体行为
    float mSeparationWeight = 1.5;   // 分离规则权重
    float mSeparationRadius = 40.0;  // 感知半径
    float mAlignmentWeight  = 1.0;   // 对齐规则权重
    float mAlignmentRadius  = 50.0;  // 感知半径
    float mCohesionWeight   = 1.0;   // 聚集规则权重
    float mCohesionRadius   = 100.0; // 感知半径

    std::vector<BoidSprite2D*> mNeighborBoids;
    uint64_t mClusterHash = 0;

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
    float mPursuitWeight          = 1.0; // 追逐规则权重
    BoidSprite2D* mPursuitTargetV = nullptr;
    uint64_t mPursuitTargetHash   = 0;

    // evade: 躲避
    float mEvadeWeight          = 1.0;  // 躲避规则权重
    float mEvadeRadius          = 50.0; // 躲避距离
    BoidSprite2D* mEvadeTargetV = nullptr;
    uint64_t mEvadeTargetHash   = 0;

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
    std::vector<Node2D*> mPredatorV;
    uint64_t mPredatorHash = 0;

    // following path : 跟随路径
    float mFollowingPathWeight = 1.0; // 跟随路径规则权重
    std::vector<Vector2> mPathPoints;
    uint64_t mFollowingHash = 0;
    uint64_t mPathIndex     = 0;

    BoidManagerNode* mBoidManager = nullptr; // 引用BoidManagerNode
    float mTimeEmit               = 0.0;
};

template <typename U>
auto BoidSprite2D::_flocking(const std::vector<U*>& neighbors) -> Vector2 {
    auto separation    = Vector2(0, 0);
    auto alignment     = Vector2(0, 0);
    auto cohesion      = Vector2(0, 0);
    auto sepTotal      = 0;
    auto alignTotal    = 0;
    auto cohesionTotal = 0;

    for (auto* other : neighbors) {
        auto distance = std::fabs(this->get_position().distance_to(other->get_position()));
        if (distance > 0) {
            if (distance < mSeparationRadius) {
                Vector2 dic = (this->get_position() - other->get_position());
                if (dic.is_zero_approx()) {
                    dic = Vector2(1, 0); // 避免除以零
                }
                separation += dic.normalized() * (mMaxSpeed * mSeparationRadius / (mSeparationRadius - distance));
                sepTotal++;
            }
            if (distance < mAlignmentRadius) {
                alignment += other->velocity();
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
        alignment = (alignment - this->velocity());
    }
    if (cohesionTotal > 0) {
        cohesion /= static_cast<float>(cohesionTotal);
        cohesion = _seek_target(cohesion);
    }

    return (separation * mSeparationWeight) + (alignment - velocity()) * mAlignmentWeight + cohesion * mCohesionWeight;
}

template <typename U>
auto BoidSprite2D::_leave_from_predator(const std::vector<U*>& predators) -> Vector2 {
    auto steering_predator = Vector2(0, 0);
    for (const auto& predator : predators) {
        auto distance = predator->get_position().distance_to(this->get_position());
        if (distance < mFleeFromPredatorDistance) {
            auto force = _leave_target(predator->get_position());
            steering_predator += force;
        }
    }
    return steering_predator;
}

template <typename U>
auto BoidSprite2D::_pursuit(const U& target) -> Vector2 {
    auto to_target         = target.get_position() - this->get_position();
    auto relative_heading  = velocity().dot(target.velocity());
    double prediction_time = 0;
    if (relative_heading < -0.95f && to_target.length() > 0) {
        prediction_time = to_target.length() / mMaxSpeed;
    } else {
        prediction_time = to_target.length() / (target.velocity().length() + 0.001); // 避免除以零
    }

    auto predicted_target_position = target.get_position() + target.velocity() * prediction_time;
    return _seek_target(predicted_target_position);
}

template <typename U>
auto BoidSprite2D::_evade(const U& target) -> Vector2 {
    auto to_target = target.get_position() - this->get_position();
    auto distance  = to_target.length();
    if (distance > mEvadeRadius) {
        return {0, 0};
    }
    auto prediction_time            = distance / (target.velocity().length() + 0.001); // 避免除以零
    auto predicted_pursuer_position = target.get_position() + target.velocity() * prediction_time;
    return _leave_target(predicted_pursuer_position);
}

} // namespace godot