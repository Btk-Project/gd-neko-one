#include "player.hpp"

namespace godot {
void Player::_bind_methods() {
    // _time_passed
    ClassDB::bind_method(D_METHOD("timePassed"), &Player::timePassed);
    ClassDB::bind_method(D_METHOD("setTimePassed", "timePassed"), &Player::setTimePassed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "timePassed"), "setTimePassed", "timePassed");

    // _amplitude
    ClassDB::bind_method(D_METHOD("amplitude"), &Player::amplitude);
    ClassDB::bind_method(D_METHOD("setAmplitude", "amplitude"), &Player::setAmplitude);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "amplitude"), "setAmplitude", "amplitude");

    // _speed
    ClassDB::bind_method(D_METHOD("speed"), &Player::speed);
    ClassDB::bind_method(D_METHOD("setSpeed", "speed"), &Player::setSpeed);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed"), "setSpeed", "speed");

    ADD_SIGNAL(MethodInfo("position_changed", PropertyInfo(Variant::OBJECT, "node"),
                          PropertyInfo(Variant::VECTOR2, "new_pos")));
}

Player::Player() {
    mTimePassed = 0.0;
    mAmplitude  = 10.0;
    mSpeed      = 1;
}

Player::~Player() {}

void Player::setAmplitude(double amplitude) { mAmplitude = amplitude; }

double Player::amplitude() const { return mAmplitude; }

void Player::setTimePassed(double timePassed) { mTimePassed = timePassed; }

double Player::timePassed() const { return mTimePassed; }

void Player::setSpeed(double speed) { mSpeed = speed; }

double Player::speed() const { return mSpeed; }

void Player::_process(double delta) {
    mTimePassed += mSpeed * delta;

    Vector2 newPosition =
        Vector2(mAmplitude + (mAmplitude * sin(mTimePassed * 2.0)), mAmplitude + (mAmplitude * cos(mTimePassed * 1.5)));

    set_position(newPosition);

    mTimeEmit += delta;
    if (mTimeEmit > 1.0) {
        emit_signal("position_changed", this, newPosition);

        mTimeEmit = 0.0;
    }
}

} // namespace godot