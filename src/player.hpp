#pragma once

#include <godot_cpp/classes/sprite2d.hpp>

namespace godot {
class Player : public Sprite2D {
    // NOLINTBEGIN
    GDCLASS(Player, Sprite2D)
    // NOLINTEND
    friend class Node;

public:
    Player();
    ~Player();
    void setAmplitude(double amplitude);
    double amplitude() const;
    void setTimePassed(double timePassed);
    double timePassed() const;
    void setSpeed(double pSpeed);
    double speed() const;

protected:
    void _process(double delta) override;
    static void _bind_methods();

private:
    double mTimePassed;
    double mAmplitude;
    double mSpeed;
    double mTimeEmit;
};
} // namespace godot