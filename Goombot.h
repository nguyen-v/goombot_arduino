#ifndef GOOMBOT_H
#define GOOMBOT_H

#include "Arduino.h"
#include "Wheel.h"

#define WHEEL_SPACING 0.342f // m

class Goombot {
  public:
    Goombot(Wheel& wheel_left_, Wheel& wheel_right_, float wheel_spacing_);
    Goombot(Wheel& wheel_left_, Wheel& wheel_right_);
    ~Goombot();
    void set_speed(float speed_left, float speed_right);
    void set_speed(float speed);
    void rotate(float angular_speed);
    Wheel& wheel_left;
    Wheel& wheel_right;

  private:
    float wheel_spacing;
};

#endif // GOOMBOT_H