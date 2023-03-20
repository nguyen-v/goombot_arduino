#ifndef GOOMBOT_H
#define GOOMBOT_H

#include "Wheel.h"

class Goombot {
  public:
    Goombot();
    void set_speed(float speed_left, float speed_right);
    void rotate(float speed);

  private:
    Wheel wheel_left;
    Wheel wheel_right;
};

#endif // GOOMBOT_H
