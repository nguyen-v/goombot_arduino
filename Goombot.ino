#include "Wheel.h"

class Goombot {
  public:
    Goombot() : wheel_left(MotorController()), wheel_right(MotorController()) {}

    void set_speed(float speed) {
      wheel_left.set_speed(speed);
      wheel_right.set_speed(speed);
    }

    void rotate(float angular_velocity) {
      float r = wheel_left.get_diameter() / 2.0 ;
      float speed = angular_velocity * r ;
      wheel_left.set_speed(-speed);
      wheel_right.set_speed(speed);
    }

  private:
    Wheel wheel_left;
    Wheel wheel_right;
};
