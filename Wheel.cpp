#include "Wheel.h"

Wheel::Wheel(float diameter_, MotorController& controller_, bool reversed_) :
  diameter(diameter_),
  controller(controller_),
  reversed(reversed_) {}
  

Wheel::Wheel(MotorController& controller_, bool reversed): Wheel(DEF_WHEEL_DIAMETER, controller_, reversed) {}

Wheel::Wheel(MotorController& controller_): Wheel(DEF_WHEEL_DIAMETER, controller_, false) {}

Wheel::~Wheel() {
  set_speed(0);  
}

void Wheel::set_speed(float speed) {
  float speed_ = reversed ? -speed : speed;
  controller.set_rpm_shaft(60.*speed_ / (PI*diameter));
}

float Wheel::get_speed() {
  float speed = controller.get_rpm_shaft()*(PI*diameter)/60.;
  float speed_ = reversed ? -speed : speed; 
  return (abs(speed_) < SPEED_LOW_THRESHOLD ? 0. : speed_);
}
