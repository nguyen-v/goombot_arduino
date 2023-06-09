#include "Wheel.h"

Wheel::Wheel(float diameter_, MotorController& controller_, bool reversed_) :
  diameter(diameter_),
  controller(controller_),
  reversed(reversed_),
  speed_offset(0) {
  }
  

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
  return (abs(speed_) < SPEED_LOW_THRESHOLD ? 0. : speed_)*SPEED_CORRECTION_FACTOR;
}

float Wheel::get_speed_avg(uint8_t num_samples) {
  float speed_avg = 0;
  for (uint8_t i = 0; i < num_samples; ++i) {
    speed_avg += get_speed();
  }
  speed_avg /= num_samples;
  return speed_avg;
}

float Wheel::get_speed_avg_calibrated(uint8_t num_samples) {
  return get_speed_avg(num_samples) - speed_offset;
}

void Wheel::calibrate_speed() {
  set_speed(0);
  speed_offset = get_speed_avg(100);
}

//int Wheel::get_ticks() {
//  return reversed? -controller.get_ticks() : controller.get_ticks();
//}
