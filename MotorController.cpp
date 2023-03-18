#include "MotorController.h"

MotorController::~MotorController() {
  set_rpm(0);
  disable_controller();
}

void MotorController::enable_controller() {
  digitalWrite(enable_pin, HIGH);
  enabled = true;
}

void MotorController::disable_controller() {
  digitalWrite(enable_pin, LOW);
  enabled = false;
}

void MotorController::set_direction(bool direction) {
  if (direction == CW)
    digitalWrite(direction_pin, HIGH); 
  else
    digitalWrite(direction_pin, LOW);
}

void MotorController::set_rpm(uint16_t rpm, bool direction) {
  if (enabled) {
    set_direction(direction);
    if (rpm >= rpm_max)
      analogWrite(output_pin, 255*(pwm_max/100.));
    else if (rpm <= rpm_min)
      analogWrite(output_pin, 255*(pwm_min/100.));
    else {
      float duty_cycle = rpm*((float)(pwm_max-pwm_min)/(rpm_max-rpm_min)) + pwm_min/100.;
      analogWrite(output_pin, 255*duty_cycle);
    }
  }
}

void MotorController::set_rpm_shaft(float rpm) {
  set_rpm(rpm*reduction_ratio, (rpm >= 0) ? CW : CCW);
}
