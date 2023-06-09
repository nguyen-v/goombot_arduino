#include "MotorController.h"

MotorController::MotorController(
                    uint16_t reduction_ratio_, 
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_,
                    uint8_t output_pin_,
                    uint8_t analog_pin_, 
                    uint8_t commutation_pin_,
                    uint16_t rpm_min_,
                    uint16_t rpm_max_,
                    uint8_t pwm_min_,
                    uint8_t pwm_max_,
                    float analog_output_min_,
                    float analog_output_max_) :
                    reduction_ratio(reduction_ratio_),
                    enable_pin(enable_pin_),
                    direction_pin(direction_pin_),
                    output_pin(output_pin_),
                    analog_pin(analog_pin_),
                    commutation_pin(commutation_pin_),
                    rpm_min(rpm_min_),
                    rpm_max(rpm_max_),
                    pwm_min(pwm_min_),
                    pwm_max(pwm_max_),
                    analog_output_min(analog_output_min_),
                    analog_output_max(analog_output_max_)
//                    rpm_offset(0)
//                    tick(0) 
  {
    pinMode(enable_pin_, OUTPUT);
    pinMode(direction_pin_, OUTPUT);
    pinMode(output_pin_, OUTPUT);
    pinMode(analog_pin_, INPUT);
    pinMode(commutation_pin_, INPUT_PULLUP);
    set_rpm(0);
    set_direction(CW);
//    calibrate_offset();
  }

MotorController::MotorController(
                    uint16_t reduction_ratio_, 
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_,
                    uint8_t analog_pin_,
                    uint8_t commutation_pin_) :
                    MotorController(
                      reduction_ratio_,
                      enable_pin_,
                      direction_pin_,
                      output_pin_,
                      analog_pin_,
                      commutation_pin_,
                      RPM_MIN,
                      RPM_MAX,
                      PWM_MIN,
                      PWM_MAX,
                      ANALOG_OUTPUT_MIN,
                      ANALOG_OUTPUT_MAX) {}


MotorController::MotorController(
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_,
                    uint8_t analog_pin_,
                    uint8_t commutation_pin_) :
                    MotorController(
                      DEF_RED_RATIO,
                      enable_pin_,
                      direction_pin_,
                      output_pin_,
                      analog_pin_,
                      commutation_pin_) {}

MotorController::~MotorController() {
  set_rpm(0);
  disable_controller();
}

void MotorController::enable_controller() {
  digitalWrite(enable_pin, HIGH);
}

void MotorController::disable_controller() {
  digitalWrite(enable_pin, LOW);
}

void MotorController::reset_controller() {
  disable_controller();
  delay(100);
  enable_controller();
}

void MotorController::set_direction(bool direction_) {
  direction = direction_;
  if (direction_ == CW)
    digitalWrite(direction_pin, HIGH); 
  else
    digitalWrite(direction_pin, LOW);
}



void MotorController::set_rpm(uint16_t rpm, bool direction_) {
    enable_controller();
    set_direction(direction_);
    
    if (rpm > rpm_max) {
      analogWrite(output_pin, 255*(pwm_max/100.));
    } else if (rpm <= rpm_min) {
      disable_controller();
      analogWrite(output_pin, 255*(pwm_min/100.));
    } else {
      float duty_cycle = (rpm-rpm_min)*((float)(pwm_max-pwm_min)/(rpm_max-rpm_min)) + pwm_min;
      analogWrite(output_pin, 255*duty_cycle/100.);
    }
}

//void MotorController::calibrate_offset() {
//  set_rpm(0);
////  reset_controller();
////  delay(1000);
////  float offset = 0;
////  for (int i = 0; i < 100; ++i) {
////    offset += get_rpm();
////  }
////  rpm_offset = (int)(offset/100.);
////  rpm_offset 
//}

int MotorController::get_rpm() {
  float analog_output = analogRead(analog_pin)/ADC_FULL_SCALE*MCU_VOLTAGE;
  return (analog_output - analog_output_min)*((float)(2*rpm_max))/(analog_output_max-analog_output_min) - (int)rpm_max;
}

//int MotorController::get_rpm_calibrated() {
//  return get_rpm() - rpm_offset;
//}

void MotorController::set_rpm_shaft(float rpm) {
  set_rpm(abs(rpm)*reduction_ratio, (rpm >= 0) ? CW : CCW);
}

float MotorController::get_rpm_shaft() {
  return (float)get_rpm()/reduction_ratio;
}
//
//int MotorController::get_tick() {
//  return tick;
//}

//void MotorController::increment_tick() {
//  bool direction = get_rpm_shaft() > 0 ? CW : CCW;
//  if (direction == CW) {
//    tick++;
//  }
//}
