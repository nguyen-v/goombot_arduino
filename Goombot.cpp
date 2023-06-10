#include "Goombot.h"

Goombot::Goombot(Wheel& wheel_left_, Wheel& wheel_right_, float wheel_spacing_) : 
  wheel_left(wheel_left_), 
  wheel_right(wheel_right_), 
  wheel_spacing(wheel_spacing_)
//  ADS(0x48)
{
  wheel_left.set_speed(0);
  wheel_right.set_speed(0);
//  ADS.setMode(1);
//  ADS.requestADC(1);
//  ADS.setGain(1);
}

Goombot::Goombot(Wheel& wheel_left_, Wheel& wheel_right_) : Goombot(wheel_left_, wheel_right_, WHEEL_SPACING)
{
}

Goombot::~Goombot() {
  wheel_left.set_speed(0);
  wheel_right.set_speed(0);

}

void Goombot::set_speed(float speed_left, float speed_right) {
  wheel_left.set_speed(speed_left);
  wheel_right.set_speed(speed_right);
}

void Goombot::set_speed(float speed) {
  set_speed(speed, speed);
}

void Goombot::rotate(float angular_speed) {
  float rotation_speed = angular_speed * (wheel_spacing/2);
  set_speed(-rotation_speed, rotation_speed);
}

void Goombot::set_speed_lin_ang(float lin_speed, float ang_speed) {
  float rotation_speed = ang_speed * (wheel_spacing/2);
  set_speed(lin_speed - rotation_speed, lin_speed + rotation_speed);
}

//float Goombot::get_speed_left() {
//  if (ADS.isReady())
//  {
//    float value = ADS.toVoltage();
//    ADS.requestADC(1);
//    return value;
//  }
//}
