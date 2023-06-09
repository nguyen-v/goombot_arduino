#include "Goombot.h"
#include <ServoEasing.h>


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
  
  liftingServo_right.attach(RIGHTSERVO_PIN);  
  liftingServo_left.attach(LEFTSERVO_PIN);  

  liftingServo_right.setSpeed(SERVO_SPEED);  
  liftingServo_left.setSpeed(SERVO_SPEED);  
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

void Goombot::lift_up(){
  liftingServo_right.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_right.startEaseTo(RIGHT_UP);
  liftingServo_left.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_left.startEaseTo(LEFT_UP);

  while (liftingServo_right.isMoving() || liftingServo_left.isMoving()) {
    liftingServo_right.update();
    liftingServo_left.update();
  }
}
void Goombot::liftDown() {
  liftingServo_right.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_right.startEaseTo(RIGHT_DOWN);
  liftingServo_left.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_left.startEaseTo(LEFT_DOWN);

  while (liftingServo_right.isMoving() || liftingServo_left.isMoving()) {
    liftingServo_right.update();
    liftingServo_left.update();
  }
}
