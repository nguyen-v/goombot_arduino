#ifndef GOOMBOT_H
#define GOOMBOT_H

#include "Arduino.h"
#include "Wheel.h"
#include "ADS1X15.h"
#include <ServoEasing.h>


#define RIGHTSERVO_PIN 8
#define LEFTSERVO_PIN 9
#define SERVO_SPEED 50
#define LEFT_UP 0
#define LEFT_DOWN 120
#define RIGHT_UP 120
*define LEFT_DOWN 0

//#define WHEEL_SPACING 0.324f // 
#define WHEEL_SPACING 0.34f
class Goombot {
  public:
    Goombot(Wheel& wheel_left_, Wheel& wheel_right_, float wheel_spacing_);
    Goombot(Wheel& wheel_left_, Wheel& wheel_right_);
    ~Goombot();
    void set_speed(float speed_left, float speed_right);
    void set_speed(float speed);
    void rotate(float angular_speed);
    void set_speed_lin_ang(float lin_speed, float ang_speed);
    void Goombot::lift_up();
    void Goombot::liftDown();
    Wheel& wheel_left;
    Wheel& wheel_right;
//    float get_speed_left(); // uses the ADS1113

  private:
    float wheel_spacing;
    ServoEasing liftingServo_right;
    ServoEasing liftingServo_left;

//    ADS1113 ADS;
};

#endif // GOOMBOT_H
