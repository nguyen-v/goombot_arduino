#ifndef GOOMBOT_H
#define GOOMBOT_H

#include "Arduino.h"
#include "Wheel.h"
#include "ADS1X15.h"
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
    Wheel& wheel_left;
    Wheel& wheel_right;
//    float get_speed_left(); // uses the ADS1113

  private:
    float wheel_spacing;

//    ADS1113 ADS;
};

#endif // GOOMBOT_H
