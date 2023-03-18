#ifndef WHEEL_H
#define WHEEL_H

#include "Arduino.h"
#include "MotorController.h"

// Wheel default parameters
#define DEF_WHEEL_DIAMETER 120 // mm

class Wheel {
  public:

    Wheel(MotorController controller_) :
      diameter(DEF_WHEEL_DIAMETER), controller(controller_) {}
    
    Wheel(float diameter_, MotorController controller_) : 
      diameter(diameter_), controller(controller_) {}

    ~Wheel();
    
    void set_speed(float speed); // m/s
    
  private:
    float diameter;
    MotorController controller;
    
};

#endif // WHEEL_H
