#ifndef WHEEL_H
#define WHEEL_H

#include "Arduino.h"
#include "MotorController.h"
#include <math.h>

// Wheel default parameters
#define DEF_WHEEL_DIAMETER 0.12f // m

class Wheel {
  public:

    Wheel(MotorController controller_) :
      diameter(DEF_WHEEL_DIAMETER), controller(controller_) {}
    
    Wheel(float diameter_, MotorController controller_) : 
      diameter(diameter_), controller(controller_) {}

    ~Wheel();
    
    void set_speed(float speed); // m/s

    float get_diameter() const;
    
  private:
    float diameter;
    MotorController controller;
    
};

#endif // WHEEL_H
