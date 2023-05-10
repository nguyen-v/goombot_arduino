#ifndef WHEEL_H
#define WHEEL_H

#include "Arduino.h"
#include "MotorController.h"

// Wheel default parameters
#define DEF_WHEEL_DIAMETER 0.12f // m

#define SPEED_LOW_THRESHOLD 0.005f //m/s

class Wheel {
  public:

    Wheel(MotorController& controller_);

    Wheel(MotorController& controller_, bool reversed_);
    
    Wheel(float diameter_, MotorController& controller_, bool reversed_);

    ~Wheel();
    
    MotorController& controller;
    void set_speed(float speed); // m/s
    float get_speed(); // m/s
    float get_speed_avg(uint8_t num_samples);
//    int get_ticks();
    
  private:
    float diameter;
    bool reversed;
    
};

#endif // WHEEL_H
