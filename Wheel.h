#ifndef WHEEL_H
#define WHEEL_H

#include "Arduino.h"
#include "MotorController.h"

// Wheel default parameters
#define DEF_WHEEL_DIAMETER 0.1185f // m

#define SPEED_LOW_THRESHOLD 0.005f //m/s

#define SPEED_CORRECTION_FACTOR 1.0f

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
    float get_speed_avg_calibrated(uint8_t num_samples);
    float speed_offset;

    void calibrate_speed();
//    int get_ticks();
    
  private:
    float diameter;
    bool reversed;
    
};

#endif // WHEEL_H
