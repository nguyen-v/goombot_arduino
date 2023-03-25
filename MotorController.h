#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Arduino.h"

// Motor default parameters
#define DEF_RED_RATIO 60  // Motor reduction ratio
#define PWM_MAX 90        // Maximum PWM percentage (%)
#define PWM_MIN 10        // Minimum PWM percentage (%)

#define RPM_MIN 0         // Min speed before reduction (RPM)
#define RPM_MAX 6000      // Max speed before reduction (RPM)

#define CW true           // Clockwise direction
#define CCW false         // Counter-clockwise direction

#define ANALOG_OUTPUT_MIN 0 // V
#define ANALOG_OUTPUT_MAX 4.f // V
#define RPM_OFFSET 95;

// Arduino specific configuration
#define MCU_VOLTAGE 5 // V (arduino uses 5V)
#define ADC_FULL_SCALE 1023.0f // (arduino has 10 bits ADCs)


class MotorController {
  
  public:

    MotorController(uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_,
                    uint8_t analog_pin_);

      
    MotorController(uint16_t reduction_ratio_, 
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_,
                    uint8_t analog_pin_);

    MotorController(uint16_t reduction_ratio_, 
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_,
                    uint8_t output_pin_,
                    uint8_t analog_pin_, 
                    uint16_t rpm_min_,
                    uint16_t rpm_max_,
                    uint8_t pwm_min_,
                    uint8_t pwm_max_,
                    float analog_output_min_,
                    float analog_output_max_);
      
    ~MotorController();
    
    void set_rpm_shaft(float rpm);
    float get_rpm_shaft();
    void reset_controller();
    
  private:
    uint16_t reduction_ratio;
    uint8_t enable_pin;
    uint8_t direction_pin;
    uint8_t output_pin;
    uint8_t analog_pin;

    uint16_t rpm_min;
    uint16_t rpm_max;

    uint8_t pwm_min;
    uint8_t pwm_max;

    float analog_output_min;
    float analog_output_max;
    
    bool direction;

    void enable_controller();
    void disable_controller();
    void set_direction(bool direction_);
    void set_rpm(uint16_t rpm, bool direction_=CW);
    int get_rpm();
};


#endif // MOTOR_CONTROLLER_H
