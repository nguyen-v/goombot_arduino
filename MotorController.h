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


class MotorController {
  public:

    MotorController(uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_) : 
      reduction_ratio(DEF_RED_RATIO), 
      enable_pin(enable_pin_), 
      direction_pin(direction_pin_), 
      output_pin(output_pin_),
      rpm_min(RPM_MIN),
      rpm_max(RPM_MAX),
      pwm_min(PWM_MIN),
      pwm_max(PWM_MAX)
      {
        set_rpm(0);
        enable_controller();
      }

      
    MotorController(uint16_t reduction_ratio_, 
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_) :
      reduction_ratio(reduction_ratio_), 
      enable_pin(enable_pin_), 
      direction_pin(direction_pin_), 
      output_pin(output_pin_),
      rpm_min(RPM_MIN),
      rpm_max(RPM_MAX),
      pwm_min(PWM_MIN),
      pwm_max(PWM_MAX)
      {
        set_rpm(0);
        enable_controller();
      }


    MotorController(uint16_t reduction_ratio_, 
                    uint8_t enable_pin_, 
                    uint8_t direction_pin_, 
                    uint8_t output_pin_,
                    uint16_t rpm_min_,
                    uint16_t rpm_max_,
                    uint8_t pwm_min_,
                    uint8_t pwm_max_) :
      reduction_ratio(reduction_ratio_), 
      enable_pin(enable_pin_), 
      direction_pin(direction_pin_), 
      output_pin(output_pin_),
      rpm_min(rpm_min_),
      rpm_max(rpm_max_),
      pwm_min(pwm_min_),
      pwm_max(pwm_max_)
      {
        set_rpm(0);
        enable_controller();
      }
      
    ~MotorController();
    
    void set_rpm_shaft(float rpm);
    
  private:
    uint16_t reduction_ratio;
    uint8_t enable_pin;
    uint8_t direction_pin;
    uint8_t output_pin;

    uint16_t rpm_min;
    uint16_t rpm_max;

    uint8_t pwm_min;
    uint8_t pwm_max;

    bool enabled;

    void enable_controller();
    void disable_controller();
    void set_direction(bool direction);
    void set_rpm(uint16_t rpm, bool direction=CW);
};


#endif // MOTOR_CONTROLLER_H
