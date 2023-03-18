
// ROS includes
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Custom headers
#include "MotorController.h"


// Create ROS node handle
ros::NodeHandle nh;

// Create a motor controller instance
MotorController left_motor(31, 33, 2);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
//  for (int i=21; i<229; ++i) {
//    analogWrite(2, i);
//    delay(10);
//  }
//
//  for (int i=229; i>21; --i) {
//    analogWrite(2, i);
//    delay(10);
//  }
//
//  for (int i = 0; i < 100; ++i) {
//    left_motor.set_rpm_shaft(i/200.);
//  delay(10);
//  }
//  for (int i = 100; i > 0; --i) {
//    left_motor.set_rpm_shaft(i/200.);
//  delay(10);
//  }
//left_motor.set_rpm_shaft(100);
}
