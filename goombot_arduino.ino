
 
// ROS includes
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

// Custom headers
#include "MotorController.h"
#include "Wheel.h"
#include "Goombot.h"


// Create ROS node handle
ros::NodeHandle nh;


// Create a motor controller instance
MotorController left_motor(40, 42, 3, A0);
MotorController right_motor(35, 33, 2, A15);

// Create wheel instances
Wheel left_wheel(left_motor);
Wheel right_wheel(right_motor, true);

// Create a goombot instance
Goombot goombot(left_wheel, right_wheel);

// ROS callbacks
void set_velocity_callback(const geometry_msgs::Twist& cmd_vel) {

  // Check if the robot has to rotate first
  if (cmd_vel.angular.z != 0) {
    goombot.rotate(cmd_vel.angular.z);

  // Go straight
  } else {
    goombot.set_speed(cmd_vel.linear.x);
  }
}

void reset_controller_callback(const std_msgs::String& reset_controller){
  const char *controller_name = reset_controller.data;
  if(strcmp(controller_name, "left") == 0) {
    goombot.wheel_left.controller.reset_controller();
  }
  else if (strcmp(controller_name, "right") == 0) {
    goombot.wheel_right.controller.reset_controller();
  }
}

// ROS subscribers

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &set_velocity_callback);
ros::Subscriber<std_msgs::String> reset_controller_sub("reset_controller", &reset_controller_callback);

// ROS publishers

std_msgs::Float32 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("left_wheel_vel", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("right_wheel_vel", &right_wheel_vel);


void setup() {
  
  // reset controllers
  goombot.wheel_left.controller.reset_controller();
  goombot.wheel_right.controller.reset_controller();
  nh.getHardware()->setBaud(921600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(reset_controller_sub);

  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);

}

void loop() {
  // Read and publish wheel speed
  left_wheel_vel.data = goombot.wheel_left.get_speed();
  right_wheel_vel.data = goombot.wheel_right.get_speed();
  left_wheel_vel_pub.publish(&left_wheel_vel);
  right_wheel_vel_pub.publish(&right_wheel_vel);
  delay(5);
  nh.spinOnce();
}
