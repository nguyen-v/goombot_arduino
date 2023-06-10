
 // ROS includes
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

// Custom headers
#include "MotorController.h"
#include "Wheel.h"
#include "Goombot.h"
#include <ServoEasing.hpp>


#define RIGHTSERVO_PIN 8
#define LEFTSERVO_PIN 9

#define SERVO_SPEED 100

#define LEFT_UP 0
#define LEFT_IDLE 115
#define LEFT_DOWN 120

#define RIGHT_UP 120
#define RIGHT_IDLE 5
#define RIGHT_DOWN 0





// Create ROS node handle
ros::NodeHandle nh;

// Create a motor controller instance
MotorController left_motor(40, 42, 3, A0, 18);
MotorController right_motor(35, 33, 2, A15, 19);

ServoEasing liftingServo_right;
ServoEasing liftingServo_left;

// Create wheel instances
Wheel left_wheel(left_motor);
Wheel right_wheel(right_motor, true);

// Create a goombot instance
Goombot goombot(left_wheel, right_wheel);

// ROS callbacks
void set_velocity_callback(const geometry_msgs::Twist& cmd_vel) {
//
//  // Check if the robot has to rotate first
//  if (abs(cmd_vel.angular.z) > 0.05) {
//    goombot.rotate(cmd_vel.angular.z);
//
//  // Go straight
//  } else {
//    goombot.set_speed(cmd_vel.linear.x);
//  }
  goombot.set_speed_lin_ang(cmd_vel.linear.x, cmd_vel.angular.z);
}

void lift_command_callback(const std_msgs::String& lift_command) {
  if (strcmp(lift_command.data, "LIFT_UP") == 0) {
    lift_up();
  }
  else if (strcmp(lift_command.data, "LIFT_DOWN") == 0) {
    lift_down();
  }
  else if (strcmp(lift_command.data, "LIFT_IDLE") == 0) {
    lift_idle();
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
  else if (strcmp(controller_name, "speed") == 0) {
    goombot.wheel_right.calibrate_speed();
    goombot.wheel_left.calibrate_speed();
  }
}


// ROS subscribers

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &set_velocity_callback);
ros::Subscriber<std_msgs::String> reset_controller_sub("reset_controller", &reset_controller_callback);
ros::Subscriber<std_msgs::String> lift_command_sub("lift_command", &lift_command_callback);
// ROS publishers

std_msgs::Float32 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("left_wheel_vel", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("right_wheel_vel", &right_wheel_vel);

std_msgs::Int16 left_ticks;
ros::Publisher left_wheel_tick_pub("left_ticks", &left_ticks);

std_msgs::Int16 right_ticks;
ros::Publisher right_wheel_tick_pub("right_ticks", &right_ticks);


const int intervalPub = 35;
const int intervalConv = 30;
long previousMillis = 0;
long currentMillis = 0;
long previousMillisConv = 0;
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

void setup() {

  attachInterrupt(digitalPinToInterrupt(18), increment_tick_left, RISING);
  attachInterrupt(digitalPinToInterrupt(19), increment_tick_right, RISING);
  liftingServo_right.attach(RIGHTSERVO_PIN);  
  liftingServo_left.attach(LEFTSERVO_PIN);  

  liftingServo_right.setSpeed(SERVO_SPEED);  
  liftingServo_left.setSpeed(SERVO_SPEED);  
  
  // reset controllers
  goombot.wheel_left.controller.reset_controller();
  goombot.wheel_right.controller.reset_controller();
  nh.getHardware()->setBaud(921600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(reset_controller_sub);

  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);

  nh.advertise(left_wheel_tick_pub);
  nh.advertise(right_wheel_tick_pub);

  nh.initNode();

}

void loop() {
//  // Read and publish wheel speed
//  left_wheel_vel.data = goombot.wheel_left.get_speed();
//  right_wheel_vel.data = goombot.wheel_right.get_speed();
//  left_wheel_vel_pub.publish(&left_wheel_vel);
//  right_wheel_vel_pub.publish(&right_wheel_vel);

//  // Read and publish tick
//  left_ticks.data = goombot.wheel_left.get_ticks();
//  right_ticks.data = goombot.wheel_right.get_ticks();


  currentMillis = millis();

  if (currentMillis - previousMillisConv > intervalConv) {
     
    previousMillisConv = currentMillis;
    left_wheel_vel.data = goombot.wheel_left.get_speed_avg_calibrated(15);
    nh.spinOnce();
//    left_wheel_vel.data = goombot.get_speed_left();
    right_wheel_vel.data = goombot.wheel_right.get_speed_avg_calibrated(15);
    nh.spinOnce();
  }
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > intervalPub) {
     
    previousMillis = currentMillis;
     
    left_wheel_tick_pub.publish(&left_ticks);
    right_wheel_tick_pub.publish(&right_ticks);
    left_wheel_vel_pub.publish(&left_wheel_vel);
    right_wheel_vel_pub.publish(&right_wheel_vel);
    nh.spinOnce();
  }
   if (liftingServo_right.isMoving() || liftingServo_left.isMoving()) {
    liftingServo_right.update();
    liftingServo_left.update();
  }
  
  delay(1);
  nh.spinOnce();

}

void increment_tick_left() {
   
  // Read the value for the encoder for the left wheel
  bool direction_left = goombot.wheel_left.get_speed_avg_calibrated(1) > 0 ? true : false;

   
  if (direction_left) {
     
    if (left_ticks.data == encoder_maximum) {
      left_ticks.data = encoder_minimum;
    }
    else {
      left_ticks.data++;  
    }    
  }
  else {
    if (left_ticks.data == encoder_minimum) {
      left_ticks.data = encoder_maximum;
    }
    else {
      left_ticks.data--;  
    }   
  }
}

void increment_tick_right() {
   
  // Read the value for the encoder for the right wheel
  bool direction_right = goombot.wheel_right.get_speed_avg_calibrated(1) > 0 ? true : false;
   
  if (direction_right) {
     
    if (right_ticks.data == encoder_maximum) {
      right_ticks.data = encoder_minimum;
    }
    else {
      right_ticks.data++; 
    }
  }
  else {
    if (right_ticks.data == encoder_minimum) {
      right_ticks.data = encoder_maximum;
    }
    else {
      right_ticks.data--;
    }   
  }
}

void lift_up(){
  liftingServo_right.setEasingType(EASE_CUBIC_IN);
  liftingServo_right.startEaseTo(RIGHT_UP);
  liftingServo_left.setEasingType(EASE_CUBIC_IN);
  liftingServo_left.startEaseTo(LEFT_UP);
 }
void lift_down() {
  liftingServo_right.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_right.startEaseTo(RIGHT_DOWN);
  liftingServo_left.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_left.startEaseTo(LEFT_DOWN);
 }
 
 void lift_idle() {
  liftingServo_right.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_right.startEaseTo(RIGHT_IDLE);
  liftingServo_left.setEasingType(EASE_CUBIC_IN_OUT);
  liftingServo_left.startEaseTo(LEFT_IDLE);
 }
