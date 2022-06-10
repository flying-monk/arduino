#include <Arduino.h>
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Twist.h>
#include "Motors.h"

#define ROS_LOOP_TIME_DELAY 50
#define MAXIMUM_BREAK_COUNT 3


ros::NodeHandle nh;

std_msgs::Int16 int_msg_right;
std_msgs::Int16 int_msg_left;
std_msgs::Int16 left_wheel;
std_msgs::Int16 right_wheel;

unsigned long ros_loop_stamp = 100;

void left_cmd_vel_CB(const std_msgs::Int16& tspeed){
  Motors::left_motor(tspeed.data);
  
}

void right_cmd_vel_CB(const std_msgs::Int16& tspeed){
  Motors::right_motor(tspeed.data);

}

ros::Publisher lwheel_encoder("lwheel_encoder",&left_wheel);
ros::Publisher rwheel_encoder("rwheel_encoder",&right_wheel);

ros::Publisher motor_right_pub("motor_right", &int_msg_right);
ros::Publisher motor_left_pub("motor_left", &int_msg_left);

ros::Subscriber<std_msgs::Int16> left_motor_cmd("left_motor_cmd",&left_cmd_vel_CB);
ros::Subscriber<std_msgs::Int16> right_motor_cmd("right_motor_cmd",&right_cmd_vel_CB);


void setup() {
  Motors::init_motors();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(lwheel_encoder);
  nh.advertise(rwheel_encoder);
  nh.subscribe(left_motor_cmd);
  nh.subscribe(right_motor_cmd);
}

void loop() {
  Motors::updateState();
  
  left_wheel.data = Motors::get_left_motor_pulse();
  lwheel_encoder.publish(&left_wheel);
  right_wheel.data = Motors::get_right_motor_pulse();
  rwheel_encoder.publish(&right_wheel);
  nh.spinOnce();
  delay(100);

  ros_loop_stamp = millis();
  if(ros_loop_stamp < ROS_LOOP_TIME_DELAY) delay(ROS_LOOP_TIME_DELAY-ros_loop_stamp);
  ros_loop_stamp = millis();
  
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
