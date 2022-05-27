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

#define LOOPTIME 100 
#define ROS_LOOP_TIME_DELAY 50
#define MAXIMUM_BREAK_COUNT 3

std_msgs::Int16 left_wheel;
std_msgs::Int16 right_wheel;

ros::NodeHandle nh;
//ros::Publisher pub;

unsigned long ros_loop_stamp = 100;

int break_count = 0;

void left_cmd_vel_CB(const std_msgs::Int16& tspeed){
  Motors::left_motor(tspeed.data);
}

void right_cmd_vel_CB(const std_msgs::Int16& tspeed){
  Motors::right_motor(tspeed.data);
}

ros::Publisher left_wheel_encoder("left_wheel_encoder",&left_wheel);
ros::Publisher right_wheel_encoder("right_wheel_encoder",&right_wheel);

ros::Subscriber<std_msgs::Int16> left_motor_cmd("left_motor_cmd",&left_cmd_vel_CB);
ros::Subscriber<std_msgs::Int16> right_motor_cmd("right_motor_cmd",&right_cmd_vel_CB);

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                          
  
  speed_req = cmd_vel.linear.x;      

  angular_speed_req = cmd_vel.angular.z;  
  
  speed_req_left = speed_req - angular_speed_req*(wheelbase/2);     
  speed_req_right = speed_req + angular_speed_req*(wheelbase/2);    
}

void cmd_vel_cb(const geometry_msgs::Twist & msg) {

  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;
  //  MoveDifferential(x,z_rotation);
  if (x > 0) {
    if (state == FWD){
      state_vel +=10;
      state_vel = min(state_vel, 255);
    }
    Motors::MoveFwd(state_vel);
    state = FWD;
  }
  else if (x < 0) {
    if (state == BWD){
      state_vel +=10;
      state_vel = min(state_vel, 255);

    }
    Motors::MoveBwd(state_vel);
    state = BWD;
  }
  else if (z_rotation < 0) {
    if (state == RIGHT){
      state_vel +=10;
      state_vel = min(state_vel, 255);

    }
    Motors::MoveRight (state_vel);
    state = RIGHT;
  }  
  else if (z_rotation > 0) {
    if (state == LEFT){
      state_vel += 10;
      state_vel = min(state_vel, 255);

    }
    Motors::MoveLeft (state_vel);
    state = LEFT;
  }
  else {
    Motors::MoveStop();
    state_vel = default_vel;
    state = STOP;
  }

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {
  Motors::init_motors();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(left_wheel_encoder);
  nh.advertise(right_wheel_encoder);
  nh.subscribe(left_motor_cmd);
  nh.subscribe(right_motor_cmd);
  nh.subscribe(sub);
}

void loop() {
  Motors::updateState();
  
  left_wheel.data = Motors::get_left_motor_pulse();
  left_wheel_encoder.publish(&left_wheel);
  right_wheel.data = Motors::get_right_motor_pulse();
  right_wheel_encoder.publish(&right_wheel);
  nh.spinOnce();
  delay(100);

  ros_loop_stamp = millis();
  if(ros_loop_stamp < ROS_LOOP_TIME_DELAY) delay(ROS_LOOP_TIME_DELAY-ros_loop_stamp);
  ros_loop_stamp = millis();
  
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
