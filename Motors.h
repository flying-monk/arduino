#include <Arduino.h>

#define encoder_left_pin_A 2
#define encoder_left_pin_B 4

#define encoder_right_pin_A 3
#define encoder_right_pin_B 5

#define left_motor_direction 6
#define right_motor_direction 8

#define left_motor_pwm 10
#define right_motor_pwm 11

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

boolean Direction_left = true;
boolean Direction_right = true;

const bool left_fwd = true;
const bool right_fwd = false;

unsigned long lastMilli = 0;
const double radius = 0.1;                   
const double wheelbase = 0.030;               

double speed_req = 0;                        
double angular_speed_req = 0;               

double speed_req_left = 0;             
double speed_act_left = 0;               
double speed_cmd_left = 0;               

double speed_req_right = 0;                  
double speed_act_right = 0;                  
double speed_cmd_right = 0;

volatile float pos_left = 0;       
volatile float pos_right = 0; 

const double max_speed = 0.4;

const int default_vel = 80;
int state_vel = default_vel;
enum State {FWD, BWD, RIGHT, LEFT, STOP};
State state;
const int max_vel = 255;

unsigned int noCommLoops = 0;

int PWM_leftMotor = 0;                
int PWM_rightMotor = 0;

namespace Motors{
  byte motor_state[2] = {1, 1};
  uint8_t motor_dir[2] = {LOW, HIGH};
  int motor_data[2] = {0, 0};
  unsigned long state_stamp[2] = {0, 0};
  
  volatile byte current_left_motor_pulse;
  volatile byte current_right_motor_pulse;

  int left_motor_pulse;
  int right_motor_pulse;

  void left_motor_count(){
    if (digitalRead(encoder_left_pin_A) == digitalRead(encoder_left_pin_B)) current_left_motor_pulse++;
    else current_left_motor_pulse--;
  }

  void right_motor_count(){
    if (digitalRead(encoder_right_pin_A) == digitalRead(encoder_right_pin_B)) current_right_motor_pulse++;
    else current_right_motor_pulse--;
  }

  int get_left_motor_pulse(){
    return left_motor_pulse;
  }

  int get_right_motor_pulse(){
    return right_motor_pulse;
  }

  void init_state_motors(){
    digitalWrite(left_motor_direction, LOW);
    digitalWrite(right_motor_direction, LOW);
    digitalWrite(left_motor_pwm, LOW);
    digitalWrite(right_motor_pwm, LOW);
    digitalWrite(encoder_left_pin_A, HIGH);
    digitalWrite(encoder_left_pin_B, HIGH);
    digitalWrite(encoder_right_pin_A, HIGH);
    digitalWrite(encoder_right_pin_B, HIGH);
    
  }

  void init_motors(){
    pinMode(left_motor_direction, motor_dir[0]);
    pinMode(right_motor_direction, motor_dir[1]);
    pinMode(left_motor_pwm, OUTPUT);
    pinMode(right_motor_pwm, OUTPUT);
    pinMode(encoder_left_pin_A, INPUT);
    pinMode(encoder_left_pin_B, INPUT);
    pinMode(encoder_right_pin_A, INPUT);
    pinMode(encoder_right_pin_B, INPUT);
    attachInterrupt(0, left_motor_count, RISING);
    attachInterrupt(1, right_motor_count, RISING);
    init_state_motors();
  }

  void stop_motors(){
    if (motor_state[0]==0){
      analogWrite(left_motor_pwm, 0);
      analogWrite(right_motor_pwm, 0);
      motor_data[0] = 0;
      motor_state[0] = 1;
      state_stamp[0] = millis();
    }
    if (motor_state[1]==0){
      analogWrite(left_motor_pwm, 0);
      analogWrite(right_motor_pwm, 0);
      motor_data[0] = 0;
      motor_state[0] = 1;
      state_stamp[0] = millis();
    }
  }

  void left_motor(int speed_data){
    motor_data[0] = speed_data;
    state_stamp[0] = millis();
  }

  void right_motor(int speed_data){
    motor_data[1] = speed_data;
    state_stamp[1] = millis();
  }

  void updateMotorState(byte i, byte mdir, byte mvrm){
    if (millis()-state_stamp[i] > 10){
      if (motor_data[i] != 0){
        analogWrite(mvrm, byte(abs(motor_data[i])));
        digitalWrite(mdir, (motor_dir[i] = (motor_data[i] < 0)?HIGH:LOW));

        motor_state[i] = 0;
        state_stamp[i] = millis();
      }
      else if(motor_state[i] == 0){
        analogWrite(left_motor_pwm, 0);
        analogWrite(right_motor_pwm, 0);
        motor_data[i] = 0;
        motor_state[i] = 1;
        state_stamp[i] = millis();
      }
    }
    
  }

  void updateState(){
    left_motor_pulse += (motor_dir[0] == LOW ? current_left_motor_pulse : -current_left_motor_pulse);
    current_left_motor_pulse = 0;

    right_motor_pulse += (motor_dir[0] == LOW ? current_right_motor_pulse : -current_right_motor_pulse);
    current_right_motor_pulse = 0;

    updateMotorState(0, left_motor_direction, left_motor_pwm);
    updateMotorState(1, right_motor_direction, right_motor_pwm);
    
  }

  void MoveBwd(const size_t speed) {
    digitalWrite(right_motor_direction, right_fwd);
    digitalWrite(left_motor_direction, !left_fwd);
    analogWrite(right_motor_pwm, speed);
    analogWrite(left_motor_pwm, speed);
  }
  
  void MoveFwd(const size_t speed) {
    digitalWrite(right_motor_direction, !right_fwd);
    digitalWrite(left_motor_direction, left_fwd);
    analogWrite(right_motor_pwm, speed);
    analogWrite(left_motor_pwm, speed);
  }
  
  void MoveRight(const size_t speed) {
    digitalWrite(right_motor_direction, right_fwd);
    digitalWrite(left_motor_direction, left_fwd);
    analogWrite(right_motor_pwm, speed);
    analogWrite(left_motor_pwm, speed);
  }
  
  void MoveLeft(const size_t speed) {
    digitalWrite(right_motor_direction, !right_fwd);
    digitalWrite(left_motor_direction, !left_fwd);
    analogWrite(right_motor_pwm, speed);
    analogWrite(left_motor_pwm, speed);
  }
  
  void MoveStop() {
    digitalWrite(right_motor_direction, right_fwd);
    digitalWrite(left_motor_direction, left_fwd);
    analogWrite(right_motor_pwm, 0);
    analogWrite(left_motor_pwm, 0);
  }
}
