#include <Arduino.h>

#define encoder_left_pin_A 2
#define encoder_left_pin_B 3

#define encoder_right_pin_A 20
#define encoder_right_pin_B 21

#define left_motor_direction A0
#define right_motor_direction A1

#define left_motor_pwm 10
#define right_motor_pwm 11         

namespace Motors{
  byte motor_state[2] = {1, 0};
  uint8_t motor_dir[2] = {LOW, LOW};
  int motor_data[2] = {0, 0};
  unsigned long state_stamp[2] = {0, 0};
  
  volatile byte current_left_motor_pulse;
  volatile byte current_right_motor_pulse;

  int left_motor_pulse;
  int right_motor_pulse;

  volatile int lastEncoded = 0;
  volatile int lastEncoded_right = 0;
  int sig;

//  void left_motor_count(){
//    if (digitalRead(encoder_left_pin_A) == digitalRead(encoder_left_pin_B)) current_left_motor_pulse++;
//    else current_left_motor_pulse--;
//  }

  void updateEncoder(){
  int MSB = digitalRead(encoder_left_pin_A); //MSB = most significant bit
  int LSB = digitalRead(encoder_left_pin_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) current_left_motor_pulse ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) current_left_motor_pulse --;

  lastEncoded = encoded; //store this value for next time
  }

//  void right_motor_count(){
//    if (digitalRead(encoder_right_pin_A) == digitalRead(encoder_right_pin_B)) current_right_motor_pulse++;
//    else current_right_motor_pulse--;
//  }

  void updateEncoder_right(){
  int MSB_right = digitalRead(encoder_right_pin_A); //MSB = most significant bit
  int LSB_right = digitalRead(encoder_right_pin_B); //LSB = least significant bit

  int encoded_right = (MSB_right << 1) |LSB_right; //converting the 2 pin value to single number
  int sum_right  = (lastEncoded_right << 2) | encoded_right; //adding it to the previous encoded value

  if(sum_right == 0b1101 || sum_right == 0b0100 || sum_right == 0b0010 || sum_right == 0b1011) current_right_motor_pulse ++;
  if(sum_right == 0b1110 || sum_right == 0b0111 || sum_right == 0b0001 || sum_right == 0b1000) current_right_motor_pulse --;

  lastEncoded_right = encoded_right; //store this value for next time
  }

  int get_left_motor_pulse(){
    return left_motor_pulse;
  }

  int get_right_motor_pulse(){
    return right_motor_pulse;
  }

  void init_state_motors(){
    analogWrite(left_motor_direction, motor_dir[1]);
    analogWrite(right_motor_direction, motor_dir[0]);
    digitalWrite(left_motor_pwm, LOW);
    digitalWrite(right_motor_pwm, LOW);
    digitalWrite(encoder_left_pin_A, HIGH);
    digitalWrite(encoder_left_pin_B, HIGH);
    digitalWrite(encoder_right_pin_A, HIGH);
    digitalWrite(encoder_right_pin_B, HIGH);
    
  }

  void init_motors(){
    pinMode(left_motor_direction, OUTPUT);
    pinMode(right_motor_direction, OUTPUT);
    pinMode(left_motor_pwm, OUTPUT);
    pinMode(right_motor_pwm, OUTPUT);
    pinMode(encoder_left_pin_A, INPUT_PULLUP);
    pinMode(encoder_left_pin_B, INPUT_PULLUP);
    pinMode(encoder_right_pin_A, INPUT_PULLUP);
    pinMode(encoder_right_pin_B, INPUT_PULLUP);
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder, CHANGE);
    attachInterrupt(2, updateEncoder_right, CHANGE);
    attachInterrupt(3, updateEncoder_right, CHANGE);
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
        digitalWrite(mdir, (motor_dir[i] = (motor_data[i] < 0)?LOW:HIGH));
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
    updateMotorState(0, left_motor_direction, left_motor_pwm);

    right_motor_pulse += (motor_dir[1] == HIGH ? current_right_motor_pulse : -current_right_motor_pulse);
    current_right_motor_pulse = 0;    
    updateMotorState(1, right_motor_direction, right_motor_pwm);
    
  }
}
