#include "01_program.h"
#include "03_motor_control.h"
#include "define.h"
#include <Arduino.h>
#include <stdio.h>

#define kp 5.0f
#define ki 0.1f
#define kd 0.0f
float setpoint;
/*Private Marco*/
Motor_Encoder motor_FL;
Motor_Encoder motor_FR;
Motor_Encoder motor_BL;
Motor_Encoder motor_BR;
/*Private Variables*/
void set_up(void) {
  Serial.begin(115200);
  Serial.println("Unit test position");
  /*Config Front Left*/
  motor_FL.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_FL_A;
  motor_FL.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_FL_B; 
  motor_FL.Motor_Encoder_Pin.encoder.id   = 0;

  motor_FL.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_FL_RPWM;
  motor_FL.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_FL_LPWM;
  motor_FL.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_FL_REN;
  motor_FL.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_FL_LEN;

  Motor_Encoder_Init(&motor_FL);
  Motor_Encoder_SetPositionPID(&motor_FL, kp, ki, kd, 255.0f);
  Motor_Encoder_ResetPositionPID(&motor_FL);
  /*Config Front Right*/
  motor_FR.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_FR_A;
  motor_FR.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_FR_B;
  motor_FR.Motor_Encoder_Pin.encoder.id   = 1;

  motor_FR.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_FR_RPWM;
  motor_FR.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_FR_LPWM;
  motor_FR.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_FR_REN;
  motor_FR.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_FR_LEN;

  Motor_Encoder_Init(&motor_FR);
  Motor_Encoder_SetPositionPID(&motor_FR, kp, ki, kd, 255.0f);
  Motor_Encoder_ResetPositionPID(&motor_FR);
  /*Config Back Left*/
  motor_BL.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_BL_A;
  motor_BL.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_BL_B;
  motor_BL.Motor_Encoder_Pin.encoder.id   = 2;

  motor_BL.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_BL_RPWM;
  motor_BL.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_BL_LPWM;
  motor_BL.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_BL_REN;
  motor_BL.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_BL_LEN;

  Motor_Encoder_Init(&motor_BL);
  Motor_Encoder_SetPositionPID(&motor_BL, kp, ki, kd, 255.0f);
  Motor_Encoder_ResetPositionPID(&motor_BL);
  /*Config Back Right*/
    /*Config Back Left*/
  motor_BR.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_BR_A;
  motor_BR.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_BR_B;
  motor_BR.Motor_Encoder_Pin.encoder.id   = 3;

  motor_BR.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_BR_RPWM;
  motor_BR.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_BR_LPWM;
  motor_BR.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_BR_REN;
  motor_BR.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_BR_LEN;

  Motor_Encoder_Init(&motor_BR);
  Motor_Encoder_SetPositionPID(&motor_BR, kp, ki, kd, 255.0f);
  Motor_Encoder_ResetPositionPID(&motor_BR);
}
void main_loop(void) {
  if(Serial.available() > 0){
    String input = Serial.readStringUntil('\n');
    setpoint = input.toFloat();
  }
  Motor_Encoder_PositionPID_Process(&motor_FL, setpoint);
  Motor_Encoder_PositionPID_Process(&motor_FR, setpoint);
  Serial.print("FL: ");
  Serial.print(Motor_Encoder_GetPosition(&motor_FL));
  Serial.print(" | FR: ");
  Serial.println(Motor_Encoder_GetPosition(&motor_FR));

}
