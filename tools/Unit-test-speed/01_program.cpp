#include "01_program.h"
#include "03_motor_control.h"
#include "define.h"
#include <Arduino.h>
#include <stdio.h>

#define kp 30.0f
#define ki 5.0f
#define kd 0.0f
float setpoint;
/*Private Marco*/
Motor_Encoder motor_FL;

/*Private Variables*/
void set_up(void) {
  Serial.begin(115200);
  Serial.println("Unit test speed");
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
}
void main_loop(void) {
  if(Serial.available() > 0){
    String input = Serial.readStringUntil('\n');
    setpoint = input.toFloat();
  }
  Motor_Encoder_Run(&motor_FL,setpoint);
//  Motor_Encoder_Processing/(&motor_FL, setpoint);
  Serial.print(setpoint, 2);
  Serial.print(", ");
  Serial.println((Encoder_GetPosition(&motor_FL.Motor_Encoder_Pin.encoder) * 360.0f) / TICKS_PER_REV,2);
}

// void Event_SerialPrint(void){
//     char
// }
