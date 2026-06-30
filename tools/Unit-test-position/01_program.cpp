#include "01_program.h"
#include "03_motor_control.h"
#include "define.h"
#include <Arduino.h>
#include <stdio.h>

float kp = 5.0f;
float ki = 0.01f;
float kd = 0.0f;
float setpoint;
/*Private Marco*/
Motor_Encoder motor_FL;
void ProcessSerial(void);
float AngleWrap(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}
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
//  Motor_Encoder_Run(&motor_FL,setpoint);/
//  Motor_Encoder_Processing(&motor_FL, 30);///
  ProcessSerial();
  float feedback =((float)Encoder_GetPosition(&motor_FL.Motor_Encoder_Pin.encoder)* 360.0f) / TICKS_PER_REV;

  feedback = fmodf(feedback, 360.0f);

  if(feedback < 0.0f)
        feedback += 360.0f;
  Motor_Encoder_Processing(&motor_FL, setpoint);
  Serial.print(setpoint, 2);
  Serial.print(", ");
  Serial.println(feedback,2);
}


void ProcessSerial(void)
{
    if (Serial.available() <= 0)
        return;

    String input = Serial.readStringUntil('\n');
    input.trim();

    Serial.print("RX=[");
    Serial.print(input);
    Serial.println("]");

    /* setpoint:30 */
    if (input.startsWith("setpoint:"))
    {
        setpoint = input.substring(9).toFloat();

        Serial.print("Setpoint = ");
        Serial.println(setpoint, 2);
        return;
    }

    /* pid:1.0 0.1 0.01 */
    if (input.startsWith("pid:"))
    {
        String data = input.substring(4);

        int p1 = data.indexOf(' ');
        int p2 = data.indexOf(' ', p1 + 1);

        if (p1 > 0 && p2 > p1)
        {
            kp = data.substring(0, p1).toFloat();
            kd = data.substring(p1 + 1, p2).toFloat();
            ki = data.substring(p2 + 1).toFloat();

            Motor_Encoder_SetPositionPID(
                &motor_FL,
                kp,
                ki,
                kd,
                255.0f);

            Serial.print("Kp=");
            Serial.print(kp, 3);

            Serial.print(" Kd=");
            Serial.print(kd, 3);

            Serial.print(" Ki=");
            Serial.println(ki, 3);
        }
        else
        {
            Serial.println("PID format: pid:kp kd ki");
        }

        return;
    }

    Serial.println("Unknown command");
}
