#include "01_program.h"
#include "03_motor_control.h"
#include "Adafruit_BNO08x.h"
#include "define.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
/*Configure Library*/
Motor_Encoder motor_FL;
Motor_Encoder motor_FR;
Motor_Encoder motor_BL;
Motor_Encoder motor_BR;

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t imuValue;
/*Private Marco*/
#define kp 110.0f
#define ki 35.0f
#define kd 0.0f


String inputString = "";
volatile bool stringComplete;
float vTarget[4];
float vx, vy, wz;
unsigned long last_time = 0;
float imuYaw = 0.0f;
float theta_fused = 0.0f;
/*Private Variables*/
void set_up(void) {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Mototion Control Systems");
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 not detected");
    while (1);
  } else {
    Serial.println("BNO085 Init Success");
  }
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

  /*Config Front Left*/
  motor_FL.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_FL_A;
  motor_FL.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_FL_B;
  motor_FL.Motor_Encoder_Pin.encoder.id   = 0;

  motor_FL.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_FL_RPWM;
  motor_FL.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_FL_LPWM;
  motor_FL.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_FL_REN;
  motor_FL.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_FL_LEN;

  Motor_Encoder_Init(&motor_FL);
  Motor_Encoder_SetSpeedPID(&motor_FL, kp, ki, kd, 150);
  Motor_Encoder_ResetSpeedPID(&motor_FL);
  /*Config Front Right*/
  motor_FR.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_FR_A;
  motor_FR.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_FR_B;
  motor_FR.Motor_Encoder_Pin.encoder.id   = 1;

  motor_FR.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_FR_RPWM;
  motor_FR.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_FR_LPWM;
  motor_FR.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_FR_REN;
  motor_FR.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_FR_LEN;

  Motor_Encoder_Init(&motor_FR);
  Motor_Encoder_SetSpeedPID(&motor_FR, kp, ki, kd, 150);
  Motor_Encoder_ResetSpeedPID(&motor_FR);
  /*Config Back Left*/
  motor_BL.Motor_Encoder_Pin.encoder.pinA = PIN_ENCODER_BL_A;
  motor_BL.Motor_Encoder_Pin.encoder.pinB = PIN_ENCODER_BL_B;
  motor_BL.Motor_Encoder_Pin.encoder.id   = 2;

  motor_BL.Motor_Encoder_Pin.RPWM_pin = PIN_MOTOR_BL_RPWM;
  motor_BL.Motor_Encoder_Pin.LPWM_pin = PIN_MOTOR_BL_LPWM;
  motor_BL.Motor_Encoder_Pin.REN_pin  = PIN_MOTOR_BL_REN;
  motor_BL.Motor_Encoder_Pin.LEN_pin  = PIN_MOTOR_BL_LEN;

  Motor_Encoder_Init(&motor_BL);
  Motor_Encoder_SetSpeedPID(&motor_BL, kp, ki, kd, 10);
  Motor_Encoder_ResetSpeedPID(&motor_BL);
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
  Motor_Encoder_SetSpeedPID(&motor_BR, kp, ki, kd, 10);
  Motor_Encoder_ResetSpeedPID(&motor_BR);
}
void main_loop(void) {
  unsigned long now = micros();
  float dt = (now - last_time) / 1000000.0f; 
  if (dt < 0.02f) return; 
  last_time = now;
  if (stringComplete) {
    vTarget[0] = vx + vy - wz * (d - L);      // FL
    vTarget[1] = vx - vy - wz * (d - L);      // BL
    vTarget[2] = vx + vy + wz * (d - L);      // BR
    vTarget[3] = vx - vy + wz * (d - L);      // FR
    stringComplete = false;

  }

  Motor_Encoder_SpeedPID_Procces(&motor_FL, -vTarget[0]); //m/s
  Motor_Encoder_SpeedPID_Procces(&motor_BL, -vTarget[1]);
  Motor_Encoder_SpeedPID_Procces(&motor_BR, vTarget[2]);
  Motor_Encoder_SpeedPID_Procces(&motor_FR, vTarget[3]);
  float Vx_est =  (WHEEL_CIRC * (motor_FL.speedInfo.rps + motor_BL.speedInfo.rps + motor_BR.speedInfo.rps + motor_FR.speedInfo.rps)) / 4.0f;
  float Vy_est =  (WHEEL_CIRC * (motor_FL.speedInfo.rps - motor_BL.speedInfo.rps + motor_BR.speedInfo.rps - motor_FR.speedInfo.rps)) / 4.0f;
  float wz_est =  (WHEEL_CIRC * (-motor_FL.speedInfo.rps - motor_BL.speedInfo.rps + motor_BR.speedInfo.rps + motor_FR.speedInfo.rps )) / (4.0f * (d - L));
  if (bno08x.getSensorEvent(&imuValue)) {
    if (imuValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qw = imuValue.un.gameRotationVector.real;
      float qx = imuValue.un.gameRotationVector.i;
      float qy = imuValue.un.gameRotationVector.j;
      float qz = imuValue.un.gameRotationVector.k;

      imuYaw = atan2( 2.0 * (qw * qz + qx * qy),1.0 - 2.0 * (qy * qy + qz * qz));
    }
  }
  const float ALPHA = 0.95;
  float theta_enc = theta_fused + wz_est * dt;
  theta_fused = ALPHA * theta_enc + (1.0 - ALPHA) * imuYaw;
  Serial.print(Vx_est, 3); Serial.print(" ");
  Serial.print(Vy_est, 3); Serial.print(" ");
  Serial.print(wz_est, 3); Serial.print(" ");
  Serial.println(theta_fused, 3);`
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      char buf[64];
      inputString.toCharArray(buf, sizeof(buf));

      char *endString = strtok(buf, " ");
      vx = atof(endString);

      endString = strtok(NULL, " ");
      vy = atof(endString);

      endString = strtok(NULL, " ");
      wz = atof(endString);

      stringComplete = true;
      inputString = "";
    }
  }
}
