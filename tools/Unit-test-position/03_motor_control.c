#include "03_motor_control.h"

MOTOR_PRIVATE(float) AngleWrap(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}
MOTOR_PUBLIC(void) Motor_Encoder_Init(Motor_Encoder *pHandle){
    
  Encoder_Init(&pHandle->Motor_Encoder_Pin.encoder);
//   PIDControl_SetParam(&pHandle->speed_PID, 1000.0f, 1.0f, 0.1f, 0.01f, 255.0f);
  pinMode(pHandle->Motor_Encoder_Pin.RPWM_pin, OUTPUT);
  pinMode(pHandle->Motor_Encoder_Pin.LPWM_pin, OUTPUT);
  pinMode(pHandle->Motor_Encoder_Pin.LEN_pin, OUTPUT);
  pinMode(pHandle->Motor_Encoder_Pin.REN_pin, OUTPUT);
  digitalWrite(pHandle->Motor_Encoder_Pin.REN_pin, HIGH);
  digitalWrite(pHandle->Motor_Encoder_Pin.LEN_pin, HIGH);
}
MOTOR_PUBLIC(void) Motor_Encoder_Run(Motor_Encoder *pHandle, int16_t pwm){
  if(pHandle == NULL) return;
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    analogWrite(pHandle->Motor_Encoder_Pin.RPWM_pin, pwm);
    analogWrite(pHandle->Motor_Encoder_Pin.LPWM_pin, 0);
  } else {
    analogWrite(pHandle->Motor_Encoder_Pin.RPWM_pin, 0);
    analogWrite(pHandle->Motor_Encoder_Pin.LPWM_pin, -pwm);
  }
}
MOTOR_PUBLIC(void) Motor_Encoder_Stop(Motor_Encoder *pHandle){
    if(pHandle == NULL) return;
    digitalWrite(pHandle->Motor_Encoder_Pin.REN_pin, HIGH);
    digitalWrite(pHandle->Motor_Encoder_Pin.LEN_pin, HIGH);
    analogWrite(pHandle->Motor_Encoder_Pin.RPWM_pin, 0);
    analogWrite(pHandle->Motor_Encoder_Pin.LPWM_pin, 0);
}
/* Speed PID Functions */
MOTOR_PUBLIC(void) Motor_Encoder_SetSpeedPID(Motor_Encoder *pHandle, float Kp, float Ki, float Kd, float outMax){
    PIDControl_SetParam(&pHandle->speed_PID, 1000.0f, Kp, Ki, Kd, outMax);
}
MOTOR_PUBLIC(float) Motor_Encoder_CalcSpeedPID(Motor_Encoder *pHandle, float setpoint, float feedback){
    return PIDControl_Calc(&pHandle->speed_PID, setpoint, feedback);
}
MOTOR_PUBLIC(void) Motor_Encoder_ResetSpeedPID(Motor_Encoder *pHandle){
    PIDControl_Reset(&pHandle->speed_PID);
}
/* Position PID Functions */
MOTOR_PUBLIC(float) Motor_Encoder_GetPosition(Motor_Encoder *pHandle){
    float theta_mech = (Encoder_GetPosition(&pHandle->Motor_Encoder_Pin.encoder) * 360.0f)/ TICKS_PER_REV;
    return theta_mech;
}

MOTOR_PUBLIC(void) Motor_Encoder_Position_Processing(Motor_Encoder *pHandle,float setpoint){
    float feedback = Motor_Encoder_GetPosition(pHandle);

    feedback = fmodf(feedback, 360.0f);
    if(feedback < 0.0f)
        feedback += 360.0f;

    float error = AngleWrap(setpoint - feedback);

    float output =  PIDControl_Calc(&pHandle->position_PID,error, 0.0f);
    Motor_Encoder_Run(pHandle, (int16_t)output);
}
MOTOR_PUBLIC(void) Motor_Encoder_SetPositionPID(Motor_Encoder *pHandle, float Kp, float Ki, float Kd, float outMax){
    PIDControl_SetParam(&pHandle->position_PID, 1000.0f, Kp, Ki, Kd, outMax);
}
MOTOR_PUBLIC(float) Motor_Encoder_CalcPositionPID(Motor_Encoder *pHandle, float setpoint, float feedback){
    return PIDControl_Calc(&pHandle->position_PID, setpoint, feedback);
}
MOTOR_PUBLIC(void) Motor_Encoder_ResetPositionPID(Motor_Encoder *pHandle){
    PIDControl_Reset(&pHandle->position_PID);
}
