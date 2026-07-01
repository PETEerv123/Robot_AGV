#ifndef _Motor_control_h
#define _Motor_control_h

#include "04_PIDControl.h"
#include "02_Encoder.h"
#include "define.h"
#ifdef __cplusplus
extern "C"{
#endif
/*Mạch Cầu H BTS7960 */
#define MOTOR_PUBLIC(type) type
#define MOTOR_PRIVATE(type) static type


typedef struct
{
  struct{
    Encoder_Handle encoder;
    uint8_t RPWM_pin;
    uint8_t LPWM_pin;
    uint8_t LEN_pin;
    uint8_t REN_pin;
  } Motor_Encoder_Pin;
  PIDControl_handle_t speed_PID;  
  PIDControl_handle_t position_PID;  
}Motor_Encoder;
/*Basic Control Motor*/
MOTOR_PUBLIC(void) Motor_Encoder_Init(Motor_Encoder *pHandle);
MOTOR_PUBLIC(void) Motor_Encoder_Run(Motor_Encoder *pHandle, int16_t pwm);
MOTOR_PUBLIC(void) Motor_Encoder_Stop(Motor_Encoder *pHandle);
/* Speed PID Functions */
MOTOR_PUBLIC(void)  Motor_Encoder_SetSpeedPID(Motor_Encoder *pHandle, float Kp, float Ki, float Kd, float outMax);
MOTOR_PUBLIC(float) Motor_Encoder_CalcSpeedPID(Motor_Encoder *pHandle, float setpoint, float feedback);
MOTOR_PUBLIC(void)  Motor_Encoder_ResetSpeedPID(Motor_Encoder *pHandle);

/* Position PID Functions */
MOTOR_PUBLIC(float) Motor_Encoder_GetPosition(Motor_Encoder *pHandle);
MOTOR_PUBLIC(void) Motor_Encoder_Position_Processing(Motor_Encoder *pHandle,float setpoint);

MOTOR_PUBLIC(void)  Motor_Encoder_SetPositionPID(Motor_Encoder *pHandle, float Kp, float Ki, float Kd, float outMax);
MOTOR_PUBLIC(float) Motor_Encoder_CalcPositionPID(Motor_Encoder *pHandle, float setpoint, float feedback);
MOTOR_PUBLIC(void)  Motor_Encoder_ResetPositionPID(Motor_Encoder *pHandle);

#ifdef __cplusplus
}
#endif

#endif