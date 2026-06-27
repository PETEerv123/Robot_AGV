#ifndef _Motor_control_h
#define _Motor_control_h

#include "PIDControl.h"

#define MOTOR_PUBLIC(type) type
#define MOTOR_PRIVATE(type) static type


typedef struct
{

    PIDControl_handle_t speed_PID;
}Motor_Encoder;



#endif