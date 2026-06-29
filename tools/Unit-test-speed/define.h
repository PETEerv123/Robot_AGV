#ifndef __define_H
#define __define_H

#ifdef __cplusplus
extern "C"
{
#endif
/*Define Encoder Pins*/
/*Fornt Encoder Pins*/
#define PIN_ENCODER_FL_A       2
#define PIN_ENCODER_FL_B       24

#define PIN_MOTOR_FL_LPWM      4
#define PIN_MOTOR_FL_RPWM      5
#define PIN_MOTOR_FL_LEN       22
#define PIN_MOTOR_FL_REN       23


#define PIN_ENCODER_FR_A       3
#define PIN_ENCODER_FR_B       25
/*Back Encoder Pins*/
#define PIN_ENCODER_BL_A       18
#define PIN_ENCODER_BL_B       26
#define PIN_ENCODER_BR_A       19
#define PIN_ENCODER_BR_B       27

#define ENCODER_PPR      13.0f
#define GEAR_RATIO       19.2f
#define QUAD_FACTOR      4.0f
#define TICKS_PER_REV    (ENCODER_PPR * GEAR_RATIO * QUAD_FACTOR)

#define L  0.27     // meters
#define W  0.27     // meters
#define R  0.075    // meters
#define WHEEL_CIRC  2.0 * M_PI * R

#ifdef __cplusplus
}
#endif
#endif
