#ifndef __define_H
#define __define_H

#ifdef __cplusplus
extern "C"
{
#endif
/*Define Encoder Pins*/
/*Begin Fornt lèeft*/
#define PIN_ENCODER_FL_A       2
#define PIN_ENCODER_FL_B       24


#define PIN_MOTOR_FL_LPWM      6
#define PIN_MOTOR_FL_RPWM      7
#define PIN_MOTOR_FL_LEN       13
#define PIN_MOTOR_FL_REN       12
/*End Front Left*/
/*Begin Front right*/
#define PIN_ENCODER_FR_A       25
#define PIN_ENCODER_FR_B       3

#define PIN_MOTOR_FR_RPWM      5
#define PIN_MOTOR_FR_LPWM      4
#define PIN_MOTOR_FR_REN       22
#define PIN_MOTOR_FR_LEN       23
/*End Front right*/
/*Back Encoder Pins*/
/*Begin Back Left*/
#define PIN_ENCODER_BL_A       18
#define PIN_ENCODER_BL_B       26

#define PIN_MOTOR_BL_RPWM      10
#define PIN_MOTOR_BL_LPWM      11
#define PIN_MOTOR_BL_REN       16
#define PIN_MOTOR_BL_LEN       17
/*End Back Left*/
/*Begin Back right*/
#define PIN_ENCODER_BR_A       19
#define PIN_ENCODER_BR_B       27


#define PIN_MOTOR_BR_RPWM      8
#define PIN_MOTOR_BR_LPWM      9
#define PIN_MOTOR_BR_REN       14
#define PIN_MOTOR_BR_LEN       15
/*End Back right*/
/*End Config Encoder*/
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
