#ifndef _program_h
#define _program_h

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_ENCODER_FL_A       2
#define PIN_ENCODER_FL_B       24

#define ENCODER_PPR      7.0f
#define GEAR_RATIO       19.2f
#define QUAD_FACTOR      4.0f
#define TICKS_PER_REV    (ENCODER_PPR * GEAR_RATIO * QUAD_FACTOR)

#define L  0.27     // meters
#define W  0.27     // meters
#define R  0.075    // meters
#define WHEEL_CIRC  2.0 * M_PI * R

void set_up(void);
void main_loop(void);

#ifdef __cplusplus
}
#endif

#endif