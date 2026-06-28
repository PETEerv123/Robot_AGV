#ifndef _Encoder_H
#define _Encoder_H

#include "stdint.h"
#include "Arduino.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define Encoder_PUBLIC(type) type
#define Encoder_PRIVATE(type) static type

#define MAX_ENCODERS 4
typedef struct
{
	uint8_t pinA;
	uint8_t pinB;
	volatile uint8_t state;
	volatile uint32_t position;

	uint8_t id;
} Encoder_Handle;

Encoder_PUBLIC(void) Encoder_Init(Encoder_Handle *pHandle);
Encoder_PUBLIC(int32_t) Encoder_GetPosition(Encoder_Handle *pHandle);
#ifdef __cplusplus
}
#endif

#endif
