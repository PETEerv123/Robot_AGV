#ifndef _PID_H

#define _PID_H

#define PID_PUBLIC(type) type
#define PID_PRIVATE(type) static type

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float fRef;
	float fSumErr;
	float fMaxSumErr;
	float fKp;
	float fKi;
    float fKd;
	float fpreProportional;
	float fErr[2];
	float fOuputValue;
	float fOutputLimit;
    float prevT ;
    float lastT ;
//	float fTs;
}PIDControl_handle_t;

PID_PUBLIC(void) PIDControl_SetParam(PIDControl_handle_t *pHandle, float fMaxSumErr, float fKp, float fKi,float fkd ,float fOutputLimit);

PID_PUBLIC(void) PIDControl_Reset(PIDControl_handle_t *pHandle);

PID_PUBLIC(float) PIDControl_Calc(PIDControl_handle_t *pHandle, float fSetpoint, float fFeedback);
#ifdef __cplusplus
}
#endif
#endif


