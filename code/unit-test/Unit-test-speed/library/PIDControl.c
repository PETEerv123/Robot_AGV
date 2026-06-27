#include "PID.h"


PID_PUBLIC(void) PIDControl_SetParam(PIDControl_handle_t *pHandle, float fMaxSumErr, float fKp, float fKi,float fkd ,float fOutputLimit){
	pHandle->fMaxSumErr = fMaxSumErr;
	pHandle->fKp = fKp;
	pHandle->fKi = fKi;
	pHandle->fKd = fkd;
	pHandle->fOutputLimit = fOutputLimit;
}	

PID_PUBLIC(void) PIDControl_Reset(PIDControl_handle_t *pHandle){
	pHandle->fRef = 0;
	pHandle->fSumErr = 0;
	pHandle->fErr[0] = 0;
	pHandle->fErr[1] = 0;
	pHandle->fpreProportional = 0;
	pHandle->fOuputValue = 0;
    pHandle->lastT = micros();
    pHandle->prevT = pHandle->lastT;
}

PID_PUBLIC(float) PIDControl_Calc(PIDControl_handle_t *pHandle, float fSetpoint, float fFeedback){

    pHandle->prevT = micros();
	pHandle->fErr[0] = fSetpoint - fFeedback;
//	pHandle->fSumErr += pHandle->fErr[0];

    float dt = (pHandle->prevT - pHandle->lastT)/1e6;  
    if(dt < 1e-6f) dt = 1e-6f;
    pHandle->fSumErr += (pHandle->fErr[0]) * dt;
    if(pHandle->fSumErr > pHandle->fMaxSumErr)
	{
		pHandle->fSumErr = pHandle->fMaxSumErr;
	} 
	else if(pHandle->fSumErr < -pHandle->fMaxSumErr)
	{
		pHandle->fSumErr = -pHandle->fMaxSumErr;
	}
    float Intergral = pHandle->fKi*pHandle->fSumErr;

    float Derivative = pHandle->fKd*(pHandle->fErr[0]-pHandle->fErr[1])/dt;
    float Proportional = pHandle->fKp*pHandle->fErr[0];
		//pHandle->ffOuputValue[0]= pHandle->fKp*pHandle->fErr[0]+pHandle->fSum;
	
//	pHandle->ffOuputValue[0] = pHandle->ffOuputValue[1] + pHandle->fKp *(pHandle->fErr[0]-pHandle->fErr[1]) + pHandle->fKi*(pHandle->fErr[0]+pHandle->fErr[1]);
//	pHandle->ffOuputValue[0] = pHandle->ffOuputValue[1] + pHandle->fKp *(pHandle->fErr[0]-pHandle->fErr[1]) + pHandle->fKi*pHandle->fTs*(pHandle->fErr[0]+pHandle->fErr[1]);
//	pHandle->ffOuputValue[0] = pHandle->fKp *(pHandle->fErr[0]-pHandle->fErr[1]) + pHandle->fKi*pHandle->fTs*pHandle->fSumErr;
	
	pHandle->fOuputValue = Proportional + Intergral + Derivative;

	if(pHandle->fOuputValue > pHandle->fOutputLimit)
	{
		pHandle->fOuputValue = pHandle->fOutputLimit;
	}
	else if(pHandle->fOuputValue < -pHandle->fOutputLimit)
	{
		pHandle->fOuputValue = -pHandle->fOutputLimit;
	}
	
	pHandle->fErr[1]=pHandle->fErr[0];
//	pHandle->ffOuputValue[1]= pHandle->ffOuputValue[0];
    pHandle->lastT = pHandle->prevT;
	return pHandle->fOuputValue;
}
