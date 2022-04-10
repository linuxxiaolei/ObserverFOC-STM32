#ifndef __FOCSUB_H__
#define __FOCSUB_H__

#include "main.h"

extern float GetThetaE(float Theta, uint8_t Np);
extern void Cordic(float ThetaE, float* SinTheta, float* CosTheta);
extern void InvPark(float Ud, float Uq, float SinTheta, float CosTheta, float* Ux, float* Uy);
extern void InvClarke(float Ux, float Uy, float* U1, float* U2, float* U3);
extern void GetSector(float U1, float U2, float U3, uint8_t* Sector);
extern void GetCCR(MotorRealTimeInformation_str* MRT_Inf);
extern void Clarke(float Ia, float Ic, float* Ix, float* Iy);
extern void Park(float Ix, float Iy, float SinTheta, float CosTheta, float* Id, float* Iq);
extern void GetSpd(uint32_t Theta, uint32_t* Theta_Pre, float* Speed, float SpdFs);

#endif
