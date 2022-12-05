#ifndef __SENSORLESS_H
#define __SENSORLESS_H

#include "main.h"

extern void Ih_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI);
extern void ThetaE_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI);
extern void Pole_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI);

#endif
