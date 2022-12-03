#ifndef __FOC_H
#define __FOC_H

#include "main.h"

extern void FOC_Mode_Select(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, SensorData_str* SensorData,ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI);
#endif
