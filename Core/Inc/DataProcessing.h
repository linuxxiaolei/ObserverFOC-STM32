#ifndef __DATAPROCESSING_H__
#define __DATAPROCESSING_H__

#include "main.h"
#include "FOCSub.h"

extern float SMOSwitchFunction1(float E, float Error);
extern float PID_Control(PI_str* pPI, float Target, float Present);
extern float PI_Control_Err(PI_str* pPI, float Error);
extern float ObsPID_Control(PI_str* pPI, float Target, float Present);
extern float PIMAX_Control(PI_str* pPI, float Target, float Present, float MaxUp, float MaxDown);
extern void SlidingModeObserver(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO);
extern void SlidingModeObserver2(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO);
extern void SlidingModeObserver3(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO);
#endif
