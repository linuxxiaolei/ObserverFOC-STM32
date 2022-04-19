#include "DataProcessing.h"

float PID_Control(PI_str* pPI, float Target, float Present){
    pPI->Error = Target - Present;
    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * pPI->Error >= 0));
    
    pPI->up = pPI->Kp * pPI->Error;
    pPI->ui = pPI->ui + pPI->Ki * pPI->Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    float PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

float PI_Control_Err(PI_str* pPI, float Error){
    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * Error >= 0));
    
    pPI->up = pPI->Kp * Error;
    pPI->ui = pPI->ui + pPI->Ki * Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    float PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

float ObsPID_Control(PI_str* pPI, float Target, float Present){
    pPI->Error = Target-Present;

    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * pPI->Error >= 0));
    
    pPI->up = pPI->Kp * pPI->Error;
    pPI->ui = pPI->ui + pPI->Ki * pPI->Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    float PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

float PIMAX_Control(PI_str* pPI, float Target, float Present, float MaxUp, float MaxDown){
    float Error = Target - Present;
    uint8_t ui_flag = !(((pPI->Out_temp > MaxUp) || (pPI->Out_temp < MaxDown)) && (pPI->Out_temp * Error >= 0));
    
    pPI->up = pPI->Kp * Error;
    pPI->ui = pPI->ui + pPI->Ki * Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    float PIout = 0;
    
    if(pPI->Out_temp > MaxUp)
        PIout = MaxUp;
    else if(pPI->Out_temp < MaxDown)
        PIout = MaxDown;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

void LPF(float* Uo, float Ui, float Fs, float Wc){
    *Uo += Wc / Fs * (Ui - *Uo);
}

float SMOSwitchFunction1(float E, float Error){
    float SF_Out;

    SF_Out = Error / E;
    
    if(SF_Out > 1)
        SF_Out = 1;
    else if(SF_Out < -1)
        SF_Out = -1;

    return SF_Out;
}

void SlidingModeObserver(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Rms * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Rms * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix) * SMO->h1;
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy) * SMO->h1;

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Ix + MRT_Inf->Ux - SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Iy + MRT_Inf->Uy - SMO->Vy) / MotorParameter->Ls;
    
    LPF(&(SMO->Ex), SMO->Vx, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    LPF(&(SMO->Ey), SMO->Vy, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    
//    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);
//    SMO->EMF_Peak = SMO->EMF_Rms * 1.732f / 1.414f;

//    Cordic(SMO->ThetaE2, &(SMO->SinTheta), &(SMO->CosTheta));

//    SMO->de = -SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta;

//    if(SMO->EMF_Flag)
//        SMO->de = -SMO->de;

//    float SpdE = PI_Control_Err(&(SMO->SpdE_PI), SMO->de);
//    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);
//    
//    if((SMO->SpdE < 0.1f) && (SMO->SpdE > -0.1f))
//        SMO->Flux = 0;
//    else{
//        LPF(&(SMO->Flux), SMO->EMF_Peak / SMO->SpdE, CtrlCom->CurFs, 5 * 2 * PI);
//    }
//    
//    float ThetaE_temp = SMO->ThetaE + SpdE * CtrlCom->CurTs;
//    if(ThetaE_temp < 0)
//        ThetaE_temp += 2 * PI;
//    SMO->ThetaE = fmodf(ThetaE_temp, 2 * PI);
//    SMO->ThetaE2 = atan2f(-MRT_Inf->Ex , MRT_Inf->Ey);
}

void SlidingModeObserver2(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF_Rms = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->Ex = -MRT_Inf->EMF_Rms * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Rms * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Ix + MRT_Inf->Ux + 0 * SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Iy + MRT_Inf->Uy + 0 * SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->Vx, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    LPF(&(SMO->Ey), SMO->Vy, CtrlCom->CurFs, SMO->EMF_LPF_wc);

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    if(((-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) < SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux) && ((-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) > -SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux)){
        SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we;
        SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta);
    }
    else{
        SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn;
        SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) *2;
    }

    float SpdE = PI_Control_Err(&(SMO->SpdE_PI), SMO->de);
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);

    float ThetaE_temp = SMO->ThetaE + SpdE * CtrlCom->CurTs;
    if(ThetaE_temp < 0)
        ThetaE_temp += 2 * PI;
    SMO->ThetaE = fmodf(ThetaE_temp, 2 * PI);
    SMO->ThetaE2 = atan2f(-SMO->Ex, SMO->Ey);
    SMO->de = (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta);
}

void SlidingModeObserver3(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF_Rms = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->Ex = -MRT_Inf->EMF_Rms * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Rms * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - 1 * SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - 1 * SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    Cordic(SMO->ThetaE, &((SMO->SinTheta)), &(SMO->CosTheta));

    SMO->de = -SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta;

    if((SMO->de < SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux) && (SMO->de > -SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux)){
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / (SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux);
    }
    else{
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);
    }

    if(SMO->EMF_Flag)
        SMO->de = -SMO->de;

    float SpdE = PI_Control_Err(&(SMO->SpdE_PI), SMO->de);
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);

    float ThetaE_temp = SMO->ThetaE + SpdE * CtrlCom->CurTs;
    if(ThetaE_temp < 0)
        ThetaE_temp += 2 * PI;
    SMO->ThetaE = fmodf(ThetaE_temp, 2 * PI);
    SMO->ThetaE2 = atan2f(-SMO->Ex, SMO->Ey);
    SMO->de = (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta);
}
