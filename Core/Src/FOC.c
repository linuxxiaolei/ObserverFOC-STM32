#include "FOC.h"
#include "FOCSub.h"
#include "DataProcessing.h"
#include "filt.h"

void VolLoop_Stop(ControlCommand_str* CtrlCom){
    CtrlCom->Uq_Target = 0;
}

void VolLoop_Mode0(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
    GetSpd(SensorData->Theta, &(SensorData->Theta_Pre), &MRT_Inf->Spd, CtrlCom->SpdFs, CtrlCom->Spd_Tick, &CtrlCom->Start_Flag);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    MRT_Inf->Ud = 0;
    MRT_Inf->Uq = CtrlCom->Uq_Target;
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void CurLoop_Stop(ControlCommand_str* CtrlCom){
    CtrlCom->Id_Target = 0;
}

void CurLoop_Mode1(PI_str* D_PI, PI_str* Q_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
    GetSpd(SensorData->Theta, &(SensorData->Theta_Pre), &MRT_Inf->Spd, CtrlCom->SpdFs, CtrlCom->Spd_Tick, &CtrlCom->Start_Flag);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    CtrlCom->Id = CtrlCom->Id_Target;
    CtrlCom->Iq = 0;

    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SpdLoop_Mode2(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
    GetSpd(SensorData->Theta, &(SensorData->Theta_Pre), &MRT_Inf->Spd, CtrlCom->SpdFs, CtrlCom->Spd_Tick, &CtrlCom->Start_Flag);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        Spd_PI->Error = CtrlCom->Spd - MRT_Inf->Spd;
        CtrlCom->Iq = PI_Control(Spd_PI);
    }

    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void FOC_Mode_Select(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, SensorData_str* SensorData,ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    Spd_Timer(&(CtrlCom->Spd_Tick));
    
    switch(CtrlCom->Mode){
        case 0:
            if(CtrlCom->Stop_Flag == 1){
                CtrlCom->Stop_Flag = 0;
                VolLoop_Stop(CtrlCom);
            }
            else{
                VolLoop_Mode0(CtrlCom, MotorParameter, MRT_Inf, SensorData);
            }
            break;
        case 1:
            if(CtrlCom->Stop_Flag == 1){
                CtrlCom->Stop_Flag = 0;
                CurLoop_Stop(CtrlCom);
            }
            else{
                CurLoop_Mode1(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
            }
            break;
        case 2:
            SpdLoop_Mode2(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
    }
}
