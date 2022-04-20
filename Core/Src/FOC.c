#include "FOC.h"
#include "FOCSub.h"
#include "DataProcessing.h"
#include "filt.h"

void FOCwithSensor(int8_t* Spd_Cnt, PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, SensorData_str* SensorData,ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    if(*Spd_Cnt < 9){
        *Spd_Cnt = *Spd_Cnt + 1;
    }else{
        CtrlComFilter(&CtrlCom->Spd, CtrlCom->Spd_Target, 0.05f);
        
        GetSpd(SensorData->Theta, &SensorData->Theta_Pre, &MRT_Inf->Spd, CtrlCom->SpdFs);

        CtrlCom->Id = 0;
        CtrlCom->Iq = PI_Control_Err(Spd_PI, CtrlCom->Spd - MRT_Inf->Spd);
        
        *Spd_Cnt = 0;
    }
    
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    SlidingModeObserver(CtrlCom, MotorParameter, MRT_Inf, SMO);
    
    MRT_Inf->Ud = PI_Control_Err(D_PI, CtrlCom->Id - MRT_Inf->Id);
    MRT_Inf->Uq = PI_Control_Err(Q_PI, CtrlCom->Iq - MRT_Inf->Iq);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}
