#include "FOC.h"
#include "FOCSub.h"
#include "Sensorless.h"
#include "DataProcessing.h"

void VolLoop_Start(ControlCommand_str* CtrlCom){
    CtrlCom->Uq_Target = 0;
    CtrlCom->Status_Flag = RUN;
}

void VolLoop_Mode(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
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

void VolLoop_Stop(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
    if(CtrlCom->Uq != 0){
        CtrlCom->Uq_Target = 0;
        VolLoop_Mode(CtrlCom, MotorParameter, MRT_Inf, SensorData);
    }
    else{
        CtrlCom->Status_Flag = START;
    }
}

void CurLoop_Start(ControlCommand_str* CtrlCom){
    CtrlCom->Id_Target = 0;
    CtrlCom->Status_Flag = RUN;
}

void CurLoop_Mode(PI_str* D_PI, PI_str* Q_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
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

void CurLoop_Stop(PI_str* D_PI, PI_str* Q_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
    if((MRT_Inf->Id < CurLoopStop_Threshold) && (MRT_Inf->Id > -CurLoopStop_Threshold) && (MRT_Inf->Iq < CurLoopStop_Threshold) && (MRT_Inf->Iq > -CurLoopStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }
    
    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Id_Target = 0;
        CurLoop_Mode(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
    }
}

void SpdLoop_Start(ControlCommand_str* CtrlCom){
    CtrlCom->Spd = 0;
    CtrlCom->Status_Flag = RUN;
}

void SpdLoop_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
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

void SpdLoop_Stop(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SensorData_str* SensorData){
    if((MRT_Inf->Spd < SpdLoopStop_Threshold) && (MRT_Inf->Spd > -SpdLoopStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }

    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Spd = 0;
        SpdLoop_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
    }
}

void HFI_Start(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    switch (HFI->Start_Status){
    case IH_INIT:
        if((HFI->Ih_Err > -HFIIhErr_Threshold) && (HFI->Ih_Err < HFIIhErr_Threshold)){
            HFI->status_cnt++;
        }
        else{
            HFI->status_cnt = 0;
        }
        
        if(HFI->status_cnt >= 500){
            //HFI->Start_Status = THETAE_INIT;
            HFI->status_cnt = 0;
        }

        Ih_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case THETAE_INIT:
        if((HFI->ThetaE_Err > -HFIThetaEErr_Threshold) && (HFI->ThetaE_Err < HFIThetaEErr_Threshold)){
            HFI->status_cnt++;
        }
        else{
            HFI->status_cnt = 0;
        }
        
        if(HFI->status_cnt >= 500){
            //HFI->Start_Status = POLE_INIT;
            HFI->status_cnt = 0;
        }

        ThetaE_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case POLE_INIT:
        if(HFI->Pulse_cnt == 100){
            if(HFI->Ipulse_Max > -HFI->Ipulse_Min){
                HFI->Start_Status = POLE_N_WAIT;
                HFI->status_cnt = 0;
            }
            else if(HFI->Ipulse_Max < -HFI->Ipulse_Min){
                HFI->Start_Status = POLE_S_WAIT;
                HFI->status_cnt = 0;
            }
        }
        
        Pole_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case POLE_N_WAIT:
        if(HFI->Pulse_cnt == 100){
            if(HFI->Ipulse_Max > -HFI->Ipulse_Min){
                HFI->status_cnt++;
            }
            else{
                HFI->Start_Status = POLE_INIT;
                HFI->status_cnt = 0;
            }
            
            if(HFI->status_cnt >= 2){
                HFI->Start_Status = HFI_WAIT;
                HFI->status_cnt = 0;
            }
        }

        Pole_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case POLE_S_WAIT:
        if(HFI->Pulse_cnt == 100){
            if(HFI->Ipulse_Max < -HFI->Ipulse_Min){
                HFI->status_cnt++;
            }
            else{
                HFI->Start_Status = POLE_INIT;
                HFI->status_cnt = 0;
            }
            
            if(HFI->status_cnt >= 2){
                HFI->Start_Status = HFI_WAIT;
                HFI->ThetaE = fmodf(HFI->ThetaE + PI, 2 * PI);
                HFI->status_cnt = 0;
            }
        }

        Pole_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case HFI_WAIT:
        if(HFI->status_cnt >= 500){
            CtrlCom->Status_Flag = RUN;
            HFI->status_cnt = 0;
        }
        else{
            HFI->status_cnt++;
        }

        break;
    }
}

void HFI_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    
}

void FOC_Mode_Select(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, SensorData_str* SensorData, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    switch (CtrlCom->Status_Flag){
    case START:
        CtrlCom->Mode_last = CtrlCom->Mode;

        switch(CtrlCom->Mode){
        case VolLoop:
            VolLoop_Start(CtrlCom);
            break;
        case CurLoop:
            CurLoop_Start(CtrlCom);
            break;
        case SpdLoop:
            SpdLoop_Start(CtrlCom);
            break;
        case HFI_Sensorless:
            HFI_Start(CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        }
        break;
    case RUN:
        switch(CtrlCom->Mode){
        case VolLoop:
            VolLoop_Mode(CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
        case CurLoop:
            CurLoop_Mode(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
        case SpdLoop:
            SpdLoop_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
        case HFI_Sensorless:
            HFI_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        }
        break;
    case STOP:
        switch(CtrlCom->Mode_last){
        case VolLoop:
            VolLoop_Stop(CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
        case CurLoop:
            CurLoop_Stop(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
        case SpdLoop:
            SpdLoop_Stop(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SensorData);
            break;
        case HFI_Sensorless:
            
            break;
        }
        break;
    }
}
