#include "Sensorless.h"
#include "FOCSub.h"
#include "DataProcessing.h"

void Ih_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_Ih = 0;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;
    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);
    HFI->Ih_Err = HFI_Ih - HFI->Ih;
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    HFI->Ud = HFI->Vdh;
    HFI->Uq = 0;
    
    Cordic(0, &HFI->SinTheta, &HFI->CosTheta);
    arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta, HFI->CosTheta);
    //arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, 0, 1);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    
    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
}

void ThetaE_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_SpdE = 0;
    float HFI_SpdE_Rec = 0;
    float HFI_Ih = 0;
    float HFI_ThetaE_temp = 0;
    float HFI_ThetaE_Rec_temp = 0;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;

    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);

    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);

    HFI->SpdE_PI.Error = (HFI->Ixh * HFI->SinTheta - HFI->Iyh * HFI->CosTheta) / HFI->Ih;
    LPF(&(HFI->ThetaE_Err), HFI->SpdE_PI.Error, CtrlCom->CurFs, HFI->ThetaE_Err_LPF_wc);
    
    HFI_SpdE = PI_Control(&HFI->SpdE_PI);
    HFI_ThetaE_temp = HFI->ThetaE + HFI_SpdE * CtrlCom->CurTs;
    if(HFI_ThetaE_temp < 0)
        HFI_ThetaE_temp += 2 * PI;
    HFI->ThetaE = fmodf(HFI_ThetaE_temp, 2 * PI);
    
    Cordic(fmodf(HFI->Rec.b1*HFI_ThetaE_temp + HFI->Rec.c1, 2*PI), &HFI->SinThetaE_Rec, &HFI->CosThetaE_Rec);
    HFI_ThetaE_Rec_temp = HFI_ThetaE_temp + HFI->Rec.a1*HFI->SinThetaE_Rec + HFI->Rec.d1 + HFI->Rec.d2 * MRT_Inf->Iq_Ave / 2.7f;
    if(HFI_ThetaE_Rec_temp < 0)
        HFI_ThetaE_Rec_temp += 2 * PI;
    HFI->ThetaE_Rec = fmodf(HFI_ThetaE_Rec_temp, 2 * PI);
    
    HFI_SpdE_Rec = HFI->ThetaE_Rec - HFI->ThetaE_Rec_temp;
    if(HFI_SpdE_Rec > PI)
        HFI_SpdE_Rec -= 2*PI;
    else if(HFI_SpdE_Rec < -PI)
        HFI_SpdE_Rec += 2*PI;
    //HFI_SpdE_Rec += fmodf(HFI_SpdE_Rec, PI);
    HFI_SpdE_Rec = HFI_SpdE_Rec / CtrlCom->CurTs;
    LPF(&(HFI->SpdE_Rec), HFI_SpdE_Rec, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    HFI->Ud = HFI->Vdh;
    HFI->Uq = 0;
    
    arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    
    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
    HFI->ThetaE_Rec_temp = HFI->ThetaE_Rec;
}

void Pole_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, HFI->SinTheta, HFI->CosTheta);
    
    if(HFI->Pulse_cnt < 1){
        HFI->Pulse_cnt++;
        HFI->Ud = HFI->Ud;
        HFI->Uq = 0;
        
        if(HFI->Ipulse_Max < MRT_Inf->Id)
            HFI->Ipulse_Max = MRT_Inf->Id;
        
        if(HFI->Ipulse_Min > MRT_Inf->Id)
            HFI->Ipulse_Min = MRT_Inf->Id;
    }
    else if(HFI->Pulse_cnt < 124){
        HFI->Pulse_cnt++;
        HFI->Ud = 0;
        HFI->Uq = 0;
        
        if(HFI->Ipulse_Max < MRT_Inf->Id)
            HFI->Ipulse_Max = MRT_Inf->Id;
        
        if(HFI->Ipulse_Min > MRT_Inf->Id)
            HFI->Ipulse_Min = MRT_Inf->Id;
    }
    else{
        HFI->Pulse_cnt = 0;
        HFI->Dir = (HFI->Dir == 1)?(-1):(1);
        HFI->Ud = 12 * HFI->Dir;
        HFI->Uq = 0;
        if(HFI->Dir == 1){
            HFI->Ipulse_Max = 0;
        }
        else{
            HFI->Ipulse_Min = 0;
        }
    }
    
    arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
}

void HFI_Work(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_SpdE = 0;
    float HFI_SpdE_Rec = 0;
    float HFI_Ih = 0;
    float HFI_ThetaE_temp = 0;
    float HFI_ThetaE_Rec_temp = 0;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    MRT_Inf->Ix = (HFI->Ix + HFI->Ix_temp) / 2;
    MRT_Inf->Iy = (HFI->Iy + HFI->Iy_temp) / 2;
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;
    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);

    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);
    HFI->SpdE_PI.Error = (HFI->Ixh * HFI->SinTheta - HFI->Iyh * HFI->CosTheta) / HFI->Ih;

    HFI_SpdE = PI_Control(&HFI->SpdE_PI);  
    HFI_ThetaE_temp = HFI->ThetaE + HFI_SpdE * CtrlCom->CurTs;
    if(HFI_ThetaE_temp < 0)
        HFI_ThetaE_temp += 2 * PI;
    HFI->ThetaE = fmodf(HFI_ThetaE_temp, 2 * PI);
    
    Cordic(fmodf(HFI->Rec.b1*HFI_ThetaE_temp + HFI->Rec.c1, 2*PI), &HFI->SinThetaE_Rec, &HFI->CosThetaE_Rec);
    HFI_ThetaE_Rec_temp = HFI_ThetaE_temp + HFI->Rec.a1*HFI->SinThetaE_Rec + HFI->Rec.d1 + HFI->Rec.d2 * MRT_Inf->Iq_Ave / 2.7f;
    if(HFI_ThetaE_Rec_temp < 0)
        HFI_ThetaE_Rec_temp += 2 * PI;
    HFI->ThetaE_Rec = fmodf(HFI_ThetaE_Rec_temp, 2 * PI);
    
    Cordic(HFI->ThetaE_Rec, &HFI->SinTheta_Rec, &HFI->CosTheta_Rec);
    
    HFI_SpdE_Rec = HFI->ThetaE_Rec - HFI->ThetaE_Rec_temp;
    if(HFI_SpdE_Rec > PI)
        HFI_SpdE_Rec -= 2*PI;
    else if(HFI_SpdE_Rec < -PI)
        HFI_SpdE_Rec += 2*PI;
    //HFI_SpdE_Rec = fmodf(HFI_SpdE_Rec, PI);
    HFI_SpdE_Rec = HFI_SpdE_Rec / CtrlCom->CurTs;
    LPF(&(HFI->SpdE_Rec), HFI_SpdE_Rec, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
    LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    HFI->Spd_Rec = HFI->SpdE_Rec / MotorParameter->Np;
    
    CtrlCom->Id = 0;
    Spd_PI->Error = CtrlCom->Spd - HFI->Spd_Rec;
    CtrlCom->Iq = PI_Control(Spd_PI);
    
    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
    arm_inv_park_f32(HFI->Vdh, 0, &HFI->Ux, &HFI->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux + HFI->Ux, MRT_Inf->Uy + HFI->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    
    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
    
    HFI->ThetaE_Rec_temp = HFI->ThetaE_Rec;
}
