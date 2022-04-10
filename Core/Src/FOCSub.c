#include "FOCSub.h"

float GetThetaE(float Theta, uint8_t Np){
    return Theta * Np;
}

void Cordic(float ThetaE, float* SinTheta, float* CosTheta){
    *SinTheta = sinf(ThetaE);
    *CosTheta = cosf(ThetaE);
}

void InvPark(float Ud, float Uq, float SinTheta, float CosTheta, float* Ux, float* Uy){
    *Ux = CosTheta * Ud - SinTheta * Uq;
    *Uy = SinTheta * Ud + CosTheta * Uq;
}

void InvClarke(float Ux, float Uy, float* U1, float* U2, float* U3){
    *U1 =  Uy;
    *U2 =  Ux * 0.866f - Uy * 0.5f;
    *U3 = -Ux * 0.866f - Uy * 0.5f;
}

void GetSector(float U1, float U2, float U3, uint8_t* Sector){
    switch (((U3 > 0) << 2) | ((U2 > 0) << 1) | ((U1 > 0) << 0)){
    case 3: *Sector = 1; break;
    case 1: *Sector = 2; break;
    case 5: *Sector = 3; break;
    case 4: *Sector = 4; break;
    case 6: *Sector = 5; break;
    case 2: *Sector = 6; break;}
}

void GetCCR(MotorRealTimeInformation_str* MRT_Inf){
    MRT_Inf->Udc_K = MRT_Inf->Udc / 1.732f;
    
    switch(MRT_Inf->Sector){
    case 1: MRT_Inf->Tx =  MRT_Inf->U2 / MRT_Inf->Udc_K; MRT_Inf->Ty =  MRT_Inf->U1 / MRT_Inf->Udc_K; break;
    case 2: MRT_Inf->Tx = -MRT_Inf->U2 / MRT_Inf->Udc_K; MRT_Inf->Ty = -MRT_Inf->U3 / MRT_Inf->Udc_K; break;
    case 3: MRT_Inf->Tx =  MRT_Inf->U1 / MRT_Inf->Udc_K; MRT_Inf->Ty =  MRT_Inf->U3 / MRT_Inf->Udc_K; break;
    case 4: MRT_Inf->Tx = -MRT_Inf->U1 / MRT_Inf->Udc_K; MRT_Inf->Ty = -MRT_Inf->U2 / MRT_Inf->Udc_K; break;
    case 5: MRT_Inf->Tx =  MRT_Inf->U3 / MRT_Inf->Udc_K; MRT_Inf->Ty =  MRT_Inf->U2 / MRT_Inf->Udc_K; break;
    case 6: MRT_Inf->Tx = -MRT_Inf->U3 / MRT_Inf->Udc_K; MRT_Inf->Ty = -MRT_Inf->U1 / MRT_Inf->Udc_K; break;}

    MRT_Inf->Ta = (1.0f - MRT_Inf->Tx - MRT_Inf->Ty) / 2;
    MRT_Inf->Tb = (1.0f + MRT_Inf->Tx - MRT_Inf->Ty) / 2;
    MRT_Inf->Tc = (1.0f + MRT_Inf->Tx + MRT_Inf->Ty) / 2;

    switch(MRT_Inf->Sector){
    case 1: MRT_Inf->CCRa = MRT_Inf->Ta;   MRT_Inf->CCRb = MRT_Inf->Tb;   MRT_Inf->CCRc = MRT_Inf->Tc;   break;
    case 2: MRT_Inf->CCRa = MRT_Inf->Tb;   MRT_Inf->CCRb = MRT_Inf->Ta;   MRT_Inf->CCRc = MRT_Inf->Tc;   break;
    case 3: MRT_Inf->CCRa = MRT_Inf->Tc;   MRT_Inf->CCRb = MRT_Inf->Ta;   MRT_Inf->CCRc = MRT_Inf->Tb;   break;
    case 4: MRT_Inf->CCRa = MRT_Inf->Tc;   MRT_Inf->CCRb = MRT_Inf->Tb;   MRT_Inf->CCRc = MRT_Inf->Ta;   break;
    case 5: MRT_Inf->CCRa = MRT_Inf->Tb;   MRT_Inf->CCRb = MRT_Inf->Tc;   MRT_Inf->CCRc = MRT_Inf->Ta;   break;
    case 6: MRT_Inf->CCRa = MRT_Inf->Ta;   MRT_Inf->CCRb = MRT_Inf->Tc;   MRT_Inf->CCRc = MRT_Inf->Tb;   break;
    }
}

void Clarke(float Ia, float Ic, float* Ix, float* Iy){
    *Ix = Ia;
    *Iy = -((Ia + Ic * 2.0f)*0.57735f);
}

void Park(float Ix, float Iy, float SinTheta, float CosTheta, float* Id, float* Iq){
    *Id = SinTheta * Iy + CosTheta * Ix;
    *Iq = CosTheta * Iy - SinTheta * Ix;
}

int32_t delta_Theta;

void GetSpd(uint32_t Theta, uint32_t* Theta_Pre, float* Speed, float SpdFs){
    delta_Theta = (Theta - *Theta_Pre) << 15;
    delta_Theta = delta_Theta >> 15;
    
    *Speed = (2 * PI) * delta_Theta / (1 << 17) * SpdFs;
    *Theta_Pre = Theta;
}
