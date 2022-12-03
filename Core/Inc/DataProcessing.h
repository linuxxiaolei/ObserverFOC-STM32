#ifndef __DATAPROCESSING_H__
#define __DATAPROCESSING_H__

#include "main.h"
#include "FOCSub.h"

extern void CtrlComFilter(float *Target, float CtrlCom, float TickAdd);
extern void LPF(float* Uo, float Ui, float Fs, float Wc);
extern float PI_Control(PI_str* pPI);

#endif
