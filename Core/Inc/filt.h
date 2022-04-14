#ifndef __FILT_H__
#define __FILT_H__

#include "stdint.h"

typedef struct
{
	int16_t dat[100];
	uint8_t length;
	uint8_t p;
}FIFO_typedef;

void FIFO_DataUpdate(FIFO_typedef *FIFO, int16_t New_dat);
int16_t FIFO_Get_Ave(FIFO_typedef *FIFO);
int16_t FIFO_Get_Dif(FIFO_typedef *FIFO, uint8_t dif);

#endif
