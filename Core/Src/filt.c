#include "filt.h"

void FIFO_DataUpdate(FIFO_typedef *FIFO, int16_t New_dat)
{
	FIFO->dat[FIFO->p] = New_dat;
	FIFO->p++;
	if(FIFO->p >= FIFO->length)
		FIFO->p -= FIFO->length;
}

int16_t FIFO_Get_Ave(FIFO_typedef *FIFO)
{
	int32_t sum = 0;
	uint8_t i;
	
	for(i = 0; i < FIFO->length; ++i) 
	{
		sum += FIFO->dat[i];
	}
	
	return sum/FIFO->length;
}

int16_t FIFO_Get_Dif(FIFO_typedef *FIFO, uint8_t dif)
{
	int8_t p,p_last;

	(FIFO->p == 0) ? (p = FIFO->length - 1) : (p = FIFO->p - 1);

	p_last = p - dif;
	if (p_last < 0) p_last += FIFO->length;

	return FIFO->dat[p] - FIFO->dat[p_last];
}
