#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stdint.h"

int GetCurTime(void);
void Time_MeasureStart(void);
int Time_MeasureEnd(void);
void delay_ms(volatile uint32_t nTime);
void delay_us(volatile uint32_t nTime);

#endif





























