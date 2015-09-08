#include "stm32f4xx.h"
#include "delay.h"

#define CPU_FREQUENCY_MHZ    168

static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/


// static u32 PreTime = 0;
// static u32 CurTime = 0;

//void Time_MeasureStart()
//{
//    PreTime = SysTick->VAL;
//}



//int Time_MeasureEnd()//us
//{
//    CurTime = SysTick->VAL;

//    if(CurTime > PreTime)
//    {
//        return CPU_FREQUENCY_MHZ*1000 - (CurTime - PreTime);
//    }
//    else
//        return PreTime - CurTime;
//}


void delay_us(__IO uint32_t nTime)
{
    int old_val,new_val,val;

    if(nTime > 900)
    {
        for(old_val = 0; old_val < nTime/900; old_val++)
        {
            delay_us(900);
        }
        nTime = nTime%900;
    }

    old_val = SysTick->VAL;
    new_val = old_val - CPU_FREQUENCY_MHZ*nTime;
    if(new_val >= 0)
    {
        do
        {
            val = SysTick->VAL;
        }
        while((val < old_val)&&(val >= new_val));
    }
    else
    {
        new_val +=CPU_FREQUENCY_MHZ*1000;
        do
        {
            val = SysTick->VAL;
        }
        while((val <= old_val)||(val > new_val));

    }
}



void delay_ms(__IO uint32_t nTime)
{
    delay_us(1000*nTime);
}



int GetCurTime(void)
{
    return TimingDelay;
}

void SysTick_Handler(void)
{
    TimingDelay++;
}





























