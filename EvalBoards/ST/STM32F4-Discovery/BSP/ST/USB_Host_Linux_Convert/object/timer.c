//#define USBH_DEBUG_LEVEL USBH_DEBUG_TRACE
#include "os.h"
#include "usbh_config.h"
#include "string.h"
#include "timer.h"
#include "misc_cvt.h"


unsigned int TimerTotalCtr = 0;

int timer_pending(struct timer_list * timer);

static void timer_call_back(void *p_tmr, void *p_arg)
{
    struct timer_list *timer = p_arg;

    USBH_TRACE("timer_call_back timer:%p\r\n",timer);
    
    if(timer != p_tmr)
        USBH_DBG("timer_call_back serious error occur!!! timer != p_tmr %p\r\n",p_tmr);

    timer->function(timer->data);
}


int mod_timer(struct timer_list *timer, unsigned long expires)
{
    OS_ERR err;
    OS_STATE state;
    int ret = 1;
    
    

    //actived tmr
	if (timer->expires == expires && timer_pending(timer))
		return 1; 
    
    expires = expires - jiffies;
//    USBH_TRACE("mod_timer expires:%ld timer:%p\r\n",expires,timer);   
    
    state = OSTmrStateGet(&timer->tmr, &err);

    if(err == OS_ERR_TMR_INVALID_STATE)
    {
        USBH_DBG("mod_timer OS_ERR_TMR_INVALID_STATE\r\n");
    }


    expires = (expires * OSCfg_TmrTaskRate_Hz + (OSCfg_TickRate_Hz - 1))/ OSCfg_TickRate_Hz;

    
    USBH_TRACE("mod_timer timer:%p state:%d Period:%d expires:%ld\r\n",timer,state,timer->tmr.Period,expires);
    

    //first should create tmr
    if(state == OS_TMR_STATE_UNUSED)
    {
        //expires is 1 at least
        USBH_TRACE("OSTmrCreate period =%ld\r\n",expires);
        OSTmrCreate(&timer->tmr, "",expires, expires,OS_OPT_TMR_ONE_SHOT,timer_call_back, timer,&err);
        ret = 0;
    }

    OSTmrStop(&timer->tmr,OS_OPT_TMR_NONE, NULL,&err);

    OSSchedLock(&err);
    timer->tmr.Dly = expires;
    timer->tmr.Period = expires;
    
    OSTmrStart(&timer->tmr, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("mod_timer OSTmrStart Failed %d\r\n",err);
    }
    OSSchedUnlock(&err);
    
    return ret;
}




void init_timer(struct timer_list *timer)
{
    CPU_SR cpu_sr;
    
    CPU_CRITICAL_ENTER();
    TimerTotalCtr++;
    CPU_CRITICAL_EXIT();
    memset((void*)timer, 0, sizeof(struct timer_list));

}


void add_timer(struct timer_list *timer)
{
	BUG_ON(timer_pending(timer));
	mod_timer(timer, timer->expires);
}



int timer_pending(struct timer_list * timer)
{
    OS_ERR err;
    OS_STATE state;
    
    state = OSTmrStateGet(&timer->tmr, &err);
     if(err == OS_ERR_TMR_INVALID_STATE)
        USBH_DBG("timer_pending OS_ERR_TMR_INVALID_STATE\r\n");
     
	return (state == OS_TMR_STATE_RUNNING);
}

int del_timer(struct timer_list *timer)
{
    OS_ERR err;    
	int ret = 0;
    CPU_SR cpu_sr;
    



	if (timer_pending(timer)) {
			ret = 1;
	}

    CPU_CRITICAL_ENTER();
    TimerTotalCtr--;
    CPU_CRITICAL_EXIT();

    OSTmrDel(&timer->tmr, &err);
     if(err != OS_ERR_NONE)
        USBH_DBG("timer_pending OSTmrDel Failed:%d\r\n",err);  

    return ret;
}

/**
 * msleep - sleep safely even with waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
void msleep(unsigned int msecs)
{
    OS_ERR err;

    
    OSTimeDlyHMSM(0,0,0,msecs,OS_OPT_TIME_TIMEOUT | OS_OPT_TIME_HMSM_NON_STRICT,&err);
    if(err != OS_ERR_NONE)
        USBH_DBG("msleep error:%d\r\n",err);
}


// void sle1ep()
// {
//     
//     
// }

