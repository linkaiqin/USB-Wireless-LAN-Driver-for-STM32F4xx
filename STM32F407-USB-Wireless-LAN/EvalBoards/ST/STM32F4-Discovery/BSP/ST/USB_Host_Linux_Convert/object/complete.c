#include "os.h"
#include "usbh_config.h"
#include "complete.h"
#include "kthread.h"





void init_completion(struct completion *x)
{
    OS_ERR err;
    CPU_SR cpu_sr;
    
    CPU_CRITICAL_ENTER();
    x->p_tcb = OSTCBCurPtr;                   /* Set task prio as ICMP Req Msg id (see Note #3b).     */
    CPU_CRITICAL_EXIT();
    
    OSTaskSemSet(x->p_tcb,0,&err);

    if(err != OS_ERR_NONE)
        USBH_DBG("init_completion OSTaskSemSet 0 Failed %d\r\n",err);
    
//	x->done = 0;
//	init_waitqueue_head(&x->wait);
}




void complete(struct completion *x)
{
	OS_ERR err;

    OSTaskSemPost(x->p_tcb,OS_OPT_POST_NONE,&err);
    
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("complete OSTaskSemPost Faild %d\r\n",err);
    }
}

void complete_and_exit(struct completion *comp, long code)
{
    OS_ERR err;
    OSSchedLock(&err);
	if (comp)
		complete(comp);

	kthread_exit(code);
    OSSchedUnlock(&err);
}

unsigned long wait_for_completion_timeout(struct completion *x, long timeout)
{
    OS_ERR err;
    OS_TICK expire;


    expire = OSTimeGet(&err) + timeout;
    
    OSTaskSemPend(timeout,OS_OPT_PEND_BLOCKING, 0, &err);
    if(err != OS_ERR_NONE)
    {
        if(err == OS_ERR_TIMEOUT)
            return 0;
        else
            return -err;
    }

    
    timeout = expire - OSTimeGet(&err);
     
    return timeout < 0 ? 0 : timeout;
}


void  wait_for_completion(struct completion *x)
{
    OS_ERR err;
    
    OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, 0, &err);
    
    if(err != OS_ERR_NONE)
    {   
        USBH_DBG("wait_for_completion OSTaskSemPend %d\r\n",err);
    } 

    return;
}





