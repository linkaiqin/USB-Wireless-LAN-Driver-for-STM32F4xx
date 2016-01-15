#include "os.h"
#include "usbh_debug.h"
#include "usb_conf.h"
#include "irq.h"
#include "tasklet.h"
#include "bitops.h"




#define TASKLET_STATE_UNSCHED     0
#define TASKLET_STATE_SCHED       1


OS_TCB TaskletActionTCB;
CPU_STK TaskletAction_TaskStk[TASKLET_ACTION_TASK_STK_SIZE];

void tasklet_action(void *p_arg);

void tasklet_task_init()
{
    OS_ERR err;
    

    OSTaskCreate((OS_TCB     *)&TaskletActionTCB,                /* Create the start task                                    */
                 (CPU_CHAR   *)"Tasklet Action",
                 (OS_TASK_PTR ) tasklet_action,
                 (void       *) 0,
                 (OS_PRIO     ) TASKLET_ACTION_TASK_PRIO,
                 (CPU_STK    *)&TaskletAction_TaskStk[0],
                 (CPU_STK     )(TASKLET_ACTION_TASK_STK_SIZE / 10u),
                 (CPU_STK_SIZE) TASKLET_ACTION_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 10,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    
}



void tasklet_schedule(struct tasklet_struct *t)
{
    OS_ERR err;
	CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();   
    if(t->state == TASKLET_STATE_SCHED)
    {
        CPU_CRITICAL_EXIT();
        return;
    }
    t->state = TASKLET_STATE_SCHED;
    CPU_CRITICAL_EXIT();
    
    OSTaskQPost(&TaskletActionTCB ,t,sizeof(struct tasklet_struct), OS_OPT_POST_FIFO,&err);
    if(err != OS_ERR_NONE)
        USBH_DBG("tasklet_schedule OSTaskQPost Failed %d\r\n",err);
    
}




void tasklet_init(struct tasklet_struct *t,
		  void (*func)(unsigned long), unsigned long data)
{
// 	t->next = NULL;
	t->state = TASKLET_STATE_UNSCHED;
	atomic_set(&t->count, 0);
	t->func = func;
	t->data = data;
}


void tasklet_action(void *p_arg)
{
    OS_ERR err;
    OS_MSG_SIZE msg_size;
    struct tasklet_struct *t;
    CPU_SR cpu_sr;


    while(1)
    {
        t = OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &msg_size, 0,&err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("tasklet_schedule OSTaskQPend Failed %d\r\n",err);
            continue;
        }
        if(!t){
            USBH_DBG("tasklet_schedule t == NULL\r\n");
            continue;
        }

        
//        OSSchedLock(&err);
        CPU_CRITICAL_ENTER();
        if(t->state == TASKLET_STATE_UNSCHED)
            USBH_DBG("tasklet_action Error at %d\r\n",__LINE__);
        t->state = TASKLET_STATE_UNSCHED;
        CPU_CRITICAL_EXIT();      
        
        t->func(t->data);
//        OSSchedUnlock(&err);
    }

}

void flush_scheduled_work(void)
{
    USBH_DBG("flush_scheduled_work\r\n");
//	flush_workqueue(keventd_wq);
}


// static int inline tasklet_state_test_and_set(unsigned long *addr,unsigned long val)
// {
//     unsigned long temp_val;
//     temp_val = *addr;    
//     *addr = val;  
//     return (temp_val == val); 
// }

//in t->func(t->data), don't restart tasklet_schedule,oterwise tasklet_kill will not exit.
void tasklet_kill(struct tasklet_struct *t)
{
    OS_ERR err;
    CPU_SR cpu_sr;
    
	if (in_interrupt())
		USBH_DBG("tasklet_kill Attempt to kill tasklet from interrupt\r\n");

    CPU_CRITICAL_ENTER();   
	while (t->state == TASKLET_STATE_SCHED){
        t->state = TASKLET_STATE_SCHED; 
		do {
            CPU_CRITICAL_EXIT();
			OSTimeDlyHMSM(0,0,0,5, OS_OPT_TIME_DLY,&err);
            CPU_CRITICAL_ENTER();  
		} while (t->state == TASKLET_STATE_SCHED);
	}
    CPU_CRITICAL_EXIT();
    
    
//	while (test_and_set_bit(TASKLET_STATE_SCHED, &t->state)) {
//		do {
//			yield();
//		} while (test_bit(TASKLET_STATE_SCHED, &t->state));
//	}
//	tasklet_unlock_wait(t);
//	clear_bit(TASKLET_STATE_SCHED, &t->state);
}



