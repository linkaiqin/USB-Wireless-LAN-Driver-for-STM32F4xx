#define USBH_DEBUG_LEVEL USBH_DEBUG_ERROR
#include "os.h"
#include "usbh_debug.h"
#include "usb_conf.h"
#include "kthread.h"
#include "memory.h"
#include "list.h"


#define KERNEL_THREAD_USE_POOL
#define KERNEL_THREAD_NUM_MAX           3



#ifdef KERNEL_THREAD_USE_POOL
static CPU_STK KthreadStkPool[KERNEL_THREAD_NUM_MAX][KERNEL_THREAD_STK_SIZE];
static struct task_struct KthreadTaskPool[KERNEL_THREAD_NUM_MAX];
static OS_MEM KthreadStkMem,KthreadTaskMem;
#endif

unsigned int KthreadTotalCtr = 0;

LIST_HEAD(kernel_task_list_head);


#if OS_CFG_SEM_PEND_ABORT_EN > 0u
OS_OBJ_QTY  KthreadSemPendAbort (OS_SEM  *p_sem,
                            OS_TCB        *p_tcb,
                            OS_ERR  *p_err)
{
    OS_PEND_LIST  *p_pend_list;
    
    CPU_TS         ts;
    OS_OBJ_QTY     nbr_tasks;
    CPU_SR_ALLOC();



//#ifdef OS_SAFETY_CRITICAL
//    if (p_err == (OS_ERR *)0) {
//        OS_SAFETY_CRITICAL_EXCEPTION();
//        return ((OS_OBJ_QTY)0u);
//    }
//#endif

//#if OS_CFG_CALLED_FROM_ISR_CHK_EN > 0u
//    if (OSIntNestingCtr > (OS_NESTING_CTR)0u) {             /* Not allowed to Pend Abort from an ISR                  */
//       *p_err =  OS_ERR_PEND_ABORT_ISR;
//        return ((OS_OBJ_QTY)0u);
//    }
//#endif

//#if OS_CFG_ARG_CHK_EN > 0u
//    if (p_sem == (OS_SEM *)0) {                             /* Validate 'p_sem'                                       */
//       *p_err =  OS_ERR_OBJ_PTR_NULL;
//        return ((OS_OBJ_QTY)0u);
//    }
//    switch (opt) {                                          /* Validate 'opt'                                         */
//        case OS_OPT_PEND_ABORT_1:
//        case OS_OPT_PEND_ABORT_ALL:
//        case OS_OPT_PEND_ABORT_1   | OS_OPT_POST_NO_SCHED:
//        case OS_OPT_PEND_ABORT_ALL | OS_OPT_POST_NO_SCHED:
//             break;

//        default:
//            *p_err =  OS_ERR_OPT_INVALID;
//             return ((OS_OBJ_QTY)0u);
//    }
//#endif

//#if OS_CFG_OBJ_TYPE_CHK_EN > 0u
//    if (p_sem->Type != OS_OBJ_TYPE_SEM) {                   /* Make sure semaphore was created                        */
//       *p_err =  OS_ERR_OBJ_TYPE;
//        return ((OS_OBJ_QTY)0u);
//    }
//#endif

    CPU_CRITICAL_ENTER();
    p_pend_list = &p_sem->PendList;
    if (p_pend_list->NbrEntries == (OS_OBJ_QTY)0u) {        /* Any task waiting on semaphore?                         */
        CPU_CRITICAL_EXIT();                                /* No                                                     */
       *p_err =  OS_ERR_PEND_ABORT_NONE;
        return ((OS_OBJ_QTY)0u);
    }

    OS_CRITICAL_ENTER_CPU_EXIT();
    nbr_tasks = 0u;
    ts        = OS_TS_GET();                                /* Get local time stamp so all tasks get the same time    */
//    while (p_pend_list->NbrEntries > (OS_OBJ_QTY)0u) {
//        p_tcb = p_pend_list->HeadPtr->TCBPtr;
        OS_PendAbort((OS_PEND_OBJ *)((void *)p_sem),
                     p_tcb,
                     ts);
        nbr_tasks++;
//        if (opt != OS_OPT_PEND_ABORT_ALL) {                 /* Pend abort all tasks waiting?                          */
//            break;                                          /* No                                                     */
//        }
//    }
    OS_CRITICAL_EXIT_NO_SCHED();

//    if ((opt & OS_OPT_POST_NO_SCHED) == (OS_OPT)0u) {
//        OSSched();                                          /* Run the scheduler                                      */
//    }

   *p_err = OS_ERR_NONE;
    return (nbr_tasks);
}
#endif



int kill_pid(struct pid *pid, int sig, int priv)
{
    struct task_struct *task; 
    OS_ERR err;
    OS_STATE task_state;
    CPU_SR cpu_sr;

    task  = container_of(pid,struct task_struct,pid);   
    CPU_CRITICAL_ENTER();
    task->is_killed = 1;
    task_state = task->tcb.TaskState;
    if((task_state == OS_TASK_STATE_PEND) || (task_state == OS_TASK_STATE_PEND_TIMEOUT))
    {
        if((task->tcb.PendOn == OS_TASK_PEND_ON_SEM)&&(task->tcb.PendDataTblEntries == 1)&&(task->tcb.PendDataTblPtr->PendObjPtr->Type ==  OS_OBJ_TYPE_SEM))
        {
            USBH_TRACE("KthreadSemPendAbort sem:%p\r\n",task->tcb.PendDataTblPtr);
            KthreadSemPendAbort((OS_SEM *)task->tcb.PendDataTblPtr->PendObjPtr,&task->tcb,&err);
        }
        else
        {
            USBH_DBG("Error! TaskState:%d PendOn:%d!=OS_TASK_PEND_ON_SEM PendDataTblEntries:%d PendObjPtr->Type:%d\r\n",
                                                            task->tcb.TaskState,task->tcb.PendOn,task->tcb.PendDataTblEntries,task->tcb.PendDataTblPtr->PendObjPtr->Type);
        }
    }    
    CPU_CRITICAL_EXIT();
    
    USBH_TRACE("kill_pid:%p task:%s TaskState:%d\r\n",pid,task->name,task_state);

    OSSched();
    

    return 0;
}

pid_t pid_nr(struct pid *pid)
{
	pid_t nr = 0;
	if (pid)
		nr = pid->pid_nr;
	return nr;
}



void kthread_exit(long code)
{
    struct task_struct *task;
    OS_TCB *tcb;
    CPU_SR cpu_sr;



    CPU_CRITICAL_ENTER();
    KthreadTotalCtr--;
    tcb = OSTCBCurPtr;
    CPU_CRITICAL_EXIT();
    
    task  = container_of(tcb,struct task_struct,tcb);
    USBH_TRACE("thread_exit %s\r\n",task->name); 
    OSTaskDel(0,&task->exit_err);
    if(task->exit_err)
        USBH_DBG("task:%s exit failed:%d\r\n", task->name, task->exit_err);
    kthread_del(task);
}


void kthread_del(struct task_struct *task)
{   
#ifdef KERNEL_THREAD_USE_POOL
    OS_ERR err;    
    list_del(&task->task_list);  
    OSMemPut(&KthreadStkMem,task->stk,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("OSMemPut KthreadStkMem Failed!!! %d task->stk:%p\r\n",err,task->stk);        
    }
    OSMemPut(&KthreadTaskMem,task,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("OSMemPut KthreadTaskMem Failed!!! %d task:%p\r\n",err,task);    
    }
#else
    list_del(&task->task_list);  
    usbh_mem_free(task->stk);
    usbh_mem_free(task);
#endif
}



pid_t __kernel_thread(char *name, int (*fn)(void *), void *arg, unsigned long flags)
{
    OS_ERR err;
    struct task_struct *task;
    CPU_STK *stk;
    OS_PRIO prio;
    CPU_SR cpu_sr;
#ifdef KERNEL_THREAD_USE_POOL
    static char is_first = 1;
#endif

    CPU_CRITICAL_ENTER();
    prio =  KERNEL_THREAD_PRIO_BEGIN;// + KthreadTotalCtr;
    CPU_CRITICAL_EXIT();
    
    USBH_TRACE("kernel_thread create %s prio:%d stk_size:%d\r\n",name,prio,KERNEL_THREAD_STK_SIZE);

#ifdef KERNEL_THREAD_USE_POOL
    if(is_first)
    {
        is_first = 0;
        OSMemCreate(&KthreadTaskMem, "KthreadTaskMem", &KthreadTaskPool[0], KERNEL_THREAD_NUM_MAX, sizeof(struct task_struct), &err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("OSMemCreate KthreadTaskMem Failed!!! %d\r\n",err);
            return -1;
        }
        OSMemCreate(&KthreadStkMem, "KthreadStkMem", &KthreadStkPool[0][0], KERNEL_THREAD_NUM_MAX, KERNEL_THREAD_STK_SIZE*sizeof(CPU_STK), &err);
        if(err != OS_ERR_NONE)
        {       
            USBH_DBG("OSMemCreate KthreadStkMem Failed!!! %d\r\n",err);
            return -1;
        }        
    }
    task = OSMemGet(&KthreadTaskMem, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("OSMemGet KthreadTaskMem Failed!!! %d\r\n",err);
        return -1;
    }
    stk = OSMemGet(&KthreadStkMem, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("OSMemGet KthreadStkMem Failed!!! %d\r\n",err);
        return -1;
    }
#else
    task = usbh_mem_calloc(1,sizeof(struct task_struct));
    if(task == NULL)
    {
        USBH_DBG("usbh_mem_malloc sizeof(struct task_struct) Failed!!!\r\n");
        return -1;
    }        
    stk = usbh_mem_calloc(1,KERNEL_THREAD_STK_SIZE*sizeof(CPU_STK));
    if(stk == NULL)
    {
        usbh_mem_free(task);
        USBH_DBG("usbh_mem_malloc stk Failed!!! size:%d\r\n",KERNEL_THREAD_STK_SIZE*sizeof(CPU_STK));
        return -1;
    }
#endif
    
    //make pid_nr > 0
    task->pid.pid_nr = (int)(((unsigned int)&task->tcb) & (unsigned int)(~((unsigned int)1 << 31)));
    task->prio = prio;
    task->stk = stk;
    task->is_killed = 0;
    task->exit_err = OS_ERR_OS_RUNNING; //debug
    task->name = name;
    task->pids[PIDTYPE_PID].pid = &task->pid;
    
    INIT_LIST_HEAD(&task->task_list);
    USBH_TRACE("kernel_thread pid->pid_nr:%d stk:%p\r\n",task->pid.pid_nr,task->stk);


    OSSchedLock(&err);
    OSTaskCreate((OS_TCB     *)&task->tcb,                /* Create the start task                                    */
                 (CPU_CHAR   *)name,
                 (OS_TASK_PTR ) fn,
                 (void       *) arg,
                 (OS_PRIO     ) prio,
                 (CPU_STK    *) stk,
                 (CPU_STK     )(KERNEL_THREAD_STK_SIZE / 10u),
                 (CPU_STK_SIZE) KERNEL_THREAD_STK_SIZE,
                 (OS_MSG_QTY  ) 5,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);    

    
    if(err != OS_ERR_NONE){
        USBH_DBG("kernel_thread create %s Failed!!! prio:%d stk_size:%d\r\n",name,prio,KERNEL_THREAD_STK_SIZE);
        OSSchedUnlock(&err);
        return -1;
    }
    else{
        
        CPU_CRITICAL_ENTER();  
        KthreadTotalCtr++;
        list_add_tail(&task->task_list,&kernel_task_list_head);
        CPU_CRITICAL_EXIT();   
        OSSchedUnlock(&err);
        
        return task->pid.pid_nr;
    }
}













