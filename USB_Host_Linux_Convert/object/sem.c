#define USBH_DEBUG_LEVEL USBH_DEBUG_TRACE
#include "os.h"
#include "usbh_debug.h"
#include "errno.h"
#include "sem.h"
#include "kthread.h"
#include "misc_cvt.h"



unsigned int SemaTotalCtr = 0;



void _sema_init(char *name, struct semaphore *sem, int val)
{
    OS_ERR err;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
    SemaTotalCtr++;
#if OS_CFG_DBG_EN > 0u   
    //The procedure _sema_init()-->OSSemCreate()-->OS_SemDbgListAdd() is dangerous,
    //because some linux driver don't call sema_destroy(), the semaphore instance will
    //still exsit in OSSemDbgListPtr after freed. Alway set OSSemDbgListPtr to zero.
    OSSemDbgListPtr = NULL;
#endif
    CPU_CRITICAL_EXIT();

    OSSemCreate(&sem->p_os_sem, name, val, &err);
    if(err != OS_ERR_NONE)
        USBH_DBG("sema_init OSSemCreate Failed %d\r\n",err);

}



int _down_interruptible(struct semaphore *sem)
{
    OS_ERR err;
    struct task_struct *task;
    CPU_SR cpu_sr;


    CPU_CRITICAL_ENTER();
    task  = container_of(OSTCBCurPtr,struct task_struct,tcb);
    if(task->is_killed)
    {
        CPU_CRITICAL_EXIT();
        USBH_TRACE("_down_interruptible %s is_killed\r\n",task->name);
        return -EINTR;
    }
    CPU_CRITICAL_EXIT();
         

    OSSemPend(&sem->p_os_sem,0,OS_OPT_PEND_BLOCKING,0, &err);
    if(err == OS_ERR_NONE)
        return 0;

    if(err == OS_ERR_PEND_ABORT)
    {
        USBH_TRACE("_down_interruptible OS_ERR_PEND_ABORT %s may be killed\r\n",task->name);
        return -EINTR;
    }
    else
    {
        USBH_DBG("down_interruptible OSSemPend Failed %d\r\n",err);
        return -EINTR;
    }
}


void _up(struct semaphore *sem)
{
    OS_ERR err;
    
    OSSemPost(&sem->p_os_sem, OS_OPT_POST_1, &err);

    if(err != OS_ERR_NONE){
        USBH_DBG("_up OSSemPost Failed %d\r\n",err);
    }
}




void _sema_destroy(struct semaphore *sem)
{

    OS_ERR err;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
    SemaTotalCtr--;
    CPU_CRITICAL_EXIT();

    OSSemDel(&sem->p_os_sem,OS_OPT_DEL_NO_PEND,&err);
    
    if(err != OS_ERR_NONE){
        USBH_DBG("_sema_destroy OSSemPend Failed %d\r\n",err);
    }

}


/**
 * down_trylock - try to acquire the semaphore, without waiting
 * @sem: the semaphore to be acquired
 *
 * Try to acquire the semaphore atomically.  Returns 0 if the mutex has
 * been acquired successfully or 1 if it it cannot be acquired.
 *
 * NOTE: This return value is inverted from both spin_trylock and
 * mutex_trylock!  Be careful about this when converting code.
 *
 * Unlike mutex_trylock, this function can be used from interrupt context,
 * and the semaphore can be released by any task or interrupt.
 */
int down_trylock(struct semaphore *sem)
{
// 	unsigned long flags;
// 	int count;
    OS_ERR err;

    OSSemPend(&sem->p_os_sem,0,OS_OPT_PEND_NON_BLOCKING,0,&err);

    if(err == OS_ERR_NONE)
        return 0;
    else if(err == OS_ERR_PEND_WOULD_BLOCK)
        return 1;
    else
    {
        USBH_DBG("down_trylock err:%d should not occur\r\n",err);
        return 1;
    }
}


/**
 * down - acquire the semaphore
 * @sem: the semaphore to be acquired
 *
 * Acquires the semaphore.  If no more tasks are allowed to acquire the
 * semaphore, calling this function will put the task to sleep until the
 * semaphore is released.
 *
 * Use of this function is deprecated, please use down_interruptible() or
 * down_killable() instead.
 */
void down(struct semaphore *sem)
{
    OS_ERR err;
    
    OSSemPend(&sem->p_os_sem,0,OS_OPT_PEND_BLOCKING,0, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("down OSSemPend Failed %d\r\n",err);
    }
}






