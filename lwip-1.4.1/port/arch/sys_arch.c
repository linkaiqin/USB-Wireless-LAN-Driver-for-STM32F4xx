
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"

#include <string.h>


OS_MEM LwipTCBMem;
OS_MEM LwipStkMem;
CPU_STK LwipStkPool[LWIP_THREAD_NUM_MAX][LWIP_THREAD_STK_SIZE];
OS_TCB LwipTCBPool[LWIP_THREAD_NUM_MAX];


/*-----------------------------------------------------------------------------------*/
//  Creates an empty mailbox.
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    OS_ERR err;
    
    OSQCreate (mbox, "lwip queue", size, &err);

    if(err != OS_ERR_NONE)
    {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_new: err:%d \n", err));
        return ERR_MEM;
    }
    return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void sys_mbox_free(sys_mbox_t *mbox)
{
    OS_ERR err;

    OSQDel(mbox, OS_OPT_DEL_ALWAYS, &err);

    if(err != OS_ERR_NONE)
    {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_free: err:%d \n", err));
    }
}

/*-----------------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
void sys_mbox_post(sys_mbox_t *mbox, void *data)
{
    OS_ERR err;

    while(1)
    {
        OSQPost(mbox, data, 0, OS_OPT_POST_FIFO, &err);

        if(err == OS_ERR_NONE)
            return;

        LWIP_ASSERT( "sys_mbox_post error\n", (err == OS_ERR_MSG_POOL_EMPTY) || (err == OS_ERR_Q_MAX) );              
    }
}


/*-----------------------------------------------------------------------------------*/
//   Try to post the "msg" to the mailbox.
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    OS_ERR err;    

    OSQPost(mbox, msg, 0, OS_OPT_POST_FIFO, &err);    

    if(err != OS_ERR_NONE)
    {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mbox_trypost: err:%d \n", err));
        return ERR_MEM;
    }
    return ERR_OK; 
}

/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread until a message arrives in the mailbox, but does
  not block the thread longer than "timeout" milliseconds (similar to
  the sys_arch_sem_wait() function). The "msg" argument is a result
  parameter that is set by the function (i.e., by doing "*msg =
  ptr"). The "msg" parameter maybe NULL to indicate that the message
  should be dropped.

  The return values are the same as for the sys_arch_sem_wait() function:
  Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
  timeout.

  Note that a function with a similar name, sys_mbox_fetch(), is
  implemented by lwIP.
*/
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    OS_ERR err;
    OS_TICK StartTime;
    OS_MSG_SIZE  msg_size;    
    void *dummyptr;

    if(!msg)
        msg = &dummyptr;

    StartTime = OSTimeGet(&err);
    *msg = OSQPend(mbox, timeout, OS_OPT_PEND_BLOCKING, &msg_size, NULL, &err);

    if(err == OS_ERR_TIMEOUT)
        return SYS_ARCH_TIMEOUT;
    else if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_arch_mbox_fetch: err:%d \n", err));        

    return (OSTimeGet(&err) - StartTime);
}

/*-----------------------------------------------------------------------------------*/
/*
  Similar to sys_arch_mbox_fetch, but if message is not ready immediately, we'll
  return with SYS_MBOX_EMPTY.  On success, 0 is returned.
*/
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    OS_ERR err;
    OS_MSG_SIZE  msg_size;    
    void *dummyptr;

    if(!msg)
        msg = &dummyptr;

    *msg = OSQPend(mbox, 0, OS_OPT_PEND_NON_BLOCKING, &msg_size, NULL, &err);

    if(err == OS_ERR_NONE)
        return ERR_OK;
    else if(err != OS_ERR_PEND_WOULD_BLOCK)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_arch_mbox_tryfetch: err:%d \n", err));      
    
    return SYS_MBOX_EMPTY;
}

/** 
  * Check if an mbox is valid/allocated: 
  * @param sys_mbox_t *mbox pointer mail box
  * @return 1 for valid, 0 for invalid 
  */ 
int sys_mbox_valid(sys_mbox_t *mbox)
{
  if(mbox->NamePtr)  
    return (strcmp(mbox->NamePtr,"?Q"))? 1:0;
  else
    return 0;
}

/** 
  * Set an mbox invalid so that sys_mbox_valid returns 0 
  */      
void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  if(sys_mbox_valid(mbox))
    sys_mbox_free(mbox);
}   

/*-----------------------------------------------------------------------------------*/
//  Creates a new semaphore. The "count" argument specifies
//  the initial state of the semaphore.
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    OS_ERR err;

    OSSemCreate(sem, "lwip sem", count, &err);

    if(err != OS_ERR_NONE)
    {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_sem_new: err:%d \n", err));
        return ERR_MEM;
    }
    return ERR_OK;    
}

/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread while waiting for the semaphore to be
  signaled. If the "timeout" argument is non-zero, the thread should
  only be blocked for the specified time (measured in
  milliseconds).

  If the timeout argument is non-zero, the return value is the number of
  milliseconds spent waiting for the semaphore to be signaled. If the
  semaphore wasn't signaled within the specified time, the return value is
  SYS_ARCH_TIMEOUT. If the thread didn't have to wait for the semaphore
  (i.e., it was already signaled), the function may return zero.

  Notice that lwIP implements a function with a similar name,
  sys_sem_wait(), that uses the sys_arch_sem_wait() function.
*/
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    OS_ERR err;
    OS_TICK StartTime;


    StartTime = OSTimeGet(&err);
    OSSemPend(sem, timeout, OS_OPT_PEND_BLOCKING, NULL, &err);
    
    if(err == OS_ERR_TIMEOUT)
        return SYS_ARCH_TIMEOUT;
    else if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_arch_sem_wait: err:%d \n", err));        

    return (OSTimeGet(&err) - StartTime);    
}


/*-----------------------------------------------------------------------------------*/
// Signals a semaphore
void sys_sem_signal(sys_sem_t *sem)
{
    OS_ERR err;

    OSSemPost(sem, OS_OPT_POST_1, &err);
    if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_sem_signal: err:%d \n", err));         
}

/*-----------------------------------------------------------------------------------*/
// Deallocates a semaphore
void sys_sem_free(sys_sem_t *sem)
{
    OS_ERR err;

    OSSemDel(sem, OS_OPT_DEL_ALWAYS, &err);
    if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_sem_free: err:%d \n", err));     
}


int sys_sem_valid(sys_sem_t *sem)
{
  if(sem->NamePtr)
    return (strcmp(sem->NamePtr,"?SEM"))? 1:0;
  else
    return 0;
}

/** Set a semaphore invalid so that sys_sem_valid returns 0 */
void sys_sem_set_invalid(sys_sem_t *sem)
{
  if(sys_sem_valid(sem))
    sys_sem_free(sem);
}



/*-----------------------------------------------------------------------------------*/
// Initialize sys arch
void sys_init(void)
{
    OS_ERR err;

    OSMemCreate(&LwipTCBMem, "LwipTCBMem", &LwipTCBPool[0], LWIP_THREAD_NUM_MAX, sizeof(OS_TCB), &err);
    
    LWIP_ASSERT( "sys_init: failed OSMemCreate TCB", err == OS_ERR_NONE);

    OSMemCreate(&LwipStkMem, "LwipStkMem", &LwipStkPool[0][0], LWIP_THREAD_NUM_MAX, LWIP_THREAD_STK_SIZE*sizeof(CPU_STK), &err);

    LWIP_ASSERT( "sys_init: failed OSMemCreate STK", err == OS_ERR_NONE);
}

/*-----------------------------------------------------------------------------------*/
                                      /* Mutexes*/
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
#if LWIP_COMPAT_MUTEX == 0
/* Create a new mutex*/
err_t sys_mutex_new(sys_mutex_t *mutex) {
    OS_ERR err;

    OSMutexCreate(mutex,"lwip mutex", &err);

    if(err != OS_ERR_NONE)
    {
        LWIP_DEBUGF(SYS_DEBUG, ("sys_sem_new: err:%d \n", err));
        return ERR_MEM;
    }
    return ERR_OK;    
}



/*-----------------------------------------------------------------------------------*/
/* Deallocate a mutex*/
void sys_mutex_free(sys_mutex_t *mutex)
{
    OS_ERR err;
    
    OSMutexDel(mutex, OS_OPT_DEL_ALWAYS, &err);

    if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mutex_free: err:%d \n", err));
}
/*-----------------------------------------------------------------------------------*/
/* Lock a mutex*/
void sys_mutex_lock(sys_mutex_t *mutex)
{
    OS_ERR err;

    OSMutexPend(mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    
    if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mutex_lock: err:%d \n", err));
}

/*-----------------------------------------------------------------------------------*/
/* Unlock a mutex*/
void sys_mutex_unlock(sys_mutex_t *mutex)
{
    OS_ERR err;
    
	OSMutexPost(mutex,OS_OPT_POST_NONE, &err);
    if(err != OS_ERR_NONE)
        LWIP_DEBUGF(SYS_DEBUG, ("sys_mutex_lock: err:%d \n", err));    
}

#endif /*LWIP_COMPAT_MUTEX*/

/*-----------------------------------------------------------------------------------*/
// TODO
/*-----------------------------------------------------------------------------------*/
/*
  Starts a new thread with priority "prio" that will begin its execution in the
  function "thread()". The "arg" argument will be passed as an argument to the
  thread() function. The id of the new thread is returned. Both the id and
  the priority are system dependent.
*/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread , void *arg, int stacksize, int prio)
{
    OS_ERR err;
    CPU_STK *stk;
    OS_TCB *tcb;

    LWIP_ASSERT("stacksize > KERNEL_THREAD_STK_SIZE\n",stacksize <= LWIP_THREAD_STK_SIZE);

    tcb = OSMemGet(&LwipTCBMem,&err);

    LWIP_ASSERT("sys_thread_new: tcb is NULL\n",tcb != NULL);

    stk = OSMemGet(&LwipStkMem, &err);

    OSTaskCreate((OS_TCB     *)tcb,                /* Create the start task                                    */
                 (CPU_CHAR   *)name,
                 (OS_TASK_PTR ) thread,
                 (void       *) arg,
                 (OS_PRIO     ) prio,
                 (CPU_STK    *) stk,
                 (CPU_STK     )(stacksize / 10u),
                 (CPU_STK_SIZE) stacksize,
                 (OS_MSG_QTY  ) 5,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);  


    return tcb;
}


/**
 * Sleep for some ms. Timeouts are NOT processed while sleeping.
 *
 * @param ms number of milliseconds to sleep
 */
void
sys_msleep(u32_t ms)
{
    OS_ERR      err;

    OSTimeDlyHMSM(0u, 0u, 0u, ms,
              OS_OPT_TIME_HMSM_NON_STRICT,
              &err);
}





