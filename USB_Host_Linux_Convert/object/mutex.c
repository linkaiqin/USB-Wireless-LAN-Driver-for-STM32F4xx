#include "os.h"
#include "stdio.h"
#include "mutex.h"






void _mutex_init(char *name, struct mutex *mutex)
{
    OS_ERR err;
    CPU_SR cpu_sr;

#if OS_CFG_DBG_EN > 0u
    //The procedure _mutex_init()-->OSMutexCreate()-->OS_MutexDbgListAdd() is dangerous,
    //because some linux driver don't call mutex_destroy(), the mutex instance will
    //still exsit in OSMutexDbgListPtr after freed. Alway set OSMutexDbgListPtr to zero.
    CPU_CRITICAL_ENTER();
    OSMutexDbgListPtr = NULL;
    CPU_CRITICAL_EXIT();
#endif
    OSMutexCreate(&mutex->mutex, name, &err);

    if(err != OS_ERR_NONE)
    {
        printf("_mutex_init failed:%d hang!!\r\n",err);
        while(1);
    }
}

void mutex_destroy(struct mutex *mutex)
{
    OS_ERR err;
    
    OSMutexDel (&mutex->mutex, OS_OPT_DEL_ALWAYS, &err);
    
    if(err != OS_ERR_NONE)
    {
        printf("mutex_destroy failed:%d hang!!\r\n",err);
        while(1);
    }    
}

void mutex_lock(struct mutex *lock)
{
    OS_ERR err;

    OSMutexPend(&lock->mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    if(err != OS_ERR_NONE)
    {
        printf("mutex_lock failed:%d hang!!\r\n",err);
        while(1);
    }
}

void mutex_unlock(struct mutex *lock)
{
    OS_ERR err;

    OSMutexPost(&lock->mutex, OS_OPT_POST_NONE, &err);
    if(err != OS_ERR_NONE)
    {
        printf("mutex_unlock failed:%d hang!!\r\n",err);
        while(1);
    }
}





