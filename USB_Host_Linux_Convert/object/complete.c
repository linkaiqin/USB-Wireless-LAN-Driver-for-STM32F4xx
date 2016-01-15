#include "os.h"
#include "sem.h"
#include "complete.h"
#include "kthread.h"



void init_completion(struct completion *x)
{
    sema_init(&x->sem, 0);
//	x->done = 0;
//	init_waitqueue_head(&x->wait);
}




void complete(struct completion *x)
{
    up(&x->sem);
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


void  wait_for_completion(struct completion *x)
{
    down(&x->sem);
}





