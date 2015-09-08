#ifndef _TASKLET_H
#define _TASKLET_H

#include "atomic.h"



struct tasklet_struct
{
//	struct tasklet_struct *next;
	unsigned long state;
	atomic_t count;
	void (*func)(unsigned long);
	unsigned long data;
};

#define tasklet_hi_schedule tasklet_schedule

void tasklet_schedule(struct tasklet_struct *t);

void tasklet_init(struct tasklet_struct *t,
		  void (*func)(unsigned long), unsigned long data);


void flush_scheduled_work(void);

void tasklet_task_init(void);




#endif
