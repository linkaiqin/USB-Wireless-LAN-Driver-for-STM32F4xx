 #ifndef _COMPLETE_H
#define _COMPLETE_H





struct completion {
    OS_TCB *p_tcb;
//	unsigned int done;
//	wait_queue_head_t wait;
};




void complete(struct completion *x);
void  wait_for_completion(struct completion *x);
void init_completion(struct completion *x);
void complete_and_exit(struct completion *comp, long code);










#endif

