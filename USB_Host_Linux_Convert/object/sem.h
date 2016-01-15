#ifndef _SEM_H
#define _SEM_H




struct semaphore {
    OS_SEM p_os_sem;
};


#define DECLARE_MUTEX(name)	\
	struct semaphore name


void _sema_init(char *name, struct semaphore *sem, int val);
int _down_interruptible(struct semaphore *sem);
void _up(struct semaphore *sem);
void _sema_destroy(struct semaphore *sem);
int down_trylock(struct semaphore *sem);
void down(struct semaphore *sem);


#define init_MUTEX(sem)		sema_init(sem, 1)
#define init_MUTEX_LOCKED(sem)	sema_init(sem, 0)


#define sema_init(sem,val)   _sema_init(#sem, sem, val)
#define down_interruptible _down_interruptible
#define up _up
#define sema_destroy _sema_destroy


#define rcu_read_lock()
#define rcu_read_unlock()

#define rtnl_lock()
#define rtnl_unlock()




#endif
