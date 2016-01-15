#ifndef _MUTEX_H
#define _MUTEX_H






struct mutex {
    OS_MUTEX mutex;
};




void _mutex_init(char *name, struct mutex *mutex);
#define mutex_init(mutex) _mutex_init(#mutex, mutex)


void mutex_lock(struct mutex *lock);
void mutex_unlock(struct mutex *lock);



#endif
