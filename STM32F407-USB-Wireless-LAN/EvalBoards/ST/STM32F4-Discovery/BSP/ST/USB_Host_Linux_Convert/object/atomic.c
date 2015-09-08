#include "os.h"
#include "atomic.h"


int atomic_sub_return(int i, atomic_t *v)
{
	int ret;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
	ret = v->counter -= i;
    CPU_CRITICAL_EXIT();

	return ret;
}

int atomic_add_return(int i, atomic_t *v)
{
	int ret;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
	ret = v->counter += i;
    CPU_CRITICAL_EXIT();

	return ret;
}








