
#ifndef _ATOMIC_H
#define _ATOMIC_H


typedef struct {
	volatile int counter;
} atomic_t;


#define atomic_read(v)		((v)->counter)
#define atomic_set(v, i)	(((v)->counter) = (i))

#define atomic_inc(v)		(atomic_add_return(1, (v)))
#define atomic_dec(v)		(atomic_sub_return(1, (v)))

#define atomic_add(i, v)	(atomic_add_return(i, (v)))
#define atomic_sub(i, v)	(atomic_sub_return(i, (v)))

#define atomic_inc_return(v)	(atomic_add_return(1, (v)))
#define atomic_dec_return(v)	(atomic_sub_return(1, (v)))

#define atomic_inc_and_test(v)	(atomic_add_return(1, (v)) == 0)
#define atomic_dec_and_test(v)	(atomic_sub_return(1, (v)) == 0)


int atomic_sub_return(int i, atomic_t *v);
int atomic_add_return(int i, atomic_t *v);




#endif
