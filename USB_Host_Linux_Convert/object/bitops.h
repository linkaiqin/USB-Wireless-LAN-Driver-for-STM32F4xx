#ifndef _BITOPS_H
#define _BITOPS_H



#include "os.h"


/*
 * These functions are the basis of our bit ops.
 *
 * First, the atomic bitops. These use native endian.
 */
static inline void ____atomic_set_bit(unsigned int bit, volatile unsigned long *p)
{
	CPU_SR cpu_sr;
	unsigned long mask = 1UL << (bit & 31);

	p += bit >> 5;

	CPU_CRITICAL_ENTER();
	*p |= mask;
	CPU_CRITICAL_EXIT();
}

static inline void ____atomic_clear_bit(unsigned int bit, volatile unsigned long *p)
{
	CPU_SR cpu_sr;
	unsigned long mask = 1UL << (bit & 31);

	p += bit >> 5;

	CPU_CRITICAL_ENTER();
	*p &= ~mask;
	CPU_CRITICAL_EXIT();
}

static inline void ____atomic_change_bit(unsigned int bit, volatile unsigned long *p)
{
	CPU_SR cpu_sr;
	unsigned long mask = 1UL << (bit & 31);

	p += bit >> 5;

	CPU_CRITICAL_ENTER();
	*p ^= mask;
	CPU_CRITICAL_EXIT();
}

static inline int
____atomic_test_and_set_bit(unsigned int bit, volatile unsigned long *p)
{
	CPU_SR cpu_sr;
	unsigned int res;
	unsigned long mask = 1UL << (bit & 31);

	p += bit >> 5;

	CPU_CRITICAL_ENTER();
	res = *p;
	*p = res | mask;
	CPU_CRITICAL_EXIT();

	return res & mask;
}

static inline int
____atomic_test_and_clear_bit(unsigned int bit, volatile unsigned long *p)
{
	CPU_SR cpu_sr;
	unsigned int res;
	unsigned long mask = 1UL << (bit & 31);

	p += bit >> 5;

	CPU_CRITICAL_ENTER();
	res = *p;
	*p = res & ~mask;
	CPU_CRITICAL_EXIT();

	return res & mask;
}

static inline int
____atomic_test_and_change_bit(unsigned int bit, volatile unsigned long *p)
{
	CPU_SR cpu_sr;
	unsigned int res;
	unsigned long mask = 1UL << (bit & 31);

	p += bit >> 5;

	CPU_CRITICAL_ENTER();
	res = *p;
	*p = res ^ mask;
	CPU_CRITICAL_EXIT();

	return res & mask;
}

#define BITS_PER_LONG 32
#define BIT_WORD(nr)		((nr) / BITS_PER_LONG)

/**
 * test_bit - Determine whether a bit is set
 * @nr: bit number to test
 * @addr: Address to start counting from
 */
static inline int test_bit(int nr, const volatile unsigned long *addr)
{
	return 1UL & (addr[BIT_WORD(nr)] >> (nr & (BITS_PER_LONG-1)));
}





#define ATOMIC_BITOP_LE(name,nr,p)	____atomic_##name(nr,p)

/*
 * These are the little endian, atomic definitions.
 */
#define set_bit(nr,p)			ATOMIC_BITOP_LE(set_bit,nr,p)
#define clear_bit(nr,p)			ATOMIC_BITOP_LE(clear_bit,nr,p)
#define change_bit(nr,p)		ATOMIC_BITOP_LE(change_bit,nr,p)
#define test_and_set_bit(nr,p)		ATOMIC_BITOP_LE(test_and_set_bit,nr,p)
#define test_and_clear_bit(nr,p)	ATOMIC_BITOP_LE(test_and_clear_bit,nr,p)
#define test_and_change_bit(nr,p)	ATOMIC_BITOP_LE(test_and_change_bit,nr,p)



#endif
