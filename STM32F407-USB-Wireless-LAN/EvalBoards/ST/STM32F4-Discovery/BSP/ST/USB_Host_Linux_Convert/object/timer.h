#ifndef _TIMER_H
#define _TIMER_H

#include "os_cfg_app.h"
#include "stdint.h"
#include "delay.h"

extern volatile uint32_t jiffies;
#define HZ   OS_CFG_TICK_RATE_HZ
#define udelay   delay_us


#define mdelay delay_ms

struct timer_list {
    OS_TMR tmr;
    char *name;
	unsigned long expires;

	void (*function)(unsigned long);
	unsigned long data;
};

//#define init_timer(timer) _init_timer(#)

//#define mod_timer(timer, expires) _mod_timer(#timer, timer, expires)

#define del_timer_sync(t)		del_timer(t)
int mod_timer(struct timer_list *timer, unsigned long expires);

void init_timer(struct timer_list *timer);

void add_timer(struct timer_list *timer);
int timer_pending(struct timer_list * timer);

int del_timer(struct timer_list *timer);

void msleep(unsigned int msecs);




#endif
