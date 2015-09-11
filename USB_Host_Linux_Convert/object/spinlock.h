#ifndef _SPINLOCK_H
#define _SPINLOCK_H

#include "usbh_config.h"
#include "cpu_cfg.h"


#ifdef   CPU_CFG_INT_DIS_MEAS_EN
#define  CPU_CRITICAL_ENTER_EX(irqflags)  do { irqflags = CPU_SR_Save();         \
                                                CPU_IntDisMeasStart();     }  while (0)
                                                                        /* Stop & measure   interrupts disabled time,   */
                                                                        /* ...  & re-enable interrupts.                 */
#define  CPU_CRITICAL_EXIT_EX(irqflags)   do { CPU_IntDisMeasStop();  \
                                                CPU_SR_Restore(irqflags);  }  while (0)
#else
#define  CPU_CRITICAL_ENTER_EX(irqflags)  do { irqflags = CPU_SR_Save(); } while (0)          /* Disable   interrupts.                        */
#define  CPU_CRITICAL_EXIT_EX(irqflags)   do { CPU_SR_Restore(irqflags); } while (0)          /* Re-enable interrupts.                        */
#endif



typedef int spinlock_t;


#define SPIN_LOCK_UNLOCKED   0
#define SPIN_LOCK_LOCKED     1

#define spin_lock_init(lock)					\
	do { *(lock) = SPIN_LOCK_UNLOCKED; } while (0)


#define spin_lock_bh  spin_lock
#define spin_unlock_bh spin_unlock

#define spin_lock(lock)\
do{\
    OS_ERR err;\
    OSSchedLock(&err);\
    if(err != OS_ERR_NONE) USBH_DBG("spin_lock OSSchedLock Failed %d\r\n",err);\
    if(*(lock) == SPIN_LOCK_LOCKED) USBH_DBG("spin_lock Failed spinlock has been locked\r\n");\
    *(lock) = SPIN_LOCK_LOCKED;\
}while(0)


#define spin_unlock(lock)\
do {\
    OS_ERR err;\
    if(*(lock) == SPIN_LOCK_UNLOCKED) USBH_DBG("spin_unlock Failed spinlock has been unlocked\r\n");\
    *(lock) = SPIN_LOCK_UNLOCKED;\
    OSSchedUnlock(&err);\
    if((err != OS_ERR_NONE) && (err != OS_ERR_SCHED_LOCKED)) USBH_DBG("spin_unlock OSSchedUnlock Failed %d\r\n",err);\
} while (0)


//do OSSchedLock(), prevent call ucos post func may lead to sched fail
#define spin_lock_irqsave(lock, irqflags)\
do {\
    OS_ERR err;\
    CPU_CRITICAL_ENTER_EX(irqflags);\
    OSSchedLock(&err);\
    if(err != OS_ERR_NONE) USBH_DBG("spin_lock_irqsave OSSchedLock Failed %d\r\n",err);\
    if(*(lock) == SPIN_LOCK_LOCKED) USBH_DBG("spin_lock_irqsave Failed spinlock has been locked\r\n");\
    *(lock) = SPIN_LOCK_LOCKED;\
} while (0)





#define spin_unlock_irqrestore(lock,irqflags)\
do {\
    OS_ERR err;\
    if(*(lock) == SPIN_LOCK_UNLOCKED) USBH_DBG("spin_unlock_irqrestore Failed spinlock has been unlocked\r\n");\
    *(lock) = SPIN_LOCK_UNLOCKED;\
    OSSchedUnlock(&err);\
    if((err != OS_ERR_NONE) && (err != OS_ERR_SCHED_LOCKED)) USBH_DBG("spin_unlock_irqrestore OSSchedUnlock Failed %d\r\n",err);\
    CPU_CRITICAL_EXIT_EX(irqflags);\
} while (0)




#endif



