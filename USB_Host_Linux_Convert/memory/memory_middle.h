#ifndef _MEMORY_MIDDLE_H
#define _MEMORY_MIDDLE_H
#include "os.h"
#include "stdio.h"
#include "memory.h"


#ifndef USE_LWIP_MALLOC

#define MEM_SIZE                USBH_MEM_SIZE
#define MEM_DEBUG               USBH_MEM_DEBUG
#define MEM_USE_POOLS           USBH_MEM_USE_POOLS


#define MEM_USE_POOLS_TRY_BIGGER_POOL         1


#define ram_heap usbh_ram_heap


#define mem_init  usbh_mem_init
#define mem_trim  usbh_mem_trim

#define mem_malloc  usbh_mem_malloc
#define mem_calloc  usbh_mem_calloc
#define mem_free  usbh_mem_free
#define mem_realloc  usbh_mem_realloc


#define NO_SYS                           0  
#define MEM_ALIGNMENT                   4
#define MEM_LIBC_MALLOC                 0
#define MEM_STATS_AVAIL(x, y)
#define MEM_STATS_INC(x)
#define STATS_INC_USED(x, y) do { memory_used += y; \
                                if (memory_used_max < memory_used) { \
                                    memory_used_max = memory_used; \
                                } \
                             } while(0)
#define MEM_STATS_INC_USED(x, y) STATS_INC_USED(mem, y)
#define MEM_STATS_DEC_USED(x, y) memory_used -= y





typedef unsigned   char    u8_t;
typedef signed     char    s8_t;
typedef unsigned   short   u16_t;
typedef signed     short   s16_t;
typedef unsigned   long    u32_t;
typedef signed     long    s32_t;
typedef u32_t mem_ptr_t;
typedef usbh_mem_size_t mem_size_t;

#define LWIP_DBG_TRACE              0
#define LWIP_DBG_LEVEL_SERIOUS      0
#define LWIP_DBG_LEVEL_SEVERE      0
#define S16_F         "d"
#define ERR_OK 0

#define LWIP_ASSERT(message, assertion) do { if(!(assertion)) \
  printf(message); } while(0)
#define LWIP_DEBUGF(debug, message) do { \
                               if ( \
                                 debug) { \
                                 printf message; \
                                 printf("\r\n");\
                               } \
                             } while(0)


                          
#endif

#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT 1 
#define SYS_ARCH_DECL_PROTECT(lev)  CPU_SR cpu_sr
#define SYS_ARCH_PROTECT(lev)       CPU_CRITICAL_ENTER()
#define SYS_ARCH_UNPROTECT(lev)     CPU_CRITICAL_EXIT()




typedef unsigned   char  sys_mutex_t;



#define sys_mutex_new(mu) __sys_mutex_new(mu, #mu)
static __inline char __sys_mutex_new(sys_mutex_t *mu,char *name)
{
//     OS_ERR err;
//     OSSemCreate(mu,name,1,&err);
//     if(err != OS_ERR_NONE)
//         return ERR_FAIL;
//     else
//         return ERR_OK;
    return 0;
}
static __inline signed char sys_mutex_lock(sys_mutex_t *mu)
{
    OS_ERR err;
    OSSchedLock(&err);
//    printf("sys_mutex_lock %d  ",err);
//    OSSemPend(mu,0,OS_OPT_PEND_BLOCKING,0,&err);
    if(err != OS_ERR_NONE)
    {
        printf("memory_middle.h: sys_mutex_lock error:%d\r\n",err);
        return -1;
    }
    else
        return 0;    
}
static __inline signed char sys_mutex_unlock(sys_mutex_t *mu)
{
    OS_ERR err;
//    OSSemPost(mu, OS_OPT_POST_1, &err);
    OSSchedUnlock(&err);
//    printf("sys_mutex_unlock %d\r\n",err);
    if((err != OS_ERR_NONE)&&(err != OS_ERR_SCHED_LOCKED))
    {
        printf("memory_middle.h: sys_mutex_unlock error:%d\r\n",err);
        return -1;
    }
    else
        return 0;    
}









#endif
