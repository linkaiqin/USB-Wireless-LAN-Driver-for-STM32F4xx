#ifndef _MEMORY_H
#define _MEMORY_H


#ifdef __cplusplus
extern "C" {
#endif


#define GFP_NOIO    0
#define GFP_KERNEL  1 
#define GFP_ATOMIC   2

    
//Merge lwip malloc function with USB_Linux_Convert malloc funtion.This macro option 
//will decrease the usage of memory.
//NOTE:If this option is defined, you should not complie the file mem.c of lwip, since 
//     USB_Linux_Convert malloc is modified to non-blocking, but lwip malloc is not.      
#define USE_LWIP_MALLOC






#ifndef USE_LWIP_MALLOC

#define USBH_MEM_SIZE                (90*1024)//(34*1024)
#define USBH_MEM_DEBUG                       1
#define USBH_MEM_USE_POOLS                   0



typedef unsigned   short   u16_t;
typedef unsigned   long    u32_t;


#if USBH_MEM_SIZE > 64000L
typedef u32_t usbh_mem_size_t;
#else
typedef u16_t usbh_mem_size_t;
#endif /* MEM_SIZE > 64000 */


extern usbh_mem_size_t memory_used;
extern usbh_mem_size_t memory_used_max;



#if USBH_MEM_USE_POOLS
/** mem_init is not used when using pools instead of a heap */
#define usbh_mem_init()
/** mem_trim is not used when using pools instead of a heap:
    we can't free part of a pool element and don't want to copy the rest */
#define usbh_mem_trim(mem, size) (mem)
#else /* MEM_USE_POOLS */
/* lwIP alternative malloc */
void  usbh_mem_init(void);
void *usbh_mem_trim(void *mem, usbh_mem_size_t size);
#endif /* MEM_USE_POOLS */
void *usbh_mem_malloc(usbh_mem_size_t size);
void *usbh_mem_calloc(usbh_mem_size_t count, usbh_mem_size_t size);
void  usbh_mem_free(void *mem);
void *usbh_mem_realloc(void *ptr, usbh_mem_size_t size);




#include "stdio.h"
static inline void* usbh_mem_malloc_dbg(usbh_mem_size_t size,const char* func)
{
    void* mem;

    mem = usbh_mem_malloc(size);
    if(size > 200)
    {
     printf("kmalloc:%p size:%ld in %s\r\n ",mem,size,func);
    }
//    printf("mem:%p\r\n",mem);
    return mem;
}
static inline void* usbh_mem_calloc_dbg(usbh_mem_size_t count, usbh_mem_size_t size,const char* func)
{
    void* mem;    
    printf("kcalloc size:%ld in %s\r\n",count*size,func);
    mem = usbh_mem_calloc(count,size);   
//    printf("mem:%p\r\n",mem);
    return mem;
}

static inline void  usbh_mem_free_dbg(void *mem,const char* func)
{
    printf("kfree mem:%p in %s\r\n",mem,func);
    return usbh_mem_free(mem);
}


//#define kmalloc(size,flags)     usbh_mem_malloc_dbg(size,__FUNCTION__)
// #define kzalloc(size,flags)   usbh_mem_calloc_dbg(1,size,__FUNCTION__)
// #define kcalloc(n,size,flags)   usbh_mem_calloc_dbg(n,size,__FUNCTION__)
// #define vmalloc(size)           usbh_mem_malloc_dbg(size,__FUNCTION__)

// #define kfree(mem)              usbh_mem_free_dbg(mem,__FUNCTION__)
// #define vfree(mem)		       usbh_mem_free_dbg(mem,__FUNCTION__)




#define kmalloc(size,flags)     usbh_mem_malloc(size)
#define kzalloc(size,flags)   usbh_mem_calloc(1,size)
#define kcalloc(n,size,flags)   usbh_mem_calloc(n,size)
#define vmalloc(size)           usbh_mem_malloc(size)
#define krealloc(size)     usbh_mem_realloc(size)


#define kfree(mem)              usbh_mem_free(mem)
#define vfree(mem)		        usbh_mem_free(mem)
#else




#include "os.h"


//notice that MEM_SIZE <= 64000L
typedef unsigned   short   mem_size_t;  //typedef u16_t mem_size_t;





void *mem_malloc(mem_size_t size);

void *mem_calloc(mem_size_t count, mem_size_t size);

void mem_free(void *rmem);

void *mem_realloc(void *ptr, mem_size_t size);

//static inline void* mem_malloc_dbg(mem_size_t size)
//{
//    void* mem;    
//    if(OSSchedLockNestingCtr)
//    {
//        printf("mem_malloc_dbg OSSchedLockNestingCtr>0 hang!\r\n");
//        while(1);
//    }
//    mem = mem_malloc(size);   
//    return mem;
//}

//static inline void* mem_calloc_dbg(mem_size_t count, mem_size_t size)
//{
//    void* mem;    
//    if(OSSchedLockNestingCtr)
//    {
//        printf("mem_malloc_dbg OSSchedLockNestingCtr>0 hang!\r\n");
//        while(1);
//    }

//    mem = mem_calloc(count,size);   

//    return mem;
//}


//#define kmalloc(size,flags)     mem_malloc_dbg(size)
//#define kcalloc(n,size,flags)   mem_calloc_dbg(n,size)
//#define vmalloc(size)           mem_malloc_dbg(size)



#define kmalloc(size,flags)     mem_malloc(size)   //none blocking
#define kzalloc(size,flags)   mem_calloc(1,size)
#define kcalloc(n,size,flags)   mem_calloc(n,size)
#define vmalloc(size)           mem_malloc(size)
#define krealloc(size)          mem_realloc(size)



#define kfree(mem)              mem_free(mem)  //none blocking
#define vfree(mem)		        mem_free(mem)




#define memory_used  lwip_stats.mem.used
#define memory_used_max  lwip_stats.mem.max


#endif



#ifdef __cplusplus
}
#endif


#endif
