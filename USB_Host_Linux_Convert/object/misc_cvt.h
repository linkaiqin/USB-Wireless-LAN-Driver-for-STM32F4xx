#ifndef _MISC_CVT_H
#define _MISC_CVT_H
#include "stdio.h"
#include "string.h"
#include "type.h"

#define smp_rmb()
#define unlikely
#define likely
#define __user
#define __init
#define __exit


#define	KERN_EMERG	"<0>"	/* system is unusable			*/
#define	KERN_ALERT	"<1>"	/* action must be taken immediately	*/
#define	KERN_CRIT	"<2>"	/* critical conditions			*/
#define	KERN_ERR	"<3>"	/* error conditions			*/
#define	KERN_WARNING	"<4>"	/* warning conditions			*/
#define	KERN_NOTICE	"<5>"	/* normal but significant condition	*/
#define	KERN_INFO	"<6>"	/* informational			*/
#define	KERN_DEBUG	"<7>"	/* debug-level messages			*/

//#define printk printf
#define printk(...) do{printf("[%d.%03d] ",  OSTickCtr/1000, OSTickCtr%1000);printf(__VA_ARGS__); printf("\r\n");}while(0)


#define EXPORT_SYMBOL(x) 

#define offsetof offset_of
#define offset_of(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

#define container_of(obj, type, memb) \
	((type *)(((char *)obj) - offset_of(type, memb)))



//#define test_bit(nr, addr)   (*addr & (1L << nr))
//#define __set_bit(nr, addr)   *addr |=  (1L << nr)
//#define __clr_bit(nr, addr)   *addr &=  ~(1L << nr)
//#define set_bit __set_bit
//#define clear_bit __clr_bit

/**
 * assert() equivalent, that is always enabled.
 */
#define misc_assert(cond) do {                                           \
    if (!(cond)) {                                                      \
        printf("Assertion %s failed at %s:%d\r\n",                        \
               #cond, __func__, __LINE__);                              \
        while(1);                                                        \
    }                                                                   \
} while (0)


#define BUG() do{printf("bug\r\n");while(1);}while(0)
#define BUG_ON(x) 			misc_assert(!(x))
#define WARN_ON(x) 		misc_assert(!(x))
#define BUILD_BUG_ON(x)	misc_assert(!(x))

#define mb()
#define module_init(init_func) 
#define module_exit(exit_func)

#define module_put(x)
#define try_module_get(x)  1
#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
#define module_param(x,y,z)
#define MODULE_PARM_DESC(x,y)
#define MODULE_AUTHOR(x)
#define MODULE_DESCRIPTION(x)
#define MODULE_LICENSE(x)

#define __cpu_to_le16(x) ((__u16)(__u16)(x))
#define le16_to_cpu(val) (val)

/*
 * Check at compile time that something is of a particular type.
 * Always evaluates to 1 so you may use it easily in comparisons.
 */
#define typecheck(type,x) \
({	type __dummy; \
	typeof(x) __dummy2; \
	(void)(&__dummy == &__dummy2); \
	1; \
})

#include <ctype.h>
// #define islower ASCII_IS_LOWER
// #define isspace ASCII_IS_SPACE
// #define isdigit ASCII_IS_DIG
// #define isxdigit ASCII_IsDigHex
#define simple_strtol Str_ParseNbr_Int32U



#define LINUX_VERSION_CODE  KERNEL_VERSION(2,6,30)
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))



/* ARP protocol HARDWARE identifiers. */
#define ARPHRD_NETROM	0		/* from KA9Q: NET/ROM pseudo	*/
#define ARPHRD_ETHER 	1		/* Ethernet 10Mbps		*/
#define	ARPHRD_EETHER	2		/* Experimental Ethernet	*/
#define	ARPHRD_AX25	3		/* AX.25 Level 2		*/
#define	ARPHRD_PRONET	4		/* PROnet token ring		*/
#define	ARPHRD_CHAOS	5		/* Chaosnet			*/
#define	ARPHRD_IEEE802	6		/* IEEE 802.2 Ethernet/TR/TB	*/
#define	ARPHRD_ARCNET	7		/* ARCnet			*/
#define	ARPHRD_APPLETLK	8		/* APPLEtalk			*/
#define ARPHRD_DLCI	15		/* Frame Relay DLCI		*/
#define ARPHRD_ATM	19		/* ATM 				*/
#define ARPHRD_METRICOM	23		/* Metricom STRIP (new IANA id)	*/
#define	ARPHRD_IEEE1394	24		/* IEEE 1394 IPv4 - RFC 2734	*/
#define ARPHRD_EUI64	27		/* EUI-64                       */
#define ARPHRD_INFINIBAND 32		/* InfiniBand			*/
	/* 787->799 reserved for fibrechannel media types */
#define ARPHRD_IEEE802_TR 800		/* Magic type ident for TR	*/
#define ARPHRD_IEEE80211 801		/* IEEE 802.11			*/
#define ARPHRD_IEEE80211_PRISM 802	/* IEEE 802.11 + Prism2 header  */
#define ARPHRD_IEEE80211_RADIOTAP 803	/* IEEE 802.11 + radiotap header */




#define VERIFY_READ	0
#define VERIFY_WRITE	1



//static inline int ate_copy_to_user(
//	char * payload,
//	char * msg,
//	int    len)
//{
//	memmove(payload, msg, len);
//	return 0;
//}
//#define copy_to_user(x,y,z) ate_copy_to_user((char *)x, (char *)y, z)
//#define copy_from_user(to,from,n)	(memcpy(to, (void __force *)from, n), 0)


#define __copy_from_user(to,from,n)	memcpy(to, from, n)
#define __copy_to_user(to,from,n)	memcpy(to, from, n)
#define __clear_user(addr,n)		memset(addr, 0, n)

static inline unsigned long  copy_from_user(void *to, const void  *from, unsigned long n)
{

	__copy_from_user(to, from, n);
	return 0;
}

static inline unsigned long  copy_to_user(void  *to, const void *from, unsigned long n)
{
    __copy_to_user(to, from, n);
	return 0;
}

#define __range_ok(addr,size)	(0)
#define access_ok(type,addr,size)	(__range_ok(addr,size) == 0)


char *strsep(char **s, const char *ct);

//#include "lwip/sockets.h"
//#include "lwip/def.h"

//temp  need to uncoment==========================================================================//
//temp define
typedef unsigned short	sa_family_t;

struct sockaddr {
	sa_family_t	sa_family;	/* address family, AF_xxx	*/
	char		sa_data[14];	/* 14 bytes of protocol address	*/
};
#if 1

#define  NET_UTIL_HOST_TO_NET_32(val)            NET_UTIL_NET_TO_HOST_32(val)
#define  NET_UTIL_HOST_TO_NET_16(val)            NET_UTIL_NET_TO_HOST_16(val)


#define  NET_UTIL_NET_TO_HOST_32(val)                   (((((CPU_INT32U)(val)) & 0xFF000000) >> 24) | \
                                                         ((((CPU_INT32U)(val)) & 0x00FF0000) >>  8) | \
                                                         ((((CPU_INT32U)(val)) & 0x0000FF00) <<  8) | \
                                                         ((((CPU_INT32U)(val)) & 0x000000FF) << 24))

#define  NET_UTIL_NET_TO_HOST_16(val)                   (((((CPU_INT16U)(val)) &     0xFF00) >>  8) | \
                                                         ((((CPU_INT16U)(val)) &     0x00FF) <<  8))
                                                         

#define  ntohs(val)                      NET_UTIL_NET_TO_HOST_16(val)
#define  ntohl(val)                      NET_UTIL_NET_TO_HOST_32(val)

#define  htons(val)                      NET_UTIL_HOST_TO_NET_16(val)
#define  htonl(val)                      NET_UTIL_HOST_TO_NET_32(val)


#endif




#endif
