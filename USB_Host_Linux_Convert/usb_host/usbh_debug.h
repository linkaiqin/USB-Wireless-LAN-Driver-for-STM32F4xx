#ifndef _USBH_DEBUG_H
#define _USBH_DEBUG_H


#include "stdio.h"


#define DEBUG




#define	USB_HALT	"<halt>"	/* system is unusable			*/
#define	USB_EMERG	"<emerge>"	/* system is unusable			*/
#define	USB_ALERT	"<alert>"	/* action must be taken immediately	*/
#define	USB_CRIT	"<crit>"	/* critical conditions			*/
#define	USB_ERR	"<err>"	/* error conditions			*/
#define	USB_WARNING	"<warn>"	/* warning conditions			*/
#define	USB_NOTICE	"<notice>"	/* normal but significant condition	*/
#define	USB_INFO	"<info>"	/* informational			*/
#define	USB_DEBUG	"<debug>"	/* debug-level messages			*/




#define usb_printk(level, dev, format, arg...)	\
	printf(level "%s: " format ,  \
	          ((dev) != NULL)?(dev)->name:"" , ## arg)
	          
#define usb_halt(dev, format, arg...)		\
        do{usb_printk(USB_HALT , dev , "%s " format , __func__, ## arg);while(1);}while(0)
#define usb_emerg(dev, format, arg...)		\
	usb_printk(USB_EMERG , dev , "%s "format , __func__, ## arg)
#define usb_alert(dev, format, arg...)		\
	usb_printk(USB_ALERT , dev , format , ## arg)
#define usb_crit(dev, format, arg...)		\
	usb_printk(USB_CRIT , dev , format , ## arg)
#define usb_err(dev, format, arg...)		\
	usb_printk(USB_ERR , dev , format , ## arg)
#define usb_warn(dev, format, arg...)		\
	usb_printk(USB_WARNING , dev , format , ## arg)
#define usb_notice(dev, format, arg...)		\
	usb_printk(USB_NOTICE , dev , format , ## arg)
#define usb_info(dev, format, arg...)		\
	usb_printk(USB_INFO , dev , format , ## arg)

#if defined(DEBUG)
#define usb_dbg(dev, format, arg...)		\
	usb_printk(USB_DEBUG , dev , format , ## arg)
#else
#define usb_dbg(dev, format, arg...)		\
	({ if (0) usb_printk(USB_DEBUG, dev, format, ##arg); 0; })
#endif



#ifndef USBH_DEBUG_LEVEL
#define USBH_DEBUG_LEVEL USBH_DEBUG_ERROR
#endif

#define USBH_DEBUG_OFF     0
#define USBH_DEBUG_ERROR   1
#define USBH_DEBUG_TRACE   2


#define USBH_TRACE(...)   ((USBH_DEBUG_LEVEL >= USBH_DEBUG_TRACE)  ? (void)(printf(__VA_ARGS__)) : (void)0)
#define USBH_DBG(...)   ((USBH_DEBUG_LEVEL >= USBH_DEBUG_ERROR)  ? (void)(printf(__VA_ARGS__)) : (void)0)




#endif
