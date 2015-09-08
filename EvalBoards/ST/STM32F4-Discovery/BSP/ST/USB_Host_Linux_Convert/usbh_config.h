#ifndef _USBH_CONFIG_H
#define _USBH_CONFIG_H

#include "stdio.h"

#define USBH_URB_NUM_MAX           15
#define USBH_REQ_QUEUE_NUM_MAX     15



#define USBH_MAX_NUM_ENDPOINTS                HC_MAX//10
#define USBH_MAX_NUM_INTERFACES               1









#ifndef USBH_DEBUG_LEVEL
#define USBH_DEBUG_LEVEL USBH_DEBUG_ERROR
#endif

#define USBH_DEBUG_OFF     0
#define USBH_DEBUG_ERROR   1
#define USBH_DEBUG_TRACE   2


#define USBH_TRACE(...)   ((USBH_DEBUG_LEVEL >= USBH_DEBUG_TRACE)  ? (void)(printf(__VA_ARGS__)) : (void)0)
#define USBH_DBG(...)   ((USBH_DEBUG_LEVEL >= USBH_DEBUG_ERROR)  ? (void)(printf(__VA_ARGS__)) : (void)0)



#endif
