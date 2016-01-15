/* 
  Copyright (C) 2014-2015 Kaiqin Lin <linkaiqin@sina.com>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details:

  http://www.gnu.org/licenses/gpl.txt
*/


#define USBH_DEBUG_LEVEL USBH_DEBUG_TRACE
#include "os.h"
#include "usbh_debug.h"
#include "usbh_linux.h"
#include "usb_core.h"
#include "usb_bsp.h"
#include "usb_hcd.h"
#include "usb_hcd_int.h"
#include "usbh_def.h"
#include "usbh_ioreq.h"
#include "memory.h"
#include "string.h"
#include "list.h"
#include "timer.h"
#include "errno.h"

#ifdef USE_LWIP_MALLOC
#include "lwip/stats.h"
#endif


#define cpu_to_le16(val)  (val)
#define le16_to_cpu(val)  (val)



LIST_HEAD(usb_device_list_head);
LIST_HEAD(usb_driver_list_head);
LIST_HEAD(usb_hotplug_list_head);


OS_TMR USBH_ProbeTmr;
OS_Q USBH_ProbeQ;
OS_MUTEX USBH_MutexLock;


OS_MEM URB_Mem;
OS_MEM ReqQueueMem;
OS_TCB USBHTaskTCB;
OS_TCB USBHProbeTaskTCB;

CPU_STK  USBH_TaskStk[USBH_TASK_STK_SIZE];
CPU_STK  USBH_ProbeTaskStk[USBH_PROBE_TASK_STK_SIZE];
struct urb USBH_URBPool[USBH_URB_NUM_MAX];
struct usbh_task_req ReqQueuePool[USBH_REQ_QUEUE_NUM_MAX];

extern struct list_head kernel_task_list_head;

void USBH_Task(void *p_arg);
void USBH_ProbeTask(void *p_arg);
void USBH_EpQueueInit(struct ep_queue *queue);
void USBH_EpQueueDump(struct ep_queue *queue);


void USBH_Handler(struct urb *urb, struct usbh_task_req *req);
void USBH_UrbProgressCallBack(struct urb * urb, int status);
int USBH_UrbTaskReqListFlush(struct usb_host_channel *ch, struct urb* urb);
void USBH_DisconCheck(struct usb_device * dev);


int USBH_CtrlSetupReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitSetupRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitDataInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlDataOutReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitDataOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitStatusOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitStatusInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);

int USBH_BulkIntSubmitSentReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_BulkIntWaitSubmitSentRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_BulkIntSubmitRcvReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_BulkIntWaitSubmitRcvRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);

int USBH_IsocSubmitSentReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_IsocWaitSubmitSentRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_IsocSubmitRcvReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_IsocWaitSubmitRcvRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);


extern int usb_get_configuration(struct usb_device *dev);
extern void usb_destroy_configuration(struct usb_device *dev);
extern void usb_release_interface_cache(struct kref *ref);


void MachineSateFuncInit(struct usb_device *dev, u16 machine_state, u16 msg_type, MachineStateFunc func)
{
    dev->MachineStateFunc[machine_state][msg_type] = func;
}


void USBH_TmrStart(OS_TMR *tmr, u32 dly_ms, OS_TMR_CALLBACK_PTR p_callback, void *p_arg)
{
    OS_ERR err;
    CPU_SR cpu_sr;
    OS_TICK dly;


    dly = (dly_ms * OSCfg_TmrTaskRate_Hz + (1000 - 1))/ 1000;


    CPU_CRITICAL_ENTER();
    tmr->Dly = dly;
    tmr->Period = dly;
    tmr->CallbackPtr = p_callback;
    tmr->CallbackPtrArg = p_arg;
    CPU_CRITICAL_EXIT();

    OSTmrStart(tmr, &err);

    if(err != OS_ERR_NONE)
        USBH_DBG("USBH_TmrStart OSTmrStart Failed %d tmr:%p\r\n",err,tmr);
}




void usb_device_add(struct usb_device *dev, USB_OTG_CORE_HANDLE *otg_core, int coreID, char *name)
{
    OS_ERR err;
    int i;

    memset(dev, 0, sizeof(struct usb_device));

    dev->USB_OTG_Core = otg_core;
    dev->USB_OTG_Core->parent = dev;
    dev->state = USB_STATE_NOTATTACHED;
    dev->name = name;

    for(i = 0; i < USBH_CHANNEL_MAX; i++)
    {
        dev->usb_ch_pool[i].hc_num = i;
        dev->usb_ch_pool[i].dev = dev;
        dev->usb_ch_pool[i].urb = NULL;
        dev->usb_ch_pool[i].xfer_mgt = NULL;
        dev->usb_ch_pool[i].state = USBH_CHANNEL_STATE_IDLE;
        INIT_LIST_HEAD(&dev->usb_ch_pool[i].urb_task_req_list);
        INIT_LIST_HEAD(&dev->usb_ch_pool[i].hc_list);
    }

    OSMemCreate(&dev->usb_ch_mem, "usb channel mem", dev->usb_ch_pool, USBH_CHANNEL_MAX, sizeof(struct usb_host_channel), &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_Init OSMemCreate Faild %d stop!!\r\n",err);
        while(1);
    }

    OSSemCreate(&dev->lock, "usb device lock", 1, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_Init OSSemCreate usb device lock Failed %d stop!!\r\n",err);
        while(1);
    }


    USBH_EpQueueInit(&dev->nperiod_xfer_mgt.ep_queue);
    INIT_LIST_HEAD(&dev->nperiod_xfer_mgt.frame_wait_ep_list);
    INIT_LIST_HEAD(&dev->nperiod_xfer_mgt.out_hc_wait_list);
    INIT_LIST_HEAD(&dev->nperiod_xfer_mgt.out_hc_process_list);


    USBH_EpQueueInit(&dev->period_xfer_mgt.ep_queue);
    INIT_LIST_HEAD(&dev->period_xfer_mgt.frame_wait_ep_list);
    INIT_LIST_HEAD(&dev->period_xfer_mgt.out_hc_wait_list);
    INIT_LIST_HEAD(&dev->period_xfer_mgt.out_hc_process_list);


    MachineSateFuncInit(dev, USBH_CONTRL_MACHINE, USBH_CTRL_SETUP_REQ, USBH_CtrlSetupReqAction);
    MachineSateFuncInit(dev, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_SETUP_RSP, USBH_CtrlWaitSetupRspAction);
    MachineSateFuncInit(dev, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_DATA_IN_RSP, USBH_CtrlWaitDataInRspAction);
    MachineSateFuncInit(dev, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_DATA_OUT_RSP, USBH_CtrlWaitDataOutRspAction);
    MachineSateFuncInit(dev, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_STATUS_OUT_RSP, USBH_CtrlWaitStatusOutRspAction);
    MachineSateFuncInit(dev, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_STATUS_IN_RSP, USBH_CtrlWaitStatusInRspAction);

    MachineSateFuncInit(dev, USBH_BULK_INT_MACHINE, USBH_BULK_INT_SUBMIT_SENT_REQ, USBH_BulkIntSubmitSentReqAction);
    MachineSateFuncInit(dev, USBH_BULK_INT_MACHINE, USBH_BULK_INT_WAIT_SUBMIT_SENT_RSP, USBH_BulkIntWaitSubmitSentRspAction);
    MachineSateFuncInit(dev, USBH_BULK_INT_MACHINE, USBH_BULK_INT_SUBMIT_RCV_REQ, USBH_BulkIntSubmitRcvReqAction);
    MachineSateFuncInit(dev, USBH_BULK_INT_MACHINE, USBH_BULK_INT_WAIT_SUBMIT_RCV_RSP, USBH_BulkIntWaitSubmitRcvRspAction);

    MachineSateFuncInit(dev, USBH_ISOC_MACHINE, USBH_ISOC_SUBMIT_SENT_REQ, USBH_IsocSubmitSentReqAction);
    MachineSateFuncInit(dev, USBH_ISOC_MACHINE, USBH_ISOC_WAIT_SUBMIT_SENT_RSP, USBH_IsocWaitSubmitSentRspAction);
    MachineSateFuncInit(dev, USBH_ISOC_MACHINE, USBH_ISOC_SUBMIT_RCV_REQ, USBH_IsocSubmitRcvReqAction);
    MachineSateFuncInit(dev, USBH_ISOC_MACHINE, USBH_ISOC_WAIT_SUBMIT_RCV_RSP, USBH_IsocWaitSubmitRcvRspAction);


    /* Hardware Init */
    USB_OTG_BSP_Init(coreID);

    /* configure GPIO pin used for switching VBUS power */
    USB_OTG_BSP_ConfigVBUS(coreID);

    /* Start the USB OTG core */
    HCD_Init(dev->USB_OTG_Core ,coreID);

    /* Enable Interrupts */
    USB_OTG_BSP_EnableInterrupt(dev->USB_OTG_Core, coreID);


    INIT_LIST_HEAD(&dev->list);
    OSSchedLock(&err);
    list_add_tail(&dev->list, &usb_device_list_head);
    OSSchedUnlock(&err);
}

#ifdef USE_USB_OTG_FS
struct usb_device _usb_device_fs;
USB_OTG_CORE_HANDLE _usb_device_fs_otg_core;
#endif

#ifdef USE_USB_OTG_HS
struct usb_device _usb_device_hs;
USB_OTG_CORE_HANDLE _usb_device_hs_otg_core;
#endif

void usb_start()
{
#ifdef USE_USB_OTG_FS    
    usb_device_add(&_usb_device_fs, &_usb_device_fs_otg_core, USB_OTG_FS_CORE_ID, "usbfs");
#endif

#ifdef USE_USB_OTG_HS
    usb_device_add(&_usb_device_hs, &_usb_device_hs_otg_core, USB_OTG_HS_CORE_ID, "usbhs");
#endif
}

void usb_init()
{
    OS_ERR err;
    u8 i;

    USBH_TRACE("USBH_Init---->\r\n");

    OSMemCreate(&URB_Mem, "URB_Mem", USBH_URBPool, USBH_URB_NUM_MAX, sizeof(struct urb), &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_Init OSMemCreate Faild %d\r\n",err);
    }


    OSMemCreate(&ReqQueueMem, "ReqQueueMem", ReqQueuePool, USBH_REQ_QUEUE_NUM_MAX, sizeof(struct usbh_task_req), &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_Init OSMemCreate Faild %d\r\n",err);
    }


    for(i = 0; i < USBH_URB_NUM_MAX; i++)
    {
        OSTmrCreate(&USBH_URBPool[i].tmr,"",~(u32)0, ~(u32)0, OS_OPT_TMR_ONE_SHOT, NULL,NULL,&err);

        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_Init OSTmrCreate Failed %d\r\n",err);
        }
    }


    OSMutexCreate(&USBH_MutexLock, "USBH Mutex Lock", &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_Init OSMutexCreate Failed %d\r\n",err);
    }

    
    OSTaskCreate((OS_TCB     *)&USBHTaskTCB,                /* Create the start task                                    */
                 (CPU_CHAR   *)"USBH_Task",
                 (OS_TASK_PTR ) USBH_Task,
                 (void       *) 0,
                 (OS_PRIO     ) USBH_TASK_PRIO,
                 (CPU_STK    *)&USBH_TaskStk[0],
                 (CPU_STK     )(USBH_TASK_STK_SIZE / 10u),
                 (CPU_STK_SIZE) USBH_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 10,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);


    OSTaskCreate((OS_TCB     *)&USBHProbeTaskTCB,                /* Create the start task                                    */
                 (CPU_CHAR   *)"USBH Probe Task",
                 (OS_TASK_PTR ) USBH_ProbeTask,
                 (void       *) 0,
                 (OS_PRIO     ) USBH_PROBE_TASK_PRIO,
                 (CPU_STK    *)&USBH_ProbeTaskStk[0],
                 (CPU_STK     )(USBH_PROBE_TASK_STK_SIZE / 10u),
                 (CPU_STK_SIZE) USBH_PROBE_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 10,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);


    USBH_TRACE("USBH_Init<----\r\n");
}






#define SET_ADDRESS_TRIES       6
#define GET_DESCRIPTOR_TRIES    2

#define usb_sndaddr0pipe()  (PIPE_CONTROL << 30)
#define usb_rcvaddr0pipe()  ((PIPE_CONTROL << 30) | USB_DIR_IN)


/* USB device locking */
void usb_lock_device(struct usb_device *udev)
{
    OS_ERR err;

    OSSemPend(&udev->lock,0,OS_OPT_PEND_BLOCKING,0, &err);
    if(err != OS_ERR_NONE)
    {
        usb_halt(udev, "OSSemPend Failed %d\r\n",err);
    }
}

void usb_unlock_device(struct usb_device *udev)
{
    OS_ERR err;

    OSSemPost(&udev->lock, OS_OPT_POST_1, &err);
    if(err != OS_ERR_NONE)
    {
        usb_halt(udev, "OSSemPost Failed %d\r\n",err);
    }
}

void usb_set_device_state(struct usb_device *udev,
                          enum usb_device_state new_state)
{
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
    udev->state = new_state;
    CPU_CRITICAL_EXIT();
}



static int usb_port_reset(struct usb_device *dev)
{
    USB_OTG_HPRT0_TypeDef  hprt0;
    int retry = 10;
    

    HCD_ResetPort(dev->USB_OTG_Core);
    do
    {
        msleep(20);
        hprt0.d32 = USB_OTG_READ_REG32(dev->USB_OTG_Core->regs.HPRT0);
    }
    while(!hprt0.b.prtena && (--retry > 0));

    if(!retry)
        return -1;

    msleep(200);

    dev->speed = (enum usb_device_speed)HCD_GetCurrentSpeed(dev->USB_OTG_Core);

    usb_set_device_state(dev, USB_STATE_DEFAULT);

    return 0;
}

void usb_ep0_reinit(struct usb_device *udev)
{
    usb_disable_endpoint(udev, 0 + USB_DIR_IN, 1);
    usb_disable_endpoint(udev, 0 + USB_DIR_OUT, 1);
    usb_enable_endpoint(udev, &udev->ep0, 1);
}

static void update_address(struct usb_device *udev, int devnum)
{
    udev->devnum = devnum;
}

static int usb_set_address(struct usb_device *udev, int devnum)
{
    int retval;

    if (udev->state == USB_STATE_ADDRESS)
        return 0;
    if (udev->state != USB_STATE_DEFAULT)
        return -EINVAL;
    retval = usb_control_msg(udev, usb_sndaddr0pipe(),
                             USB_REQ_SET_ADDRESS, 0, devnum, 0,
                             NULL, 0, USB_CTRL_SET_TIMEOUT);
    if (retval == 0)
    {
        /* Device now using proper address. */
        update_address(udev,devnum);
        usb_set_device_state(udev, USB_STATE_ADDRESS);
        usb_ep0_reinit(udev);
    }
    return retval;
}

struct usb_host_interface *usb_altnum_to_altsetting(
    const struct usb_interface *intf,
    unsigned int altnum)
{
    int i;

    for (i = 0; i < intf->num_altsetting; i++)
    {
        if (intf->altsetting[i].desc.bAlternateSetting == altnum)
            return &intf->altsetting[i];
    }
    return NULL;
}

static void usb_release_dev(struct kref *kref)
{
    struct usb_device *udev;

    udev = container_of(kref, struct usb_device, kref);

    usb_destroy_configuration(udev);
}


struct usb_device *usb_get_dev(struct usb_device *dev)
{
    if (dev)
        kref_get(&dev->kref);
    return dev;
}



void usb_put_dev(struct usb_device *dev)
{
    if (dev)
        kref_put(&dev->kref, usb_release_dev);
}

static void usb_release_interface(struct kref *kref)
{
    struct usb_interface *intf = container_of(kref, struct usb_interface, kref);
    struct usb_interface_cache *intfc =
        altsetting_to_usb_interface_cache(intf->altsetting);

    kref_put(&intfc->ref, usb_release_interface_cache);
    kfree(intf);
}


struct usb_interface *usb_get_intf(struct usb_interface *intf)
{
    if (intf)
        kref_get(&intf->kref);
    return intf;
}


void usb_put_intf(struct usb_interface *intf)
{
    if (intf)
        kref_put(&intf->kref, usb_release_interface);
}




void usb_disconnect(struct usb_device   *udev)
{

    /* mark the device as inactive, so any further urb submissions for
     * this device (and any of its children) will fail immediately.
     * this quiesces everyting except pending urbs.
     */
    usb_set_device_state(udev, USB_STATE_NOTATTACHED);
    usb_info (udev, "USB disconnect, address %d\n", udev->devnum);

    usb_lock_device(udev);



    /* deallocate hcd/hardware state ... nuking all pending urbs and
     * cleaning up all state associated with the current configuration
     * so that the hardware is now fully quiesced.
     */
    usb_dbg (udev, "unregistering device\n");
    usb_disable_device(udev, 0);

    usb_unlock_device(udev);

    /* Unregister the device.  The device driver is responsible
     * for de-configuring the device and invoking the remove-device
     * notifier chain (used by usbfs and possibly others).
     */
//  device_del(&udev->dev);


    /* Free the device number and delete the parent's children[]
     * (or root_hub) pointer.
     */
//  release_address(udev);


    usb_put_dev(udev);
}


static int
usb_port_init (struct usb_device *udev)
{
    int         i, j, retval;
    char            *speed;
    int         devnum = udev->devnum;


    /* Reset the device; full speed may morph to high speed */
    retval = usb_port_reset(udev);
    if (retval < 0)     /* error or disconnect */
        goto fail;


    /* USB 2.0 section 5.5.3 talks about ep0 maxpacket ...
     * it's fixed size except for full speed devices.
     * For Wireless USB devices, ep0 max packet is always 512 (tho
     * reported as 0xff in the device descriptor). WUSB1.0[4.8.1].
     */
    switch (udev->speed)
    {
        case USB_SPEED_HIGH:        /* fixed at 64 */
            udev->ep0.desc.wMaxPacketSize = cpu_to_le16(64);
            break;
        case USB_SPEED_FULL:        /* 8, 16, 32, or 64 */
            /* to determine the ep0 maxpacket size, try to read
             * the device descriptor to get bMaxPacketSize0 and
             * then correct our initial guess.
             */
            udev->ep0.desc.wMaxPacketSize = cpu_to_le16(64);
            break;
        case USB_SPEED_LOW:     /* fixed at 8 */
            udev->ep0.desc.wMaxPacketSize = cpu_to_le16(8);
            break;
        default:
            goto fail;
    }


    switch (udev->speed)
    {
        case USB_SPEED_LOW:
            speed = "low";
            break;
        case USB_SPEED_FULL:
            speed = "full";
            break;
        case USB_SPEED_HIGH:
            speed = "high";
            break;
        default:
            speed = "?";
            break;
    }

    usb_info (udev,"%s %s speed USB device using address %d\r\n",
              (udev->config) ? "reset" : "new", speed, devnum);


    for (i = 0; i < GET_DESCRIPTOR_TRIES; (++i, msleep(100)))
    {

        for (j = 0; j < SET_ADDRESS_TRIES; ++j)
        {
            retval = usb_set_address(udev, devnum);
            if (retval >= 0)
                break;
            msleep(200);
        }
        if (retval < 0)
        {
            usb_err(udev,
                    "device not accepting address %d, error %d\n",
                    devnum, retval);
            goto fail;
        }

        /* cope with hardware quirkiness:
         *  - let SET_ADDRESS settle, some device hardware wants it
         *  - read ep0 maxpacket even for high and low speed,
         */
        msleep(10);



        retval = usb_get_device_descriptor(udev, 8);
        if (retval < 8)
        {
            usb_err(udev,
                    "device descriptor read/8, error %d\n",
                    retval);
            if (retval >= 0)
                retval = -EMSGSIZE;
        }
        else
        {
            retval = 0;
            break;
        }



    }
    if (retval)
        goto fail;

    i = udev->descriptor.bMaxPacketSize0 == 0xff?   /* wusb device? */
        512 : udev->descriptor.bMaxPacketSize0;
    if (le16_to_cpu(udev->ep0.desc.wMaxPacketSize) != i)
    {
        if (udev->speed != USB_SPEED_FULL ||
            !(i == 8 || i == 16 || i == 32 || i == 64))
        {
            usb_err(udev, "ep0 maxpacket = %d\n", i);
            retval = -EMSGSIZE;
            goto fail;
        }
        usb_dbg(udev, "ep0 maxpacket = %d\n", i);
        udev->ep0.desc.wMaxPacketSize = cpu_to_le16(i);
        usb_ep0_reinit(udev);
    }

    retval = usb_get_device_descriptor(udev, USB_DT_DEVICE_SIZE);
    if (retval < (signed)sizeof(udev->descriptor))
    {
        usb_err(udev, "device descriptor read/all, error %d\n",
                retval);
        if (retval >= 0)
            retval = -ENOMSG;
        goto fail;
    }

    retval = 0;

fail:
    if (retval)
    {
        update_address(udev, devnum);   /* for disconnect processing */
    }

    return retval;
}






void usb_init_dev(struct usb_device *dev)
{
    kref_init(&dev->kref);

    dev->devnum = 0;
    dev->state = USB_STATE_ATTACHED;
    dev->speed = USB_SPEED_UNKNOWN;
    memset(dev->toggle, 0, sizeof(dev->toggle));

    memset(&dev->ep0, 0, sizeof(dev->ep0));
    memset(&dev->descriptor, 0, sizeof(dev->descriptor));
    dev->config = NULL;
    dev->actconfig = NULL;
    memset(dev->ep_in, 0, sizeof(dev->ep_in));
    memset(dev->ep_out, 0, sizeof(dev->ep_out));

    memset(dev->product, 0, sizeof(dev->product));
    memset(dev->manufacturer, 0, sizeof(dev->manufacturer));
    memset(dev->serial, 0, sizeof(dev->serial));

    dev->frame_number = 0;

    INIT_LIST_HEAD(&dev->ep0.urb_list);
    dev->ep0.state = USBH_EP_IDLE;
    dev->ep0.xfer_mgt = &dev->nperiod_xfer_mgt;
    dev->ep0.desc.bLength = USB_DT_ENDPOINT_SIZE;
    dev->ep0.desc.bDescriptorType = USB_DT_ENDPOINT;
    /* ep0 maxpacket comes later, from device descriptor */
    usb_enable_endpoint(dev, &dev->ep0, 0);
}


static int usb_configure_device(struct usb_device *udev)
{
    int err = 0;

    if (udev->config == NULL)
    {
        err = usb_get_configuration(udev);
        if (err < 0)
        {
            usb_err(udev, "can't read configurations, error %d\n",
                    err);
            goto fail;
        }
    }

    /* read the standard strings and cache them if present */
    usb_string(udev, udev->descriptor.iProduct, udev->product, sizeof(udev->product));
    usb_string(udev, udev->descriptor.iManufacturer, udev->manufacturer, sizeof(udev->manufacturer));
    usb_string(udev, udev->descriptor.iSerialNumber, udev->serial, sizeof(udev->serial));
fail:
    return err;
}

static void announce_device(struct usb_device *udev)
{
    usb_info(udev, "New USB device found, idVendor=%04x, idProduct=%04x\n",
             le16_to_cpu(udev->descriptor.idVendor),
             le16_to_cpu(udev->descriptor.idProduct));
    usb_info(udev,
             "New USB device strings: Mfr=%d, Product=%d, SerialNumber=%d\n",
             udev->descriptor.iManufacturer,
             udev->descriptor.iProduct,
             udev->descriptor.iSerialNumber);
    usb_info(udev, "Product: %s\r\n", udev->product);
    usb_info(udev, "Manufacturer: %s\r\n", udev->manufacturer);
    usb_info(udev, "SerialNumber: %s\r\n", udev->serial);
}

int usb_new_device(struct usb_device *udev)
{
    int err;


    err = usb_configure_device(udev);   /* detect & probe dev/intfs */
    if (err < 0)
        goto fail;

    /* Tell the world! */
    announce_device(udev);


    /* Register the device.  The device driver is responsible
     * for configuring the device and invoking the add-device
     * notifier chain (used by usbfs and possibly others).
     */
    usb_lock_device(udev);

    err = usb_set_configuration(udev, udev->config[0].desc.bConfigurationValue);
    if (err)
    {
        usb_err(udev, "can't set config #%d, error %d\n",0, err);
        /* This need not be fatal.  The user can try to
        * set other configurations. */
    }

    usb_unlock_device(udev);


fail:
    return err;
//  usb_set_device_state(udev, USB_STATE_NOTATTACHED);
//  return err;
}


int usb_register(struct usb_driver *driver)
{
    OS_ERR err;

    INIT_LIST_HEAD(&driver->list);

    OSSchedLock(&err);
    list_add_tail(&driver->list, &usb_driver_list_head);
    OSSchedUnlock(&err);

    return 0;
//  return usb_register_driver(driver, THIS_MODULE, KBUILD_MODNAME);
}

void usb_deregister(struct usb_driver *driver)
{
    OS_ERR err;

    USBH_TRACE("deregistering interface driver %s\n",
               driver->name);

    OSSchedLock(&err);
    list_del(&driver->list);
    OSSchedUnlock(&err);
}

/* returns 0 if no match, 1 if match */
int usb_match_device(struct usb_device *dev, const struct usb_device_id *id)
{
    if ((id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
        id->idVendor != le16_to_cpu(dev->descriptor.idVendor))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_PRODUCT) &&
        id->idProduct != le16_to_cpu(dev->descriptor.idProduct))
        return 0;
#if 0
    /* No need to test id->bcdDevice_lo != 0, since 0 is never
       greater than any unsigned number. */
    if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_LO) &&
        (id->bcdDevice_lo > le16_to_cpu(dev->descriptor.bcdDevice)))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_HI) &&
        (id->bcdDevice_hi < le16_to_cpu(dev->descriptor.bcdDevice)))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_CLASS) &&
        (id->bDeviceClass != dev->descriptor.bDeviceClass))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_SUBCLASS) &&
        (id->bDeviceSubClass != dev->descriptor.bDeviceSubClass))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_PROTOCOL) &&
        (id->bDeviceProtocol != dev->descriptor.bDeviceProtocol))
        return 0;
#endif
    return 1;
}

int usb_match_one_id(struct usb_interface *interface,
                     const struct usb_device_id *id)
{
    struct usb_host_interface *intf;
    struct usb_device *dev;

    /* proc_connectinfo in devio.c may call us with id == NULL. */
    if (id == NULL)
        return 0;

    intf = interface->cur_altsetting;
    dev = interface_to_usbdev(interface);

    if (!usb_match_device(dev, id))
        return 0;

    /* The interface class, subclass, and protocol should never be
     * checked for a match if the device class is Vendor Specific,
     * unless the match record specifies the Vendor ID. */
    if (dev->descriptor.bDeviceClass == USB_CLASS_VENDOR_SPEC &&
        !(id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
        (id->match_flags & (USB_DEVICE_ID_MATCH_INT_CLASS |
                            USB_DEVICE_ID_MATCH_INT_SUBCLASS |
                            USB_DEVICE_ID_MATCH_INT_PROTOCOL)))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_CLASS) &&
        (id->bInterfaceClass != intf->desc.bInterfaceClass))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_SUBCLASS) &&
        (id->bInterfaceSubClass != intf->desc.bInterfaceSubClass))
        return 0;

    if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_PROTOCOL) &&
        (id->bInterfaceProtocol != intf->desc.bInterfaceProtocol))
        return 0;

    return 1;
}

const struct usb_device_id *usb_match_id(struct usb_interface *interface,
        const struct usb_device_id *id)
{
    /* proc_connectinfo in devio.c may call us with id == NULL. */
    if (id == NULL)
        return NULL;

    /* It is important to check that id->driver_info is nonzero,
       since an entry that is all zeroes except for a nonzero
       id->driver_info is the way to create an entry that
       indicates that the driver want to examine every
       device and interface. */
    for (; id->idVendor || id->idProduct || /*id->bDeviceClass ||*/
         id->bInterfaceClass /*|| id->driver_info*/; id++)
    {
        if (usb_match_one_id(interface, id))
            return id;
    }

    return NULL;
}





void usb_do_hotplug(struct usb_interface *interface, int is_connect)
{
    struct usb_hotplug *hotplug;
    const struct usb_device_id *id;

    list_for_each_entry(hotplug, &usb_hotplug_list_head, list)
    {
        id = usb_match_id(interface, hotplug->id);
        if(id && hotplug->call_back)
        {
            hotplug->call_back(interface, hotplug->arg, is_connect);
        }
    }
}


void usb_hotplug_add(struct usb_hotplug *hotplug)
{
    list_add_tail(&hotplug->list, &usb_hotplug_list_head);
}



struct usb_host_channel *USBH_AllocChannel(struct usb_device *dev)
{
    struct usb_host_channel *ch;
    OS_ERR err;

    ch = OSMemGet(&dev->usb_ch_mem, &err);
    if((err != OS_ERR_NONE) || (ch == NULL))
    {
        USBH_DBG("USBH_Sched alloc channel failed %d\r\n",err);
        return NULL;
    }

    if(!list_empty(&ch->urb_task_req_list))
    {
        USBH_DBG("USBH_AllocChannel urb_task_req_list is not empty stop!! ch:%p\r\n",ch);
        while(1);
    }

    kref_init(&ch->kref);

    INIT_LIST_HEAD(&ch->hc_list);
    ch->state = USBH_CHANNEL_STATE_PROCESS;
    ch->dev = dev;

    return ch;
}


void USBH_DestroyChannel(struct kref *kref)
{
    struct usb_host_channel *ch = container_of(kref, struct usb_host_channel, kref);
    OS_ERR err;
    CPU_SR cpu_sr;


    CPU_CRITICAL_ENTER();
    USBH_UrbTaskReqListFlush(ch, ch->urb);
    OSMemPut(&ch->dev->usb_ch_mem, ch, &err);
    ch->state = USBH_CHANNEL_STATE_IDLE;
    ch->xfer_mgt = NULL;
    ch->urb = NULL;
    CPU_CRITICAL_EXIT();
}


void USBH_FreeChannel(struct usb_device *dev, struct usb_host_channel *ch)
{
    kref_put(&ch->kref, USBH_DestroyChannel);
}



void USBH_InitChannel  (struct usb_device *dev,
                        struct usb_host_channel *ch,
                        struct usb_endpoint_descriptor *epd)
{
    memset(&dev->USB_OTG_Core->host.hc[ch->hc_num], 0, sizeof(USB_OTG_HC));
    dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num = usb_endpoint_num(epd);
    dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = usb_endpoint_dir_in(epd);
    dev->USB_OTG_Core->host.hc[ch->hc_num].dev_addr = usb_pipedevice(ch->urb->pipe);
    dev->USB_OTG_Core->host.hc[ch->hc_num].ep_type = usb_endpoint_type(epd);
    dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet = le16_to_cpu(epd->wMaxPacketSize) & 0x07ff;
    dev->USB_OTG_Core->host.hc[ch->hc_num].speed = dev->speed;
    dev->USB_OTG_Core->host.hc[ch->hc_num].multi_count = 1;

    if((dev->speed == USB_SPEED_HIGH)
       && !usb_endpoint_dir_in(epd))
    {
        //only for high speed device and out endpoint of control or bulk .
        dev->USB_OTG_Core->host.hc[ch->hc_num].do_ping = 1;
    }

    USB_OTG_HC_Init(dev->USB_OTG_Core, ch->hc_num) ;
}





void USBH_Sched(struct usb_device *dev)
{
    OS_ERR err;
    struct usb_host_channel *ch;
    struct usb_host_endpoint *ep;
    struct urb *urb;
    struct usbh_task_req *task_req;
    struct urbs_xfer_mgt *xfer_mgt;
    CPU_SR cpu_sr;


    while(1)
    {

        if(dev->period_xfer_mgt.ep_queue.num)
        {
            xfer_mgt = &dev->period_xfer_mgt;
        }
        else if(dev->nperiod_xfer_mgt.ep_queue.num)
        {
            xfer_mgt = &dev->nperiod_xfer_mgt;
        }
        else
        {
            break;
        }


        if((ch = USBH_AllocChannel(dev)) == NULL)
        {
            USBH_TRACE("USBH_Sched alloc channel failed \r\n");
            return;
        }

        ch->xfer_mgt = xfer_mgt;

        task_req = OSMemGet(&ReqQueueMem,&err);
        if((err != OS_ERR_NONE) || (task_req == NULL))
        {
            USBH_DBG("usb_submit_urb OSMemGet ReqQueueMem Faild %d\r\n",err);
            USBH_FreeChannel(dev, ch);
            return;
        }

        if(!list_empty(&ch->urb_task_req_list))
        {
            usb_halt(dev, " urb_task_req_list is not emmpty stop!! ch:%p\r\n",ch);
        }

        OSSchedLock(&err);

        if((ep = USBH_EpDequeue(&xfer_mgt->ep_queue)) == NULL)
        {
            usb_halt(dev, " USBH_EpDequeue ep is NULL stop!!\r\n");
        }

        if(list_empty(&ep->urb_list))
        {
            usb_halt(dev, " can't get urb from ep stop!! ch:%p\r\n",ch);
        }

        urb = list_first_entry(&ep->urb_list, struct urb, urb_list);


        if(urb->priv_state != USBH_URB_WAIT_PROGRESS)
        {
            usb_halt(dev, " urb:%p->priv_state:%d != USBH_URB_WAIT_PROGRESS stop!!\r\n",urb,urb->priv_state);
        }

        if(ep->state != USBH_EP_WAIT_PROGRESS)
        {
            usb_halt(dev, " ep:%p->state:%d != USBH_EP_WAIT_PROGRESS stop!!\r\n",ep,ep->state);
        }



        ep->state = USBH_EP_PROGRESS;
        urb->priv_state = USBH_URB_PROGRESS;
        urb->ch = ch;
        ch->urb = urb;

        OSSchedUnlock(&err);

        USBH_InitChannel(dev, ch, &ep->desc);

        if(usb_endpoint_dir_out(&ep->desc))
        {
            CPU_CRITICAL_ENTER();
            list_add_tail(&ch->hc_list, &xfer_mgt->out_hc_wait_list);
            if(!list_empty(&xfer_mgt->out_hc_process_list))
            {
                /*usb host controll is already processing a out channel, add it to the out channel waiting list*/
                CPU_CRITICAL_EXIT();
                OSMemPut(&ReqQueueMem,task_req, &err);

                continue;
            }

            ch = list_first_entry(&xfer_mgt->out_hc_wait_list, struct usb_host_channel, hc_list);
            list_move_tail(&ch->hc_list, &xfer_mgt->out_hc_process_list);
            CPU_CRITICAL_EXIT();
        }



        task_req->req_type = USBH_TASK_REQ_CH_PROCESS;
        task_req->req_msg = USBH_TASK_REQ_FROM_TASK;
        INIT_LIST_HEAD(&task_req->list);
        CPU_CRITICAL_ENTER();
        list_add_tail(&task_req->list, &ch->urb_task_req_list);
        CPU_CRITICAL_EXIT();

        usb_get_channel(ch);
        OSTaskQPost(&USBHTaskTCB, ch, USBH_PROCESS_CHANNEL, OS_OPT_POST_FIFO, &err);
        if(err != OS_ERR_NONE)
        {
            usb_dbg(dev, "USBH_Sched OSTaskQPost Failed %d\r\n",err);
            usb_put_channel(ch);
            if(usb_endpoint_dir_out(&ep->desc))
            {
                CPU_CRITICAL_ENTER();
                list_del(&task_req->list);
                OSMemPut(&ReqQueueMem,task_req, &err);
                list_move(&ch->hc_list, &xfer_mgt->out_hc_wait_list);
                CPU_CRITICAL_EXIT();
            }
            else
            {
                CPU_CRITICAL_ENTER();
                list_del(&task_req->list);
                CPU_CRITICAL_EXIT();
                OSMemPut(&ReqQueueMem,task_req, &err);
                USBH_FreeChannel(dev, ch);
                urb->ch = NULL;
                urb->priv_state = USBH_URB_WAIT_PROGRESS;

                OSSchedLock(&err);
                ep->state = USBH_EP_WAIT_PROGRESS;
                USBH_EpEnqueue(&xfer_mgt->ep_queue, ep);
                OSSchedUnlock(&err);
            }
            return;
        }
    }
    u8 is_period;


    for(is_period = 0; is_period <= 1; is_period++)
    {
        xfer_mgt = is_period?&dev->period_xfer_mgt:&dev->nperiod_xfer_mgt;

        CPU_CRITICAL_ENTER();
        if(!list_empty(&xfer_mgt->out_hc_wait_list)
           && list_empty(&xfer_mgt->out_hc_process_list))
        {
            if(!list_empty(&xfer_mgt->out_hc_wait_list))
            {
                task_req = OSMemGet(&ReqQueueMem,&err);
                if((err != OS_ERR_NONE) || (task_req == NULL))
                {
                    CPU_CRITICAL_EXIT();
                    usb_dbg(dev,"usb_submit_urb OSMemGet ReqQueueMem Faild %d\r\n",err);
                    return;
                }
                ch = list_first_entry(&xfer_mgt->out_hc_wait_list, struct usb_host_channel, hc_list);
                list_move_tail(&ch->hc_list, &xfer_mgt->out_hc_process_list);


                task_req->req_type = USBH_TASK_REQ_CH_PROCESS;
                task_req->req_msg = USBH_TASK_REQ_FROM_TASK;
                INIT_LIST_HEAD(&task_req->list);
                list_add_tail(&task_req->list, &ch->urb_task_req_list);
                CPU_CRITICAL_EXIT();

                usb_get_channel(ch);
                OSTaskQPost(&USBHTaskTCB, ch, USBH_PROCESS_CHANNEL, OS_OPT_POST_FIFO, &err);
                if(err != OS_ERR_NONE)
                {
                    usb_put_channel(ch);
                    CPU_CRITICAL_ENTER();
                    list_del(&task_req->list);
                    OSMemPut(&ReqQueueMem,task_req, &err);
                    list_move(&ch->hc_list, &xfer_mgt->out_hc_wait_list);
                    CPU_CRITICAL_EXIT();

                    return;
                }
            }
            else
            {
                CPU_CRITICAL_EXIT();
            }
        }
        else
        {
            CPU_CRITICAL_EXIT();
        }

    }
}


void USBH_DisconCheck(struct usb_device * dev)
{
    int i, all_dev_removed = 1;
    OS_ERR err;


    //let other task run for a while to realse memory.
    OSTimeDlyHMSM(0u, 0u, 0u, 200u, OS_OPT_TIME_HMSM_STRICT,&err);


    if(dev->usb_ch_mem.NbrFree != USBH_CHANNEL_MAX)
    {
        USBH_DBG("dev->usb_ch_mem.NbrFree:%d !=USBH_CHANNEL_MAX\r\n",dev->usb_ch_mem.NbrFree);

    }
    for ( i = 0 ; i < USBH_CHANNEL_MAX; i++ )
    {
        if(dev->usb_ch_pool[i].state != USBH_CHANNEL_STATE_IDLE)
        {
            USBH_DBG("ch:%d state != USBH_CHANNEL_STATE_IDLE\r\n",i);
        }
    }

    if(!list_empty(&dev->nperiod_xfer_mgt.out_hc_process_list))
        USBH_DBG("nperiod_xfer_mg out_hc_process_list is not empty \r\n");

    if(!list_empty(&dev->period_xfer_mgt.out_hc_process_list))
        USBH_DBG("period_xfer_mgt out_hc_process_list is not empty \r\n");


    if(dev->nperiod_xfer_mgt.ep_queue.num != 0)
    {
        USBH_EpQueueDump(&dev->nperiod_xfer_mgt.ep_queue);
        USBH_DBG("nperiod_xfer_mgt.num:%d != 0 \r\n",dev->nperiod_xfer_mgt.ep_queue.num);
    }


    if(dev->period_xfer_mgt.ep_queue.num != 0)
    {
        USBH_EpQueueDump(&dev->period_xfer_mgt.ep_queue);
        USBH_DBG("period_xfer_mgt.num:%d != 0 \r\n",dev->period_xfer_mgt.ep_queue.num);
    }
    
    
    list_for_each_entry(dev, &usb_device_list_head, list)
    {
        if(dev->state != USB_STATE_NOTATTACHED)
            all_dev_removed = 0;
    }

    if(all_dev_removed)
    {
        if(URB_Mem.NbrFree != USBH_URB_NUM_MAX)
        {
            void *p;
            struct urb *urb;

            USBH_DBG("URB_Mem.NbrFree:%d !=USBH_URB_NUM_MAX\r\n",URB_Mem.NbrFree);


            for(i = 0; i < USBH_URB_NUM_MAX; i++)
            {

                for(p = URB_Mem.FreeListPtr; p != NULL; p = *(void **)p)
                {
                    urb = (struct urb *)p;
                    if(&USBH_URBPool[i] == urb)//find
                        break;
                }
                if(p == NULL)//do not find
                    USBH_DBG("index:%d urb:%p state:%d kref:%d\r\n",i,&USBH_URBPool[i], USBH_URBPool[i].priv_state, USBH_URBPool[i].kref.refcount.counter);
            }
        }

        if(ReqQueueMem.NbrFree != USBH_REQ_QUEUE_NUM_MAX)
        {
            void *p;
            struct usbh_task_req *task_req;


            USBH_DBG("ReqQueueMem.NbrFree:%d !=USBH_REQ_QUEUE_NUM_MAX\r\n",ReqQueueMem.NbrFree);


            for(i = 0; i < USBH_REQ_QUEUE_NUM_MAX; i++)
            {

                for(p = ReqQueueMem.FreeListPtr; p != NULL; p = *(void **)p)
                {
                    task_req = (struct usbh_task_req *)p;
                    if(&ReqQueuePool[i] == task_req)//find
                        break;
                }
                if(p == NULL)//do not find
                    USBH_DBG("-->req_type:%d req_msg:%d\r\n",ReqQueuePool[i].req_type, ReqQueuePool[i].req_msg);
            }
        }

        if(memory_used)
        {
            USBH_DBG("memory_used:%d !=0 \r\n",memory_used);
//             mem_check();
        }

        printf("memory used:%d  max_used:%d  \r\n",memory_used,memory_used_max);
//       printf("lwip_stats.mem.used:%d  max:%d \r\n ",lwip_stats.mem.used,lwip_stats.mem.max);
    }
}


/*
 * Store the current frame number in uhci->frame_number if the controller
 * is runnning.  Expand from 11 bits (of which we use only 10) to a
 * full-sized integer.
 *
 * Like many other parts of the driver, this code relies on being polled
 * more than once per second as long as the controller is running.
 */
void usb_get_current_frame_number(struct usb_device *dev)
{
    CPU_SR cpu_sr;

    if (dev->state >= USB_STATE_POWERED)
    {
        unsigned delta;

        CPU_CRITICAL_ENTER();
        delta = (USB_OTG_READ_REG32(&dev->USB_OTG_Core->regs.HREGS->HFNUM)&0xffff - dev->frame_number) &
                (0x4000 - 1);
        dev->frame_number += delta;
        CPU_CRITICAL_EXIT();
    }
}


void usb_add_to_frame_wait_list(struct usb_device *dev, struct urb *urb)
{
    USB_OTG_GINTMSK_TypeDef  intmsk;
    OS_ERR err;
    struct usb_host_channel *ch;
    struct usb_host_endpoint *ep;
    CPU_SR cpu_sr;


    ch = urb->ch;
    ep = urb->ep;

    OSSchedLock(&err);

    if(ep->state != USBH_EP_PROGRESS)
    {
        usb_halt(dev, " ep->state != USBH_EP_PROGRESS stop!! ep:%p type:%d\r\n",ep, usb_endpoint_type(&ep->desc));
    }

    urb->priv_state = USBH_URB_WAIT_PROGRESS;
    ep->state = USBH_EP_WAIT_FRAME_TIMEOUT;

    USBH_UrbTaskReqListFlush(ch, urb);
    OSSchedUnlock(&err);

    CPU_CRITICAL_ENTER();
    if(list_empty(&ep->xfer_mgt->frame_wait_ep_list))
    {
        intmsk.d32 = 0;
        intmsk.b.sofintr    = 1;
        USB_OTG_MODIFY_REG32(&dev->USB_OTG_Core->regs.GREGS->GINTMSK, 0, intmsk.d32);
    }
    list_add_tail(&ep->frame_wait_ep_list, &ep->xfer_mgt->frame_wait_ep_list);
    CPU_CRITICAL_EXIT();

    USBH_FreeChannel(dev, ch);
    USBH_Sched(dev);
}

void USBH_ConnectProbeCallback(void *p_tmr, void *p_arg)
{
    u32 con_sts;
    struct usb_device *dev;
    OS_ERR err;
    CPU_SR cpu_sr;

    list_for_each_entry(dev, &usb_device_list_head, list)
    {
        struct usb_host_channel *ch = NULL;

        CPU_CRITICAL_ENTER();
        con_sts = dev->USB_OTG_Core->host.ConnSts;
        CPU_CRITICAL_EXIT();

        usb_get_current_frame_number(dev);

        if((con_sts == dev->USB_OTG_Core->host.PreConnSts) && (con_sts == USB_STATE_NOTATTACHED))
        {
            dev->state = USB_STATE_NOTATTACHED;
        }

        if(dev->state == USB_STATE_NOTATTACHED)
            dev->USB_OTG_Core->host.disconnet_time++;
        else
            dev->USB_OTG_Core->host.disconnet_time = 0;


        CPU_CRITICAL_ENTER();
        if((dev->state == USB_STATE_NOTATTACHED)
           && dev->USB_OTG_Core->host.disconnet_time*1000/OSCfg_TmrTaskRate_Hz >= 400 //ms
           && (dev->usb_ch_mem.NbrFree != USBH_CHANNEL_MAX || !list_empty(&dev->period_xfer_mgt.frame_wait_ep_list)))
        {
            int i;

            //fix bug
            for ( i = 0 ; i < USBH_CHANNEL_MAX ; i++ )
            {
                ch = &dev->usb_ch_pool[i];
                if(ch->state == USBH_CHANNEL_STATE_PROCESS)
                {
                    if(dev->USB_OTG_Core->host.disconnet_time*1000/OSCfg_TmrTaskRate_Hz > 800) //800ms
                    {
                        struct usbh_task_req *task_req;
                        
                        usb_dbg(dev, "---->%s ch:%d do USB_OTG_HC_Halt force\r\n",__func__, i);
                        USB_OTG_HC_Halt(dev->USB_OTG_Core, ch->hc_num);
                        
                        task_req = OSMemGet(&ReqQueueMem,&err);
                        if(err != OS_ERR_NONE)
                        {
                            usb_halt(dev, "OSMemGet ReqQueueMem Faild %d\r\n",err);
                        }
                        task_req->req_type = USBH_TASK_REQ_CH_PROCESS;
                        task_req->req_msg = USBH_TASK_REQ_FROM_TASK;
                        INIT_LIST_HEAD(&task_req->list);
                        list_add_tail(&task_req->list, &ch->urb_task_req_list);
                        dev->USB_OTG_Core->host.URB_State[ch->hc_num] = URB_ERROR; 
                        
                        usb_get_channel(ch);
                        OSTaskQPost(&USBHTaskTCB, ch, USBH_PROCESS_CHANNEL, OS_OPT_POST_FIFO, &err);
                        if(err != OS_ERR_NONE)
                        {
                            usb_put_channel(ch);
                            /*if failed, this channel may not be processed forever in task*/
                            list_del(&task_req->list);
                            OSMemPut(&ReqQueueMem, task_req, &err);
                        }
                    }
                }
            }


            struct usb_host_endpoint *ep,*next;

            list_for_each_entry_safe(ep, next, &dev->period_xfer_mgt.frame_wait_ep_list, frame_wait_ep_list)
            {
                if(list_empty(&ep->urb_list))
                    usb_halt(dev, "ep->urb_list is empty\r\n");

                OSTaskQPost(&USBHTaskTCB, ep, USBH_PROCESS_ISOC_SOF, OS_OPT_POST_FIFO, &err);
                if(err != OS_ERR_NONE)
                {
                    usb_dbg(dev, "%s OSTaskQPost failed:%d\r\n",__func__, err);
                    break;
                }

                list_del_init(&ep->frame_wait_ep_list);
            }
        }
        CPU_CRITICAL_EXIT();


        if((con_sts == dev->USB_OTG_Core->host.PreConnSts)
           && (con_sts != dev->USB_OTG_Core->host.pre_post_state))
        {
            OSSchedLock(&err);
            OSQPost(&USBH_ProbeQ, (void *)dev, con_sts, OS_OPT_POST_FIFO, &err);
            if(err!= OS_ERR_NONE)
            {
                USBH_DBG("USBH_ConnectProbeCallback  OSQPost failed:%d\r\n",err);
            }
            else
            {
                dev->USB_OTG_Core->host.pre_post_state = con_sts;
            }
            OSSchedUnlock(&err);

        }
        dev->USB_OTG_Core->host.PreConnSts = con_sts;
    }

}



void USBH_ProbeTask(void *p_arg)
{
    struct usb_device *udev;
    OS_ERR err;
    OS_TICK dly;
    

    dly = (200 * OSCfg_TmrTaskRate_Hz + (1000 - 1))/ 1000;// 200ms

    OSQCreate(&USBH_ProbeQ, "USBH_ProbeQ", 5, &err);
    OSTmrCreate(&USBH_ProbeTmr, "USBH_ProbeTmr",0, dly, OS_OPT_TMR_PERIODIC, USBH_ConnectProbeCallback, NULL,&err);
    OSTmrStart(&USBH_ProbeTmr,&err);


    while(1)
    {
        OS_MSG_SIZE con_sts;


        udev = OSQPend(&USBH_ProbeQ, 0, OS_OPT_PEND_BLOCKING, &con_sts, NULL, &err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_ProbeTask OSQPend Failed:%d\r\n",err);
            continue;
        }


        if(con_sts == USB_STATE_ATTACHED)
        {
            int status;

            USBH_TRACE("USBH_ProbeTask USBH_DEV_ATTACHED \r\n");
            USBH_TRACE("memory used:%d  max_used:%d  \r\n",memory_used,memory_used_max);

             
            usb_init_dev(udev);

            usb_set_device_state(udev, USB_STATE_POWERED);

            update_address(udev, USBH_DEVICE_ADDRESS);
            
            /*wait more than 100ms*/
            msleep(200);
            /* reset and get descriptor */
            status = usb_port_init(udev);
            if (status)
            {
                continue;
            }

            status = usb_new_device(udev);
            if (status)
            {
                continue;
            }
        }
        else if(con_sts == USB_STATE_NOTATTACHED)
        {

            USBH_TRACE("USBH_ProbeTask USBH_DEV_DISCONNECTED \r\n");

            //Let other threads run for a while.
            msleep(100);

            usb_disconnect(udev);

            for(;;)
            {
                //Wait until no one hold the usb device reference count.
                if(!atomic_read(&udev->kref.refcount))
                    break;

                msleep(200);
            }

            //check memory
            USBH_DisconCheck(udev);
            
        }
    }
}



struct usbh_task_req *USBH_GetTaskReq(struct list_head *head)
{
    struct usbh_task_req *req,*next;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();

    list_for_each_entry_safe(req, next, head, list)
    {
        list_del(&req->list);
        CPU_CRITICAL_EXIT();
        return req;
    }

    CPU_CRITICAL_EXIT();
    return NULL;
}





void USBH_Task(void *p_arg)
{
    int ret;
    OS_ERR err;
    struct usbh_task_req *req;
    OS_MSG_SIZE  proc_type;
    struct usb_host_channel *ch;
    void *data;


    while(1)
    {
        data = OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &proc_type, NULL, &err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_Task OSTaskSemPend Failed %d\r\n",err);
            continue;
        }

        if(proc_type == USBH_PROCESS_CHANNEL)
        {
            ch = (struct usb_host_channel *)data;

            ret = usb_put_channel(ch);
            if(ret)
            {
                if(!list_empty(&ch->urb_task_req_list))
                {
                    usb_halt(ch->dev, " ch:%d->urb_task_req_list is not empty stop!!\r\n",ch->hc_num);
                }
                //Channel is destroyed really.
                //Note:This channel may still exist in USBH Task Queue despite USBH_FreeChannel() is called, since kref count is not zero.

                usb_dbg(ch->dev, "%s channel:%d is destroyed really, call USBH_Sched\r\n",__func__, ch->hc_num);
                USBH_Sched(ch->dev);
                continue;
            }

            if((req = USBH_GetTaskReq(&ch->urb_task_req_list)) != NULL)
            {
                USBH_Handler(ch->urb, req);

                OSMemPut(&ReqQueueMem, req, &err);
            }
        }
        else if(proc_type == USBH_PROCESS_SCHED)
        {
            struct usb_device *dev;

            dev = (struct usb_device *)data;
            USBH_Sched(dev);

            usb_put_dev(dev);
        }
        else if(proc_type == USBH_PROCESS_ISOC_SOF)
        {
            struct usb_host_endpoint *ep;

            ep = (struct usb_host_endpoint *)data;

            OSSchedLock(&err);

            if(list_empty(&ep->urb_list))
                usb_halt((struct usb_device *)NULL, "endpoint:%d urb_list is empty\r\n",ep->desc.bEndpointAddress);

            if(ep->state != USBH_EP_WAIT_FRAME_TIMEOUT)
                usb_halt((struct usb_device *)NULL, "endpoint:%d state:%d != USBH_EP_WAIT_FRAME_TIMEOUT\r\n",ep->desc.bEndpointAddress, ep->state);

            USBH_EpEnqueue(&ep->xfer_mgt->ep_queue, ep);

            OSSchedUnlock(&err);

            USBH_Sched(list_first_entry(&ep->urb_list, struct urb, urb_list)->dev);
        }
    }

}




void USBH_EpQueueInit(struct ep_queue *queue)
{
    queue->ep_head = NULL;
    queue->ep_tail = NULL;
    queue->num = 0;
    queue->max_num_record = 0;
}




struct usb_host_endpoint *USBH_EpDequeue(struct ep_queue *queue)
{
    struct usb_host_endpoint * ep;


    if(!queue->ep_head)
        return NULL;

    if(queue->ep_head == queue->ep_tail)
    {
        ep = queue->ep_head;
        queue->ep_head = NULL;
        queue->ep_tail = NULL;
        queue->num--;
    }
    else
    {
        ep = queue->ep_head;
        queue->ep_head = queue->ep_head->next;
        queue->num--;
    }

    return ep;
}


void USBH_EpQueueDump(struct ep_queue *queue)
{
    struct usb_host_endpoint *temp_ep;
    struct urb *urb;

    USBH_DBG("USBH_UrbQueueDump:\r\n");

    USBH_DBG("queue->num:%d \r\n",queue->num);

    for(temp_ep = queue->ep_head; temp_ep != NULL; temp_ep = temp_ep->next)
    {
        USBH_DBG("ep:%p addr:0x%x is in queue\r\n",temp_ep, temp_ep->desc.bEndpointAddress);
        list_for_each_entry(urb, &temp_ep->urb_list, urb_list)
        {
            USBH_DBG("  urb:%p is in the endpoint:0x%x\r\n",urb,temp_ep->desc.bEndpointAddress);
        }

    }

}


void USBH_EpEnqueue(struct ep_queue *queue, struct usb_host_endpoint *ep)
{

    ep->state = USBH_EP_WAIT_PROGRESS;

    if(!queue->ep_head)
    {
        ep->next = NULL;
        queue->ep_head = ep;
        queue->ep_tail = ep;
        queue->num ++;
    }
    else
    {
        ep->next = NULL;
        queue->ep_tail->next = ep;
        queue->ep_tail = ep;
        queue->num ++;
    }
    if(queue->num > queue->max_num_record)
    {
        queue->max_num_record = queue->num;
    }

}



void USBH_Handler(struct urb *urb, struct usbh_task_req *req)
{

    // USBH_TRACE("USBH_NperiodHandler req_type:%d  req_msg:%d MachineState:%d MsgType:%d\r\n",req_type,req_msg,urb->MachineState,urb->MsgType);

    if(!urb)
        usb_halt(urb->dev, "urb is NULL\r\n");

    if(urb->priv_state == USBH_URB_IDLE)
    {
        usb_halt(urb->dev, "urb->priv_state==USBH_URB_IDLE should not occur urb:%p req_type:%d req_msg:%d\r\n",urb,req->req_type,req->req_msg);
    }
    else if(urb->priv_state != USBH_URB_PROGRESS)
    {
        usb_halt(urb->dev, "Unknown urb->priv_state:%d urb:%p stop!!\r\n",urb->priv_state,urb);
    }


    if(req->req_type != USBH_TASK_REQ_CH_PROCESS)
        usb_halt(urb->dev, "req_type:%d == USBH_TASK_REQ_CH_PROCESS",req->req_type);


    if(urb->unlinked)
    {
        usb_dbg(urb->dev, "USBH_Handler usb is unlinked:%d do call_back\r\n",urb->unlinked);
        usb_dbg(urb->dev, "MachineState:%d MsgType:%d urb_state:%d\r\n",urb->MachineState,urb->MsgType,HCD_GetURB_State(urb->dev->USB_OTG_Core, urb->ch->hc_num));
        USBH_UrbProgressCallBack(urb, urb->unlinked);
    }
    else
        (urb->dev->MachineStateFunc[urb->MachineState][urb->MsgType])(urb->dev, urb, &urb->MsgType);

}




void USBH_ClearUrb(struct urb* urb)
{
    if(!urb->ep)
    {
        usb_halt(urb->dev, "USBH_ClearUrb urb->ep is NULL urb:%p\r\n",urb);
    }

    urb->priv_state = USBH_URB_IDLE;

    list_del_init(&urb->urb_list);

    urb->ep = NULL;
    urb->ch = NULL;
}


int USBH_FreeUrbBuf(struct urb* urb)
{
    int is_out;
    int xfertype;

    if(urb->transfer_buffer_length == 0)
        return -1;


    xfertype = usb_endpoint_type(&urb->ep->desc);
    if (xfertype == USB_ENDPOINT_XFER_CONTROL)
    {
        struct usb_ctrlrequest *setup =
            (struct usb_ctrlrequest *) urb->setup_packet;

        is_out = !(setup->bRequestType & USB_DIR_IN) ||
                 !setup->wLength;
    }
    else
    {
        is_out = usb_endpoint_dir_out(&urb->ep->desc);
    }


    if(is_out && !(urb->transfer_flags & URB_FREE_BUFFER))
    {
        kfree(urb->transfer_buffer);
    }

    return 0;
}


int USBH_UrbTaskReqListFlush(struct usb_host_channel *ch, struct urb* urb)
{
    OS_ERR err;
    struct usbh_task_req *req, *next;
    u16 num = 0;
    CPU_SR cpu_sr;


    CPU_CRITICAL_ENTER();

    list_for_each_entry_safe(req, next, &ch->urb_task_req_list, list)
    {
        num++;
        list_del(&req->list);

        OSMemPut(&ReqQueueMem, req, &err);
    }

    CPU_CRITICAL_EXIT();

    if(num)
        usb_dbg((struct usb_device *)NULL, "%s flush task req ch_num:%d num:%d  urb:%p\r\n",__func__,ch->hc_num,num,urb);

    return num;
}



void USBH_UrbProgressCallBack(struct urb * urb, int status)
{
    OS_ERR err;
    struct usb_host_channel *ch;
    struct usb_host_endpoint *ep;


    USBH_FreeUrbBuf(urb);
    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);

    ch = urb->ch;
    ep = urb->ep;

    OSSchedLock(&err);

    if(ep->state != USBH_EP_PROGRESS)
    {
        USBH_DBG("USBH_UrbProgressCallBack ep->state != USBH_EP_PROGRESS stop!! ep:%p type:%d\r\n",ep, usb_endpoint_type(&ep->desc));
        while(1);
    }

    ep->state = USBH_EP_IDLE;

    USBH_ClearUrb(urb);
    USBH_UrbTaskReqListFlush(ch, urb);
    if(!list_empty(&ep->urb_list))
        USBH_EpEnqueue(&ep->xfer_mgt->ep_queue, ep);

    //ch->hc_list is intialized in USBH_AllocChannel(), it is no matter whether the channel is associate with (struct urbs_xfer_mgt) channel list .
    //just remove from list.
    list_del_init(&ch->hc_list);


    OSSchedUnlock(&err);


    if (urb->unlinked)
        status = urb->unlinked;
    else if ((urb->transfer_flags & URB_SHORT_NOT_OK) &&
             urb->actual_length < urb->transfer_buffer_length &&
             !status)
        status = -EREMOTEIO;

    /* pass ownership to the completion handler */
    urb->status = status;
    urb->complete (urb);
    atomic_dec (&urb->use_count);

    USBH_FreeChannel(urb->dev, ch);
    USBH_Sched(urb->dev);

    if (atomic_read(&urb->reject))
        wake_up_kill_urb (&usb_kill_urb_queue);
    usb_put_urb (urb);

}


//=======================================CONTROL TRANSFER======================

void USBH_ReqTimeOut(void *p_tmr, void *p_arg)
{
    struct urb *urb;
    USB_OTG_HCTSIZn_TypeDef  hctsiz;
    USB_OTG_HCINTn_TypeDef     hcint;
    USB_OTG_HCINTMSK_TypeDef  hcintmsk;


    urb = (struct urb *)p_arg;

    usb_dbg(urb->dev, "USBH_ReqTimeOut urb:%p endpoint_type:%d  dir_in:%d actual_length:%d buffer_length:%d\r\n",urb,usb_endpoint_type(&urb->ep->desc),usb_endpoint_dir_in(&urb->ep->desc)
            ,urb->actual_length, urb->transfer_buffer_length);

    //for debug
    hctsiz.d32 = USB_OTG_READ_REG32(&urb->dev->USB_OTG_Core->regs.HC_REGS[urb->ch->hc_num]->HCTSIZ);
    printf("hcint.b.ack remain_length:%d  hctsiz.b.pktcnt:%d\r\n",hctsiz.b.xfersize,hctsiz.b.pktcnt);

    hcint.d32 = USB_OTG_READ_REG32(&urb->dev->USB_OTG_Core->regs.HC_REGS[urb->ch->hc_num]->HCINT);
    hcintmsk.d32 = USB_OTG_READ_REG32(&urb->dev->USB_OTG_Core->regs.HC_REGS[urb->ch->hc_num]->HCINTMSK);
    printf("hcint.d32:0x%x  hcintmsk.d32:0x%x\r\n",hcint.d32,hcintmsk.d32);



    usb_hcd_unlink_urb(urb, -ETIME);
}


int USBH_CtrlSetupReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
//    dev->USB_OTG_Core->host.hc[urb->ep->hc_ctrl_num_out].ep_is_in = 0;

    if(dev->USB_OTG_Core->host.hc[urb->ch->hc_num].ep_is_in != 0)
    {
        dev->USB_OTG_Core->host.hc[urb->ch->hc_num].ep_is_in = 0;
        USB_OTG_HC_Halt(dev->USB_OTG_Core, urb->ch->hc_num);
        USB_OTG_HC_Init(dev->USB_OTG_Core, urb->ch->hc_num);
    }

    dev->USB_OTG_Core->host.hc[urb->ch->hc_num].data_pid = HC_PID_SETUP;
    dev->USB_OTG_Core->host.hc[urb->ch->hc_num].xfer_buff = (u8 *)&urb->setup_request;
    dev->USB_OTG_Core->host.hc[urb->ch->hc_num].xfer_len = 8; //USBH_SETUP_PKT_SIZE

    HCD_SubmitRequest (dev->USB_OTG_Core, urb->ch);

    *msg_type = USBH_CTRL_WAIT_SETUP_RSP;
    return 0;
}


int USBH_CtrlWaitSetupRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    static u16 timeout = 0;
    u8  direction;


    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;
    
    /* case SETUP packet sent successfully */
    if(URB_Status == URB_DONE)
    {
        direction  = (urb->setup_request.bRequestType & USB_DIR_IN);

        /* check if there is a data stage */
        if (urb->setup_request.wLength != 0 )
        {
            timeout = DATA_STAGE_TIMEOUT;
            if (direction == USB_DIR_IN)
            {
                /* Start DATA in transfer */
                usb_settoggle(dev, dev->USB_OTG_Core->host.hc[urb->ch->hc_num].ep_num, 0, 1);

                /* Data Direction is IN */
                USBH_CtlReceiveData(dev,
                                    urb->transfer_buffer,
                                    urb->transfer_buffer_length,
                                    urb->ch);

                *msg_type = USBH_CTRL_WAIT_DATA_IN_RSP;
            }
            else
            {
                /* Start DATA out transfer */
                usb_settoggle(dev, dev->USB_OTG_Core->host.hc[urb->ch->hc_num].ep_num, 1, 1);

                *msg_type = USBH_CTRL_DATA_OUT_REQ;
                USBH_CtrlDataOutReqAction(dev, urb, msg_type);

            }
        }
        /* No DATA stage */
        else
        {
            timeout = NODATA_STAGE_TIMEOUT;

            /* If there is No Data Transfer Stage */
            if (direction == USB_DIR_IN)
            {
                USBH_CtlSendData (dev,
                                  0,
                                  0,
                                  urb->ch);

                *msg_type = USBH_CTRL_WAIT_STATUS_OUT_RSP;
                /* Data Direction is IN */
            }
            else
            {

                /* Send 0 bytes out packet */
                USBH_CtlReceiveData (dev,
                                     0,
                                     0,
                                     urb->ch);

                *msg_type = USBH_CTRL_WAIT_STATUS_IN_RSP;

                /* Data Direction is OUT */
            }
        }
        /* Set the delay timer to enable timeout for data stage completion */
        USBH_TmrStart(&urb->tmr, timeout, USBH_ReqTimeOut, urb);
    }
    else if(URB_Status == URB_ERROR)
    {
//        usb_dbg(dev,"USBH_CtrlWaitSetupRspAction URB_ERROR!!! error_count:%d urb:%p\r\n",urb->error_count, urb);
        if(++urb->error_count < 3)
        {
            USBH_CtrlSetupReqAction(dev, urb, msg_type);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        } 
    }
    else
    {
        USBH_DBG("USBH_CtrlWaitSetupRspAction Unknow URB_Status:%d stop!! urb:%p  ch_num:%d\r\n",URB_Status,urb,urb->ch->hc_num);
        while(1);
    }


    return 0;
}




int USBH_CtrlWaitDataInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;

//    urb->actual_length = dev->host.hc[urb->ep->hc_ctrl_num_in].xfer_count;
    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;


    urb->actual_length += dev->USB_OTG_Core->host.XferCnt[urb->ch->hc_num];
    /* check is DATA packet transfered successfully */
    if  (URB_Status == URB_DONE)
    {
        USBH_CtlSendData (dev,
                          0,
                          0,
                          urb->ch);
        *msg_type = USBH_CTRL_WAIT_STATUS_OUT_RSP;
    }

    /* manage error cases*/
    else if  (URB_Status == URB_STALL)
    {
        usb_dbg(dev,"USBH_CtrlWaitDataInRspAction URB_STALL!!! urb:%p\r\n",urb);
        USBH_UrbProgressCallBack(urb, -EPIPE);
    }
    else if (URB_Status == URB_NAK) //add by LKQ
    {
        /* Device NAK, reactive transfer */
//        usb_dbg(dev,"USBH_CtrlWaitDataInRspAction receive URB_NAK buffer_length:%d actual_length:%d\r\n",urb->transfer_buffer_length,urb->actual_length);
        USBH_CtlReceiveData(dev,
                            (u8 *)urb->transfer_buffer + urb->actual_length,
                            urb->transfer_buffer_length - urb->actual_length,
                            urb->ch);
    }
    else if (URB_Status == URB_ERROR)
    {
        /* Device error */
        usb_dbg(dev,"USBH_CtrlWaitDataInRspAction URB_ERROR!!! error_count:%d urb:%p\r\n",urb->error_count, urb);
        if(++urb->error_count < 3)
        {
            USBH_CtlReceiveData(dev,
                                (u8 *)urb->transfer_buffer + urb->actual_length,
                                urb->transfer_buffer_length - urb->actual_length,
                                urb->ch);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        }  
    }    
    else
    {
        USBH_DBG("USBH_CtrlWaitDataInRspAction Unknow URB_Status:%d stop!! urb:%p hc_num:%d\r\n",URB_Status,urb,urb->ch->hc_num);
        while(1);
    }
    return 0;
}




int USBH_CtrlDataOutReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    int remain_length;
    int write_length;

    remain_length = urb->transfer_buffer_length - urb->actual_length;

    write_length = USBH_CtlSendData (dev,
                                     (u8 *)urb->transfer_buffer + urb->actual_length,
                                     remain_length,
                                     urb->ch);

    if(write_length >= 0)
    {
        *msg_type= USBH_CTRL_WAIT_DATA_OUT_RSP;
    }

    return write_length;

}


int USBH_CtrlWaitDataOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;

//    urb->actual_length = dev->host.hc[urb->ep->hc_ctrl_num_out].xfer_count;
    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;

    if  (URB_Status == URB_DONE)
    {
        urb->actual_length += dev->USB_OTG_Core->host.XferCnt[urb->ch->hc_num];
        /* If the Setup Pkt is sent successful, then change the state */

        USBH_CtlReceiveData (dev,
                             0,
                             0,
                             urb->ch);
        *msg_type = USBH_CTRL_WAIT_STATUS_IN_RSP;
    }

    /* handle error cases */
    else if  (URB_Status == URB_STALL)
    {
        usb_dbg(dev,"USBH_CtrlWaitDataOutRspAction URB_STALL!!! urb:%p\r\n",urb);
        USBH_UrbProgressCallBack(urb, -EPIPE);
    }
    else if  (URB_Status == URB_NOTREADY)
    {
        /* Start DATA out transfer (only one DATA packet)*/
        *msg_type = USBH_CTRL_DATA_OUT_REQ;
        USBH_CtrlDataOutReqAction(dev, urb, msg_type);
    }
    else if (URB_Status == URB_ERROR)
    {
        /* device error */
        usb_dbg(dev,"USBH_CtrlWaitDataOutRspAction URB_ERROR!!! error_count:%d urb:%p\r\n",urb->error_count, urb);
        if(++urb->error_count < 3)
        {
            *msg_type = USBH_CTRL_DATA_OUT_REQ;
            USBH_CtrlDataOutReqAction(dev, urb, msg_type);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        }  
    }
    else
    {
        USBH_DBG("USBH_CtrlWaitDataOutRspAction Unknow URB_Status:%d stop!! urb:%p num:%d\r\n",URB_Status,urb,urb->ch->hc_num);
        while(1);
    }
    return 0;

}


int USBH_CtrlWaitStatusOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;

    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        usb_halt(dev, "OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    if  ( URB_Status == URB_DONE)
    {
//        USBH_TRACE("USBH_CtrlWaitStatusOutRspAction StatusOutRsp success urb:%p\r\n",urb);
        /* Control transfers completed*/
        USBH_UrbProgressCallBack(urb, 0);
    }
    else if(URB_Status == URB_STALL)
    {
        usb_dbg(dev,"USBH_CtrlWaitStatusOutRspAction URB_STALL!!! urb:%p\r\n",urb);
        USBH_UrbProgressCallBack(urb, -EPIPE);
    }
    else if(URB_Status == URB_NOTREADY)  //add by LKQ, resent.
    {
        USBH_CtlSendData (dev,
                          0,
                          0,
                          urb->ch);
    }
    else if (URB_Status == URB_ERROR)
    {
        usb_dbg(dev,"USBH_CtrlWaitStatusOutRspAction URB_ERROR!!! error_count:%d urb:%p\r\n",urb->error_count, urb);
        if(++urb->error_count < 3)
        {
            USBH_CtlSendData (dev,
                              0,
                              0,
                              urb->ch);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        }  
    }    
    else
    {
        USBH_DBG("USBH_CtrlWaitStatusOutRspAction Unknow URB_Status:%d stop!! urb:%p hc_num:%d\r\n",URB_Status,urb,urb->ch->hc_num);
        while(1);

    }
    return 0;

}



int USBH_CtrlWaitStatusInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;

    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        USBH_DBG("USBH_CtrlWaitStatusInRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    if  ( URB_Status == URB_DONE)
    {
//        USBH_TRACE("USBH_CtrlWaitStatusInRspAction StatusInRsp success urb:%p\r\n",urb);
        /* Control transfers completed*/
        USBH_UrbProgressCallBack(urb, 0);
    }
    else if(URB_Status == URB_STALL)
    {
        usb_dbg(dev,"USBH_CtrlWaitStatusInRspAction URB_STALL!!! urb:%p\r\n",urb);
        USBH_UrbProgressCallBack(urb, -EPIPE);
    }
    else if (URB_Status == URB_NAK)  //add by LKQ
    {
        USBH_CtlReceiveData (dev,
                             0,
                             0,
                             urb->ch);
    }
    else if (URB_Status == URB_ERROR)
    {
        usb_dbg(dev,"USBH_CtrlWaitStatusInRspAction URB_ERROR!!! error_count:%d urb:%p\r\n",urb->error_count, urb);
        if(++urb->error_count < 3)
        {
            USBH_CtlReceiveData (dev,
                                 0,
                                 0,
                                 urb->ch);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        }
    }    
    else
    {
        USBH_DBG("USBH_CtrlWaitStatusInRspAction Unknow URB_Status:%d stop!! urb:%p  num:%d\r\n",URB_Status,urb,urb->ch->hc_num);
        while(1);
    }
    return 0;
}




//=======================================BULK AND INTTREPURT TRANSFER======================




int USBH_BulkIntSubmitSentReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    int write_length = 0;
    int remain_length;
    

    USBH_TmrStart(&urb->tmr, USBH_BULK_SENT_TIMEOUT, USBH_ReqTimeOut, urb);

    remain_length = urb->transfer_buffer_length - urb->actual_length;

    if(usb_endpoint_type(&urb->ep->desc) == USB_ENDPOINT_XFER_BULK)
    {
        write_length = USBH_BulkIntSendData (dev,
                                             (u8 *)urb->transfer_buffer + urb->actual_length,
                                             remain_length,
                                             urb->ch);

        *msg_type= USBH_BULK_INT_WAIT_SUBMIT_SENT_RSP;
    }
    else
    {
        if(urb->start_frame - 1 <= dev->frame_number)
        {
            int max = le16_to_cpu(urb->ep->desc.wMaxPacketSize);

            /* "high bandwidth" mode, 1-3 packets/uframe? */
            if (dev->speed == USB_SPEED_HIGH)
            {
                int mult = 1 + ((max >> 11) & 0x03);
                max &= 0x07ff;
                max *= mult;
            }

            if(remain_length > max)
            {
                write_length = USBH_BulkIntSendData (dev,
                                                     (u8 *)urb->transfer_buffer + urb->actual_length,
                                                     max,
                                                     urb->ch);
            }
            else
            {
                write_length = USBH_BulkIntSendData (dev,
                                                     (u8 *)urb->transfer_buffer + urb->actual_length,
                                                     remain_length,
                                                     urb->ch);
            }

            *msg_type= USBH_BULK_INT_WAIT_SUBMIT_SENT_RSP;
        }
        else
        {
            usb_add_to_frame_wait_list(dev, urb);
            *msg_type= USBH_BULK_INT_SUBMIT_SENT_REQ;
        }
    }
//    usb_dbg(dev, "USBH_BulkIntSubmitSentReqAction remain_length:%d write_length:%d urb:%p\r\n",remain_length,write_length,urb);


    return write_length;

}


int USBH_BulkIntWaitSubmitSentRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;
    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        usb_halt(dev, "USBH_BulkWaitSubmitSentRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;

    if  (URB_Status == URB_DONE)
    {
        urb->actual_length += dev->USB_OTG_Core->host.XferCnt[urb->ch->hc_num];

//        usb_dbg(dev,"USBH_BulkIntWaitSubmitSentRspAction actual_length:%d buffer_length:%d urb:%p\r\n",urb->actual_length,urb->transfer_buffer_length,urb);
        if(urb->actual_length < urb->transfer_buffer_length)
        {
            USBH_BulkIntSubmitSentReqAction(dev, urb, msg_type);
        }
        else
        {
//            usb_dbg(dev, "USBH_BulkWaitSubmitSentRspAction do call back\r\n\r\n");
//            USBH_TRACE("USBH_BulkWaitSubmitSentRspAction success sent,usb:%p actual_length:%d,buffer_length:%d\r\n",
//                                                urb,urb->actual_length,urb->transfer_buffer_length);
            USBH_UrbProgressCallBack(urb, 0);
        }
    }
    /* handle error cases */
    else if  (URB_Status == URB_STALL)
    {
        usb_dbg(dev,"USBH_BulkWaitSubmitSentRspAction URB_STALL!!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                urb,urb->actual_length,urb->transfer_buffer_length);

        USBH_UrbProgressCallBack(urb, -EPIPE);
    }
    else if  (URB_Status == URB_NOTREADY)
    {
        /* Device not ready, restart DATA out transfer*/

        USBH_BulkIntSubmitSentReqAction(dev, urb, msg_type); 
    }
    else if (URB_Status == URB_ERROR)
    {
        usb_dbg(dev,"USBH_BulkWaitSubmitSentRspAction URB_ERROR!!! error_count:%d ,urb:%p actual_length:%d,buffer_length:%d\r\n",urb->error_count,
                urb,urb->actual_length,urb->transfer_buffer_length);
        /* device error */ 
        if(++urb->error_count < 3)
        {
            USBH_BulkIntSubmitSentReqAction(dev, urb, msg_type);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        }  
    }
    else
    {
        USBH_DBG("USBH_BulkWaitSubmitSentRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }
    return 0;
}




int USBH_BulkIntSubmitRcvReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    int remain_length;

//    USBH_TRACE("USBH_BulkIntSubmitRcvReqAction  urb:%p actual_length:%d buffer_length:%d \r\n", urb,urb->actual_length,urb->transfer_buffer_length);

    remain_length = urb->transfer_buffer_length - urb->actual_length;


    if(usb_endpoint_type(&urb->ep->desc) == USB_ENDPOINT_XFER_BULK)
    {
        USBH_BulkIntReceiveData (dev,
                                 (u8 *)urb->transfer_buffer + urb->actual_length,
                                 remain_length,
                                 urb->ch);

        *msg_type= USBH_BULK_INT_WAIT_SUBMIT_RCV_RSP;
    }
    else
    {
        //USB_ENDPOINT_XFER_INT
        if(urb->start_frame - 1 <= dev->frame_number)
        {
            int max = le16_to_cpu(urb->ep->desc.wMaxPacketSize);

            /* "high bandwidth" mode, 1-3 packets/uframe? */
            if (dev->speed == USB_SPEED_HIGH)
            {
                int mult = 1 + ((max >> 11) & 0x03);
                max &= 0x07ff;
                max *= mult;
            }

            if(remain_length > max)
            {
                USBH_BulkIntReceiveData (dev,
                                         (u8 *)urb->transfer_buffer + urb->actual_length,
                                         max,
                                         urb->ch);
            }
            else
            {
                USBH_BulkIntReceiveData (dev,
                                         (u8 *)urb->transfer_buffer + urb->actual_length,
                                         remain_length,
                                         urb->ch);
            }

            *msg_type= USBH_BULK_INT_WAIT_SUBMIT_RCV_RSP;
        }
        else
        {
            usb_add_to_frame_wait_list(dev, urb);
            *msg_type= USBH_BULK_INT_SUBMIT_RCV_REQ;
        }
    }


    return 0;
}

//receive
int USBH_BulkIntWaitSubmitRcvRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    OS_ERR err;
    URB_STATE URB_Status = URB_IDLE;


    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);
    if(URB_Status != URB_ERROR)
        urb->error_count = 0;


    urb->actual_length += dev->USB_OTG_Core->host.XferCnt[urb->ch->hc_num];

    if  (URB_Status == URB_DONE)
    {
//        USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction success ,usb:%p actual_length:%d,buffer_length:%d\r\n\r\n",
//                                    urb,urb->actual_length,urb->transfer_buffer_length);

        USBH_UrbProgressCallBack(urb, 0);
    }
    else if(URB_Status == URB_NAK)
    {
        if(urb->actual_length)
        {
            usb_dbg(dev, "USBH_BulkWaitSubmitRcvRspAction NAK recv,but urb->actual_length:%d >0,buffer_length:%d  do complete\r\n",
                    urb->actual_length,urb->transfer_buffer_length);

            USBH_UrbProgressCallBack(urb, 0);
        }
        else
        {
            if((usb_endpoint_type(&urb->ep->desc) == USB_ENDPOINT_XFER_INT)
               && !(urb->start_frame - 1 <= dev->frame_number))
            {
                usb_add_to_frame_wait_list(dev, urb);
                *msg_type= USBH_BULK_INT_SUBMIT_RCV_REQ;
            }
            else
            {
                /*reative transfer*/
                struct usb_host_channel *ch;
                struct usb_host_endpoint *ep;

                //            USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction urb:%p URB_Status == URB_NAK do USBH_UrbEnqueue\r\n",urb);
                ch = urb->ch;
                ep = urb->ep;

                OSSchedLock(&err);

                if(ep->state != USBH_EP_PROGRESS)
                {
                    usb_halt(dev, " ep->state != USBH_EP_PROGRESS stop!! ep:%p type:%d\r\n",ep, usb_endpoint_type(&ep->desc));
                }

                urb->priv_state = USBH_URB_WAIT_PROGRESS;
                ep->state = USBH_EP_IDLE;
                *msg_type= USBH_BULK_INT_SUBMIT_RCV_REQ;

                USBH_UrbTaskReqListFlush(ch, urb);

                USBH_EpEnqueue(&ep->xfer_mgt->ep_queue, ep);

                OSSchedUnlock(&err);

                USBH_FreeChannel(urb->dev, ch);
                USBH_Sched(urb->dev);
            }

        }

    }
    else if  (URB_Status == URB_STALL)
    {
        usb_dbg(dev,"USBH_BulkWaitSubmitRcvRspAction URB_STALL!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                urb,urb->actual_length,urb->transfer_buffer_length);

        USBH_UrbProgressCallBack(urb, -EPIPE);
    }
    else if (URB_Status == URB_ERROR)
    {
        usb_dbg(dev,"USBH_BulkWaitSubmitRcvRspAction URB_ERROR!!! error_count:%d urb:%p actual_length:%d,buffer_length:%d\r\n",urb->error_count,
                urb,urb->actual_length,urb->transfer_buffer_length);
        /* device error */ 
        if(++urb->error_count < 3)
        {
            USBH_BulkIntSubmitRcvReqAction(dev, urb, msg_type);
        }
        else
        {
            USBH_UrbProgressCallBack(urb, -EILSEQ);
        } 
    }
    else
    {
        usb_halt(dev, "USBH_BulkWaitSubmitRcvRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }
    return 0;
}


 

int USBH_IsocSubmitSentReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    int write_length = 0;

    usb_get_current_frame_number(dev);

    if(urb->start_frame - 1 <= dev->frame_number)
    {
//        int mult = 1;
//
//        if(dev->speed == USB_SPEED_HIGH)
//        {
//            int max = le16_to_cpu(urb->ep->desc.wMaxPacketSize)&0x07ff;
//
//            mult = (urb->iso_frame_desc[urb->iso_frame_index].length + max - 1)/max;
//        }

        //usb host will send isoc data in next frame.
        write_length = USBH_IsocSendData (dev,
                                          (u8 *)urb->transfer_buffer + urb->iso_frame_desc[urb->iso_frame_index].offset,
                                          urb->iso_frame_desc[urb->iso_frame_index].length,
                                          urb->ch);

        if(urb->iso_frame_index == 0 && urb->isoc_submited)
            urb->isoc_submited(urb);

        *msg_type= USBH_ISOC_WAIT_SUBMIT_SENT_RSP;
    }
    else
    {
        usb_add_to_frame_wait_list(dev, urb);
        *msg_type= USBH_ISOC_SUBMIT_SENT_REQ;
    }

    return write_length;
}



int USBH_IsocWaitSubmitSentRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        usb_halt(dev, "USBH_BulkWaitSubmitSentRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);

    if  (URB_Status == URB_DONE)
    {
        if(urb->iso_frame_index == urb->number_of_packets - 1)
        {
            USBH_UrbProgressCallBack(urb, 0);
        }
        else
        {
            urb->iso_frame_index++;
            USBH_IsocSubmitSentReqAction(dev, urb, msg_type);
        }
    }
    /* handle error cases */
    else if (URB_Status == URB_ERROR)
    {
        usb_dbg(dev,"USBH_BulkWaitSubmitSentRspAction URB_ERROR!!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                urb,urb->actual_length,urb->transfer_buffer_length);
        USBH_UrbProgressCallBack(urb, -EILSEQ);
        /* device error */
    }
    else
    {
        USBH_DBG("USBH_BulkWaitSubmitSentRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }
    return 0;
}




int USBH_IsocSubmitRcvReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    usb_get_current_frame_number(dev);

    if(urb->start_frame - 1 <= dev->frame_number)
    {
        //usb host will receive isoc data in next frame.
        USBH_IsocReceiveData (dev,
                              (u8 *)urb->transfer_buffer + urb->iso_frame_desc[urb->iso_frame_index].offset,
                              urb->iso_frame_desc[urb->iso_frame_index].length,
                              urb->ch);

        if(urb->iso_frame_index == 0 && urb->isoc_submited)
            urb->isoc_submited(urb);

        *msg_type= USBH_ISOC_WAIT_SUBMIT_RCV_RSP;
    }
    else
    {
        usb_add_to_frame_wait_list(dev, urb);
        *msg_type= USBH_ISOC_SUBMIT_RCV_REQ;
    }

    return 0;
}


int USBH_IsocWaitSubmitRcvRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        usb_halt(dev, "USBH_BulkWaitSubmitSentRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    URB_Status = HCD_GetURB_State(dev->USB_OTG_Core, urb->ch->hc_num);

    if  (URB_Status == URB_DONE)
    {
        if(urb->iso_frame_index == urb->number_of_packets - 1)
        {
            USBH_UrbProgressCallBack(urb, 0);
        }
        else
        {
            urb->iso_frame_index++;
            USBH_IsocSubmitRcvReqAction(dev, urb, msg_type);
        }
    }
    /* handle error cases */
    else if (URB_Status == URB_ERROR)
    {
        usb_dbg(dev,"USBH_BulkWaitSubmitSentRspAction URB_ERROR!!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                urb,urb->actual_length,urb->transfer_buffer_length);
        USBH_UrbProgressCallBack(urb, -EILSEQ);
        /* device error */
    }
    else
    {
        USBH_DBG("USBH_BulkWaitSubmitSentRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }
    return 0;
}


