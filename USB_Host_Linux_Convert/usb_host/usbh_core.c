#define USBH_DEBUG_LEVEL USBH_DEBUG_ERROR//USBH_DEBUG_TRACE
#include "os.h"
#include "usbh_core.h"
#include "usbh_config.h"
#include "usbh_linux.h"
#include "errno.h"
#include "usb_bsp.h"
#include "usb_hcd.h"
#include "usb_hcd_int.h"
#include "usbh_def.h"
#include "usbh_ioreq.h"
#include "usbh_stdreq.h"
#include "memory.h"
#include "kthread.h"
#include "misc_cvt.h"
#include "net_cvt.h"
#include "tasklet.h"
#include "includes.h"
#include "string.h"

#ifdef USE_LWIP_MALLOC
#include "lwip/stats.h"
#endif

#define USBH_DEV_ATTACHED           0
#define USBH_DEV_DISCONNECTED      1
#define USBH_DEVICE_ADDRESS_DEFAULT                     0
#define USBH_DEVICE_ADDRESS                             1

struct usb_device _usb_device;
struct usb_interface _usb_interface;
struct usb_host_config   _usb_config;  
struct usb_host_interface _usb_altsetting;
OS_TMR USBH_ProbeTmr;
OS_SEM USBH_ProbeSem;
volatile int USBH_ConState = USBH_DEV_DISCONNECTED;


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
void USBH_UrbQueueInit(struct urb_queue *queue);
void USBH_NperiodHandler(struct usb_device *dev, struct urb *urb, int req_type,int req_msg,void *req_data);
void USBH_NperiodUrbProgressCallBack(struct usb_device * dev,struct urb * urb);



int USBH_CtrlSetupReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitSetupRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitDataInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitDataOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitStatusOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_CtrlWaitStatusInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);

int USBH_BulkSubmitSentReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_BulkWaitSubmitSentRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_BulkSubmitRcvReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_BulkWaitSubmitRcvRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);

int USBH_ErrorHcHaltReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);
int USBH_ErrorWaitHcHaltRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type);


void MachineSateFuncInit(struct usb_device *dev, u16 machine_state, u16 msg_type, MachineStateFunc func)
{
    dev->MachineStateFunc[machine_state][msg_type] = func;
}


//void USBH_TmrCreate(OS_TMR *tmr)
//{
//    OS_ERR err;
//    OSTmrCreate(tmr,"",~(u32)0, ~(u32)0,, OS_OPT_TMR_ONE_SHOT, NULL,NULL,&err);

//    if(err != OS_ERR_NONE)
//        USBH_DBG("USBH_TmrInit OSTmrCreate Failed %d\r\n",err);

//}



//delay_ms = OSCfg_TickRate_Hz*dly
void USBH_TmrStart(OS_TMR *tmr, u32 dly_ms, OS_TMR_CALLBACK_PTR p_callback, void *p_arg)
{
    OS_ERR err;
    CPU_SR cpu_sr;
    OS_TICK dly;

//    tick_rate = OSCfg_TickRate_Hz;
//    ticks     = (OSCfg_TickRate_Hz * ((OS_TICK)dly_ms + (OS_TICK)500u / OSCfg_TickRate_Hz)) / (OS_TICK)1000u;    

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


void USBH_Init()
{
    OS_ERR err;
    u8 i;

    USBH_TRACE("USBH_Init---->\r\n");
//     usbh_mem_init();
    
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
        USBH_URBPool[i].dev = &_usb_device;

        OSTmrCreate(&USBH_URBPool[i].tmr,"",~(u32)0, ~(u32)0, OS_OPT_TMR_ONE_SHOT, NULL,NULL,&err);

        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_Init OSTmrCreate Failed %d\r\n",err);
        }

        OSSemCreate(&USBH_URBPool[i].sem,"",0,&err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_Init OSSemCreate Failed %d\r\n",err);
        }
    }
    _usb_interface.altsetting = _usb_interface.cur_altsetting = &_usb_altsetting;
    _usb_interface.altsetting->endpoint = &_usb_device.ep[1]; //ep0 is removed
    _usb_interface.usb_dev = &_usb_device;
    
    _usb_device.intf = &_usb_interface;
    _usb_device.config = &_usb_config;
    _usb_device.pUSBHTaskTCB = &USBHTaskTCB;
    _usb_device.usb_nperiod_xfer_state= USBH_XFER_STATE_IDLE;
    _usb_device.CurProessUrb = NULL;
//     _usb_device.usb_nperiod_xfer_state= USBH_XFER_STATE_IDLE;
    _usb_device.usb_nperiod_task_req_num = 0;
//     _usb_device.usb_nperiod_task_req_num = 0;
    for(i = 0; i < HC_MAX; i++)
    {
        _usb_device.urb[i] = NULL;
    }


    USBH_UrbQueueInit(&_usb_device.urb_nperiod_queue);
//     USBH_UrbQueueInit(&_usb_device.urb_nperiod_queue);


    MachineSateFuncInit(&_usb_device, USBH_CONTRL_MACHINE, USBH_CTRL_SETUP_REQ, USBH_CtrlSetupReqAction);
    MachineSateFuncInit(&_usb_device, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_SETUP_RSP, USBH_CtrlWaitSetupRspAction);

    MachineSateFuncInit(&_usb_device, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_DATA_IN_RSP, USBH_CtrlWaitDataInRspAction);
    MachineSateFuncInit(&_usb_device, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_DATA_OUT_RSP, USBH_CtrlWaitDataOutRspAction);

    MachineSateFuncInit(&_usb_device, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_STATUS_OUT_RSP, USBH_CtrlWaitStatusOutRspAction);
    MachineSateFuncInit(&_usb_device, USBH_CONTRL_MACHINE, USBH_CTRL_WAIT_STATUS_IN_RSP, USBH_CtrlWaitStatusInRspAction);

    MachineSateFuncInit(&_usb_device, USBH_BULK_MACHINE, USBH_BULK_SUBMIT_SENT_REQ, USBH_BulkSubmitSentReqAction);
    MachineSateFuncInit(&_usb_device, USBH_BULK_MACHINE, USBH_BULK_WAIT_SUBMIT_SENT_RSP, USBH_BulkWaitSubmitSentRspAction);

    MachineSateFuncInit(&_usb_device, USBH_BULK_MACHINE, USBH_BULK_SUBMIT_RCV_REQ, USBH_BulkSubmitRcvReqAction);
    MachineSateFuncInit(&_usb_device, USBH_BULK_MACHINE, USBH_BULK_WAIT_SUBMIT_RCV_RSP, USBH_BulkWaitSubmitRcvRspAction);

    MachineSateFuncInit(&_usb_device, USBH_ERROR_MACHINE, USBH_ERROR_HC_HALT_REQ, USBH_ErrorHcHaltReqAction);
    MachineSateFuncInit(&_usb_device, USBH_ERROR_MACHINE, USBH_ERROR_WAIT_HC_HALT_RSP, USBH_ErrorWaitHcHaltRspAction);




    /* Hardware Init */
    USB_OTG_BSP_Init(&_usb_device.USB_OTG_Core);

    /* configure GPIO pin used for switching VBUS power */
    USB_OTG_BSP_ConfigVBUS(0);


    /* Host de-initializations */
//      USBH_DeInit(pdev, phost);



    /* Start the USB OTG core */
    HCD_Init(&_usb_device.USB_OTG_Core ,
#ifdef USE_USB_OTG_FS
             USB_OTG_FS_CORE_ID
#else
             USB_OTG_HS_CORE_ID
#endif
            );


    tasklet_task_init();

    OSTaskCreate((OS_TCB     *)&USBHTaskTCB,                /* Create the start task                                    */
                 (CPU_CHAR   *)"USBH_Task",
                 (OS_TASK_PTR ) USBH_Task,
                 (void       *) &_usb_device,
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
                 (void       *) &_usb_device,
                 (OS_PRIO     ) USBH_PROBE_TASK_PRIO,
                 (CPU_STK    *)&USBH_ProbeTaskStk[0],
                 (CPU_STK     )(USBH_PROBE_TASK_STK_SIZE / 10u),
                 (CPU_STK_SIZE) USBH_PROBE_TASK_STK_SIZE,
                 (OS_MSG_QTY  ) 10,
                 (OS_TICK     ) 0,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);


    /* Enable Interrupts */
    USB_OTG_BSP_EnableInterrupt(&_usb_device.USB_OTG_Core);
    USBH_TRACE("USBH_Init<----\r\n");
}

int USBH_EP_Free(struct usb_device *dev, struct usb_host_endpoint *ep)
{
    int xfertype;
 
    xfertype = usb_endpoint_type(&ep->desc);

    if(xfertype == USB_ENDPOINT_XFER_CONTROL)
    {
        USB_OTG_HC_Halt(&dev->USB_OTG_Core, ep->hc_ctrl_num_out);
        USB_OTG_HC_Halt(&dev->USB_OTG_Core, ep->hc_ctrl_num_in);        
        USBH_Free_Channel(&dev->USB_OTG_Core, ep->hc_ctrl_num_out);
        USBH_Free_Channel(&dev->USB_OTG_Core, ep->hc_ctrl_num_in);
    }
    else if(xfertype == USB_ENDPOINT_XFER_BULK)
    {  
        USB_OTG_HC_Halt(&dev->USB_OTG_Core, ep->hc_num);         
        USBH_Free_Channel(&dev->USB_OTG_Core, ep->hc_num);       
    }
    else
    {
        USBH_DBG("USBH_EP_Init unspport endpoint type:%d\r\n",xfertype);
    }    

    return 1;
}

int USBH_EP_Init(struct usb_device *dev, struct usb_host_endpoint *ep)
{
    int xfertype;
    u8 ep_addr;
    
    xfertype = usb_endpoint_type(&ep->desc);
    ep_addr = ep->desc.bEndpointAddress & 0x7F;

    if(xfertype == USB_ENDPOINT_XFER_CONTROL)
    {
        ep->hc_ctrl_num_out = USBH_Alloc_Channel(&dev->USB_OTG_Core, 0x00|ep_addr);
        ep->hc_ctrl_num_in = USBH_Alloc_Channel(&dev->USB_OTG_Core, 0x80|ep_addr);
        
        USBH_Open_Channel (&dev->USB_OTG_Core,
                           ep->hc_ctrl_num_in,
                           dev->address,
                           dev->speed,
                           xfertype,
                           ep->desc.wMaxPacketSize);

        USBH_Open_Channel (&dev->USB_OTG_Core,
                           ep->hc_ctrl_num_out,
                           dev->address,
                           dev->speed,
                           xfertype,
                           ep->desc.wMaxPacketSize);


    }
    else if(xfertype == USB_ENDPOINT_XFER_BULK)
    {
        if(usb_endpoint_dir_in(&ep->desc))
        {  
            ep->hc_num = USBH_Alloc_Channel(&dev->USB_OTG_Core, 0x80|ep_addr);
        }
        else
        {
            ep->hc_num = USBH_Alloc_Channel(&dev->USB_OTG_Core, ep_addr);
        }

        USBH_Open_Channel (&dev->USB_OTG_Core,
                           ep->hc_num,
                           dev->address,
                           dev->speed,
                           xfertype,
                           ep->desc.wMaxPacketSize);
        
    }
    else
    {
        USBH_DBG("USBH_EP_Init unspport endpoint type:%d\r\n",xfertype);

    }


    return 1;
}


u8 USBH_HandleEnum(struct usb_device *dev)
{
//     struct usb_host_endpoint *ep0 = &dev->ep[0];
//     OS_MSG_SIZE  msg_size;
//     OS_ERR err;

//=============================USBH_Get_DevDesc=========================/
    USBH_TRACE("USBH_HandleEnum-->USBH_Get_DevDesc8\r\n");    
    if(USBH_Get_DevDesc(dev,8) != USBH_OK)
    {
        USBH_DBG("USBH_HandleEnum USBH_Get_DevDesc 8 Failed\r\n");
        return DEF_FAIL;
    }

    dev->ep[0].desc.wMaxPacketSize = dev->descriptor.bMaxPacketSize;
    USBH_TRACE("USBH_Get_DevDesc8 Dev_Desc.bMaxPacketSize:%d\r\n",dev->descriptor.bMaxPacketSize);
    /* Issue Reset  */
    HCD_ResetPort(&dev->USB_OTG_Core);
//     OSTaskQPend(0,OS_OPT_PEND_BLOCKING,&msg_size,0,&err);    
    /* modify control channels configuration for MaxPacket size */
    USBH_Modify_Channel (&dev->USB_OTG_Core,
                         dev->ep[0].hc_ctrl_num_out,
                         0,
                         0,
                         0,
                         dev->ep[0].desc.wMaxPacketSize);

    USBH_Modify_Channel (&dev->USB_OTG_Core,
                         dev->ep[0].hc_ctrl_num_in,
                         0,
                         0,
                         0,
                         dev->ep[0].desc.wMaxPacketSize);
    
//=============================ENUM_GET_FULL_DEV_DESC=========================//
    USBH_TRACE("USBH_HandleEnum-->ENUM_GET_FULL_DEV_DESC\r\n");     
    if ( USBH_Get_DevDesc(dev, USB_DEVICE_DESC_SIZE)\
         != USBH_OK)
    {
        USBH_DBG("USBH_HandleEnum USBH_Get_DevDesc FULL Failed\r\n");
        return DEF_FAIL;
    }
//=============================ENUM_SET_ADDR=========================//
    if ( USBH_SetAddress(dev, USBH_DEVICE_ADDRESS) != USBH_OK)
    {
        USBH_DBG("USBH_HandleEnum USBH_SetAddress Failed\r\n");
        return DEF_FAIL;
    }
    USB_OTG_BSP_mDelay(2);
    dev->address = USBH_DEVICE_ADDRESS;



    /* modify control channels to update device address */
    USBH_Modify_Channel (&dev->USB_OTG_Core,
                         dev->ep[0].hc_ctrl_num_in,
                         dev->address,
                         0,
                         0,
                         0);

    USBH_Modify_Channel (&dev->USB_OTG_Core,
                         dev->ep[0].hc_ctrl_num_out,
                         dev->address,
                         0,
                         0,
                         0);
//=============================ENUM_GET_CFG_DESC=========================//
    /* get standard configuration descriptor */
    if ( USBH_Get_CfgDesc(dev,
                          USB_CONFIGURATION_DESC_SIZE) != USBH_OK)
    {
        USBH_DBG("USBH_HandleEnum USBH_Get_CfgDesc Failed\r\n");
        return DEF_FAIL;
    }
    //=============================ENUM_GET_FULL_CFG_DESC=========================//
    /* get FULL config descriptor (config, interface, endpoints) */
    if (USBH_Get_CfgDesc(dev,
                         dev->config->desc.wTotalLength) != USBH_OK)
    {
        USBH_DBG("USBH_HandleEnum USBH_Get_CfgDesc Failed\r\n");
        return DEF_FAIL;

    }
    //=============================ENUM_GET_MFC_STRING_DESC=========================//
    if (dev->descriptor.iManufacturer != 0)
    {
        /* Check that Manufacturer String is available */

        if ( USBH_Get_StringDesc(dev,
                                 dev->descriptor.iManufacturer,
                                 (u8*)dev->manufacturer ,
                                 30) != USBH_OK)
        {
            USBH_DBG("USBH_HandleEnum ENUM_GET_MFC_STRING_DESC Failed\r\n");
            return DEF_FAIL;
        }
    }
    else
    {
        strcpy(dev->manufacturer,"N/A");
    }
//    USBH_TRACE("iManufacturer:%s\r\n",dev->manufacturer);
    //=============================ENUM_GET_PRODUCT_STRING_DESC=========================//

    if (dev->descriptor.iProduct != 0)
    {
        /* Check that Product string is available */
        if ( USBH_Get_StringDesc(dev,
                                 dev->descriptor.iProduct,
                                 (u8*)dev->product,
                                 30) != USBH_OK)
        {
            USBH_DBG("USBH_HandleEnum ENUM_GET_PRODUCT_STRING_DESC Failed\r\n");
            return DEF_FAIL;
        }
    }
    else
    {
        strcpy(dev->product,"N/A");
    }
//    USBH_TRACE("iProduct:%s \r\n",dev->product);
    //=============================ENUM_GET_SERIALNUM_STRING_DESC=========================//
    if (dev->descriptor.iSerialNumber != 0)
    {
        /* Check that Serial number string is available */
        if ( USBH_Get_StringDesc(dev,
                                 dev->descriptor.iSerialNumber,
                                 (u8*)dev->serial,
                                 30) != USBH_OK)
        {
            USBH_DBG("USBH_HandleEnum ENUM_GET_SERIALNUM_STRING_DESC Failed\r\n");
            return DEF_FAIL;
        }
    }
    else
    {
        strcpy(dev->serial,"N/A");
    }
//    USBH_TRACE("iSerialNumber:%s\r\n",dev->serial);
//=============================ENUM_SET_CONFIGURATION=========================//
    /* set configuration  (default config) */
    if (USBH_SetCfg(dev,
                    dev->config->desc.bConfigurationValue) != USBH_OK)
    {
        USBH_DBG("USBH_HandleEnum USBH_SetCfg Failed\r\n");
        return DEF_FAIL;
    }

    USBH_TRACE("USBH_HandleEnum Success!\r\n");
    return DEF_OK;

}

extern unsigned int TimerTotalCtr;
extern unsigned int SemaTotalCtr;
extern unsigned int KthreadTotalCtr;
extern unsigned int skbTotalCtr;


// extern void mem_check(void);


void USBH_DisconCheck(struct usb_device * dev)
{
    int i;
    OS_ERR err;


    //wait for usbh_task release resource. 
    OSTimeDlyHMSM(0u, 0u, 0u, 200u, OS_OPT_TIME_HMSM_STRICT,&err);  

//    USBH_TRACE("urb_nperiod_queue max:%d\r\n",dev->urb_nperiod_queue.max_num_record);
//    
    if(dev->usb_nperiod_xfer_state != USBH_XFER_STATE_IDLE)
        USBH_DBG("usb_nperiod_xfer_state:%d != USBH_XFER_STATE_IDLE \r\n",dev->usb_nperiod_xfer_state);

//     if(dev->usb_nperiod_xfer_state != USBH_XFER_STATE_IDLE)
//         USBH_DBG("usb_nperiod_xfer_state:%d != USBH_XFER_STATE_IDLE \r\n",dev->usb_nperiod_xfer_state);

    if(dev->usb_nperiod_task_req_num != 0)
        USBH_DBG("usb_nperiod_task_req_num:%d != 0 \r\n",dev->usb_nperiod_task_req_num);
    
//     if(dev->usb_nperiod_task_req_num != 0)
//         USBH_DBG("usb_nperiod_task_req_num:%d != 0 \r\n",dev->usb_nperiod_task_req_num);

    if(dev->urb_nperiod_queue.num != 0)
        USBH_DBG("urb_nperiod_queue.num:%d != 0 \r\n",dev->urb_nperiod_queue.num);    

//     if(dev->urb_nperiod_queue.num != 0)
//         USBH_DBG("urb_nperiod_queue.num:%d != 0 \r\n",dev->urb_nperiod_queue.num);  
    if(TimerTotalCtr)
        USBH_DBG("TimerTotalCtr:%d !=0\r\n",TimerTotalCtr);


    if(SemaTotalCtr)
         USBH_DBG("SemaTotalCtr:%d !=0\r\n",SemaTotalCtr);  

    if(KthreadTotalCtr)
        USBH_DBG("KthreadTotalCtr:%d !=0\r\n",KthreadTotalCtr);

    if(memory_used)
    {
        USBH_DBG("memory_used:%d !=0 \r\n",memory_used);  
//        mem_check();
        USBH_DBG("-->skbTotalCtr:%d \r\n",skbTotalCtr);     
    }

    
    if(URB_Mem.NbrFree != USBH_URB_NUM_MAX)
         USBH_DBG("URB_Mem.NbrFree:%d !=USBH_URB_NUM_MAX\r\n",URB_Mem.NbrFree);

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
//    for(i = 0; i < USBH_URB_NUM_MAX; i++)
//    {
//        if(USBH_URBPool[i].dev != &_usb_device)
//            USBH_DBG("USBH_URBPool[%d].dev:%p != &_usb_device \r\n",i,USBH_URBPool[i].dev);

//        state = OSTmrStateGet(&USBH_URBPool[i].tmr, &err);
//        if(state != OS_TMR_STATE_UNUSED)
//            USBH_DBG("&USBH_URBPool[%d].timer->tmr state:%d error\r\n",i,state);

//    }

    for(i = 0; i < HC_MAX; i++)
    {
        if(dev->urb[i] != NULL)
            USBH_DBG("dev->urb[%d]:%p != NULL \r\n",i,dev->urb[i]);        
    }
}

u32 USBH_ConnectProbe(u32 status,USB_OTG_CORE_HANDLE *pdev)
{
    OS_ERR err;
    u32 pre_con_sts,con_sts;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
    pre_con_sts = con_sts = pdev->host.ConnSts;
    CPU_CRITICAL_EXIT();
    OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_DLY,&err);
    while(1)
    {
        CPU_CRITICAL_ENTER();
        con_sts = pdev->host.ConnSts;   
        CPU_CRITICAL_EXIT();
        if((con_sts == pre_con_sts) && (con_sts != status))
        {
            return con_sts; 
        }
        pre_con_sts = con_sts;
        OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_DLY,&err);
    }

}

void USBH_ConnectProbeCallback(void *p_tmr, void *p_arg)
{
    static u32 pre_con_sts = USBH_DEV_DISCONNECTED;
    static u32 con_sts = USBH_DEV_DISCONNECTED;    
    USB_OTG_CORE_HANDLE *p_dev;
    OS_ERR err;
    CPU_SR cpu_sr; 

    p_dev = (USB_OTG_CORE_HANDLE *)p_arg;
    CPU_CRITICAL_ENTER();
    con_sts = p_dev->host.ConnSts;   
    CPU_CRITICAL_EXIT();    
    if((con_sts == pre_con_sts) && (con_sts != USBH_ConState))
    {
        USBH_ConState = con_sts;
        OSTmrStop((OS_TMR  *)p_tmr,OS_OPT_TMR_NONE,NULL,&err);
        OSSemPost(&USBH_ProbeSem,OS_OPT_POST_1,&err);
    }    
    pre_con_sts = con_sts;
}


struct usb_driver *_pusb_driver = NULL;
extern struct net_device *_pnet_device;



int usb_register(struct usb_driver *driver)
{
    _pusb_driver = driver;
    return 0;
//	return usb_register_driver(driver, THIS_MODULE, KBUILD_MODNAME);
}

void usb_deregister(struct usb_driver *driver)
{
	USBH_DBG("deregistering interface driver %s\n",
			 driver->name);

    _pusb_driver = NULL;
}

static int inline USBH_ProbeUSB(struct usb_device *dev)
{
    const struct usb_device_id *id;
    int match = 0;
    int ret;
    
    if(!_pusb_driver)
    {
        USBH_DBG("USBH_ProbeUSB unregister usb driver\r\n");
        return -1;
    }

    id = _pusb_driver->id_table;
	for (; id->idVendor || id->idProduct; id++) {
        if ((id->idVendor == dev->descriptor.idVendor) && 
            (id->idProduct == dev->descriptor.idProduct))
        {
            match = 1;
            break;
        }
	}

    if(!match)
    {
        USBH_DBG("USBH_ProbeUSB match idVendor:%d idProduct:%d Failed\r\n",dev->descriptor.idVendor,
                                                                dev->descriptor.idProduct);
        return -1;
    }

    if(!_pusb_driver->probe)
    {
        USBH_DBG("USBH_ProbeUSB _usb_driver->probe is NULL\r\n");
        return -1;
    }

    ret = _pusb_driver->probe(_usb_device.intf, id);
    if(ret)
    {
        USBH_DBG("USBH_ProbeUSB _usb_driver->probe Failed:%d\r\n",ret);      
        return ret;
    }

     if(!_pnet_device)
    {
        USBH_DBG("USBH_ProbeUSB _pnet_device is NULL\r\n");
        return -1;
    }        
    _pnet_device->flags |= IFF_UP;

    ret = (*(_pnet_device->open))(_pnet_device);
    if(ret)
    {
        USBH_DBG("USBH_ProbeUSB _pnet_device->open Failed:%d\r\n",ret);      
        return ret;
    }
    

    return 0;
}


static int inline USBH_DisconUSB(struct usb_device *dev)
{

    if(_pnet_device)
    {        
        USBH_TRACE("USBH_DisconUSB do (_pnet_device->stop)\r\n");
        (_pnet_device->stop)(_pnet_device);      
    }   
    
 
    if(_pusb_driver)
    {
        USBH_TRACE("USBH_DisconUSB do _pusb_driver->disconnect\r\n");        
        _pusb_driver->disconnect(_usb_device.intf);        
        return 0;
    }

    return -1;
}

extern int __init rtusb_init(void);

void USBH_ProbeTask(void *p_arg)
{
    struct usb_device *dev;
//     OS_MSG_SIZE msg_size;
    OS_ERR err;
    u8 num_ep = 0;
//     u8 usb_probe_ok = DEF_FAIL;
    struct task_struct *task, *next;
    int i;
    OS_TICK dly;
    CPU_SR cpu_sr;
    dev = (struct usb_device *)p_arg;
    
//    (*_usb_init_func)();//module init
   
    
    CPU_CRITICAL_ENTER();
    USBH_ConState = USBH_DEV_DISCONNECTED;
    dev->USB_OTG_Core.host.ConnSts = USBH_DEV_DISCONNECTED;
    CPU_CRITICAL_EXIT();
        
    
    dly = (200 * OSCfg_TmrTaskRate_Hz + (1000 - 1))/ 1000;// 200ms
    OSSemCreate(&USBH_ProbeSem,"USBH_ProbeSem",0,&err);
    OSTmrCreate(&USBH_ProbeTmr, "USBH_ProbeTmr",0, dly, OS_OPT_TMR_PERIODIC, USBH_ConnectProbeCallback,&dev->USB_OTG_Core,&err);
    OSTmrStart(&USBH_ProbeTmr,&err);

    rtusb_init();


    HCD_ResetPort(&dev->USB_OTG_Core);
    while(1)
    {
        // ensure the return status changed.
//        USBH_ConState = USBH_ConnectProbe(USBH_ConState, &dev->USB_OTG_Core);
        OSSemPend(&USBH_ProbeSem,0,OS_OPT_PEND_BLOCKING,0,&err);
        
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_ProbeTask OSSemPend Failed:%d\r\n",err);
            continue;
        }
        
        if(USBH_ConState == USBH_DEV_ATTACHED)
        {    
            USBH_TRACE("memory used:%d  max_used:%d  \r\n",memory_used,memory_used_max);
                
            USBH_TRACE("USBH_ProbeTask USBH_DEV_ATTACHED \r\n");   
            
            USB_OTG_BSP_mDelay(100);

            /* Reset USB Device */
            HCD_ResetPort(&dev->USB_OTG_Core);
//             OSTaskQPend(0,OS_OPT_PEND_BLOCKING,&msg_size,0,&err);
            
            dev->speed= HCD_GetCurrentSpeed(&dev->USB_OTG_Core);
            dev->address = USBH_DEVICE_ADDRESS_DEFAULT;

            memset(&dev->ep[0],0,sizeof(struct usb_host_endpoint));
            dev->ep[0].desc.bEndpointAddress = 0;
            dev->ep[0].desc.bmAttributes = USB_ENDPOINT_XFER_CONTROL;
            dev->ep[0].desc.wMaxPacketSize = USB_OTG_MAX_EP0_SIZE;


            //ep0 init,in order to enum.
            USBH_EP_Init(dev,&dev->ep[0]);

            if(USBH_HandleEnum(dev) == DEF_OK)
            {
                num_ep = dev->intf->altsetting->desc.bNumEndpoints;

                USBH_TRACE("idVendor:%x\r\n",dev->descriptor.idVendor);
                USBH_TRACE("idProduct:%x\r\n",dev->descriptor.idProduct);                
                USBH_TRACE("iManufacturer:%s\r\n",dev->manufacturer);
                USBH_TRACE("iProduct:%s \r\n",dev->product);
                USBH_TRACE("iSerialNumber:%s\r\n",dev->serial);
                USBH_TRACE("bNumEndpoints:%d\r\n",num_ep);

                OSTmrStart(&USBH_ProbeTmr,&err);
                //&dev->ep[i] array index may not be equal to (bEndpointAddress & 0x7F)
                //ep0 init in front, only init other endpoint.
                if(num_ep < USBH_MAX_NUM_ENDPOINTS)
                {
                    for(i = 1; i < num_ep; i++)
                    {
                        USBH_EP_Init(dev,&dev->ep[i]);
                    }

                    
                    USBH_ProbeUSB(dev);
                    
//                     if(!USBH_ProbeUSB(dev))
//                         usb_probe_ok = DEF_YES;
//                     else
//                         usb_probe_ok = DEF_FAIL;    

                     //call usb_driver->probe.
                     //                       
                }
                else
                {
                    USBH_DBG("USBH_ProbeTask num_ep > USBH_MAX_NUM_ENDPOINTS\r\n");
                    num_ep = USBH_MAX_NUM_ENDPOINTS;
                }
            
            }
            else
            {
                USBH_EP_Free(dev,&dev->ep[0]);

                CPU_CRITICAL_ENTER();
                USBH_ConState = USBH_DEV_DISCONNECTED;
                CPU_CRITICAL_EXIT();
                OSTmrStart(&USBH_ProbeTmr,&err);
            }
        }
         else if(USBH_ConState == USBH_DEV_DISCONNECTED)
         {
             USBH_TRACE("USBH_ProbeTask USBH_DEV_DISCONNECTED \r\n");  

//             if(usb_probe_ok == DEF_YES)
//             {
                USBH_TRACE("USBH_ProbeTask do USBH_DisconUSB \r\n");  
                USBH_DisconUSB(dev);
//                 usb_probe_ok = DEF_FAIL;
//             }
             
             //
             //call usb_driver->disconected.
             //
             for(i = 0; i < num_ep; i++)
             {
                USB_OTG_HC_Halt(&dev->USB_OTG_Core, dev->ep[i].hc_num);
                USBH_EP_Free(dev,&dev->ep[i]);
             }

             //kernel thread memory free.
             OSSchedLock(&err);
             list_for_each_entry_safe(task,next,&kernel_task_list_head,task_list)
             {
                if((task->exit_err != OS_ERR_NONE) || (task->tcb.TaskState != OS_TASK_STATE_DEL))
                {
                    USBH_DBG("USBH_ProbeTask %s Del failed:%d\r\n",task->name,task->exit_err);
                }
                else
                {
                    kthread_del(task);
                }
             }
             OSSchedUnlock(&err);
             
             USBH_DisconCheck(dev);

             OSTmrStart(&USBH_ProbeTmr,&err);
         }
    }



}



struct urb * DbgCurProessUrb;


void USBH_Task(void *p_arg)
{
    OS_MSG_SIZE msg_size;
    OS_ERR err;
    struct urb *urb;
    struct usbh_task_req *req;
    u8 xfer_type;
    struct usb_device *dev;


    dev = (struct usb_device *)p_arg;

    while(1)
    {
        req = OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &msg_size, 0, &err);

        if(err != OS_ERR_NONE)
            USBH_DBG("USBH_Task OSTaskQPend Failed %d\r\n",err);


        urb = req->urb;
        xfer_type = usb_endpoint_type(&urb->ep->desc);
        
        DbgCurProessUrb = urb;
        if((xfer_type == USB_ENDPOINT_XFER_CONTROL) || (xfer_type == USB_ENDPOINT_XFER_BULK))
        {
            USBH_NperiodHandler(dev, urb, req->req_type, req->req_msg, req->data);

        }

        OSMemPut(&ReqQueueMem, req, &err);
    }

}




void USBH_UrbQueueInit(struct urb_queue *queue)
{
    queue->urb_head = NULL;
    queue->urb_tail = NULL;
    queue->num = 0;
    queue->max_num_record = 0;
}




struct urb *USBH_UrbDequeue(struct urb_queue *queue)
{
    struct urb * urb;
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();

    if(!queue->urb_head)
        return NULL;

    if(queue->urb_head == queue->urb_tail)
    {
        urb = queue->urb_head;
        queue->urb_head = NULL;
        queue->urb_tail = NULL;
        queue->num--;
    }
    else
    {
        urb = queue->urb_head;
        queue->urb_head = queue->urb_head->next;
        queue->num--;
    }

    CPU_CRITICAL_EXIT();

    return urb;
}


void USBH_UrbQueueDump(struct urb_queue *queue)
{
    struct urb *temp_urb;
    CPU_SR cpu_sr;

    USBH_DBG("USBH_UrbQueueDump:\r\n");
    CPU_CRITICAL_ENTER();

    USBH_DBG("queue->num:%d urb_head:%p urb_tail:%p\r\n",queue->num,queue->urb_head,queue->urb_tail);
    
    for(temp_urb = queue->urb_head; temp_urb != NULL; temp_urb = temp_urb->next)
    {
        USBH_DBG("urb:%p urb->priv_state:%d MachineState:%d MsgType:%d\r\n",temp_urb,temp_urb->priv_state,temp_urb->MachineState,temp_urb->MsgType);
        
    }

    CPU_CRITICAL_EXIT();
}

u8 USBH_UrbQueueUrbDel(struct urb_queue *queue, struct urb *urb)
{ 
//     u16 i;
    struct urb *temp_urb, *temp_urb_pre;
    CPU_SR cpu_sr;

    if(urb == NULL)
        return DEF_FAIL;

    CPU_CRITICAL_ENTER();

    if(queue->urb_head == NULL)
    {
        CPU_CRITICAL_EXIT();
        return DEF_FAIL;
    }
    else if((queue->urb_head == urb) && (queue->urb_tail == urb))
    {
        queue->urb_head = NULL;
        queue->urb_tail = NULL;
        queue->num--;
        CPU_CRITICAL_EXIT();
        return DEF_OK;
    }
    else if(queue->urb_head == urb)
    {
        queue->urb_head = queue->urb_head->next;
        queue->num--;        
        return DEF_OK;
    }

    //the number of urb in queue >= 3
    temp_urb_pre = queue->urb_head;
    temp_urb = temp_urb_pre->next;

    for(; temp_urb != NULL; temp_urb = temp_urb->next,temp_urb_pre = temp_urb_pre->next)
    {
        if(temp_urb == urb)
        {
            temp_urb_pre->next = temp_urb->next;
            queue->num--;
            if(temp_urb == queue->urb_tail)
            {
                queue->urb_tail = temp_urb_pre;
            }
            CPU_CRITICAL_EXIT();
            return DEF_OK;
        }
    }
    CPU_CRITICAL_EXIT();
    return DEF_FAIL;
}

void USBH_UrbEnqueue(struct urb_queue *queue, struct urb *urb)
{
    CPU_SR cpu_sr;


    CPU_CRITICAL_ENTER();

    if(!queue->urb_head)
    {
        urb->next = NULL;
        queue->urb_head = urb;
        queue->urb_tail = urb;
        queue->num ++;
    }
    else
    {
        urb->next = NULL;
        queue->urb_tail->next = urb;
        queue->urb_tail = urb;
        queue->num ++;
    }
    if(queue->num > queue->max_num_record)
    {
        queue->max_num_record = queue->num;
    }
    CPU_CRITICAL_EXIT();
}



void USBH_NperiodHandler(struct usb_device *dev, struct urb *urb, int req_type,int req_msg,void *req_data)
{
    u8 xfer_type;
    OS_ERR err;
    u8 urb_is_first_handle = 0;
    
   // USBH_TRACE("USBH_NperiodHandler req_type:%d  req_msg:%d MachineState:%d MsgType:%d\r\n",req_type,req_msg,urb->MachineState,urb->MsgType);
    


    if((urb->priv_state == USBH_URB_WAIT_PROGRESS) && (dev->usb_nperiod_xfer_state == USBH_XFER_STATE_PROGRESS))
    {
        USBH_TRACE("USBH_NperiodHandler urb->priv_state==USBH_URB_WAIT_PROGRESS  in USBH_XFER_STATE_PROGRESS status may be occur\r\n");
        USBH_TRACE("CurProessUrb:%p  urb:%p req_type:%d req_msg:%d may be timeout,just ingore it\r\n",dev->CurProessUrb,urb,req_type,req_msg);        
//        USBH_DBG("call USBH_UrbEnqueue to enqueue this request\r\n");

//        USBH_UrbEnqueue(&dev->urb_nperiod_queue, urb);
        return;
    }

    // note: if urb is in USBH_URB_WAIT_PROGRESS priv_state, any req_type can occur!!
    if(urb->priv_state == USBH_URB_WAIT_PROGRESS)//first progress
    {
        urb_is_first_handle = 1;
        urb->priv_state = USBH_URB_PROGRESS;
        dev->usb_nperiod_xfer_state = USBH_XFER_STATE_PROGRESS;
        dev->CurProessUrb = urb;
        if(usb_endpoint_xfer_control(&urb->ep->desc))
        {
            if((urb->ep->hc_ctrl_num_in > HC_MAX) || (urb->ep->hc_ctrl_num_out> HC_MAX ))
            {
                USBH_DBG("USBH_NperiodHandler hc_ctrl_num > HC_MAX in:%d out:%d\r\n",urb->ep->hc_ctrl_num_in,urb->ep->hc_ctrl_num_out);
                while(1);
            }
            
            dev->urb[urb->ep->hc_ctrl_num_in] = urb;
            dev->urb[urb->ep->hc_ctrl_num_out] = urb;
        }
        else
        {
            if(urb->ep->hc_num > HC_MAX)
            {
                USBH_DBG("USBH_NperiodHandler hc_num > HC_MAX hum:%d\r\n",urb->ep->hc_num);
                while(1);
            }
            
            dev->urb[urb->ep->hc_num] = urb;
        }
    }
     //may be when urb has done and the urb->priv_state == USBH_URB_IDLE, USB_Task queue still has task_req(ie: USBH_REQ_TYPE_CH_HALT by unlink_urb or kill_urb)
     //urb->compeletion has called, no need to call it.
     //req_type =  USBH_REQ_TYPE_TIMEOUT timeout may be occurd.
    else if(urb->priv_state == USBH_URB_IDLE)
    {
        USBH_TRACE("USBH_NperiodHandler USBH_URB_IDLE may be occurd req_type:%d req_msg:%d urb:%p\r\n",req_type,req_msg,urb);

        if((req_type == USBH_REQ_TYPE_CH_HALT) && (req_msg == -ENOENT)) 
        {
            USBH_TRACE("USBH_NperiodHandler req_type==USBH_REQ_TYPE_CH_HALT do OSSemPost. urb:%p\r\n",urb);
            //synchronize
            OSSemPost((OS_SEM *)req_data,OS_OPT_POST_1,&err);//URB was canceled by  kill_urb
            if(err != OS_ERR_NONE)
            {
                USBH_DBG("USBH_ErrorWaitHcHaltRspAction OSSemPost Faild %d\r\n",err);
            }
        }      
        return;
    }
    else if(urb->priv_state != USBH_URB_PROGRESS)
    {
        USBH_DBG("USBH_NperiodHandler error!!! Unknown urb->priv_state:%d urb:%p\r\n",urb->priv_state,urb);
        return;
    }


    xfer_type = usb_endpoint_type(&urb->ep->desc);


    switch(xfer_type)
    {
        case USB_ENDPOINT_XFER_CONTROL:

        case USB_ENDPOINT_XFER_BULK:
            //is a bug? needn't halt channal?
            //2015.1.19 yes we need halt channal
            if(req_type== USBH_REQ_TYPE_TIMEOUT)
            {
                USBH_TRACE("USBH_NperiodHandler USBH_URB_PROGRESS_TIMEOUT do USB_OTG_HC_Halt do callback. urb:%p\r\n",urb);
                USB_OTG_HC_Halt(&dev->USB_OTG_Core, urb->ep->hc_num);                             
                //HOST_INT_CHH is masked,after delay for a while, assume hc complete hc_halt.
//                OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_DLY,&err);
                urb->status = -ETIMEDOUT;
                USBH_NperiodUrbProgressCallBack(dev, urb);
            }
            else if(req_type == USBH_REQ_TYPE_CH_HALT)
            {
                USBH_TRACE("USBH_NperiodHandler req_type==USBH_REQ_TYPE_CH_HALT urb:%p\r\n",urb);
                if(urb->MachineState != USBH_ERROR_MACHINE) //first progress USBH_URB_STOP when urb is in USBH_URB_PROGRESS
                {
                    urb->priv_state = USBH_URB_WAIT_STOP;
                    urb->status = req_msg;
                    urb->kill_sem = (req_msg == -ENOENT)?(OS_SEM *)req_data:NULL;
                    urb->MachineState = USBH_ERROR_MACHINE;
//                    urb->MsgType = USBH_ERROR_HC_HALT_REQ
                    urb->MsgType = urb_is_first_handle?USBH_ERROR_WAIT_HC_HALT_RSP : USBH_ERROR_HC_HALT_REQ;

                    //if urb is first be handled(likely not occur), it may needn't to request halt host channel?
                    //if endpointadress == 0(ep0), no need halt channel.
                    if(urb_is_first_handle || (urb->ep->desc.bEndpointAddress == 0))
                    {
                        urb->MsgType = USBH_ERROR_WAIT_HC_HALT_RSP;
                    }
                    else
                    {
                        urb->MsgType = USBH_ERROR_HC_HALT_REQ;

                    }
                }
                
                (*dev->MachineStateFunc[urb->MachineState][urb->MsgType])(dev, urb, &urb->MsgType);
            }
            else if(req_type == USBH_REQ_TYPE_RROGRESS)
            {
                //if urb->priv_state has changed, usbh_task_req may be post to USBH_Task Halt hc, we will ignore this request.
                if((urb->priv_state == USBH_URB_WAIT_STOP) || (urb->priv_state == USBH_URB_IDLE))
                {
                    USBH_DBG("USBH_NperiodHandler urb->priv_state:%d may be wait halt stop, ignore USBH_REQ_TYPE_RROGRESS ",urb->priv_state);
                    break;
                }

                if(USBH_ConState == USBH_DEV_ATTACHED)
                {
                    (*dev->MachineStateFunc[urb->MachineState][urb->MsgType])(dev, urb, &urb->MsgType);
                }
                else
                {
                    USBH_TRACE("USBH_NperiodHandler USBH_DEV_DISCONNECTED urb:%p\r\n",urb);
                    urb->status = -ENODEV;
                    USBH_NperiodUrbProgressCallBack(dev, urb);
                }
            }
            break;
        default:
            break;
    }



}


void USBH_NperiodQueuePostTask(struct usb_device *dev)
{
    struct urb *dequeue_urb;
    struct usbh_task_req *task_req;
    OS_ERR err;
    
    while(dev->urb_nperiod_queue.num > 0)
    {
        dequeue_urb = USBH_UrbDequeue(&dev->urb_nperiod_queue);
        if(!dequeue_urb)
        {
            USBH_DBG("USBH_NperiodUrbProgressCallBack  USBH_UrbDequeue Serious occur urb_nperiod_queue.num:%d stop!!\r\n",dev->urb_nperiod_queue.num);
            while(1);
 //           break;
        }

        if(dequeue_urb->priv_state == USBH_URB_WAIT_PROGRESS)
        {
            task_req = OSMemGet(&ReqQueueMem,&err);
            if(err != OS_ERR_NONE)
            {
                USBH_DBG("USBH_NperiodUrbProgressCallBack OSMemGet ReqQueueMem Faild %d\r\n",err);
                break;
            }

            task_req->req_type = USBH_REQ_TYPE_RROGRESS;
            task_req->req_msg = USBH_RROGRESS_FROM_TASK;                
            task_req->urb = dequeue_urb;
            OSTaskQPost(dev->pUSBHTaskTCB, task_req, sizeof(struct usbh_task_req), OS_OPT_POST_FIFO, &err);
            if(err != OS_ERR_NONE)
            {
                USBH_DBG("USBH_NperiodUrbProgressError OSTaskQPost Failed %d \r\n",err);
                OSMemPut(&ReqQueueMem,task_req, &err);
            }
            else
            {
                dev->usb_nperiod_task_req_num ++;
            }
            break;
        }
        else if(dequeue_urb->priv_state == USBH_URB_IDLE)//cancel by unlink
        {
            USBH_DBG("USBH_NperiodUrbProgressError dequeue_urb:%p was canceled by unlink_urb should never occur\r\n",dequeue_urb);
            while(1);
        }
        else
        {
            USBH_DBG("USBH_NperiodUrbProgressError dequeue_urb->priv_state = %d should never occur\r\n",dequeue_urb->priv_state);
            while(1);
        }
    }
}


int USBH_NperiodFreeUrbBuf(struct urb* urb)
{
    if(urb->transfer_buffer_length == 0)
        return -1;

    if(usb_endpoint_xfer_control(&urb->ep->desc))
    {
        if((urb->setup_request.bRequestType & USB_REQ_DIR_MASK) == USB_D2H)
        {
            memcpy(urb->user_actual_buffer,urb->transfer_buffer,urb->transfer_buffer_length);
        }
        kfree(urb->transfer_buffer); 
//            mem_free(urb->transfer_buffer);                       
    }//other endpoint type
    else if(usb_endpoint_dir_out(&urb->ep->desc))
    {
        kfree(urb->transfer_buffer);
    }
    return 0;
}


int USBH_NperiodTaskQFlush()
{
    OS_MSG_SIZE msg_size;
    OS_ERR err;
    struct usbh_task_req *req;
    u16 num = 0;

    while(1)
    {
        req = OSTaskQPend(0, OS_OPT_PEND_NON_BLOCKING, &msg_size, 0, &err);
        
        if(err == OS_ERR_PEND_WOULD_BLOCK)
        {
            break;
        }
        else if(err == OS_ERR_NONE)
        {
            num++;
            USBH_TRACE("USBH_NperiodTaskQFlush req_type:%d req_msg:%d urb:%p\r\n",req->req_type,req->req_msg,req->urb);
            OSMemPut(&ReqQueueMem, req, &err);
        }
        else
        {
            USBH_DBG("USBH_NperiodTaskQFlush Failed!!!:%d \r\n",err);                  
        }
    }    
    return num;
}


void USBH_NperiodUrbProgressCallBack(struct usb_device *dev, struct urb * urb)
{
    OS_ERR err;
//     CPU_SR cpu_sr;


    USBH_NperiodFreeUrbBuf(urb);
//    if(usb_endpoint_dir_out(&ep->desc) || usb_endpoint_xfer_control(&ep->desc)){
//        if(urb->transfer_buffer_length != 0){
//            buffer = mem_malloc(urb->transfer_buffer_length);
//            if(buffer == NULL)
//            {
//                USBH_DBG("usb_submit_urb mem_malloc Failed LINE:%d\r\n",__LINE__);
//                return -ENOSPC;
//            }

//        }
//    }


    OSSchedLock(&err);

    if(usb_endpoint_xfer_control(&urb->ep->desc))
    {
        if((urb->ep->hc_ctrl_num_in > HC_MAX) || (urb->ep->hc_ctrl_num_out> HC_MAX ))
        {
            USBH_DBG("USBH_NperiodUrbProgressCallBack hc_ctrl_num > HC_MAX in:%d out:%d\r\n",urb->ep->hc_ctrl_num_in,urb->ep->hc_ctrl_num_out);
            while(1);
        }
            
        dev->urb[urb->ep->hc_ctrl_num_in] = NULL;
        dev->urb[urb->ep->hc_ctrl_num_out] = NULL;
    }
    else
    {
        if(urb->ep->hc_num > HC_MAX)
        {
            USBH_DBG("USBH_NperiodUrbProgressCallBack hc_num > HC_MAX hum:%d\r\n",urb->ep->hc_num);
            while(1);
        }
        dev->urb[urb->ep->hc_num] = NULL;
    }
    dev->usb_nperiod_task_req_num --;
    dev->usb_nperiod_xfer_state = USBH_XFER_STATE_IDLE;
    dev->CurProessUrb = NULL;
    urb->priv_state = USBH_URB_IDLE;
    //every time usbh only progress one task_reqm ,when usb_nperiod_task_req_num > 1,usbh may be occur some error.
    //if this case occur, we call the urb callback function.
    if(dev->usb_nperiod_task_req_num > 0)
    {
        USBH_DBG("USBH_NperiodUrbProgressCallBack usb_nperiod_task_req_num > 1\r\n");
        (*urb->complete)(urb);
        OSSchedUnlock(&err);
        return;
    }
    USBH_NperiodTaskQFlush();
    USBH_NperiodQueuePostTask(dev);

    (*urb->complete)(urb);

    OSSchedUnlock(&err);
}


//void USBH_NperiodUrbProgressError(struct usb_device *dev, struct urb *urb)
//{
//    u8 priv_state;

//    priv_state = urb->priv_state;

//    if(priv_state == USBH_URB_PROGRESS_TIMEOUT)
//    {
//        USBH_DBG("USBH_NperiodUrbProgressError USBH_URB_PROGRESS_TIMEOUT\r\n");
//        urb->status = -ETIMEDOUT;
//    }
//    else if(priv_state == USBH_URB_STOP)
//    {
//        USBH_DBG("USBH_NperiodUrbProgressError urb:%p was canceled by unlink_urb\r\n",urb);
//        urb->status = -ENOENT; //"URB was canceled by unlink_urb"
//    }
//    else
//    {
//        USBH_DBG("USBH_NperiodUrbProgressError urb->priv_state = %d should never occur\r\n",urb->priv_state);
//    }


//    USBH_NperiodUrbProgressCallBack(dev, urb);

//}


//=======================================CONTROL TRANSFER======================

void USBH_ReqTimeOut(void *p_tmr, void *p_arg)
{
    struct urb *urb;
    struct usbh_task_req *task_req;
    OS_ERR err;

    urb = (struct urb *)p_arg;
    
    USBH_TRACE("USBH_ReqTimeOut urb:%p endpoint_type:%d CurProessUrb:%p\r\n",urb,usb_endpoint_type(&urb->ep->desc),urb->dev->CurProessUrb);
//    urb->state = USBH_URB_PROGRESS_TIMEOUT;

    task_req = OSMemGet(&ReqQueueMem,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_CtrlReqTimeOut OSMemGet ReqQueueMem Faild %d\r\n",err);
        return;
    }

    task_req->req_type = USBH_REQ_TYPE_TIMEOUT;
    task_req->urb = urb;
    OSTaskQPost(&USBHTaskTCB, task_req, sizeof(struct usbh_task_req), OS_OPT_POST_FIFO, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("USBH_CtrlReqTimeOut OOSTaskQPost Faild %d\r\n",err);
        OSMemPut(&ReqQueueMem,task_req, &err);
        return;
    }

}


int USBH_CtrlSetupReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].ep_is_in = 0;
    dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].data_pid = HC_PID_SETUP;
    dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].xfer_buff = (u8 *)&urb->setup_request;
    dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].xfer_len = USBH_SETUP_PKT_SIZE;

    HCD_SubmitRequest (&dev->USB_OTG_Core, urb->ep->hc_ctrl_num_out);

    *msg_type = USBH_CTRL_WAIT_SETUP_RSP;
    return 0;
}


int USBH_CtrlWaitSetupRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    static u16 timeout = 0;
    u8  direction;


    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_num);
    /* case SETUP packet sent successfully */
    if(URB_Status == URB_DONE)
    {
        direction  = (urb->setup_request.bRequestType & USB_REQ_DIR_MASK);

        /* check if there is a data stage */
        if (urb->setup_request.wLength != 0 )
        {
            timeout = DATA_STAGE_TIMEOUT;
            if (direction == USB_D2H)
            {
                /* Data Direction is IN */
                USBH_CtlReceiveData(&dev->USB_OTG_Core,
                                    urb->transfer_buffer,
                                    urb->transfer_buffer_length,
                                    urb->ep->hc_ctrl_num_in);

                *msg_type = USBH_CTRL_WAIT_DATA_IN_RSP;
//          *state = USBH_CTRL_DATA_IN_REQ;
            }
            else
            {
                /* Start DATA out transfer (only one DATA packet)*/
                dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].toggle_out = 1;

                USBH_CtlSendData (&dev->USB_OTG_Core,
                                  urb->transfer_buffer,
                                  urb->transfer_buffer_length,
                                  urb->ep->hc_ctrl_num_out);


                *msg_type = USBH_CTRL_WAIT_DATA_OUT_RSP;

                /* Data Direction is OUT */
//          *state = USBH_CTRL_DATA_OUT_REQ;
            }
        }
        /* No DATA stage */
        else
        {
            timeout = NODATA_STAGE_TIMEOUT;

            /* If there is No Data Transfer Stage */
            if (direction == USB_D2H)
            {

                dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].toggle_out ^= 1;

                USBH_CtlSendData (&dev->USB_OTG_Core,
                                  0,
                                  0,
                                  urb->ep->hc_ctrl_num_out);

                *msg_type = USBH_CTRL_WAIT_STATUS_OUT_RSP;
                /* Data Direction is IN */
//          *state = USBH_CTRL_STATUS_OUT_REQ;
            }
            else
            {
                /* Send 0 bytes out packet */
                USBH_CtlReceiveData (&dev->USB_OTG_Core,
                                     0,
                                     0,
                                     urb->ep->hc_ctrl_num_in);

                *msg_type = USBH_CTRL_WAIT_STATUS_IN_RSP;

                /* Data Direction is OUT */
//          *state = USBH_CTRL_STATUS_IN_REQ;
            }
        }
        /* Set the delay timer to enable timeout for data stage completion */
        USBH_TmrStart(&urb->tmr, timeout, USBH_ReqTimeOut, urb);
//      phost->Control.timer = HCD_GetCurrentFrame(pdev);
    }
    else if(URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_CtrlWaitSetupRspAction URB_ERROR!!! urb:%p\r\n",urb);
        urb->status = -EPIPE;
        USBH_NperiodUrbProgressCallBack(dev, urb);
//        phost->Control.state = CTRL_ERROR;
//        phost->Control.status = CTRL_XACTERR;
    }else
    {
        USBH_DBG("USBH_CtrlWaitSetupRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }


    return 0;
}


//int USBH_CtrlDataInReqAction(struct usb_device *dev, struct urb *urb, u16 *state)
//{
//    USBH_CtlReceiveData(&dev->USB_OTG_Core,
//                    urb->transfer_buffer,,
//                    urb->transfer_buffer_length,
//                    urb->ep->hc_num);
//}


int USBH_CtrlWaitDataInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;

//    urb->actual_length = dev->host.hc[urb->ep->hc_ctrl_num_in].xfer_count;
    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_ctrl_num_in);

    /* check is DATA packet transfered successfully */
    if  (URB_Status == URB_DONE)
    {
        urb->actual_length += dev->USB_OTG_Core.host.XferCnt[urb->ep->hc_ctrl_num_in];

        dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].toggle_out ^= 1;

        USBH_CtlSendData (&dev->USB_OTG_Core,
                          0,
                          0,
                          urb->ep->hc_ctrl_num_out);
        *msg_type = USBH_CTRL_WAIT_STATUS_OUT_RSP;
//        *state = USBH_CTRL_STATUS_OUT_REQ;
//      phost->Control.state = CTRL_STATUS_OUT;
    }

    /* manage error cases*/ //add else
    else if  (URB_Status == URB_STALL)
    {
        USBH_TRACE("USBH_CtrlWaitDataInRspAction URB_STALL!!! urb:%p\r\n",urb);        
        urb->status = -EPIPE;
        USBH_NperiodUrbProgressCallBack(dev, urb);
//        /* In stall case, return to previous machine state*/
//        phost->gState =   phost->gStateBkp;
    }
    else if (URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_CtrlWaitDataInRspAction URB_ERROR!!! urb:%p\r\n",urb);             
        urb->status = -EILSEQ;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* Device error */
//        phost->Control.state = CTRL_ERROR;
    }else
    {
        USBH_DBG("USBH_CtrlWaitDataInRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }
//    else if ((HCD_GetCurrentFrame(pdev)- phost->Control.timer) > timeout)
//    {
//      /* timeout for IN transfer */
//      phost->Control.state = CTRL_ERROR;
//    }
    return 0;
}


//int USBH_CtrlDataOutReqAction(struct usb_device *dev, struct urb *urb, u16 *state)
//{

//}


int USBH_CtrlWaitDataOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;

//    urb->actual_length = dev->host.hc[urb->ep->hc_ctrl_num_out].xfer_count;
    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_ctrl_num_out);

    if  (URB_Status == URB_DONE)
    {
        urb->actual_length += dev->USB_OTG_Core.host.XferCnt[urb->ep->hc_ctrl_num_out];
        /* If the Setup Pkt is sent successful, then change the state */

        USBH_CtlReceiveData (&dev->USB_OTG_Core,
                             0,
                             0,
                             urb->ep->hc_ctrl_num_in);
        *msg_type = USBH_CTRL_WAIT_STATUS_IN_RSP;

//          phost->Control.state = CTRL_STATUS_IN;
    }

    /* handle error cases */
    else if  (URB_Status == URB_STALL)
    {
        USBH_TRACE("USBH_CtrlWaitDataOutRspAction URB_STALL!!! urb:%p\r\n",urb);            
        urb->status = -EPIPE;
        USBH_NperiodUrbProgressCallBack(dev, urb);

//        /* In stall case, return to previous machine state*/
//        phost->gState =   phost->gStateBkp;
//        phost->Control.state = CTRL_STALLED;
    }
    else if  (URB_Status == URB_NOTREADY)
    {
        /* Start DATA out transfer (only one DATA packet)*/
        dev->USB_OTG_Core.host.hc[urb->ep->hc_ctrl_num_out].toggle_out = 1;

        USBH_CtlSendData (&dev->USB_OTG_Core,
                          urb->transfer_buffer,
                          urb->transfer_buffer_length,
                          urb->ep->hc_ctrl_num_out);
        *msg_type = USBH_CTRL_WAIT_DATA_OUT_RSP;
        /* Nack received from device */
//        phost->Control.state = CTRL_DATA_OUT;
    }
    else if (URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_CtrlWaitDataOutRspAction URB_ERROR!!! urb:%p\r\n",urb);        
        urb->status = -EILSEQ;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* device error */
//        phost->Control.state = CTRL_ERROR;
    }else
    {
        USBH_DBG("USBH_CtrlWaitDataOutRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }
    return 0;

}



//int USBH_CtrlStatusOutReqAction(struct usb_device *dev, struct urb *urb, u16 *state)
//{

//}

int USBH_CtrlWaitStatusOutRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;

    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_ctrl_num_out);

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        USBH_DBG("USBH_CtrlWaitStatusOutRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    if  ( URB_Status == URB_DONE)
    {
//        USBH_TRACE("USBH_CtrlWaitStatusOutRspAction StatusOutRsp success urb:%p\r\n",urb); 
        urb->status = 0;

        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* Control transfers completed, Exit the State Machine */
//        phost->gState =   phost->gStateBkp;
//        phost->Control.state = CTRL_COMPLETE;
    }

    else if (URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_CtrlWaitStatusOutRspAction URB_ERROR!!! urb:%p\r\n",urb);            
        urb->status = -EILSEQ;
        USBH_NperiodUrbProgressCallBack(dev, urb);
//        phost->Control.state = CTRL_ERROR;
    }

//    else if((HCD_GetCurrentFrame(pdev)\
//      - phost->Control.timer) > timeout)
//    {
//      phost->Control.state = CTRL_ERROR;
//    }
    else if(URB_Status == URB_STALL)
    {
        USBH_TRACE("USBH_CtrlWaitStatusOutRspAction URB_STALL!!! urb:%p\r\n",urb);          
        urb->status = -EPIPE;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* Control transfers completed, Exit the State Machine */
//        phost->gState =   phost->gStateBkp;
//        phost->Control.status = CTRL_STALL;
//        status = USBH_NOT_SUPPORTED;
    }
    else
    {
        USBH_DBG("USBH_CtrlWaitStatusOutRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }    
    return 0;

}

//int USBH_CtrlStatusInReqAction(struct usb_device *dev, struct urb *urb, u16 *state)
//{

//}

int USBH_CtrlWaitStatusInRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;

    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_ctrl_num_in);

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        USBH_DBG("USBH_CtrlWaitStatusInRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    if  ( URB_Status == URB_DONE)
    {
//        USBH_TRACE("USBH_CtrlWaitStatusInRspAction StatusInRsp success urb:%p\r\n",urb); 
        urb->status = 0;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* Control transfers completed, Exit the State Machine */
//        phost->gState =   phost->gStateBkp;
//        phost->Control.state = CTRL_COMPLETE;
    }

    else if (URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_CtrlWaitStatusInRspAction URB_ERROR!!! urb:%p\r\n",urb);          
        urb->status = -EILSEQ;
        USBH_NperiodUrbProgressCallBack(dev, urb);
//        phost->Control.state = CTRL_ERROR;
    }

//    else if((HCD_GetCurrentFrame(pdev)\
//      - phost->Control.timer) > timeout)
//    {
//      phost->Control.state = CTRL_ERROR;
//    }
    else if(URB_Status == URB_STALL)
    {
        USBH_TRACE("USBH_CtrlWaitStatusInRspAction URB_STALL!!! urb:%p\r\n",urb);            
        urb->status = -EPIPE;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* Control transfers completed, Exit the State Machine */
//        phost->gState =   phost->gStateBkp;
//        phost->Control.status = CTRL_STALL;
//        status = USBH_NOT_SUPPORTED;
    }
    else
    {
        USBH_DBG("USBH_CtrlWaitStatusInRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }        
    return 0;
}




//=======================================BULK TRANSFER======================

//MachineSateFuncInit(USBH_BULK_MACHINE, USBH_BULK_SUBMIT_REQ, USBH_BulkSubmitReqAction);
//MachineSateFuncInit(USBH_BULK_MACHINE, USBH_BULK_WAIT_SUBMIT_RSP, USBH_BulkWaitSubmitRspAction);



int USBH_BulkSubmitSentReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    int remain_length;

    USBH_TmrStart(&urb->tmr, USBH_BULK_SENT_TIMEOUT, USBH_ReqTimeOut, urb);
    remain_length = urb->transfer_buffer_length - urb->actual_length;

    if(remain_length > urb->ep->desc.wMaxPacketSize)
    {

        USBH_BulkSendData (&dev->USB_OTG_Core,
                           (u8 *)urb->transfer_buffer + urb->actual_length,
                           urb->ep->desc.wMaxPacketSize,
                           urb->ep->hc_num);

    }
    else
    {
        USBH_BulkSendData (&dev->USB_OTG_Core,
                           (u8 *)urb->transfer_buffer + urb->actual_length,
                           remain_length,
                           urb->ep->hc_num);
    }

    *msg_type= USBH_BULK_WAIT_SUBMIT_SENT_RSP;
    return 0;
}


int USBH_BulkWaitSubmitSentRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    URB_STATE URB_Status = URB_IDLE;
    OS_ERR err;
    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        USBH_DBG("USBH_BulkWaitSubmitSentRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }

    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_num);

    if  (URB_Status == URB_DONE)
    {
        urb->actual_length += dev->USB_OTG_Core.host.XferCnt[urb->ep->hc_num];

        if(urb->actual_length >= urb->transfer_buffer_length)
        {
//            USBH_TRACE("USBH_BulkWaitSubmitSentRspAction success sent,usb:%p actual_length:%d,buffer_length:%d\r\n",
//                                                urb,urb->actual_length,urb->transfer_buffer_length);              
            urb->status = 0;
            USBH_NperiodUrbProgressCallBack(dev, urb);
        }
        else
        {
            USBH_BulkSubmitSentReqAction(dev, urb, msg_type);

        }
    }
    /* handle error cases */
    else if  (URB_Status == URB_STALL)
    {
        USBH_TRACE("USBH_BulkWaitSubmitSentRspAction URB_STALL!!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                                            urb,urb->actual_length,urb->transfer_buffer_length);            
        urb->status = -EPIPE;

        USBH_NperiodUrbProgressCallBack(dev, urb);
    }
    else if  (URB_Status == URB_NOTREADY)
    {
        /* Device not ready, restart DATA out transfer*/

        USBH_BulkSubmitSentReqAction(dev, urb, msg_type);
//        USBH_BulkSendData (&dev->USB_OTG_Core,
//                          urb->transfer_buffer,
//                          urb->transfer_buffer_length,
//                          urb->ep->hc_num);
//

//        *msg_type = USBH_BULK_WAIT_SUBMIT_SENT_RSP;
        /* Nack received from device */
    }
    else if (URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_BulkWaitSubmitSentRspAction URB_ERROR!!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                                            urb,urb->actual_length,urb->transfer_buffer_length);          
        urb->status = -EILSEQ;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* device error */
    }
    else
    {
        USBH_DBG("USBH_BulkWaitSubmitSentRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }         
    return 0;
}




int USBH_BulkSubmitRcvReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    int remain_length;

//    USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction  urb:%p do USBH_BulkReceiveData\r\n", urb); 
    
    USBH_TmrStart(&urb->tmr, USBH_BULK_RECV_TIMEOUT, USBH_ReqTimeOut, urb);

    remain_length = urb->transfer_buffer_length - urb->actual_length;

    if(remain_length > urb->ep->desc.wMaxPacketSize)
    {

        USBH_BulkReceiveData (&dev->USB_OTG_Core,
                              (u8 *)urb->transfer_buffer + urb->actual_length,
                              urb->ep->desc.wMaxPacketSize,
                              urb->ep->hc_num);

//        urb->actual_length += urb->ep->desc.wMaxPacketSize;
    }
    else
    {
        USBH_BulkReceiveData (&dev->USB_OTG_Core,
                              (u8 *)urb->transfer_buffer + urb->actual_length,
                              remain_length,
                              urb->ep->hc_num);

//        urb->actual_length += remain_length;
    }

    *msg_type = USBH_BULK_WAIT_SUBMIT_RCV_RSP;

    return 0;
}

//recieve
int USBH_BulkWaitSubmitRcvRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    OS_ERR err;
    URB_STATE URB_Status = URB_IDLE;

    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);
    if((err != OS_ERR_NONE) && (err != OS_ERR_TMR_STOPPED))
    {
        USBH_DBG("USBH_BulkWaitSubmitRcvRspAction OSTmrStop Failed!! %d urb:%p\r\n",err,urb);
    }
    URB_Status = HCD_GetURB_State(&dev->USB_OTG_Core, urb->ep->hc_num);

    if  (URB_Status == URB_DONE)
    {
        urb->actual_length += dev->USB_OTG_Core.host.XferCnt[urb->ep->hc_num];

        if((urb->actual_length >= urb->transfer_buffer_length)
            ||(dev->USB_OTG_Core.host.XferCnt[urb->ep->hc_num] < urb->ep->desc.wMaxPacketSize))
        {
//            USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction success ,usb:%p actual_length:%d,buffer_length:%d\r\n",
//                                                urb,urb->actual_length,urb->transfer_buffer_length);            
            urb->status = 0;
            USBH_NperiodUrbProgressCallBack(dev, urb);
        }
        else
        {        
            USBH_BulkSubmitRcvReqAction(dev,urb,msg_type);
        }
    }
    else if(URB_Status == URB_NAK)
    {
        if(urb->actual_length)
        {
             USBH_DBG("USBH_BulkWaitSubmitRcvRspAction NAK recv,but urb->actual_length:%d >0,buffer_length:%d\r\n",
                                                 urb->actual_length,urb->transfer_buffer_length);
        }

//        USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction urb:%p URB_Status == URB_NAK do USBH_UrbEnqueue\r\n",urb);

        OSSchedLock(&err);
        dev->usb_nperiod_task_req_num --;
        dev->usb_nperiod_xfer_state = USBH_XFER_STATE_IDLE;
        dev->CurProessUrb = NULL;
        urb->priv_state = USBH_URB_WAIT_PROGRESS;
        *msg_type= USBH_BULK_SUBMIT_RCV_REQ;       
        //every time usbh only progress one task_reqm ,when usb_nperiod_task_req_num > 1,usbh may be occur some error.
        //if this case occur, we call the urb callback function.
        if(dev->usb_nperiod_task_req_num > 0)
        {
            USBH_DBG("USBH_BulkWaitSubmitRcvRspAction error! usb_nperiod_task_req_num > 1\r\n");
//            (*urb->complete)(urb);
            OSSchedUnlock(&err);
            return 0;
        }
        USBH_UrbEnqueue(&dev->urb_nperiod_queue,urb);
        USBH_NperiodTaskQFlush();
        USBH_NperiodQueuePostTask(dev);  
        
        OSSchedUnlock(&err);

    }    
    else if  (URB_Status == URB_STALL)
    {
        USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction URB_STALL!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                                            urb,urb->actual_length,urb->transfer_buffer_length);         
        urb->status = -EPIPE;

        USBH_NperiodUrbProgressCallBack(dev, urb);
    }
    else if (URB_Status == URB_ERROR)
    {
        USBH_TRACE("USBH_BulkWaitSubmitRcvRspAction URB_ERROR!!! ,usb:%p actual_length:%d,buffer_length:%d\r\n",
                                            urb,urb->actual_length,urb->transfer_buffer_length);         
        urb->status = -EILSEQ;
        USBH_NperiodUrbProgressCallBack(dev, urb);
        /* device error */
    }
    else
    {
        USBH_DBG("USBH_BulkWaitSubmitRcvRspAction Unknow URB_Status:%d\r\n",URB_Status);
    }       
    return 0;
}



int USBH_ErrorHcHaltReqAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
//     USB_OTG_CORE_HANDLE *pdev;
    OS_ERR err;

//     pdev = &dev->USB_OTG_Core;

    USBH_TRACE("USBH_NperiodUrbProgressError urb:%p was canceled by unlink_urb\r\n",urb);

//    UNMASK_HOST_INT_CHH (urb->ep->hc_num);
    USB_OTG_HC_Halt(&dev->USB_OTG_Core, urb->ep->hc_num);

    *msg_type = USBH_ERROR_WAIT_HC_HALT_RSP;
    
    //HOST_INT_CHH is masked,after delay for a while, assume hc complete hc_halt.
    OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_DLY,&err);
    USBH_ErrorWaitHcHaltRspAction(dev,urb,msg_type);
    return 0;
}

int USBH_ErrorWaitHcHaltRspAction(struct usb_device *dev, struct urb *urb, u16 *msg_type)
{
    OS_ERR err;

    USBH_TRACE("USBH_ErrorWaitHcHaltRspAction urb = %p\r\n",urb);

//    urb->status = -ENOENT;

//    else if(urb->status == -ECONNRESET)//asynchronize
//    {
//        urb->status = -ECONNRESET;//URB was canceled by unlink_urb
//    }
    OSTmrStop(&urb->tmr, OS_OPT_TMR_NONE,0,&err);

    USBH_NperiodUrbProgressCallBack(dev, urb);

    if(urb->status == -ENOENT) //synchronize
    {
        if(urb->kill_sem)
        {
            OSSemPost(urb->kill_sem,OS_OPT_POST_1,&err);//URB was canceled by  kill_urb
            if(err != OS_ERR_NONE)
            {
                USBH_DBG("USBH_ErrorWaitHcHaltRspAction OSSemPost Faild %d\r\n",err);
            }
        }
        else
        {
            USBH_DBG("USBH_ErrorWaitHcHaltRspAction urb->kill_sem is NULL shouln't occur \r\n");
        }
    }
    return 0;
}

