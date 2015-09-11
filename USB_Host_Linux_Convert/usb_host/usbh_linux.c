#define USBH_DEBUG_LEVEL USBH_DEBUG_ERROR//USBH_DEBUG_TRACE

#include "os.h"
#include "usbh_config.h"
#include "usbh_linux.h"
#include "errno.h"
#include "memory.h"
#include "includes.h"
#include "string.h"

extern OS_MEM URB_Mem;
extern OS_MEM ReqQueueMem;
extern OS_TCB USBHTaskTCB;;
extern volatile int USBH_ConState;

void *usb_buffer_alloc(struct usb_device *dev, size_t size, gfp_t mem_flags,
		       dma_addr_t *dma)
{
	if (!dev)
		return NULL;

	*dma = ~(dma_addr_t) 0;
	return kmalloc(size, mem_flags);    
}

void usb_buffer_free(struct usb_device *dev, size_t size, void *addr,
		     dma_addr_t dma)
{
	if (!dev)
		return;
	if (!addr)
		return;
    kfree(addr);
//	hcd_buffer_free(dev->bus, size, addr, dma);
}


struct urb *usb_alloc_urb(int iso_packets, u8 mem_flags)
{
	struct urb *urb;
    OS_ERR err;
    
    urb = OSMemGet(&URB_Mem,&err);
    if((err != OS_ERR_NONE) || (urb == NULL))
    {
        USBH_DBG("usb_alloc_urb OSMemGet failed %d\r\n",err);
        return NULL;
    }
    
//	memset(urb, 0, sizeof(struct urb)); do not clear it

    urb->priv_state = USBH_URB_IDLE;

	return urb;
}


void usb_free_urb(struct urb *urb)
{
    OS_ERR err;
    
    OSMemPut(&URB_Mem,urb,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("usb_free_urb OSMemPut failed %d\r\n",err);
    }
}



//when calling usb_submit_urb, don't call usb_kill_urb or usb_unlink_urb.
int usb_submit_urb(struct urb *urb, u8 mem_flags)
{
	int				xfertype;
	struct usb_device		*dev;
    struct usbh_task_req *task_req;
	struct usb_host_endpoint	*ep = NULL;
    OS_ERR err;
    int ret = 0;
    u8 i,bEndpointAddress;
    CPU_SR cpu_sr;

    
	if (!urb || !urb->complete)
		return -EINVAL;
	dev = urb->dev;
	if (!dev)
		return -ENODEV;

    if(USBH_ConState != USBH_DEV_ATTACHED)
    {
        USBH_TRACE("usb_submit_urb Device Removed! urb:%p\r\n",urb);
        return -ENODEV;
    }
        
    
    bEndpointAddress = usb_pipeendpoint(urb->pipe);

    //search ep
    for(i = 0; i < 16; i ++){
        if((dev->ep[i].desc.bEndpointAddress & 0X0F) == bEndpointAddress){
            //control endpoint ingore bit 7(direction),other endpoint must match endpoint direction.
            if((usb_pipetype(urb->pipe) == PIPE_CONTROL) || 
                ((urb->pipe & 0X80) == (dev->ep[i].desc.bEndpointAddress & 0X80)))
            {
                ep = &dev->ep[i];
                break;
            }
        }      
    }

	if (!ep)
	{
        USBH_DBG("usb_submit_urb ep is NULL \r\n");
		return -ENOENT;
	}



    CPU_CRITICAL_ENTER();//judge if already submit urb
    if(urb->priv_state != USBH_URB_IDLE){
       CPU_CRITICAL_EXIT();
       USBH_TRACE("usb_submit_urb already submit urb:%p priv_state:%d\r\n",urb,urb->priv_state);
       return -ENXIO;//URB already queued
    }
    urb->priv_state = USBH_URB_WAIT_PROGRESS;

    CPU_CRITICAL_EXIT();
    
    //after usb_contiol_msg time out, USBH may do call back, it will inc sem.ctr.
    if(urb->sem.Ctr != 0)
    {
        USBH_DBG("usb_submit_urb urb->sem.Ctr:%d !=0 may occur urb:%p\r\n",urb->sem.Ctr,urb);
    } 
    

    OSSemSet(&urb->sem,0,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("usb_submit_urb OSSemSet 0 ctr Failed %d urb:%p sem.Ctr:%d NbrEntries:%d\r\n",err,urb,urb->sem.Ctr,urb->sem.PendList.NbrEntries);
        if(urb->sem.PendList.HeadPtr)
        {
            USBH_DBG("           PendList.HeadPtr.TCBPtr=%p name:%s\r\n",urb->sem.PendList.HeadPtr->TCBPtr,urb->sem.PendList.HeadPtr->TCBPtr->NamePtr);
        }
    }



    
	urb->ep = ep;
 	urb->status = -EINPROGRESS;
	urb->actual_length = 0;
    urb->user_actual_buffer = urb->transfer_buffer;
	/* Lots of sanity checks, so HCDs can rely on clean data
	 * and don't need to duplicate tests
	 */
	xfertype = usb_endpoint_type(&ep->desc);

	if (xfertype == USB_ENDPOINT_XFER_CONTROL) {
        
//        USBH_DBG("usb_submit_urb XFER_CONTROL urb:%p  curtask:%s buffer:%p length:%d\r\n",urb,OSTCBCurPtr->NamePtr,
//                                                                    urb->transfer_buffer,urb->transfer_buffer_length);

        
        memcpy(&urb->setup_request, urb->setup_packet, sizeof(struct usb_ctrlrequest));
        CPU_CRITICAL_ENTER();
        urb->MachineState = USBH_CONTRL_MACHINE;
        urb->MsgType = USBH_CTRL_SETUP_REQ;
        CPU_CRITICAL_EXIT();

    }else if(xfertype == USB_ENDPOINT_XFER_BULK){


//        USBH_TRACE("usb_submit_urb XFER_BULK urb:%p curtask:%s dir:%d buffer:%p length:%d\r\n",urb,OSTCBCurPtr->NamePtr,usb_endpoint_dir_in(&ep->desc),
//                                                                    urb->transfer_buffer,urb->transfer_buffer_length);

        CPU_CRITICAL_ENTER();
        urb->MachineState = USBH_BULK_MACHINE;
        urb->MsgType= usb_endpoint_dir_in(&ep->desc)?USBH_BULK_SUBMIT_RCV_REQ : USBH_BULK_SUBMIT_SENT_REQ;
        CPU_CRITICAL_EXIT();
    }

    //control endpoint ignore dir.
    if(usb_endpoint_dir_out(&ep->desc) || usb_endpoint_xfer_control(&ep->desc)){
        void *buffer;

        if(xfertype == USB_ENDPOINT_XFER_BULK &&((urb->transfer_buffer == NULL) || (urb->transfer_buffer_length == 0)))
            return -EFAULT;

        if(urb->transfer_buffer_length != 0){
            buffer = kmalloc(urb->transfer_buffer_length,0);
//             buffer = mem_malloc(urb->transfer_buffer_length);
            if(buffer == NULL)
            {
                USBH_DBG("usb_submit_urb mem_malloc Failed LINE:%d\r\n",__LINE__);
                return -ENOSPC;
            }

            memcpy(buffer, urb->transfer_buffer, urb->transfer_buffer_length);
            urb->transfer_buffer = buffer;
        }
    }

//    urb->state = 0;

    OSSchedLock(&err);
    if((dev->usb_nperiod_xfer_state == USBH_XFER_STATE_PROGRESS)
        || (dev->usb_nperiod_task_req_num > 0))
    {
        //we believe the urb req_type of queue is USBH_REQ_TYPE_RROGRESS. 
        USBH_UrbEnqueue(&dev->urb_nperiod_queue, urb);     
    }
    else{
        
        task_req = OSMemGet(&ReqQueueMem,&err);
        //if failed may occur compete status.
        if(err != OS_ERR_NONE)
        {           
            USBH_DBG("usb_submit_urb OSMemGet ReqQueueMem Faild %d\r\n",err);
            ret = err;
            OSSchedUnlock(&err);
            return -ret;//if fail,mem may not be released?
        }
        
        task_req->req_type = USBH_REQ_TYPE_RROGRESS;
        task_req->req_msg = USBH_RROGRESS_FROM_TASK;        
        task_req->urb = urb;
        OSTaskQPost(dev->pUSBHTaskTCB, task_req, sizeof(struct usbh_task_req), OS_OPT_POST_FIFO, &err);
        if(err != OS_ERR_NONE){   
            ret = err;
            OSMemPut(&ReqQueueMem,task_req, &err);
            USBH_DBG("usb_submit_urb OSTaskQPost serious Failed %d\r\n",ret);     
            OSSchedUnlock(&err);
            return -ret;
        }
        dev->usb_nperiod_task_req_num++;       
    }
    OSSchedUnlock(&err);

    return 0;

//	return usb_hcd_submit_urb(urb, mem_flags);
}




static void usb_api_blocking_completion(struct urb *urb)
{
	struct usbh_msg_ctx *ctx = urb->context;
    OS_ERR err;

	ctx->status = urb->status;

    OSSemPost(&urb->sem,OS_OPT_POST_1,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("usb_api_blocking_completion OSSemPost Failed:%d urb:%p\r\n",err,urb);
    }
//	complete(&ctx->done);
}

static int usb_start_wait_urb(struct urb *urb, int timeout, int *actual_length)
{
	struct usbh_msg_ctx ctx;
	int retval;
    OS_ERR err;
    
//  init_completion(&ctx.done);
    ctx.status = -0xff;
    urb->context = &ctx;
    urb->actual_length = 0;
    retval = usb_submit_urb(urb, GFP_NOIO);
    if(retval)
    {	
        if (actual_length)
	        *actual_length = urb->actual_length;
        
        usb_free_urb(urb);
        return retval;
    }
    

    OSSemPend(&urb->sem,timeout,OS_OPT_PEND_BLOCKING,0,&err);

    retval = ctx.status;
    
    if(err != OS_ERR_NONE)
    {
        if(err == OS_ERR_TIMEOUT)
        {
            retval = (ctx.status != -0Xff ? -ETIMEDOUT : ctx.status);
            usb_kill_urb(urb);
            //if ctx.status != -0xff , it means usb_submit_urb has called the usb_api_blocking_completion before kill it.           
            USBH_DBG("usb_start_wait_urb OSSemPend timeout before kill the ctx.status:%d\r\n",ctx.status);
        }
        else
        {
            USBH_DBG("usb_start_wait_urb OSSemPend Failed %d\r\n",err);
        }
    }
 
    if (actual_length)
        *actual_length = urb->actual_length;
    
    usb_free_urb(urb);
    return retval;
}




int usb_control_msg(struct usb_device *dev, unsigned int pipe, u8 request,
		    u8 requesttype, u16 value, u16 index, void *data,
		    u16 size, int timeout)
{
	struct usb_ctrlrequest dr;
	int retv;
    int length;
    struct urb *urb;

//	dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_NOIO);

//    if (!dr)
//		return -ENOMEM;

	dr.bRequestType = requesttype;
	dr.bRequest = request;
	dr.wValue = value;
	dr.wIndex = index;
	dr.wLength = size;

	/* dbg("usb_control_msg"); */
    
    urb = usb_alloc_urb(0, GFP_NOIO);
    if (!urb)
        return -ENOMEM;
    
    usb_fill_control_urb(urb, dev, pipe, (u8*)&dr, data,
                 size, usb_api_blocking_completion, NULL);


    retv = usb_start_wait_urb(urb, timeout, &length);
	if (retv < 0)
		return retv;
	else
		return length;
    
//	ret = usb_internal_control_msg(dev, pipe, dr, data, size, timeout);
//	kfree(dr);
//	return ret;
}


int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
		 void *data, int len, int *actual_length, int timeout)
{
	struct urb *urb;
	struct usb_host_endpoint *ep = NULL;
    u8  bEndpointAddress,i;

    bEndpointAddress = usb_pipeendpoint(pipe);
    //search ep, control have two endpoint
    for(i = 0; i < 16; i ++){
        if(usb_dev->ep[i].desc.bEndpointAddress == bEndpointAddress){
            ep = &usb_dev->ep[i];
            break;
        }      
    }

//	ep = (usb_pipein(pipe) ? usb_dev->ep_in : usb_dev->ep_out)
//			[usb_pipeendpoint(pipe)];
    if (!ep || len < 0)
		return -EINVAL;
    

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return -ENOMEM;

	if ((ep->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_INT) {
        USBH_DBG("usb_bulk_msg not support int endpoint");
//		pipe = (pipe & ~(3 << 30)) | (PIPE_INTERRUPT << 30);
//		usb_fill_int_urb(urb, usb_dev, pipe, data, len,
//				usb_api_blocking_completion, NULL,
//				ep->desc.bInterval);
	} else
		usb_fill_bulk_urb(urb, usb_dev, pipe, data, len,
				usb_api_blocking_completion, NULL);

	return usb_start_wait_urb(urb, timeout, actual_length);
}


int usb_hcd_unlink_urb (OS_SEM *sem, struct urb *urb, int status)
{
    OS_ERR err,ret;
    struct usbh_task_req *task_req;    
//     CPU_SR cpu_sr;

    
    //urb is enqueue, only delete queue node.
    //if USBH_UrbQueueUrbDel fail, it means urb has been removed from urb_nperiod_queue and posted to USBH_Tsk but not be handled.
    //
    if((urb->priv_state == USBH_URB_WAIT_PROGRESS)
        && (USBH_UrbQueueUrbDel(&urb->dev->urb_nperiod_queue, urb) == DEF_OK))
    {   
        USBH_TRACE("usb_hcd_unlink_urb:%p in Queue.do urb->complete urb->priv_state:%d\r\n",urb,urb->priv_state);    
        urb->status = status;
        USBH_NperiodFreeUrbBuf(urb);        
        // usbh_task hasn't call urb->complete, now call it.   
        (*urb->complete)(urb);   
        //think seriously about it, assigning urb->priv_state dosen't occur competetion status.
        //because in the time, USBH_Task will not progress the urb.
        urb->priv_state = USBH_URB_IDLE;     
        
        if(urb->status == -ENOENT) //synchronize
        {
            OSSemPost(sem,OS_OPT_POST_1,&err);//URB was canceled by  kill_urb
            if(err != OS_ERR_NONE)
            {
                USBH_DBG("USBH_ErrorWaitHcHaltRspAction OSSemPost Faild %d\r\n",err);
            }
        }
        
        OSSchedUnlock(&err);
        return -EINPROGRESS;
    }   
    // urb->priv_state  == USBH_URB_PROGRESS || (urb->priv_state == USBH_URB_WAIT_PROGRESS and urb is posted to USBH_Task
    //but hasn't been handled)
    else
    {      
        USBH_TRACE("usb_hcd_unlink_urb:%p do OSTaskQPost urb->priv_state:%d MachineState:%d MsgType:%d CurProessUrb:%p\r\n",urb,urb->priv_state,urb->MachineState,urb->MsgType,urb->dev->CurProessUrb);


//        USBH_UrbQueueDump(&urb->dev->urb_nperiod_queue);
        
        task_req = OSMemGet(&ReqQueueMem,&err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("USBH_CtrlReqTimeOut OSMemGet ReqQueueMem Faild %d\r\n",err);
            ret = err;
            OSSchedUnlock(&err);                 
            return -ret;
        }
        
        task_req->req_type = USBH_REQ_TYPE_CH_HALT;
        task_req->req_msg = status;
        task_req->data = (void *)sem;
        task_req->urb = urb;
        
        //if failed may occur compete status.
        OSTaskQPost(&USBHTaskTCB, task_req, sizeof(struct usbh_task_req), OS_OPT_POST_FIFO, &err);
        if(err != OS_ERR_NONE)
        {
            ret = err;
            OSMemPut(&ReqQueueMem,task_req, &err);
            USBH_DBG("usb_hcd_unlink_urb OSTaskQPost Faild %d\r\n",ret);
            OSSchedUnlock(&err);                 
            return -ret;
        }else
        {
 //           urb->dev->usb_nperiod_task_req_num++;
            OSSchedUnlock(&err);
            return -EINPROGRESS;
        }
    }
//    else
//    {
//        ret = err;
//        USBH_DBG("usb_hcd_unlink_urb urb->priv_state serious problem %d\r\n",err);        
//        OSSchedUnlock(&err);           
//        return -EBUSY;
//    }

}




int usb_unlink_urb(struct urb *urb)
{
    OS_ERR err;
//     struct usbh_task_req *task_req;
    USBH_DBG("usb_unlink_urb urb:%p\r\n",urb);
	if (!urb)
		return -EINVAL;
	if (!urb->dev)
		return -ENODEV;
	if (!urb->ep)
		return -EIDRM;

    OSSchedLock(&err);
    if((urb->priv_state != USBH_URB_PROGRESS) || (urb->priv_state != USBH_URB_WAIT_PROGRESS))
    {
        OSSchedUnlock(&err);
        return -EBUSY;
    }

    return usb_hcd_unlink_urb(NULL,urb, -ECONNRESET);
}

void usb_kill_urb(struct urb *urb)
{
//	might_sleep();
    OS_ERR err;
//     struct usbh_task_req *task_req;
    int ret;
    OS_SEM sem;

    USBH_DBG("usb_kill_urb urb:%p\r\n",urb);
	if (!(urb && urb->dev && urb->ep))
		return;

    OSSemCreate(&sem,"usb_kill_urb",0,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("usb_kill_urb OSSemCreate Failed:%d\r\n",err);
        return;
    }

    OSSchedLock(&err);
    if((urb->priv_state != USBH_URB_PROGRESS) && (urb->priv_state != USBH_URB_WAIT_PROGRESS))
    {
        OSSchedUnlock(&err);
        OSSemDel(&sem,OS_OPT_DEL_NO_PEND,&err);
        if(err != OS_ERR_NONE)
            USBH_DBG("usb_kill_urb OSSemDel Failed:%d\r\n",err);        
        return;
    }

    ret = usb_hcd_unlink_urb(&sem, urb, -ENOENT);
    if(ret != -EINPROGRESS)
        USBH_DBG("usb_kill_urb usb_hcd_unlink_urb Faild %d\r\n",ret);
    //if call usb_hcd_unlink_urb,we must ensure that usb_kill_urb can aquire sem.
    OSSemPend(&sem,0, OS_OPT_PEND_BLOCKING,0,&err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("usb_kill_urb OSSemPend Faild %d\r\n",err);
    }
    
    OSSemDel(&sem,OS_OPT_DEL_NO_PEND,&err);
    if(err != OS_ERR_NONE)
        USBH_DBG("usb_kill_urb OSSemDel Failed:%d\r\n",err);
    
//	atomic_inc(&urb->reject);

//	usb_hcd_unlink_urb(urb, -ENOENT);
//	wait_event(usb_kill_urb_queue, atomic_read(&urb->use_count) == 0);

//	atomic_dec(&urb->reject);
}




