/*
  This file is ported from Linux Kernel.

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


#define USBH_DEBUG_LEVEL USBH_DEBUG_TRACE//USBH_DEBUG_ERROR

#include "os.h"
#include "usbh_debug.h"
#include "usbh_linux.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "atomic.h"
#include "errno.h"
#include "memory.h"
#include "string.h"


#define cpu_to_le16(val)  (val)
#define le16_to_cpu(val)  (val)


LIST_HEAD(usb_kill_urb_queue);

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

#define to_urb(d) container_of(d, struct urb, kref)


static void urb_destroy(struct kref *kref)
{
    struct urb *urb = to_urb(kref);
    OS_ERR err;

    if(urb->priv_state != USBH_URB_IDLE)
        usb_halt(urb->dev, "urb->priv_state:%d != USBH_URB_IDLE\r\n",urb->priv_state);

    if(urb->iso_frame_desc)
        kfree(urb->iso_frame_desc);
    
    if (urb->transfer_flags & URB_FREE_BUFFER)
        kfree(urb->transfer_buffer);
 
    OSMemPut(&URB_Mem, urb, &err);
    if(err != OS_ERR_NONE)
    {
        USBH_DBG("usb_free_urb OSMemPut failed %d\r\n",err);
    }
}


/**
 * usb_get_urb - increments the reference count of the urb
 * @urb: pointer to the urb to modify, may be NULL
 *
 * This must be  called whenever a urb is transferred from a device driver to a
 * host controller driver.  This allows proper reference counting to happen
 * for urbs.
 *
 * A pointer to the urb with the incremented reference counter is returned.
 */
struct urb *usb_get_urb(struct urb *urb)
{
    if (urb)
        kref_get(&urb->kref);
    return urb;
}

/**
 * usb_alloc_urb - creates a new urb for a USB driver to use
 * @iso_packets: number of iso packets for this urb
 * @mem_flags: the type of memory to allocate, see kmalloc() for a list of
 *  valid options for this.
 *
 * Creates an urb for the USB driver to use, initializes a few internal
 * structures, incrementes the usage counter, and returns a pointer to it.
 *
 * If no memory is available, NULL is returned.
 *
 * If the driver want to use this urb for interrupt, control, or bulk
 * endpoints, pass '0' as the number of iso packets.
 *
 * The driver must call usb_free_urb() when it is finished with the urb.
 */
struct urb *usb_alloc_urb(int iso_packets, u8 mem_flags)
{
	struct urb *urb;
    OS_ERR err;
    
    urb = OSMemGet(&URB_Mem, &err);
    if((err != OS_ERR_NONE) || (urb == NULL))
    {
        USBH_DBG("usb_alloc_urb OSMemGet failed %d\r\n",err);
        return NULL;
    }
    
    //Do not call memset to clear the structure, because urb->tmr is initialized.
    urb->priv_state = USBH_URB_IDLE;
    kref_init(&urb->kref);

    atomic_set(&urb->use_count, 0);
    atomic_set(&urb->reject, 0);
    urb->unlinked = 0;
    urb->dev = NULL;
    urb->ep = NULL;
    urb->ch = NULL;

    urb->transfer_flags = 0;
    urb->transfer_buffer = NULL;
    urb->number_of_packets = 0;
    urb->iso_frame_desc = NULL;
    urb->isoc_submited = NULL;

    if(iso_packets)
    {
        urb->iso_frame_desc = kmalloc(iso_packets * sizeof(struct usb_iso_packet_descriptor), mem_flags);
        if(!urb->iso_frame_desc)
        {
            OSMemPut(&URB_Mem, urb, &err);
            return NULL;
        }
    }

	return urb;
}

/**
 * usb_free_urb - frees the memory used by a urb when all users of it are finished
 * @urb: pointer to the urb to free, may be NULL
 *
 * Must be called when a user of a urb is finished with it.  When the last user
 * of the urb calls this function, the memory of the urb is freed.
 *
 * Note: The transfer buffer associated with the urb is not freed unless the
 * URB_FREE_BUFFER transfer flag is set.
 */
void usb_free_urb(struct urb *urb)
{
    if (urb)
        kref_put(&urb->kref, urb_destroy); 
}


static void usb_api_blocking_completion(struct urb *urb)
{
	struct usbh_msg_ctx *ctx = urb->context;
    OS_ERR err;

	ctx->status = urb->status;

    OSSemPost(&ctx->sem,OS_OPT_POST_1,&err);
    if(err != OS_ERR_NONE)
        usb_halt(urb->dev, "OSSemPost failed:%d\r\n",err);
//	complete(&ctx->done);
}


static int usb_start_wait_urb(struct urb *urb, int timeout, int *actual_length)
{
	struct usbh_msg_ctx ctx;
	int retval;
    OS_ERR err;
    

    OSSemCreate(&ctx.sem, "usb_start_wait_urb sem", 0, &err);
    if(err != OS_ERR_NONE)
    {
        retval = -ENOMEM;
        goto out;
    }
    
    ctx.status = -0xff;
    urb->context = &ctx;
    urb->actual_length = 0;
    retval = usb_submit_urb(urb, GFP_NOIO);
    if(retval)
        goto out2;
    

    OSSemPend(&ctx.sem,timeout,OS_OPT_PEND_BLOCKING, 0, &err);

    retval = ctx.status;
    
    if(err != OS_ERR_NONE)
    {
        if(err == OS_ERR_TIMEOUT)
        {
            usb_dbg(urb->dev,"usb_start_wait_urb do usb_kill_urb:%p ch:%d\r\n",urb,urb->ch->hc_num);
//            retval = (ctx.status != -0Xff ? -ETIMEDOUT : ctx.status);
            usb_kill_urb(urb);
            retval = (ctx.status == -ENOENT ? -ETIMEDOUT : ctx.status);

            usb_dbg(urb->dev,
                "%s timed out on ep%d%s len=%u/%u\n",
                __func__,
                usb_endpoint_num(&urb->ep->desc),
                usb_urb_dir_in(urb) ? "in" : "out",
                urb->actual_length,
                urb->transfer_buffer_length);
        }
        else
        {
            usb_dbg(urb->dev, "%s OSSemPend Failed %d\r\n",__func__, err);
        }
    }

out2:
    OSSemDel(&ctx.sem, OS_OPT_DEL_ALWAYS, &err);
    if(err != OS_ERR_NONE)
        usb_halt(urb->dev, "OSSemDel failed:%d\r\n",err);
out: 
    if (actual_length)
        *actual_length = urb->actual_length;
    
    usb_free_urb(urb);
    return retval;
}




/**
 * usb_control_msg - Builds a control urb, sends it off and waits for completion
 * @dev: pointer to the usb device to send the message to
 * @pipe: endpoint "pipe" to send the message to
 * @request: USB message request value
 * @requesttype: USB message request type value
 * @value: USB message value
 * @index: USB message index value
 * @data: pointer to the data to send
 * @size: length in bytes of the data to send
 * @timeout: time in msecs to wait for the message to complete before timing
 *	out (if 0 the wait is forever)
 *
 * Context: !in_interrupt ()
 *
 * This function sends a simple control message to a specified endpoint and
 * waits for the message to complete, or timeout.
 *
 * If successful, it returns the number of bytes transferred, otherwise a
 * negative error number.
 *
 * Don't use this function from within an interrupt context, like a bottom half
 * handler.  If you need an asynchronous message, or need to send a message
 * from within interrupt context, use usb_submit_urb().
 * If a thread in your driver uses this call, make sure your disconnect()
 * method can wait for it to complete.  Since you don't have a handle on the
 * URB used, you can't cancel the request.
 */
int usb_control_msg(struct usb_device *dev, unsigned int pipe, u8 request,
		    u8 requesttype, u16 value, u16 index, void *data,
		    u16 size, int timeout)
{
	struct usb_ctrlrequest dr;
	int retv;
    int length;
    struct urb *urb;


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
}



/**
 * usb_bulk_msg - Builds a bulk urb, sends it off and waits for completion
 * @usb_dev: pointer to the usb device to send the message to
 * @pipe: endpoint "pipe" to send the message to
 * @data: pointer to the data to send
 * @len: length in bytes of the data to send
 * @actual_length: pointer to a location to put the actual length transferred
 *	in bytes
 * @timeout: time in msecs to wait for the message to complete before
 *	timing out (if 0 the wait is forever)
 *
 * Context: !in_interrupt ()
 *
 * This function sends a simple bulk message to a specified endpoint
 * and waits for the message to complete, or timeout.
 *
 * If successful, it returns 0, otherwise a negative error number.  The number
 * of actual bytes transferred will be stored in the actual_length paramater.
 */
int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
		 void *data, int len, int *actual_length, int timeout)
{
	struct urb *urb;
//	struct usb_host_endpoint *ep = NULL;
//    u8  bEndpointAddress,i;

//    bEndpointAddress = usb_pipeendpoint(pipe);
//    //search ep, control have two endpoint
//    for(i = 0; i < 16; i ++){
//        if(usb_dev->ep[i].desc.bEndpointAddress == bEndpointAddress){
//            ep = &usb_dev->ep[i];
//            break;
//        }      
//    }

//	ep = (usb_pipein(pipe) ? usb_dev->ep_in : usb_dev->ep_out)
//			[usb_pipeendpoint(pipe)];
    if (len < 0)
		return -EINVAL;
    

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return -ENOMEM;

//	if ((ep->desc.bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
//			USB_ENDPOINT_XFER_INT) {
//        USBH_DBG("usb_bulk_msg not support int endpoint");
//		pipe = (pipe & ~(3 << 30)) | (PIPE_INTERRUPT << 30);
//		usb_fill_int_urb(urb, usb_dev, pipe, data, len,
//				usb_api_blocking_completion, NULL,
//				ep->desc.bInterval);
//	} else
		usb_fill_bulk_urb(urb, usb_dev, pipe, data, len,
				usb_api_blocking_completion, NULL);

	return usb_start_wait_urb(urb, timeout, actual_length);
}



int usb_interrupt_msg(struct usb_device *usb_dev, unsigned int pipe,
		      void *data, int len, int *actual_length, int timeout)
{
    struct urb *urb;
	struct usb_host_endpoint *ep;

	ep = (usb_pipein(pipe) ? usb_dev->ep_in : usb_dev->ep_out)
			[usb_pipeendpoint(pipe)];
	if (!ep || len < 0)
		return -EINVAL;


    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb)
        return -ENOMEM;


 //   pipe = (pipe & ~(3 << 30)) | (PIPE_INTERRUPT << 30);
    usb_fill_int_urb(urb, usb_dev, pipe, data, len,
    		usb_api_blocking_completion, NULL, ep->desc.bInterval);


    return usb_start_wait_urb(urb, timeout, actual_length);
}



struct kill_urb_wait_queue {
    OS_SEM *wait_sem;
	struct list_head list;
};

void wake_up_kill_urb(struct list_head *head)
{
    struct kill_urb_wait_queue *wait_queue, *next;
    OS_ERR err;

    OSSchedLock(&err);
    list_for_each_entry_safe(wait_queue, next, head, list)
    {
        OSSemPost(wait_queue->wait_sem, OS_OPT_POST_1, &err);
        if(err != OS_ERR_NONE)
            usb_halt((struct usb_device *)NULL, "OSSemPost failed:%d\r\n",err);

        list_del(&wait_queue->list);
    }
    OSSchedUnlock(&err);
}

/**
 * usb_hcd_link_urb_to_ep - add an URB to its endpoint queue
 * @urb: URB being submitted
 */
int usb_hcd_link_urb_to_ep(struct urb *urb)
{
	int		rc = 0;
    OS_ERR err; 

	OSSchedLock(&err);

	/* Check that the URB isn't being killed */
	if (atomic_read(&urb->reject)) {
		rc = -EPERM;
		goto done;
	}

	if (!urb->ep->enabled) {
		rc = -ENOENT;
		goto done;
	}

//	if (unlikely(!urb->dev->can_submit)) {
//		rc = -EHOSTUNREACH;
//		goto done;
//	}
	/*
	 * dd the URB to the endpoint's queue.
	 */
	urb->unlinked = 0;
    list_add_tail(&urb->urb_list, &urb->ep->urb_list);
 done:
	OSSchedUnlock(&err);
	return rc;
}


int usb_check_fifo(struct usb_device *dev, struct usb_host_endpoint *endpoint)
{
    //check fifo space, the fifo space of control and bulk endpoint is enough.
    if((usb_endpoint_type(&endpoint->desc) == USB_ENDPOINT_XFER_ISOC)
        || (usb_endpoint_type(&endpoint->desc) == USB_ENDPOINT_XFER_INT))
    {
        int max  = le16_to_cpu(endpoint->desc.wMaxPacketSize);
        int mult = 1 + ((max >> 11) & 0x03);  
        max &= 0x07ff;
    
        if(usb_endpoint_dir_in(&endpoint->desc)) 
        {
            int rx_fifo_size;

            rx_fifo_size = USB_OTG_READ_REG32(&dev->USB_OTG_Core->regs.GREGS->GRXFSIZ);

            //see Reference manual 31.10.2
            if(dev->speed == USB_SPEED_HIGH)
                max = 2*((max + 3)/4 + 2);
            else
                max = ((max + 3)/4 + 2);

            if(rx_fifo_size < max)
              return -EMSGSIZE;
        }
        else
        {
            USB_OTG_FSIZ_TypeDef ptxfifosize;  

            ptxfifosize.d32 = USB_OTG_READ_REG32(&dev->USB_OTG_Core->regs.GREGS->HPTXFSIZ); 
            //see Reference manual 31.10.2
            if((dev->speed == USB_SPEED_HIGH) && (mult > 1))
                max = 2*((max + 3)/4 + 2); //only in dma mode and multi transaction transfer in every frame.
            else
                max = ((max + 3)/4 + 2);

            if(ptxfifosize.b.depth < max)
              return -EMSGSIZE;       
        }
    }
//    else if(usb_endpoint_type(&endpoint->desc) == USB_ENDPOINT_XFER_INT)
//    {
//      if(usb_endpoint_dir_in(&endpoint->desc)) 
//      {
//          int rx_fifo_size;
//          
//          rx_fifo_size = USB_OTG_READ_REG32(&dev->USB_OTG_Core->regs.GREGS->GRXFSIZ);
//          //see Reference manual 31.10.2
//          if(rx_fifo_size < ((max + 3)/4 + 2))
//          {
//              return -ENOMEM;
//          }
//      }
//      else
//      {
//          USB_OTG_FSIZ_TypeDef ptxfifosize;  
//          //see Reference manual 31.10.2
//          ptxfifosize.d32 = USB_OTG_READ_REG32(&dev->USB_OTG_Core->regs.GREGS->HPTXFSIZ);     
//          if(ptxfifosize.b.depth < ((max + 3)/4 + 1))
//          {
//              return -ENOMEM;
//          }
//      }
//    }

    return 0;
}




/**
 * usb_submit_urb - issue an asynchronous transfer request for an endpoint
 * @urb: pointer to the urb describing the request
 * @mem_flags: not important, only compatible with Linux API.
 *
 * This submits a transfer request, and transfers control of the URB
 * describing that request to the USB subsystem.  Request completion will
 * be indicated later, asynchronously, by calling the completion handler.
 * The three types of completion are success, error, and unlink
 * (a software-induced fault, also called "request cancellation").
 *
 *
 * The caller must have correctly initialized the URB before submitting
 * it.  Functions such as usb_fill_bulk_urb() and usb_fill_control_urb() are
 * available to ensure that most fields are correctly initialized, for
 * the particular kind of transfer, although they will not initialize
 * any transfer flags.
 *
 * Successful submissions return 0; otherwise this routine returns a
 * negative error number.  If the submission is successful, the complete()
 * callback from the URB will be called exactly once, when the USB core and
 * Host Controller Driver (HCD) are finished with the URB.  When the completion
 * function is called, control of the URB is returned to the device
 * driver which issued the request.  The completion handler may then
 * immediately free or reuse that URB.
 *
 * With few exceptions, USB device drivers should never access URB fields
 * provided by usbcore or the HCD until its complete() is called.
 * The exceptions relate to periodic transfer scheduling.  For both
 * interrupt and isochronous urbs, as part of successful URB submission
 * urb->interval is modified to reflect the actual transfer period used
 * (normally some power of two units).  
 *
 * For control endpoints, the synchronous usb_control_msg() call is
 * often used (in non-interrupt context) instead of this call.
 * That is often used through convenience wrappers, for the requests
 * that are standardized in the USB 2.0 specification.  For bulk
 * endpoints, a synchronous usb_bulk_msg() call is available. 
 *
 */
int usb_submit_urb(struct urb *urb, u8 mem_flags)
{
	int				xfertype;
	struct usb_device		*dev;
	struct usb_host_endpoint	*ep = NULL;
    void *buffer = NULL;
    int   is_out;
    OS_ERR err;
    CPU_SR cpu_sr;

    
	if (!urb || !urb->complete)
		return -EINVAL;
	dev = urb->dev;
	if (!dev)
		return -ENODEV;

    if(urb->priv_state != USBH_URB_IDLE)
        usb_halt(urb->dev, "urb->priv_state:%d != USBH_URB_IDLE\r\n",urb->priv_state);

    if(dev->state < USB_STATE_DEFAULT)
    {
        usb_dbg(urb->dev,"usb_submit_urb Device Removed! urb:%p dev->state:%d ConnSts:%d \r\n",urb,dev->state,urb->dev->USB_OTG_Core->host.ConnSts);
        return -ENODEV;
    }
       
    /* For now, get the endpoint from the pipe.  Eventually drivers
     * will be required to set urb->ep directly and we will eliminate
     * urb->pipe.
     */
    ep = (usb_pipein(urb->pipe) ? dev->ep_in : dev->ep_out)
         [usb_pipeendpoint(urb->pipe)];
	if (!ep)
	{
        usb_dbg(dev, "usb_submit_urb ep is NULL \r\n");
		return -ENOENT;
	}


    CPU_CRITICAL_ENTER();//judge if already submit urb
    if(urb->priv_state != USBH_URB_IDLE){
       CPU_CRITICAL_EXIT();
       usb_halt(dev, "urb already submited urb:%p priv_state:%d\r\n",urb,urb->priv_state);
    }
//    urb->priv_state = USBH_URB_WAIT_PROGRESS;
    CPU_CRITICAL_EXIT();
    

    INIT_LIST_HEAD(&urb->urb_list);
    
	urb->ep = ep;
 	urb->status = -EINPROGRESS;
	urb->actual_length = 0;


    /* Lots of sanity checks, so HCDs can rely on clean data
     * and don't need to duplicate tests
     */
    xfertype = usb_endpoint_type(&ep->desc);
    if (xfertype == USB_ENDPOINT_XFER_CONTROL)
    {
        struct usb_ctrlrequest *setup =
            (struct usb_ctrlrequest *) urb->setup_packet;

        is_out = !(setup->bRequestType & USB_DIR_IN) ||
                 !setup->wLength;
    }
    else
    {
        is_out = usb_endpoint_dir_out(&ep->desc);
    }

    /* Cache the direction for later use */
    urb->transfer_flags = (urb->transfer_flags & ~URB_DIR_MASK) |
                          (is_out ? URB_DIR_OUT : URB_DIR_IN);

    //The flags is valid only for the dma disabled device.
    urb->transfer_flags |= URB_NO_SEND_DATA_IN_INTTERUPT; 
    urb->transfer_flags |= URB_NO_RECV_NAK_REACTIVE_IN_INTTERUPT;


    if (xfertype != USB_ENDPOINT_XFER_CONTROL &&
        dev->state < USB_STATE_CONFIGURED)
        return -ENODEV;


    if((xfertype != USB_ENDPOINT_XFER_CONTROL)
        &&((urb->transfer_buffer == NULL) || (urb->transfer_buffer_length == 0)))
        return -EFAULT;   
    

	if (xfertype == USB_ENDPOINT_XFER_CONTROL) {
        memcpy(&urb->setup_request, urb->setup_packet, sizeof(struct usb_ctrlrequest));
        CPU_CRITICAL_ENTER();
        urb->MachineState = USBH_CONTRL_MACHINE;
        urb->MsgType = USBH_CTRL_SETUP_REQ;
        CPU_CRITICAL_EXIT();

    }else if(xfertype == USB_ENDPOINT_XFER_BULK)
    {
        CPU_CRITICAL_ENTER();
        urb->MachineState = USBH_BULK_INT_MACHINE;
        urb->MsgType= usb_endpoint_dir_in(&ep->desc)?USBH_BULK_INT_SUBMIT_RCV_REQ : USBH_BULK_INT_SUBMIT_SENT_REQ;
        CPU_CRITICAL_EXIT();
    }
    else //USB_ENDPOINT_XFER_ISOC 
    {
        /* periodic transfers limit size per frame/uframe,
        * but drivers only control those sizes for ISO.
        * while we're checking, initialize return status.
        */
        int n, len;
        int mult;
        int max = le16_to_cpu(ep->desc.wMaxPacketSize);

        /* "high bandwidth" mode, 1-3 packets/uframe? */
        if (dev->speed == USB_SPEED_HIGH)
        {
            mult = 1 + ((max >> 11) & 0x03);
            max &= 0x07ff;
            max *= mult;

            if(((max > 1024) || (mult > 1)) && !dev->USB_OTG_Core->cfg.dma_enable)
            {
                usb_dbg(dev, "wMaxPacketSize:%d is more than 1024, but usb dma is not enabled\r\n", max);
                return -EMSGSIZE;
            }
        }
        
        usb_get_current_frame_number(dev);
        urb->start_frame = dev->frame_number;
        
        if(xfertype == USB_ENDPOINT_XFER_ISOC) 
        {
            if (urb->number_of_packets <= 0)
                return -EINVAL;
            for (n = 0; n < urb->number_of_packets; n++)
            {
                len = urb->iso_frame_desc[n].length;
                if (len < 0 || len > max)
                    return -EMSGSIZE;

                urb->iso_frame_desc[n].status = -EXDEV;
                urb->iso_frame_desc[n].actual_length = 0;
            }      
            
            urb->iso_frame_index = 0;
            CPU_CRITICAL_ENTER();
            urb->MachineState = USBH_ISOC_MACHINE;
            urb->MsgType= usb_endpoint_dir_in(&ep->desc)?USBH_ISOC_SUBMIT_RCV_REQ : USBH_ISOC_SUBMIT_SENT_REQ;
            CPU_CRITICAL_EXIT();
       }
       else
       {
            CPU_CRITICAL_ENTER();
            urb->MachineState = USBH_BULK_INT_MACHINE;
            urb->MsgType= usb_endpoint_dir_in(&ep->desc)?USBH_BULK_INT_SUBMIT_RCV_REQ : USBH_BULK_INT_SUBMIT_SENT_REQ;
            CPU_CRITICAL_EXIT();           
       }
    }

    if(usb_check_fifo(dev, ep) != 0)
    {
        usb_dbg(dev, "usb_submit_urb usb_check_fifo failed\r\n");
        return -EMSGSIZE;
    }

    if(is_out)
    {
        if((urb->transfer_buffer_length != 0) && !(urb->transfer_flags & URB_FREE_BUFFER)){
            buffer = kmalloc(urb->transfer_buffer_length,0);
            if(buffer == NULL)
            {
                usb_dbg(dev, "usb_submit_urb mem_malloc Failed LINE:%d\r\n",__LINE__);
                return -ENOSPC;
            }

            memcpy(buffer, urb->transfer_buffer, urb->transfer_buffer_length);
            urb->transfer_buffer = buffer;
        }
    }


    if(dev->USB_OTG_Core->cfg.dma_enable && (int)urb->transfer_buffer&0x3)
    {
        //if using -O2 optimzation, urb->transfer_buffer may not be aligned to 4 bytes.     
        //In this case, you can add the prefix of __attribute__((aligned)) to the transfer_buffer that will be submited.
        usb_halt(dev, "urb->transfer_buffer must be aligned to 4 bytes in dma mode\r\n");
    }

    int         status;

   /* increment urb's reference count as part of giving it to the HCD
    * (which will control it).  HCD guarantees that it either returns
    * an error or calls giveback(), but not both.
    */
    usb_get_urb(urb);


    OSSchedLock(&err);
    
    atomic_inc(&urb->use_count);
    status = usb_hcd_link_urb_to_ep(urb);   
    if(status)
     goto error;

    urb->priv_state = USBH_URB_WAIT_PROGRESS;
    urb->error_count = 0;

    if((ep->state != USBH_EP_IDLE)
       && (!list_empty(&ep->urb_list)))
    {
       usb_halt(dev, "ep->state:%d but ep urb_list is empty stop!!\r\n",ep->state);
    }

    if(ep->state == USBH_EP_IDLE) 
    {
       USBH_EpEnqueue(&ep->xfer_mgt->ep_queue, ep);   
    }
    OSSchedUnlock(&err);    


    CPU_CRITICAL_ENTER();
    status = dev->usb_ch_mem.NbrFree;
    CPU_CRITICAL_EXIT();
    
    if(status)
    {       
        usb_get_dev(dev);
        OSTaskQPost(&USBHTaskTCB, dev, USBH_PROCESS_SCHED, OS_OPT_POST_FIFO, &err);      
        if(err != OS_ERR_NONE){
            usb_put_dev(dev);
            /*if failed ?*/
            /*TODO*/
        }
    }
    return 0;

error:       
   INIT_LIST_HEAD(&urb->urb_list);
   atomic_dec(&urb->use_count);
   OSSchedUnlock(&err);

   if (atomic_read(&urb->reject))
       wake_up_kill_urb(&usb_kill_urb_queue);
   usb_put_urb(urb);

   if(buffer)
      kfree(buffer);
   return status;
}



int usb_hcd_check_unlink_urb(struct urb *urb, int status)
{
	struct list_head	*tmp;

    if(urb->priv_state == USBH_URB_IDLE)
        usb_halt(urb->dev, "urb->priv_state:%d == USBH_URB_IDLE\r\n",urb->priv_state);
    
	/* insist the urb is still queued */
	list_for_each(tmp, &urb->ep->urb_list) {
		if (tmp == &urb->urb_list)
			break;
	}
	if (tmp != &urb->urb_list)
		usb_halt(urb->dev, "urb:%p is not in endpoing queued\r\n",urb);//return -EIDRM;

	/* Any status except -EINPROGRESS means something already started to
	 * unlink this URB from the hardware.  So there's no more work to do.
	 */
	if (urb->unlinked)
		return -EBUSY;
	urb->unlinked = status;

	return 0;
}


/*
 * called in any context
 *
 * caller guarantees urb won't be recycled till both unlink()
 * and the urb's completion function return
 */
int usb_hcd_unlink_urb (struct urb *urb, int status)
{
	int			retval = -EIDRM;
	OS_ERR err;;

	/* Prevent the device and bus from going away while
	 * the unlink is carried out.  If they are already gone
	 * then urb->use_count must be 0, since disconnected
	 * devices can't have any active URBs.
	 */
	OSSchedLock(&err);
	if (atomic_read(&urb->use_count) > 0) {
        retval = usb_hcd_check_unlink_urb(urb, status);
	}
    OSSchedUnlock(&err);


	if (retval == 0)
		retval = -EINPROGRESS;
	else if (retval != -EIDRM && retval != -EBUSY)
		usb_dbg(urb->dev, "hcd_unlink_urb %p fail %d\n",
				urb, retval);
	return retval;
}



int usb_unlink_urb(struct urb *urb)
{
//     struct usbh_task_req *task_req;
    usb_dbg(urb->dev, "usb_unlink_urb urb:%p\r\n",urb);
	if (!urb)
		return -EINVAL;
	if (!urb->dev)
		return -ENODEV;
	if (!urb->ep)
		return -EIDRM;

    return usb_hcd_unlink_urb(urb, -ECONNRESET);
}





void __wait_kill_urb_event(OS_SEM *sem, struct list_head *head)
{
    struct kill_urb_wait_queue wait;
    OS_ERR err;

    OSSchedLock(&err);
    OSSemSet(sem, 0, &err);
    if(err != OS_ERR_NONE)
        usb_halt((struct usb_device *)NULL, "OSSemSet err:%d != OS_ERR_NONE\r\n",err);
    
    wait.wait_sem = sem;   
    INIT_LIST_HEAD(&wait.list);
    list_add_tail(&wait.list, head);
    OSSchedUnlock(&err);
    
    OSSemPend(sem, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    if(err != OS_ERR_NONE)
        usb_halt((struct usb_device *)NULL, "OSSemPend err:%d != OS_ERR_NONE\r\n",err);
}

/**
 * usb_kill_urb - cancel a transfer request and wait for it to finish
 * @urb: pointer to URB describing a previously submitted request,
 *  may be NULL
 *
 * This routine cancels an in-progress request.  It is guaranteed that
 * upon return all completion handlers will have finished and the URB
 * will be totally idle and available for reuse.  These features make
 * this an ideal way to stop I/O in a disconnect() callback or close()
 * function.  If the request has not already finished or been unlinked
 * the completion handler will see urb->status == -ENOENT.  
 *
 * This routine should not be called by a driver after its disconnect
 * method has returned.
 */
void usb_kill_urb(struct urb *urb)
{
//	might_sleep();
    OS_ERR err;
    OS_SEM sem;

    if (!(urb && urb->dev && urb->ep))
        return;

    OSSemCreate(&sem, "kill urb wait sem", 0, &err);
    
    atomic_inc(&urb->reject);

    usb_hcd_unlink_urb(urb, -ENOENT);


    for(;;){
        if (atomic_read(&urb->use_count) == 0)
            break;                          \
        __wait_kill_urb_event(&sem, &usb_kill_urb_queue);
    }


    atomic_dec(&urb->reject);
    
    OSSemDel(&sem, OS_OPT_DEL_ALWAYS, &err);
}



/*-------------------------------------------------------------------------*/

/* Cancel all URBs pending on this endpoint and wait for the endpoint's
 * queue to drain completely.  The caller must first insure that no more
 * URBs can be submitted for this endpoint.
 */
void usb_hcd_flush_endpoint(struct usb_device *udev,
		struct usb_host_endpoint *ep)
{
	struct urb		*urb;
    CPU_SR cpu_sr;

	if (!ep)
		return;

	/* Wait until the endpoint queue is completely empty */
	while (!list_empty (&ep->urb_list)) {
		CPU_CRITICAL_ENTER();

		/* The list may have changed while we acquired the spinlock */
		urb = NULL;
		if (!list_empty (&ep->urb_list)) {
			urb = list_entry (ep->urb_list.prev, struct urb,
					urb_list);
			usb_get_urb (urb);
		}
		CPU_CRITICAL_EXIT();

		if (urb) {
            int is_in = usb_urb_dir_in(urb);
            usb_dbg (udev,
                "shutdown urb %p ep%d%s%s\n",
                urb, usb_endpoint_num(&ep->desc),
                is_in ? "in" : "out",
                ({  char *s;
            
                     switch (usb_endpoint_type(&ep->desc)) {
                     case USB_ENDPOINT_XFER_CONTROL:
                        s = ""; break;
                     case USB_ENDPOINT_XFER_BULK:
                        s = "-bulk"; break;
                     case USB_ENDPOINT_XFER_INT:
                        s = "-intr"; break;
                     default:
                        s = "-iso"; break;
                    };
                    s;
                }));

            
			usb_kill_urb (urb);
			usb_put_urb (urb);
		}
	}
}

void usb_disable_endpoint(struct usb_device *dev, unsigned int epaddr,
		int reset_hardware)
{
	unsigned int epnum = epaddr & USB_ENDPOINT_NUMBER_MASK;
	struct usb_host_endpoint *ep;

	if (!dev)
		return;

	if (usb_endpoint_out(epaddr)) {
		ep = dev->ep_out[epnum];
		if (reset_hardware)
			dev->ep_out[epnum] = NULL;
	} else {
		ep = dev->ep_in[epnum];
		if (reset_hardware)
			dev->ep_in[epnum] = NULL;
	}
	if (ep) {
        ep->enabled = 0;
		usb_hcd_flush_endpoint(dev, ep);
	}
}

void usb_hcd_reset_endpoint(struct usb_device *udev,
			    struct usb_host_endpoint *ep)
{
	int epnum = usb_endpoint_num(&ep->desc);
	int is_out = usb_endpoint_dir_out(&ep->desc);
	int is_control = usb_endpoint_xfer_control(&ep->desc);

	usb_settoggle(udev, epnum, is_out, 0);
	if (is_control)
		usb_settoggle(udev, epnum, !is_out, 0);
}


void usb_disable_interface(struct usb_device *dev, struct usb_interface *intf,
		int reset_hardware)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i) {
		usb_disable_endpoint(dev,
				alt->endpoint[i].desc.bEndpointAddress,
				reset_hardware);
	}
}

/**
 * usb_disable_device - Disable all the endpoints for a USB device
 * @dev: the device whose endpoints are being disabled
 * @skip_ep0: 0 to disable endpoint 0, 1 to skip it.
 *
 * Disables all the device's endpoints, potentially including endpoint 0.
 * Deallocates hcd/hardware state for the endpoints (nuking all or most
 * pending urbs) and usbcore state for the interfaces, so that usbcore
 * must usb_set_configuration() before any interfaces could be used.
 */
void usb_disable_device(struct usb_device *dev, int skip_ep0)
{
	int i;

	usb_dbg(dev, "%s nuking %s URBs\n", __func__,
		skip_ep0 ? "non-ep0" : "all");
	for (i = skip_ep0; i < 16; ++i) {
		usb_disable_endpoint(dev, i, 1);
		usb_disable_endpoint(dev, i + USB_DIR_IN, 1);
	}

	/* getting rid of interfaces will disconnect
	 * any drivers bound to them (a key side effect)
	 */
	if (dev->actconfig) {
        
		for (i = 0; i < dev->actconfig->desc.bNumInterfaces; i++) {
			struct usb_interface	*interface;

			/* remove this interface if it has been registered */
			interface = dev->actconfig->interface[i];
//			if (!device_is_registered(&interface->dev))
//				continue;
			usb_dbg(dev, "unregistering interface %s\n",interface->name);
//			interface->unregistering = 1;

            if(interface->driver)
            {
                usb_do_hotplug(interface, 0); 
                
                interface->condition = USB_INTERFACE_UNBINDING;

                interface->driver->disconnect(interface);

                usb_set_intfdata(interface, NULL);

                interface->condition = USB_INTERFACE_UNBOUND;
                
            }
            interface->driver = NULL;
            
            usb_put_dev(interface->usb_dev);

//			device_del(&interface->dev);
		}

		/* Now that the interfaces are unbound, nobody should
		 * try to access them.
		 */
		for (i = 0; i < dev->actconfig->desc.bNumInterfaces; i++) {
			usb_put_intf(dev->actconfig->interface[i]);
			dev->actconfig->interface[i] = NULL;
		}
		dev->actconfig = NULL;
		if (dev->state == USB_STATE_CONFIGURED)
			usb_set_device_state(dev, USB_STATE_ADDRESS);
	}
}



void usb_enable_endpoint(struct usb_device *dev, struct usb_host_endpoint *ep,
		int reset_ep)
{
	int epnum = usb_endpoint_num(&ep->desc);
	int is_out = usb_endpoint_dir_out(&ep->desc);
	int is_control = usb_endpoint_xfer_control(&ep->desc);

	if (reset_ep)
        usb_hcd_reset_endpoint(dev,ep);
	if (is_out || is_control)
		dev->ep_out[epnum] = ep;
	if (!is_out || is_control)
		dev->ep_in[epnum] = ep;
    ep->enabled = 1;
}

void usb_enable_interface(struct usb_device *dev,
		struct usb_interface *intf, int reset_eps)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i)
		usb_enable_endpoint(dev, &alt->endpoint[i], reset_eps);
}


void usb_reset_endpoint(struct usb_device *dev, unsigned int epaddr)
{
 	unsigned int epnum = epaddr & USB_ENDPOINT_NUMBER_MASK;
	struct usb_host_endpoint *ep;

	if (usb_endpoint_out(epaddr))
		ep = dev->ep_out[epnum];
	else
		ep = dev->ep_in[epnum];
	if (ep)
        usb_hcd_reset_endpoint(dev,ep);
}


/**
 * usb_clear_halt - tells device to clear endpoint halt/stall condition
 * @dev: device whose endpoint is halted
 * @pipe: endpoint "pipe" being cleared
 * Context: !in_interrupt ()
 *
 * This is used to clear halt conditions for bulk and interrupt endpoints,
 * as reported by URB completion status.  Endpoints that are halted are
 * sometimes referred to as being "stalled".  Such endpoints are unable
 * to transmit or receive data until the halt status is cleared.  Any URBs
 * queued for such an endpoint should normally be unlinked by the driver
 * before clearing the halt condition, as described in sections 5.7.5
 * and 5.8.5 of the USB 2.0 spec.
 *
 * Note that control and isochronous endpoints don't halt, although control
 * endpoints report "protocol stall" (for unsupported requests) using the
 * same status code used to report a true stall.
 *
 * This call is synchronous, and may not be used in an interrupt context.
 *
 * Returns zero on success, or else the status code returned by the
 * underlying usb_control_msg() call.
 */
int usb_clear_halt(struct usb_device *dev, int pipe)
{
	int result;
	int endp = usb_pipeendpoint(pipe);

	if (usb_pipein(pipe))
		endp |= USB_DIR_IN;

	/* we don't care if it wasn't halted first. in fact some devices
	 * (like some ibmcam model 1 units) seem to expect hosts to make
	 * this request for iso endpoints, which can't halt!
	 */
	result = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
		USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT,
		USB_ENDPOINT_HALT, endp, NULL, 0,
		USB_CTRL_SET_TIMEOUT);




	/* don't un-halt or force to DATA0 except on success */
	if (result < 0)
		return result;

	/* NOTE:  seems like Microsoft and Apple don't bother verifying
	 * the clear "took", so some devices could lock up if you check...
	 * such as the Hagiwara FlashGate DUAL.  So we won't bother.
	 *
	 * NOTE:  make sure the logic here doesn't diverge much from
	 * the copy in usb-storage, for as long as we need two copies.
	 */

	usb_reset_endpoint(dev, endp);

	return 0;
}




int usb_get_string(struct usb_device *dev, unsigned short langid,
			  unsigned char index, void *buf, int size)
{
	int i;
	int result;

	for (i = 0; i < 3; ++i) {
		/* retry on length 0 or stall; some devices are flakey */
		result = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
			(USB_DT_STRING << 8) + index, langid, buf, size,
			USB_CTRL_GET_TIMEOUT);
		if (result == 0 || result == -EPIPE)
			continue;
		if (result > 1 && ((u8 *) buf)[1] != USB_DT_STRING) {
			result = -ENODATA;
			continue;
		}
		break;
	}
	return result;
}

int usb_string(struct usb_device *dev, int index, char *buf, size_t size)
{
	int err;
	unsigned int u, idx;

	if (size <= 0 || !buf || !index)
		return -EINVAL;
	buf[0] = 0;

	err = usb_get_string(dev, 0x0409, index, buf, size);
	if (err < 0)
		goto errout;

	size--;		/* leave room for trailing NULL char in output buffer */
	for (idx = 0, u = 2; u < err; u += 2) {
		if (idx >= size)
			break;
		if (buf[u+1])			/* high byte */
			buf[idx++] = '?';  /* non ISO-8859-1 character */
		else
			buf[idx++] = buf[u];
	}
	buf[idx] = 0;
	err = idx;

 errout:
	return err;
}


/**
 * usb_get_descriptor - issues a generic GET_DESCRIPTOR request
 * @dev: the device whose descriptor is being retrieved
 * @type: the descriptor type (USB_DT_*)
 * @index: the number of the descriptor
 * @buf: where to put the descriptor
 * @size: how big is "buf"?
 * Context: !in_interrupt ()
 *
 * Gets a USB descriptor.  Convenience functions exist to simplify
 * getting some types of descriptors.  Use
 * usb_get_string() or usb_string() for USB_DT_STRING.
 * Device (USB_DT_DEVICE) and configuration descriptors (USB_DT_CONFIG)
 * are part of the device structure.
 * In addition to a number of USB-standard descriptors, some
 * devices also use class-specific or vendor-specific descriptors.
 *
 * This call is synchronous, and may not be used in an interrupt context.
 *
 * Returns the number of bytes received on success, or else the status code
 * returned by the underlying usb_control_msg() call.
 */
int usb_get_descriptor(struct usb_device *dev, unsigned char type,
		       unsigned char index, void *buf, int size)
{
	int i;
	int result;

	memset(buf, 0, size);	/* Make sure we parse really received data */

	for (i = 0; i < 3; ++i) {
		/* retry on length 0 or error; some devices are flakey */
		result = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
				USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
				(type << 8) + index, 0, buf, size,
				USB_CTRL_GET_TIMEOUT);
		if (result <= 0 && result != -ETIMEDOUT)
			continue;
		if (result > 1 && ((u8 *)buf)[1] != type) {
			result = -ENODATA;
			continue;
		}
		break;
	}
	return result;
}


int usb_get_device_descriptor(struct usb_device *dev, unsigned int size)
{
	struct usb_device_descriptor desc;
	int ret;

	if (size > sizeof(desc))
		return -EINVAL;

	ret = usb_get_descriptor(dev, USB_DT_DEVICE, 0, &desc, size);
	if (ret >= 0)
		memcpy(&dev->descriptor, &desc, size);
    
	return ret;
}


/**
 * usb_ifnum_to_if - get the interface object with a given interface number
 * @dev: the device whose current configuration is considered
 * @ifnum: the desired interface
 *
 * This walks the device descriptor for the currently active configuration
 * and returns a pointer to the interface with that particular interface
 * number, or null.
 *
 * Note that configuration descriptors are not required to assign interface
 * numbers sequentially, so that it would be incorrect to assume that
 * the first interface in that descriptor corresponds to interface zero.
 * This routine helps device drivers avoid such mistakes.
 * However, you should make sure that you do the right thing with any
 * alternate settings available for this interfaces.
 *
 * Don't call this function unless you are bound to one of the interfaces
 * on this device or you have locked the device!
 */
struct usb_interface *usb_ifnum_to_if(const struct usb_device *dev,
				      unsigned ifnum)
{
	struct usb_host_config *config = dev->actconfig;
	int i;

	if (!config)
		return NULL;
	for (i = 0; i < config->desc.bNumInterfaces; i++)
		if (config->interface[i]->altsetting[0]
				.desc.bInterfaceNumber == ifnum)
			return config->interface[i];

	return NULL;
}


/**
 * usb_set_interface - Makes a particular alternate setting be current
 * @dev: the device whose interface is being updated
 * @interface: the interface being updated
 * @alternate: the setting being chosen.
 * Context: !in_interrupt ()
 *
 * This is used to enable data transfers on interfaces that may not
 * be enabled by default.  Not all devices support such configurability.
 * Only the driver bound to an interface may change its setting.
 *
 * Within any given configuration, each interface may have several
 * alternative settings.  These are often used to control levels of
 * bandwidth consumption.  For example, the default setting for a high
 * speed interrupt endpoint may not send more than 64 bytes per microframe,
 * while interrupt transfers of up to 3KBytes per microframe are legal.
 * Also, isochronous endpoints may never be part of an
 * interface's default setting.  To access such bandwidth, alternate
 * interface settings must be made current.
 * 
 *
 * This call is synchronous, and may not be used in an interrupt context.
 * Also, drivers must not change altsettings while urbs are scheduled for
 * endpoints in that interface; all such urbs must first be completed
 * (perhaps forced by unlinking).
 *
 * Returns zero on success, or else the status code returned by the
 * underlying usb_control_msg() call.
 */
int usb_set_interface(struct usb_device *dev, int interface, int alternate)
{
	struct usb_interface *iface;
	struct usb_host_interface *alt;
	int ret;
	int manual = 0;
	unsigned int epaddr;
	unsigned int pipe;

//	if (dev->state == USB_STATE_SUSPENDED)
//		return -EHOSTUNREACH;

	iface = usb_ifnum_to_if(dev, interface);
	if (!iface) {
		usb_dbg(dev, "selecting invalid interface %d\n",
			interface);
		return -EINVAL;
	}

	alt = usb_altnum_to_altsetting(iface, alternate);
	if (!alt) {
		usb_warn(dev, "selecting invalid altsetting %d",
			 alternate);
		return -EINVAL;
	}


	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			   USB_REQ_SET_INTERFACE, USB_RECIP_INTERFACE,
			   alternate, interface, NULL, 0, 5000);

	/* 9.4.10 says devices don't need this and are free to STALL the
	 * request if the interface only has one alternate setting.
	 */
	if (ret == -EPIPE && iface->num_altsetting == 1) {
		usb_dbg(dev,
			"manual set_interface for iface %d, alt %d\n",
			interface, alternate);
		manual = 1;
	} else if (ret < 0)
		return ret;

	/* FIXME drivers shouldn't need to replicate/bugfix the logic here
	 * when they implement async or easily-killable versions of this or
	 * other "should-be-internal" functions (like clear_halt).
	 * should hcd+usbcore postprocess control requests?
	 */


	usb_disable_interface(dev, iface, 1);

	iface->cur_altsetting = alt;

	/* If the interface only has one altsetting and the device didn't
	 * accept the request, we attempt to carry out the equivalent action
	 * by manually clearing the HALT feature for each endpoint in the
	 * new altsetting.
	 */
	if (manual) {
		int i;

		for (i = 0; i < alt->desc.bNumEndpoints; i++) {
			epaddr = alt->endpoint[i].desc.bEndpointAddress;
			pipe = __create_pipe(dev,
					USB_ENDPOINT_NUMBER_MASK & epaddr) |
					(usb_endpoint_out(epaddr) ?
					USB_DIR_OUT : USB_DIR_IN);

			usb_clear_halt(dev, pipe);
		}
	}

	/* 9.1.1.5: reset toggles for all endpoints in the new altsetting
	 *
	 * Note:
	 * Despite EP0 is always present in all interfaces/AS, the list of
	 * endpoints from the descriptor does not contain EP0. Due to its
	 * omnipresence one might expect EP0 being considered "affected" by
	 * any SetInterface request and hence assume toggles need to be reset.
	 * However, EP0 toggles are re-synced for every individual transfer
	 * during the SETUP stage - hence EP0 toggles are "don't care" here.
	 * (Likewise, EP0 never "halts" on well designed devices.)
	 */
	usb_enable_interface(dev, iface, 1);
	return 0;
}


/*
 * usb_set_configuration - Makes a particular device setting be current
 * @dev: the device whose configuration is being updated
 * @configuration: the configuration being chosen.
 * Context: !in_interrupt(), caller owns the device lock
 *
 * This is used to enable non-default device modes.  Not all devices
 * use this kind of configurability; many devices only have one
 * configuration.
 *
 * @configuration is the value of the configuration to be installed.
 * According to the USB spec (e.g. section 9.1.1.5), configuration values
 * must be non-zero; a value of zero indicates that the device in
 * unconfigured.  However some devices erroneously use 0 as one of their
 * configuration values.  To help manage such devices, this routine will
 * accept @configuration = -1 as indicating the device should be put in
 * an unconfigured state. 
 *
 * Note that USB has an additional level of device configurability,
 * associated with interfaces.  That configurability is accessed using
 * usb_set_interface().
 *
 * This call is synchronous. The calling context must be able to sleep,
 * must own the device lock, and must not hold the driver model's USB
 * bus mutex; usb interface driver probe() methods cannot use this routine.
 *
 * Returns zero on success, or else the status code returned by the
 * underlying call that failed.  On successful completion, each interface
 * in the original device configuration has been destroyed, and each one
 * in the new configuration has been probed by all relevant usb device
 * drivers currently known to the kernel.
 */
int usb_set_configuration(struct usb_device *dev, int configuration)
{
	int i, ret;
	struct usb_host_config *cp = NULL;
	struct usb_interface **new_interfaces = NULL;
	int n, nintf;

	if (configuration == -1)
		configuration = 0;
	else {
		for (i = 0; i < dev->descriptor.bNumConfigurations; i++) {
			if (dev->config[i].desc.bConfigurationValue ==
					configuration) {
				cp = &dev->config[i];
				break;
			}
		}
	}
	if ((!cp && configuration != 0))
		return -EINVAL;

	/* The USB spec says configuration 0 means unconfigured.
	 * But if a device includes a configuration numbered 0,
	 * we will accept it as a correctly configured state.
	 * Use -1 if you really want to unconfigure the device.
	 */
	if (cp && configuration == 0)
		usb_warn(dev, "config 0 descriptor??\n");

	/* Allocate memory for new interfaces before doing anything else,
	 * so that if we run out then nothing will have changed. */
	n = nintf = 0;
	if (cp) {
		nintf = cp->desc.bNumInterfaces;
		new_interfaces = kmalloc(nintf * sizeof(*new_interfaces),
				GFP_KERNEL);
		if (!new_interfaces) {
			usb_err(dev, "Out of memory\n");
			return -ENOMEM;
		}

		for (; n < nintf; ++n) {
			new_interfaces[n] = kzalloc(
					sizeof(struct usb_interface),
					GFP_KERNEL);
			if (!new_interfaces[n]) {
				usb_err(dev, "Out of memory\n");
				ret = -ENOMEM;
free_interfaces:
				while (--n >= 0)
					kfree(new_interfaces[n]);
				kfree(new_interfaces);
				return ret;
			}
		}


	}



	/* if it's already configured, clear out old state first.
	 * getting rid of old interfaces means unbinding their drivers.
	 */
	if (dev->state != USB_STATE_ADDRESS)
		usb_disable_device(dev, 1);	/* Skip ep0 */

	/* Get rid of pending async Set-Config requests for this device */
//	cancel_async_set_config(dev);

	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			      USB_REQ_SET_CONFIGURATION, 0, configuration, 0,
			      NULL, 0, USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		/* All the old state is gone, so what else can we do?
		 * The device is probably useless now anyway.
		 */
		cp = NULL;
	}

	dev->actconfig = cp;
	if (!cp) {
		usb_set_device_state(dev, USB_STATE_ADDRESS);
		goto free_interfaces;
	}
	usb_set_device_state(dev, USB_STATE_CONFIGURED);

	/* Initialize the new interface structures and the
	 * hc/hcd/usbcore interface/endpoint state.
	 */
	for (i = 0; i < nintf; ++i) {
		struct usb_interface_cache *intfc;
		struct usb_interface *intf;
		struct usb_host_interface *alt;

		cp->interface[i] = intf = new_interfaces[i];
		intfc = cp->intf_cache[i];
		intf->altsetting = intfc->altsetting;
		intf->num_altsetting = intfc->num_altsetting;
		kref_get(&intfc->ref);
        

		alt = usb_altnum_to_altsetting(intf, 0);

		/* No altsetting 0?  We'll assume the first altsetting.
		 * We could use a GetInterface call, but if a device is
		 * so non-compliant that it doesn't have altsetting 0
		 * then I wouldn't trust its reply anyway.
		 */
		if (!alt)
			alt = &intf->altsetting[0];

		intf->cur_altsetting = alt;
		usb_enable_interface(dev, intf, 1);
//		intf->dev.parent = &dev->dev;
//		intf->dev.driver = NULL;
//		intf->dev.bus = &usb_bus_type;
//		intf->dev.type = &usb_if_device_type;
//		intf->dev.groups = usb_interface_groups;
//		intf->dev.dma_mask = dev->dev.dma_mask;
//		device_initialize(&intf->dev);
        intf->usb_dev = dev;
        kref_init(&intf->kref);

        snprintf(intf->name, sizeof(intf->name), "%s:%d.%d",dev->name,configuration, alt->desc.bInterfaceNumber);
	}
	kfree(new_interfaces);



	/* Now that all the interfaces are set up, register them
	 * to trigger binding of drivers to interfaces.  probe()
	 * routines may install different altsettings and may
	 * claim() any interfaces not yet bound.  Many class drivers
	 * need that: CDC, audio, video, etc.
	 */
	for (i = 0; i < nintf; ++i) {
		struct usb_interface *intf = cp->interface[i];
        struct usb_driver *driver;
        const struct usb_device_id *id = NULL;

		usb_dbg(dev,
			"adding %s (config #%d, interface %d)\n",
			intf->name, configuration,
			intf->cur_altsetting->desc.bInterfaceNumber);
        
        
        usb_get_dev(dev);

        list_for_each_entry(driver, &usb_driver_list_head, list)
        {
            id = usb_match_id(intf, driver->id_table);
      
            if(id)
            {
                int error;
                usb_dbg(dev, "%s - got id\n", __func__);

                intf->driver = driver;
                intf->condition = USB_INTERFACE_BINDING;
            
                error = driver->probe(intf, id);
                if (error) {
                    intf->condition = USB_INTERFACE_UNBOUND;
                    intf->driver = NULL;
                    usb_err(dev,"usb probe driver failed:%d\r\n",error);
                } 
                else
                {
                    intf->condition = USB_INTERFACE_BOUND;
                    usb_do_hotplug(intf, 1);
                }
            
                break;
            }            
        }
        if(!id)
        {
            usb_err(dev,"%s no matched driver\r\n",intf->name);
        }
        

//		ret = device_add(&intf->dev);
//		if (ret != 0) {
//			dev_err(&dev->dev, "device_add(%s) --> %d\n",
//				dev_name(&intf->dev), ret);
//			continue;
//		}
	}
    
	return 0;
}



/**
 * usb_driver_claim_interface - bind a driver to an interface
 * @driver: the driver to be bound
 * @iface: the interface to which it will be bound; must be in the
 *	usb device's active configuration
 * @priv: driver data associated with that interface
 *
 * This is used by usb device drivers that need to claim more than one
 * interface on a device when probing (audio and acm are current examples).
 * No device driver should directly modify internal usb_interface or
 * usb_device structure members.
 *
 * Few drivers should need to use this routine, since the most natural
 * way to bind to an interface is to return the private data from
 * the driver's probe() method.
 *
 * Callers must own the device lock, so driver probe() entries don't need
 * extra locking, but other call contexts may need to explicitly claim that
 * lock.
 */
int usb_driver_claim_interface(struct usb_driver *driver,
				struct usb_interface *iface, void *priv)
{
	int retval = 0;

	if (iface->driver)
		return -EBUSY;

	iface->driver = driver;
	usb_set_intfdata(iface, priv);
//	iface->needs_binding = 0;

//	usb_pm_lock(udev);
	iface->condition = USB_INTERFACE_BOUND;
//	mark_active(iface);
//	iface->pm_usage_cnt = !(driver->supports_autosuspend);
//	usb_pm_unlock(udev);

	/* if interface was already added, bind now; else let
	 * the future device_add() bind it, bypassing probe()
	 */
//	if (device_is_registered(dev))
//		retval = device_bind_driver(dev);

	return retval;
}

/**
 * usb_driver_release_interface - unbind a driver from an interface
 * @driver: the driver to be unbound
 * @iface: the interface from which it will be unbound
 *
 * This can be used by drivers to release an interface without waiting
 * for their disconnect() methods to be called.  In typical cases this
 * also causes the driver disconnect() method to be called.
 *
 * This call is synchronous, and may not be used in an interrupt context.
 * Callers must own the device lock, so driver disconnect() entries don't
 * need extra locking, but other call contexts may need to explicitly claim
 * that lock.
 */
void usb_driver_release_interface(struct usb_driver *driver,
					struct usb_interface *iface)
{
	/* this should never happen, don't release something that's not ours */
	if (!iface->driver || iface->driver != driver)
		return;

	/* don't release from within disconnect() */
	if (iface->condition != USB_INTERFACE_BOUND)
		return;



    iface->condition = USB_INTERFACE_UNBINDING;
    iface->driver->disconnect(iface);
    iface->condition = USB_INTERFACE_UNBOUND;
        

//	/* don't release if the interface hasn't been added yet */
//	if (device_is_registered(dev)) {
//		iface->condition = USB_INTERFACE_UNBINDING;
//		device_release_driver(dev);
//	} else {
//		iface->condition = USB_INTERFACE_UNBOUND;
//		usb_cancel_queued_reset(iface);
//	}
	iface->driver = NULL;
	usb_set_intfdata(iface, NULL);

//	usb_pm_lock(udev);
//	iface->condition = USB_INTERFACE_UNBOUND;
//	mark_quiesced(iface);
//	iface->needs_remote_wakeup = 0;
//	usb_pm_unlock(udev);
}

