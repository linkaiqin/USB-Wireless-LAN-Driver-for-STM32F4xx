#ifndef _USBH_CORE_H
#define _USBH_CORE_H

#include "list.h"
#include "kref.h"
#include "type.h"




#define USBH_MACHINE_MAX            3
#define USBH_MACHINE_STATE_MAX     7


//Machine
#define USBH_CONTRL_MACHINE       0
#define USBH_BULK_INT_MACHINE      1
#define USBH_ISOC_MACHINE      2


//USBH_CONTRL_MACHINE  Msgtype
#define USBH_CTRL_SETUP_REQ           0
#define USBH_CTRL_WAIT_SETUP_RSP      1

#define USBH_CTRL_WAIT_DATA_IN_RSP    2

#define USBH_CTRL_DATA_OUT_REQ        3
#define USBH_CTRL_WAIT_DATA_OUT_RSP   4

#define USBH_CTRL_WAIT_STATUS_OUT_RSP   5
#define USBH_CTRL_WAIT_STATUS_IN_RSP   6


//USBH_BULK_MACHINE  Msgtype
#define USBH_BULK_INT_SUBMIT_SENT_REQ           0
#define USBH_BULK_INT_SUBMIT_RCV_REQ           1

#define USBH_BULK_INT_WAIT_SUBMIT_SENT_RSP      2
#define USBH_BULK_INT_WAIT_SUBMIT_RCV_RSP      3

//USBH_ISOC_MACHINE  Msgtype
#define USBH_ISOC_SUBMIT_SENT_REQ           0
#define USBH_ISOC_SUBMIT_RCV_REQ           1

#define USBH_ISOC_WAIT_SUBMIT_SENT_RSP      2
#define USBH_ISOC_WAIT_SUBMIT_RCV_RSP      3


//=====================================================================
#define USBH_BULK_SENT_TIMEOUT                            2500 
#define USBH_BULK_RECV_TIMEOUT                            2500
//=====================================================================

//task_req_type
#define USBH_TASK_REQ_CH_PROCESS  0
//task_req_msg
#define USBH_TASK_REQ_FROM_TASK  0  //for debug
#define USBH_TASK_REQ_FROM_INT   1  //for debug


#define USBH_PROCESS_CHANNEL      0
#define USBH_PROCESS_SCHED        1
#define USBH_PROCESS_ISOC_SOF      2





#define USBH_CHANNEL_STATE_IDLE          0
#define USBH_CHANNEL_STATE_PROCESS      1

#define USBH_EP_IDLE                        0
#define USBH_EP_WAIT_FRAME_TIMEOUT        1   //only for isoc interrupt endpoint
#define USBH_EP_WAIT_PROGRESS             2
#define USBH_EP_PROGRESS                   3

#define USBH_URB_IDLE                        0
#define USBH_URB_WAIT_PROGRESS              1
#define USBH_URB_PROGRESS                    2



struct usb_host_endpoint;
struct usb_device;
struct urb;


struct usbh_msg_ctx
{
    OS_SEM sem;
    int actual_length;
    int status;
};

struct usbh_task_req
{
    int req_type;
    int req_msg;
    void *data;
    struct list_head list;
};


struct ep_queue{
    struct usb_host_endpoint *ep_head;
    struct usb_host_endpoint *ep_tail;
    u16 num;
    u16 max_num_record;
};



struct urbs_xfer_mgt
{
    struct ep_queue ep_queue;

    struct list_head frame_wait_ep_list; //only for isoc endpoint

    struct list_head out_hc_wait_list;
    struct list_head out_hc_process_list;  //only one member in the list if not in DMA mode.
};


struct usb_host_channel
{
    //note:we need to use the structure after calling OSMemCreate(),so we remain 4 octes for ucos to link next bulk.
    void *next_bulk;
    
    struct list_head urb_task_req_list; //urb task_req list head
    
    struct kref kref;      /* reference count of the channel */

    struct urbs_xfer_mgt *xfer_mgt;
    struct urb *urb;

    struct usb_device *dev;
    u8  state;     //cur urb xfer state
    
    u8 hc_num;
    struct list_head hc_list;   //used for out endpoint.
};



void USBH_Sched(struct usb_device *dev);

void USBH_EpEnqueue(struct ep_queue *queue, struct usb_host_endpoint *ep);

struct usb_host_endpoint *USBH_EpDequeue(struct ep_queue *queue);


void usb_get_current_frame_number(struct usb_device *dev);


void USBH_DestroyChannel(struct kref *kref);



extern struct list_head usb_driver_list_head;
extern OS_MEM URB_Mem;
extern OS_MEM ReqQueueMem;
extern OS_TCB USBHTaskTCB;;



#define usb_get_channel(ch) kref_get(&ch->kref)
#define usb_put_channel(ch) kref_put(&ch->kref, USBH_DestroyChannel)


#define usb_endpoint_out(ep_dir)	(!((ep_dir) & USB_DIR_IN))


/* The D0/D1 toggle bits ... USE WITH CAUTION (they're almost hcd-internal) */
#define usb_gettoggle(dev, ep, out) (((dev)->toggle[out] >> (ep)) & 1)
#define	usb_dotoggle(dev, ep, out)  ((dev)->toggle[out] ^= (1 << (ep)))
#define usb_settoggle(dev, ep, out, bit) \
		((dev)->toggle[out] = ((dev)->toggle[out] & ~(1 << (ep))) | \
		 ((bit) << (ep)))






#endif
