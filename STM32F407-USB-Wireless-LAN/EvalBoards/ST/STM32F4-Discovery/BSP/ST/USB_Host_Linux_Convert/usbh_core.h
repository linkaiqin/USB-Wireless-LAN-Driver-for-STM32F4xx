#ifndef _USBH_CORE_H
#define _USBH_CORE_H

#include "usb_core.h"
#include "usb_hcd.h"

typedef enum {
  USBH_OK   = 0,
  USBH_BUSY,
  USBH_FAIL,
  USBH_NOT_SUPPORTED,
  USBH_UNRECOVERED_ERROR,
  USBH_ERROR_SPEED_UNKNOWN,
  USBH_APPLY_DEINIT
}USBH_Status;


#define USBH_MACHINE_MAX            4
#define USBH_MACHINE_STATE_MAX     6


//Machine
#define USBH_CONTRL_MACHINE    0
#define USBH_BULK_MACHINE      1
#define USBH_ERROR_MACHINE      2


//USBH_CONTRL_MACHINE  Msgtype
#define USBH_CTRL_SETUP_REQ           0
#define USBH_CTRL_WAIT_SETUP_RSP      1

#define USBH_CTRL_WAIT_DATA_IN_RSP    2
#define USBH_CTRL_WAIT_DATA_OUT_RSP   3

#define USBH_CTRL_WAIT_STATUS_OUT_RSP   4
#define USBH_CTRL_WAIT_STATUS_IN_RSP   5


//USBH_BULK_MACHINE  Msgtype
#define USBH_BULK_SUBMIT_SENT_REQ           0
#define USBH_BULK_SUBMIT_RCV_REQ           1

#define USBH_BULK_WAIT_SUBMIT_SENT_RSP      2
#define USBH_BULK_WAIT_SUBMIT_RCV_RSP      3


//USBH_ERROR_MACHINE  Msgtype
#define USBH_ERROR_HC_HALT_REQ                0
#define USBH_ERROR_WAIT_HC_HALT_RSP           1


//=====================================================================
#define USBH_BULK_SENT_TIMEOUT                            2500 
#define USBH_BULK_RECV_TIMEOUT                            2500
//=====================================================================

//USBH_REQUEST_QUEUE
#define USBH_REQ_TYPE_RROGRESS  0
#define USBH_REQ_TYPE_CH_HALT   1
#define USBH_REQ_TYPE_TIMEOUT   2

//USBH_REQ_TYPE_RROGRESS req_msg
#define USBH_RROGRESS_FROM_TASK  0
#define USBH_RROGRESS_FROM_INT   1




#define USBH_SETUP_PKT_SIZE   8

#define USBH_DEV_ATTACHED           0
#define USBH_DEV_DISCONNECTED      1


struct urb_queue;
struct urb;


struct usbh_task_req
{
    int req_type;
    int req_msg;
    void *data;
    struct urb *urb;
};

void USBH_UrbQueueDump(struct urb_queue *queue);
void USBH_UrbEnqueue(struct urb_queue *queue, struct urb *urb);
u8 USBH_UrbQueueUrbDel(struct urb_queue *queue, struct urb *urb);
struct urb *USBH_UrbDequeue(struct urb_queue *queue);
int USBH_NperiodFreeUrbBuf(struct urb* urb);


void USBH_Init(void);






#endif
