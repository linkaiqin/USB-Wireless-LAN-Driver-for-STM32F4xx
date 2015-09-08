
#ifndef _USBH_LINUX_H
#define _USBH_LINUX_H


#include "usbh_core.h"
#include "usb_core.h"
#include "usbh_hcs.h"
#include "usbh_def.h"
#include "type.h"

__packed
struct usb_ctrlrequest {
	u8 bRequestType;
	u8 bRequest;
	u16 wValue;
	u16 wIndex;
	u16 wLength;
};

__packed
struct usb_config_descriptor {
	u8  bLength;
	u8  bDescriptorType;

	u16 wTotalLength;
	u8  bNumInterfaces;
	u8  bConfigurationValue;
	u8  iConfiguration;
	u8  bmAttributes;
	u8  bMaxPower;
};

/* USB_DT_INTERFACE: Interface descriptor */
__packed
struct usb_interface_descriptor {
	u8  bLength;
	u8  bDescriptorType;

	u8  bInterfaceNumber;
	u8  bAlternateSetting;
	u8  bNumEndpoints;
	u8  bInterfaceClass;
	u8  bInterfaceSubClass;
	u8  bInterfaceProtocol;
	u8  iInterface;
};




/* USB_DT_ENDPOINT: Endpoint descriptor */
__packed
struct usb_endpoint_descriptor {
	u8  bLength;
	u8  bDescriptorType;

	u8  bEndpointAddress;
	u8  bmAttributes;
	u16 wMaxPacketSize;
	u8  bInterval;   
};
 

struct usb_host_endpoint {
	struct usb_endpoint_descriptor	desc;
    
    u8  hc_num;

    u8 hc_ctrl_num_out;//only use in control endpoint
    u8 hc_ctrl_num_in;
};

struct usb_host_interface {
	struct usb_interface_descriptor desc;

	/* array of desc.bNumEndpoint endpoints associated with this
	 * interface setting.  these will be in no particular order.
	 */
	struct usb_host_endpoint *endpoint;
};

struct usb_device;

struct usb_interface {
	/* array of alternate settings for this interface,
	 * stored in no particular order */
	struct usb_host_interface *altsetting;//only support one interface,so altsetting = cur_altsetting
    struct usb_host_interface *cur_altsetting;
    struct usb_device *usb_dev;
    void *intfdata;
};

struct usb_host_config {
	struct usb_config_descriptor	desc;
};


struct usb_device_id {
	u16		idVendor;
	u16		idProduct;
};


struct usb_driver {
	const char *name;

	int (*probe) (struct usb_interface *intf,
		      const struct usb_device_id *id);

	void (*disconnect) (struct usb_interface *intf);

	const struct usb_device_id *id_table;
};


#define USB_DEVICE(vend,prod) vend,prod
//\
//	.match_flags = USB_DEVICE_ID_MATCH_DEVICE, \
//	.idVendor = (vend), \
//	.idProduct = (prod)




#define usb_get_dev(dev)  (dev)
#define usb_put_dev(dev)

//#define	to_usb_interface(d) ()container_of(d, struct usb_interface, dev)
#define	interface_to_usbdev(intf) (intf->usb_dev)
//	container_of(intf->dev.parent, struct usb_device, dev)

#define usb_get_intfdata(intf) (intf->intfdata)
#define usb_set_intfdata(intf, data) (intf->intfdata = data)
//static inline void *usb_get_intfdata(struct usb_interface *intf)
//{
//	return dev_get_drvdata(&intf->dev);
//}
//static inline void usb_set_intfdata(struct usb_interface *intf, void *data)
//{
//	dev_set_drvdata(&intf->dev, data);
//}


struct urb;
typedef int (*MachineStateFunc)(struct usb_device *dev, struct urb *urb, u16 *msg_type);


struct urb_queue{
    struct urb *urb_head;
    struct urb *urb_tail;
    u16 num;
    u16 max_num_record;
};

#define USBH_XFER_STATE_IDLE          0
#define USBH_XFER_STATE_PROGRESS      1



struct usb_device {

    OS_TCB *pUSBHTaskTCB;
    USB_OTG_CORE_HANDLE USB_OTG_Core;

    
 //   struct urb_queue urb_period_queue;
    struct urb_queue urb_nperiod_queue;

//    u16 usb_period_task_req_num;
    u16 usb_nperiod_task_req_num;
    u8 usb_nperiod_xfer_state;
    struct urb* CurProessUrb;

//    u8 usb_nperiod_xfer_state;
// 	enum usb_device_speed	speed;


//	struct usb_host_endpoint ep0;



// 	struct usb_device_descriptor descriptor;
// 	struct usb_host_config *config;

// 	struct usb_host_config *actconfig;
	struct usb_host_endpoint ep[16];

    //every chanal has a urb
    struct urb *urb[HC_MAX];


//	unsigned short bus_mA;

    uint8_t                           address;
    uint8_t                           speed;

    struct usb_interface             *intf;//new add
	struct usb_host_config           *config;  


   
    USBH_DevDesc_TypeDef              descriptor;//Dev_Desc;
//    USBH_CfgDesc_TypeDef              Cfg_Desc;  


//	/* static strings from the device */
	char product[30];
	char manufacturer[30];
	char serial[30];


    MachineStateFunc MachineStateFunc[USBH_MACHINE_MAX][USBH_MACHINE_STATE_MAX];
    
};


int usb_register(struct usb_driver *driver);

void usb_deregister(struct usb_driver *driver);

void *usb_buffer_alloc(struct usb_device *dev, size_t size, gfp_t mem_flags,
		       dma_addr_t *dma);

void usb_buffer_free(struct usb_device *dev, size_t size, void *addr,
		     dma_addr_t dma);

struct urb *usb_alloc_urb(int iso_packets, u8 mem_flags);

void usb_free_urb(struct urb *urb);


int usb_submit_urb(struct urb *urb, u8 mem_flags);

void usb_kill_urb(struct urb *urb);

int usb_unlink_urb(struct urb *urb);


int usb_control_msg(struct usb_device *dev, unsigned int pipe, u8 request,
		    u8 requesttype, u16 value, u16 index, void *data,
		    u16 size, int timeout);


int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
		 void *data, int len, int *actual_length, int timeout);


#define URB_NO_TRANSFER_DMA_MAP	0x0004	/* urb->transfer_dma valid on submit */
#define USB_DIR_OUT			0		/* to device */
#define USB_DIR_IN			0x80		/* to host */



/* ----------------------------------------------------------------------- */

/*
 * For various legacy reasons, Linux has a small cookie that's paired with
 * a struct usb_device to identify an endpoint queue.  Queue characteristics
 * are defined by the endpoint's descriptor.  This cookie is called a "pipe",
 * an unsigned int encoded as:
 *
 *  - direction:	bit 7		(0 = Host-to-Device [Out],
 *					 1 = Device-to-Host [In] ...
 *					like endpoint bEndpointAddress)
 *  - device address:	bits 8-14       ... bit positions known to uhci-hcd
 *  - endpoint:		bits 15-18      ... bit positions known to uhci-hcd
 *  - pipe type:	bits 30-31	(00 = isochronous, 01 = interrupt,
 *					 10 = control, 11 = bulk)
 *
 * Given the device address and endpoint descriptor, pipes are redundant.
 */

/* NOTE:  these are not the standard USB_ENDPOINT_XFER_* values!! */

#define PIPE_ISOCHRONOUS		(u32)0
#define PIPE_INTERRUPT			(u32)1
#define PIPE_CONTROL			(u32)2
#define PIPE_BULK			(u32)3

#define usb_pipein(pipe)	((pipe) & USB_DIR_IN)
#define usb_pipeout(pipe)	(!usb_pipein(pipe))

#define usb_pipedevice(pipe)	(((pipe) >> 8) & 0x7f)
#define usb_pipeendpoint(pipe)	(((pipe) >> 15) & 0xf)

#define usb_pipetype(pipe)	(((pipe) >> 30) & 3)
#define usb_pipeisoc(pipe)	(usb_pipetype((pipe)) == PIPE_ISOCHRONOUS)
#define usb_pipeint(pipe)	(usb_pipetype((pipe)) == PIPE_INTERRUPT)
#define usb_pipecontrol(pipe)	(usb_pipetype((pipe)) == PIPE_CONTROL)
#define usb_pipebulk(pipe)	(usb_pipetype((pipe)) == PIPE_BULK)

#define __create_pipe(dev,endpoint)  (endpoint << 15)
// static inline unsigned int __create_pipe(struct usb_device *dev,
// 		unsigned int endpoint)
// {
// 	return (endpoint << 15);
// }

/* Create various pipes... */
#define usb_sndctrlpipe(dev,endpoint)	\
	((PIPE_CONTROL << 30) | __create_pipe(dev, endpoint))
#define usb_rcvctrlpipe(dev,endpoint)	\
	((PIPE_CONTROL << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)
#define usb_sndisocpipe(dev,endpoint)	\
	((PIPE_ISOCHRONOUS << 30) | __create_pipe(dev, endpoint))
#define usb_rcvisocpipe(dev,endpoint)	\
	((PIPE_ISOCHRONOUS << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)
#define usb_sndbulkpipe(dev,endpoint)	\
	((PIPE_BULK << 30) | __create_pipe(dev, endpoint))
#define usb_rcvbulkpipe(dev,endpoint)	\
	((PIPE_BULK << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)
#define usb_sndintpipe(dev,endpoint)	\
	((PIPE_INTERRUPT << 30) | __create_pipe(dev, endpoint))
#define usb_rcvintpipe(dev,endpoint)	\
	((PIPE_INTERRUPT << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)




typedef void (*usb_complete_t)(struct urb *);

#define USBH_URB_IDLE                        0

#define USBH_URB_WAIT_STOP                  1//stop asynchronize
//#define USBH_URB_STOP_SYNC                   2//stop synchronize

#define USBH_URB_WAIT_PROGRESS              3
#define USBH_URB_PROGRESS                    4
//#define USBH_URB_PROGRESS_TIMEOUT           5

struct usbh_msg_ctx
{
    int actual_length;
    int status;
};


struct urb {
    //note:we need to use the structure after calling OSMemCreate(),so we remain 4 octes for ucos to link next bulk.
    void *next_bulk;
	struct usb_device *dev; 	/* (in) pointer to associated device */
	struct usb_host_endpoint *ep;	/* (internal) pointer to endpoint */
	unsigned int pipe;		/* (in) pipe information */
    int status;	
	unsigned int transfer_flags;	/* (in) URB_SHORT_NOT_OK | ...*/
    
    //control endpoint: if(length != 0)user_actual_buffer != transfer_buffer.
    //other endpoint: if((endpoint dir out) && (length != 0)) user_actual_buffer != transfer_buffer.
    void *user_actual_buffer; //user_transfer_buffer == transfer_buffer or not
    dma_addr_t transfer_dma;	/* (in) dma addr for transfer_buffer */
    void *transfer_buffer;		/* (in) associated data buffer */

	u32 transfer_buffer_length;	/* (in) data buffer length */
	u32 actual_length;		/* (return) actual transfer length */
    
    struct usb_ctrlrequest setup_request;
	unsigned char *setup_packet;	/* (in) setup packet (control only) */


	void *context;			/* (in) context for completion */
	usb_complete_t complete;	/* (in) completion routine */

    OS_TMR tmr;
    OS_SEM sem;
    OS_SEM *kill_sem;//kill sem is created in stack, assign it in USBH_TASK.
    u8 priv_state;
    struct urb *next;
    u16 MachineState;
    u16 MsgType;
};






/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK	0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK		0x80

#define USB_ENDPOINT_XFERTYPE_MASK	0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL	0
#define USB_ENDPOINT_XFER_ISOC		1
#define USB_ENDPOINT_XFER_BULK		2
#define USB_ENDPOINT_XFER_INT		    3
#define USB_ENDPOINT_MAX_ADJUSTABLE	0x80
/**
 * usb_fill_control_urb - initializes a control urb
 * @urb: pointer to the urb to initialize.
 * @dev: pointer to the struct usb_device for this urb.
 * @pipe: the endpoint pipe
 * @setup_packet: pointer to the setup_packet buffer
 * @transfer_buffer: pointer to the transfer buffer
 * @buffer_length: length of the transfer buffer
 * @complete_fn: pointer to the usb_complete_t function
 * @context: what to set the urb context to.
 *
 * Initializes a control urb with the proper information needed to submit
 * it to a device.
 */
static __inline void usb_fill_control_urb(struct urb *urb,
					struct usb_device *dev,
					unsigned int pipe,
					unsigned char *setup_packet,
					void *transfer_buffer,
					int buffer_length,
					usb_complete_t complete_fn,
					void *context)
{
	urb->dev = dev;
	urb->pipe = pipe;
	urb->setup_packet = setup_packet;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->complete = complete_fn;
	urb->context = context;
}

/**
 * usb_fill_bulk_urb - macro to help initialize a bulk urb
 * @urb: pointer to the urb to initialize.
 * @dev: pointer to the struct usb_device for this urb.
 * @pipe: the endpoint pipe
 * @transfer_buffer: pointer to the transfer buffer
 * @buffer_length: length of the transfer buffer
 * @complete_fn: pointer to the usb_complete_t function
 * @context: what to set the urb context to.
 *
 * Initializes a bulk urb with the proper information needed to submit it
 * to a device.
 */
static __inline void usb_fill_bulk_urb(struct urb *urb,
				     struct usb_device *dev,
				     unsigned int pipe,
				     void *transfer_buffer,
				     int buffer_length,
				     usb_complete_t complete_fn,
				     void *context)
{
	urb->dev = dev;
	urb->pipe = pipe;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->complete = complete_fn;
	urb->context = context;
}

/*-------------------------------------------------------------------------*/

/**
 * usb_endpoint_num - get the endpoint's number
 * @epd: endpoint to be checked
 *
 * Returns @epd's number: 0 to 15.
 */
static __inline int usb_endpoint_num(const struct usb_endpoint_descriptor *epd)
{
	return epd->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
}

/**
 * usb_endpoint_type - get the endpoint's transfer type
 * @epd: endpoint to be checked
 *
 * Returns one of USB_ENDPOINT_XFER_{CONTROL, ISOC, BULK, INT} according
 * to @epd's transfer type.
 */
static __inline int usb_endpoint_type(const struct usb_endpoint_descriptor *epd)
{
	return epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
}

/**
 * usb_endpoint_dir_in - check if the endpoint has IN direction
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint is of type IN, otherwise it returns false.
 */
static __inline int usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN);
}

/**
 * usb_endpoint_dir_out - check if the endpoint has OUT direction
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint is of type OUT, otherwise it returns false.
 */
static __inline int usb_endpoint_dir_out(
				const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT);
}

/**
 * usb_endpoint_xfer_bulk - check if the endpoint has bulk transfer type
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint is of type bulk, otherwise it returns false.
 */
static __inline int usb_endpoint_xfer_bulk(
				const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_BULK);
}

/**
 * usb_endpoint_xfer_control - check if the endpoint has control transfer type
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint is of type control, otherwise it returns false.
 */
static __inline int usb_endpoint_xfer_control(
				const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_CONTROL);
}

/**
 * usb_endpoint_xfer_int - check if the endpoint has interrupt transfer type
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint is of type interrupt, otherwise it returns
 * false.
 */
static __inline int usb_endpoint_xfer_int(
				const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_INT);
}

/**
 * usb_endpoint_xfer_isoc - check if the endpoint has isochronous transfer type
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint is of type isochronous, otherwise it returns
 * false.
 */
static __inline int usb_endpoint_xfer_isoc(
				const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_ISOC);
}

/**
 * usb_endpoint_is_bulk_in - check if the endpoint is bulk IN
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint has bulk transfer type and IN direction,
 * otherwise it returns false.
 */
static __inline int usb_endpoint_is_bulk_in(
				const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_in(epd));
}

/**
 * usb_endpoint_is_bulk_out - check if the endpoint is bulk OUT
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint has bulk transfer type and OUT direction,
 * otherwise it returns false.
 */
static __inline int usb_endpoint_is_bulk_out(
				const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_out(epd));
}

/**
 * usb_endpoint_is_int_in - check if the endpoint is interrupt IN
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint has interrupt transfer type and IN direction,
 * otherwise it returns false.
 */
static __inline int usb_endpoint_is_int_in(
				const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_int(epd) && usb_endpoint_dir_in(epd));
}

/**
 * usb_endpoint_is_int_out - check if the endpoint is interrupt OUT
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint has interrupt transfer type and OUT direction,
 * otherwise it returns false.
 */
static __inline int usb_endpoint_is_int_out(
				const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_int(epd) && usb_endpoint_dir_out(epd));
}

/**
 * usb_endpoint_is_isoc_in - check if the endpoint is isochronous IN
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint has isochronous transfer type and IN direction,
 * otherwise it returns false.
 */
static __inline int usb_endpoint_is_isoc_in(
				const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_isoc(epd) && usb_endpoint_dir_in(epd));
}

/**
 * usb_endpoint_is_isoc_out - check if the endpoint is isochronous OUT
 * @epd: endpoint to be checked
 *
 * Returns true if the endpoint has isochronous transfer type and OUT direction,
 * otherwise it returns false.
 */
static __inline int usb_endpoint_is_isoc_out(
				const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_isoc(epd) && usb_endpoint_dir_out(epd));
}



#endif


