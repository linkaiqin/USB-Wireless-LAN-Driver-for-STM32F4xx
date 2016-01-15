
#ifndef _USBH_LINUX_H
#define _USBH_LINUX_H

#include "os.h"
#include "usbh_core.h"
#include "usbh_def.h"
#include "type.h"
#include "kref.h"
#include "list.h"

#define USBH_URB_NUM_MAX           15
#define USBH_REQ_QUEUE_NUM_MAX     15



struct usb_host_endpoint {
	struct usb_endpoint_descriptor	desc;

    struct list_head urb_list;

	unsigned char *extra;   /* Extra descriptors */
	int extralen;
    int enabled;

    //The struct member below is used by usb host controller
    u8 state;
    struct urbs_xfer_mgt *xfer_mgt;
    struct list_head frame_wait_ep_list;
    struct usb_host_endpoint *next; //link to next endpoint in ep_queue
};

struct usb_host_interface {
	struct usb_interface_descriptor desc;

	/* array of desc.bNumEndpoint endpoints associated with this
	 * interface setting.  these will be in no particular order.
	 */
	struct usb_host_endpoint *endpoint;

    unsigned char *extra;   /* Extra descriptors */
    int extralen;
};

enum usb_interface_condition {
	USB_INTERFACE_UNBOUND = 0,
	USB_INTERFACE_BINDING,
	USB_INTERFACE_BOUND,
	USB_INTERFACE_UNBINDING,
};

struct usb_device;

struct usb_interface {
    char name[12];
	/* array of alternate settings for this interface,
	 * stored in no particular order */
	struct usb_host_interface *altsetting;

	struct usb_host_interface *cur_altsetting;	/* the currently
					 * active alternate setting */
	unsigned num_altsetting;	/* number of alternate settings */

    
    enum usb_interface_condition condition;		/* state of binding */
    struct usb_device *usb_dev;
    struct usb_driver *driver;   //usb driver associate with this usb device.
    struct kref kref;
    
    void *intfdata;
};


#define	interface_to_usbdev(intf) (intf->usb_dev)
//	container_of(intf->dev.parent, struct usb_device, dev)

#define usb_get_intfdata(intf) (intf->intfdata)
#define usb_set_intfdata(intf, data) (intf->intfdata = data)


/* this maximum is arbitrary */
#define USB_MAXINTERFACES	10
#define USB_MAXIADS		USB_MAXINTERFACES/2

#define USBH_DEVICE_ADDRESS     11


struct usb_interface_cache {
	unsigned num_altsetting;	/* number of alternate settings */
	struct kref ref;		/* reference counter */

	/* variable-length array of alternate settings for this interface,
	 * stored in no particular order */
	struct usb_host_interface altsetting[0];
};

#define	ref_to_usb_interface_cache(r) \
		container_of(r, struct usb_interface_cache, ref)
#define	altsetting_to_usb_interface_cache(a) \
		container_of(a, struct usb_interface_cache, altsetting[0])


struct usb_host_config {
	struct usb_config_descriptor	desc;

    	/* the interfaces associated with this configuration,
	 * stored in no particular order */
	struct usb_interface *interface[USB_MAXINTERFACES];

	/* Interface information available even when this is not the
	 * active configuration */
	struct usb_interface_cache *intf_cache[USB_MAXINTERFACES];

    unsigned char *extra;   /* Extra descriptors */
    int extralen;
};


struct usb_device_id {
	/* which fields to match against? */
	u16		match_flags;

	/* Used for product specific matches; range is inclusive */
	u16		idVendor;
	u16		idProduct;


	/* Used for interface class matches */
	u8		bInterfaceClass;
	u8		bInterfaceSubClass;
	u8		bInterfaceProtocol;
};


struct usb_driver {
	const char *name;

	int (*probe) (struct usb_interface *intf,
		      const struct usb_device_id *id);

	void (*disconnect) (struct usb_interface *intf);

	const struct usb_device_id *id_table;

    struct list_head list;
};




/* Some useful macros to use to create struct usb_device_id */
#define USB_DEVICE_ID_MATCH_VENDOR		0x0001
#define USB_DEVICE_ID_MATCH_PRODUCT		0x0002
#define USB_DEVICE_ID_MATCH_DEV_LO		0x0004
#define USB_DEVICE_ID_MATCH_DEV_HI		0x0008
#define USB_DEVICE_ID_MATCH_DEV_CLASS		0x0010
#define USB_DEVICE_ID_MATCH_DEV_SUBCLASS	0x0020
#define USB_DEVICE_ID_MATCH_DEV_PROTOCOL	0x0040
#define USB_DEVICE_ID_MATCH_INT_CLASS		0x0080
#define USB_DEVICE_ID_MATCH_INT_SUBCLASS	0x0100
#define USB_DEVICE_ID_MATCH_INT_PROTOCOL	0x0200


/*-------------------------------------------------------------------------*/

#define USB_DEVICE_ID_MATCH_DEVICE \
		(USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT)
#define USB_DEVICE_ID_MATCH_DEV_RANGE \
		(USB_DEVICE_ID_MATCH_DEV_LO | USB_DEVICE_ID_MATCH_DEV_HI)
#define USB_DEVICE_ID_MATCH_DEVICE_AND_VERSION \
		(USB_DEVICE_ID_MATCH_DEVICE | USB_DEVICE_ID_MATCH_DEV_RANGE)
#define USB_DEVICE_ID_MATCH_DEV_INFO \
		(USB_DEVICE_ID_MATCH_DEV_CLASS | \
		USB_DEVICE_ID_MATCH_DEV_SUBCLASS | \
		USB_DEVICE_ID_MATCH_DEV_PROTOCOL)
#define USB_DEVICE_ID_MATCH_INT_INFO \
		(USB_DEVICE_ID_MATCH_INT_CLASS | \
		USB_DEVICE_ID_MATCH_INT_SUBCLASS | \
		USB_DEVICE_ID_MATCH_INT_PROTOCOL)

/**
 * USB_DEVICE - macro used to describe a specific usb device
 * @vend: the 16 bit USB Vendor ID
 * @prod: the 16 bit USB Product ID
 *
 * This macro is used to create a struct usb_device_id that matches a
 * specific device.
 */
#define USB_DEVICE(vend, prod) \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE, \
	.idVendor = (vend), \
	.idProduct = (prod)
	
/**
 * USB_INTERFACE_INFO - macro used to describe a class of usb interfaces
 * @cl: bInterfaceClass value
 * @sc: bInterfaceSubClass value
 * @pr: bInterfaceProtocol value
 *
 * This macro is used to create a struct usb_device_id that matches a
 * specific class of interfaces.
 */
#define USB_INTERFACE_INFO(cl, sc, pr) \
        .match_flags = USB_DEVICE_ID_MATCH_INT_INFO, \
        .bInterfaceClass = (cl), \
        .bInterfaceSubClass = (sc), \
        .bInterfaceProtocol = (pr)

/**
 * USB_DEVICE_INTERFACE_CLASS - describe a usb device with a specific interface class
 * @vend: the 16 bit USB Vendor ID
 * @prod: the 16 bit USB Product ID
 * @cl: bInterfaceClass value
 *
 * This macro is used to create a struct usb_device_id that matches a
 * specific interface class of devices.
 */
#define USB_DEVICE_INTERFACE_CLASS(vend, prod, cl) \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE | \
		       USB_DEVICE_ID_MATCH_INT_CLASS, \
	.idVendor = (vend), \
	.idProduct = (prod), \
	.bInterfaceClass = (cl)
/**
 * USB_DEVICE_AND_INTERFACE_INFO - describe a specific usb device with a class of usb interfaces
 * @vend: the 16 bit USB Vendor ID
 * @prod: the 16 bit USB Product ID
 * @cl: bInterfaceClass value
 * @sc: bInterfaceSubClass value
 * @pr: bInterfaceProtocol value
 *
 * This macro is used to create a struct usb_device_id that matches a
 * specific device with a specific class of interfaces.
 *
 * This is especially useful when explicitly matching devices that have
 * vendor specific bDeviceClass values, but standards-compliant interfaces.
 */
#define USB_DEVICE_AND_INTERFACE_INFO(vend, prod, cl, sc, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_INT_INFO \
		| USB_DEVICE_ID_MATCH_DEVICE, \
	.idVendor = (vend), \
	.idProduct = (prod), \
	.bInterfaceClass = (cl), \
	.bInterfaceSubClass = (sc), \
	.bInterfaceProtocol = (pr)

/**
 * USB_VENDOR_AND_INTERFACE_INFO - describe a specific usb vendor with a class of usb interfaces
 * @vend: the 16 bit USB Vendor ID
 * @cl: bInterfaceClass value
 * @sc: bInterfaceSubClass value
 * @pr: bInterfaceProtocol value
 *
 * This macro is used to create a struct usb_device_id that matches a
 * specific vendor with a specific class of interfaces.
 *
 * This is especially useful when explicitly matching devices that have
 * vendor specific bDeviceClass values, but standards-compliant interfaces.
 */
#define USB_VENDOR_AND_INTERFACE_INFO(vend, cl, sc, pr) \
	.match_flags = USB_DEVICE_ID_MATCH_INT_INFO \
		| USB_DEVICE_ID_MATCH_VENDOR, \
	.idVendor = (vend), \
	.bInterfaceClass = (cl), \
	.bInterfaceSubClass = (sc), \
	.bInterfaceProtocol = (pr)


/*
 * timeouts, in milliseconds, used for sending/receiving control messages
 * they typically complete within a few frames (msec) after they're issued
 * USB identifies 5 second timeouts, maybe more in a few cases, and a few
 * slow devices (like some MGE Ellipse UPSes) actually push that limit.
 */
#define USB_CTRL_GET_TIMEOUT	5000
#define USB_CTRL_SET_TIMEOUT	5000



struct urb;
typedef int (*MachineStateFunc)(struct usb_device *dev, struct urb *urb, u16 *msg_type);

typedef void (*usb_hotplug_call_back)(struct usb_interface *intf, void *arg, int is_connect);

struct usb_hotplug
{
    const struct usb_device_id *id;
    usb_hotplug_call_back call_back;
    void *arg;
    struct list_head list;
};



#define USBH_CHANNEL_MAX   8 

struct USB_OTG_handle;

struct usb_device {
    struct USB_OTG_handle *USB_OTG_Core;  /*USB_OTG_CORE_HANDLE*/ 

    char *name;

    struct urbs_xfer_mgt period_xfer_mgt;
    struct urbs_xfer_mgt nperiod_xfer_mgt;

    
    OS_MEM usb_ch_mem;
    struct usb_host_channel usb_ch_pool[USBH_CHANNEL_MAX];

    
    int     devnum;    //device addr
    enum usb_device_state state;
    enum usb_device_speed speed;

    unsigned int toggle[2];

	struct usb_host_endpoint ep0;

	struct usb_device_descriptor descriptor;
	struct usb_host_config *config;

	struct usb_host_config *actconfig;
	struct usb_host_endpoint *ep_in[16];
	struct usb_host_endpoint *ep_out[16];

	/* static strings from the device */
	char product[64];
	char manufacturer[64];
	char serial[30];

    struct kref kref; 
    
    unsigned int  frame_number; //only used for ISO 

    OS_SEM lock;
    struct list_head list; //usb_device list
    
    MachineStateFunc MachineStateFunc[USBH_MACHINE_MAX][USBH_MACHINE_STATE_MAX];
    
};


void usb_init(void);

void usb_start(void);


void usb_do_hotplug(struct usb_interface *interface, int is_connect);

void usb_hotplug_add(struct usb_hotplug *hotplug);


int usb_register(struct usb_driver *driver);

void usb_deregister(struct usb_driver *driver);

void *usb_buffer_alloc(struct usb_device *dev, size_t size, gfp_t mem_flags,
		       dma_addr_t *dma);

void usb_buffer_free(struct usb_device *dev, size_t size, void *addr,
		     dma_addr_t dma);


struct urb *usb_alloc_urb(int iso_packets, u8 mem_flags);
void usb_free_urb(struct urb *urb);



#define usb_put_urb usb_free_urb

struct urb *usb_get_urb(struct urb *urb);


struct usb_device *usb_get_dev(struct usb_device *dev);

void usb_put_dev(struct usb_device *dev);

int usb_submit_urb(struct urb *urb, u8 mem_flags);

int usb_hcd_unlink_urb (struct urb *urb, int status);

int usb_unlink_urb(struct urb *urb);

void usb_kill_urb(struct urb *urb);

int usb_unlink_urb(struct urb *urb);


int usb_control_msg(struct usb_device *dev, unsigned int pipe, u8 request,
		    u8 requesttype, u16 value, u16 index, void *data,
		    u16 size, int timeout);


int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
		 void *data, int len, int *actual_length, int timeout);


int usb_interrupt_msg(struct usb_device *usb_dev, unsigned int pipe,
		      void *data, int len, int *actual_length, int timeout);


int usb_clear_halt(struct usb_device *dev, int pipe);

void usb_disable_endpoint(struct usb_device *dev, unsigned int epaddr,
		int reset_hardware);


void usb_enable_endpoint(struct usb_device *dev, struct usb_host_endpoint *ep,
		int reset_ep);

void usb_disable_device(struct usb_device *dev, int skip_ep0);

int usb_get_descriptor(struct usb_device *dev, unsigned char type,
		       unsigned char index, void *buf, int size);


int usb_get_device_descriptor(struct usb_device *dev, unsigned int size);


int usb_string(struct usb_device *dev, int index, char *buf, size_t size);


int usb_set_configuration(struct usb_device *dev, int configuration);

int usb_set_interface(struct usb_device *dev, int interface, int alternate);


extern struct list_head usb_kill_urb_queue;
void wake_up_kill_urb(struct list_head *head);


extern struct usb_host_interface *usb_altnum_to_altsetting(
					const struct usb_interface *intf,
					unsigned int altnum);

extern struct usb_interface *usb_get_intf(struct usb_interface *intf);

extern void usb_put_intf(struct usb_interface *intf);

struct usb_interface *usb_ifnum_to_if(const struct usb_device *dev,
				      unsigned ifnum);

extern void usb_set_device_state(struct usb_device *udev,
                          enum usb_device_state new_state);

extern const struct usb_device_id *usb_match_id(struct usb_interface *interface,
        const struct usb_device_id *id);



int usb_driver_claim_interface(struct usb_driver *driver,
				struct usb_interface *iface, void *priv);

void usb_driver_release_interface(struct usb_driver *driver,
					struct usb_interface *iface);


#define URB_SHORT_NOT_OK	          0x0001	/* report short reads as errors */
#define URB_NO_TRANSFER_DMA_MAP	  0x0004	/* urb->transfer_dma valid on submit */
#define URB_FREE_BUFFER		0x0100	/* Free transfer buffer with the URB */



//add by LKQ, the flags is valid only for dma not enabled device.
#define URB_NO_SEND_DATA_IN_INTTERUPT		         0x0400	
#define URB_NO_RECV_NAK_REACTIVE_IN_INTTERUPT    0x0800	



#define URB_DIR_IN		0x0200	/* Transfer from device to host */
#define URB_DIR_OUT		0
#define URB_DIR_MASK		URB_DIR_IN


struct usb_iso_packet_descriptor {
	unsigned int offset;
	unsigned int length;		/* expected length */
	unsigned int actual_length;
	int status;
    unsigned int frame_number; //for debug add by LKQ
};

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

static inline unsigned int __create_pipe(struct usb_device *dev,
		unsigned int endpoint)
{
	return (dev->devnum << 8) | (endpoint << 15);
}


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




struct urb {
    //note:we need to use the structure after calling OSMemCreate(),so we remain 4 octes for ucos to link next bulk.
    void *next_bulk;

    struct kref kref;		/* reference count of the URB */
    atomic_t use_count;		/* concurrent submissions counter */
    atomic_t reject;		/* submissions will fail */
    int unlinked;			/* unlink error code */
    
    struct list_head urb_list;
    
	struct usb_device *dev; 	/* (in) pointer to associated device */
	struct usb_host_endpoint *ep;	/* (internal) pointer to endpoint */
	unsigned int pipe;		/* (in) pipe information */
    int status;	
	unsigned int transfer_flags;	/* (in) URB_SHORT_NOT_OK | ...*/
    void *transfer_buffer;		/* (in) associated data buffer */
    dma_addr_t transfer_dma;	/* (in) dma addr for transfer_buffer */ //not used

	u32 transfer_buffer_length;	/* (in) data buffer length */
	u32 actual_length;		/* (return) actual transfer length */
    struct usb_ctrlrequest setup_request;
	unsigned char *setup_packet;	/* (in) setup packet (control only) */
    int start_frame;
    int number_of_packets;		/* (in) number of ISO packets */
    int interval;           /* (modify) transfer interval */

	void *context;			/* (in) context for completion */
	usb_complete_t complete;	/* (in) completion routine */
	struct usb_iso_packet_descriptor *iso_frame_desc; /* (in) ISO ONLY */

    void *isoc_submited_context;
    void (*isoc_submited)(struct urb *);
    //The struct member below is used by usb host controller.
    int iso_frame_index;
    struct usb_host_channel *ch;
    u8 priv_state;
    u8 error_count;
    
    OS_TMR tmr;
    u16 MachineState;
    u16 MsgType;
};




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

/**
 * usb_fill_int_urb - macro to help initialize a interrupt urb
 * @urb: pointer to the urb to initialize.
 * @dev: pointer to the struct usb_device for this urb.
 * @pipe: the endpoint pipe
 * @transfer_buffer: pointer to the transfer buffer
 * @buffer_length: length of the transfer buffer
 * @complete_fn: pointer to the usb_complete_t function
 * @context: what to set the urb context to.
 * @interval: what to set the urb interval to, encoded like
 *	the endpoint descriptor's bInterval value.
 *
 * Initializes a interrupt urb with the proper information needed to submit
 * it to a device.
 * Note that high speed interrupt endpoints use a logarithmic encoding of
 * the endpoint interval, and express polling intervals in microframes
 * (eight per millisecond) rather than in frames (one per millisecond).
 */
    static inline void usb_fill_int_urb(struct urb *urb,
                        struct usb_device *dev,
                        unsigned int pipe,
                        void *transfer_buffer,
                        int buffer_length,
                        usb_complete_t complete_fn,
                        void *context,
                        int interval)
{
	urb->dev = dev;
	urb->pipe = pipe;
	urb->transfer_buffer = transfer_buffer;
	urb->transfer_buffer_length = buffer_length;
	urb->complete = complete_fn;
	urb->context = context;
	if (dev->speed == USB_SPEED_HIGH)
		urb->interval = 1 << (interval - 1);
	else
		urb->interval = interval;
	urb->start_frame = -1;
}


/**
 * usb_urb_dir_in - check if an URB describes an IN transfer
 * @urb: URB to be checked
 *
 * Returns 1 if @urb describes an IN transfer (device-to-host),
 * otherwise 0.
 */
static inline int usb_urb_dir_in(struct urb *urb)
{
	return (urb->transfer_flags & URB_DIR_MASK) == URB_DIR_IN;
}

/**
 * usb_urb_dir_out - check if an URB describes an OUT transfer
 * @urb: URB to be checked
 *
 * Returns 1 if @urb describes an OUT transfer (host-to-device),
 * otherwise 0.
 */
static inline int usb_urb_dir_out(struct urb *urb)
{
	return (urb->transfer_flags & URB_DIR_MASK) == URB_DIR_OUT;
}


#endif


