#ifndef _IW_HANDLER_DEF_H
#define _IW_HANDLER_DEF_H


#include "type.h"
#include "wireless.h"

/***************************** VERSION *****************************/
/*
 * This constant is used to know which version of the driver API is
 * available. Hopefully, this will be pretty stable and no changes
 * will be needed...
 * I just plan to increment with each new version.
 */
#define IW_HANDLER_VERSION	8

/*
 * Changes :
 *
 * V2 to V3
 * --------
 *	- Move event definition in <linux/wireless.h>
 *	- Add Wireless Event support :
 *		o wireless_send_event() prototype
 *		o iwe_stream_add_event/point() inline functions
 * V3 to V4
 * --------
 *	- Reshuffle IW_HEADER_TYPE_XXX to map IW_PRIV_TYPE_XXX changes
 *
 * V4 to V5
 * --------
 *	- Add new spy support : struct iw_spy_data & prototypes
 *
 * V5 to V6
 * --------
 *	- Change the way we get to spy_data method for added safety
 *	- Remove spy #ifdef, they are always on -> cleaner code
 *	- Add IW_DESCR_FLAG_NOMAX flag for very large requests
 *	- Start migrating get_wireless_stats to struct iw_handler_def
 *
 * V6 to V7
 * --------
 *	- Add struct ieee80211_device pointer in struct iw_public_data
 *	- Remove (struct iw_point *)->pointer from events and streams
 *	- Remove spy_offset from struct iw_handler_def
 *	- Add "check" version of event macros for ieee802.11 stack
 *
 * V7 to V8
 * ----------
 *	- Prevent leaking of kernel space in stream on 64 bits.
 */

/**************************** CONSTANTS ****************************/

/* Enhanced spy support available */
#define IW_WIRELESS_SPY
#define IW_WIRELESS_THRSPY

/* Special error message for the driver to indicate that we
 * should do a commit after return from the iw_handler */
#define EIWCOMMIT	EINPROGRESS

/* Flags available in struct iw_request_info */
#define IW_REQUEST_FLAG_COMPAT	0x0001	/* Compat ioctl call */

/* Type of headers we know about (basically union iwreq_data) */
#define IW_HEADER_TYPE_NULL	0	/* Not available */
#define IW_HEADER_TYPE_CHAR	2	/* char [IFNAMSIZ] */
#define IW_HEADER_TYPE_UINT	4	/* __u32 */
#define IW_HEADER_TYPE_FREQ	5	/* struct iw_freq */
#define IW_HEADER_TYPE_ADDR	6	/* struct sockaddr */
#define IW_HEADER_TYPE_POINT	8	/* struct iw_point */
#define IW_HEADER_TYPE_PARAM	9	/* struct iw_param */
#define IW_HEADER_TYPE_QUAL	10	/* struct iw_quality */

/* Handling flags */
/* Most are not implemented. I just use them as a reminder of some
 * cool features we might need one day ;-) */
#define IW_DESCR_FLAG_NONE	0x0000	/* Obvious */
/* Wrapper level flags */
#define IW_DESCR_FLAG_DUMP	0x0001	/* Not part of the dump command */
#define IW_DESCR_FLAG_EVENT	0x0002	/* Generate an event on SET */
#define IW_DESCR_FLAG_RESTRICT	0x0004	/* GET : request is ROOT only */
				/* SET : Omit payload from generated iwevent */
#define IW_DESCR_FLAG_NOMAX	0x0008	/* GET : no limit on request size */
/* Driver level flags */
#define IW_DESCR_FLAG_WAIT	0x0100	/* Wait for driver event */

/****************************** TYPES ******************************/

/* ----------------------- WIRELESS HANDLER ----------------------- */
/*
 * A wireless handler is just a standard function, that looks like the
 * ioctl handler.
 * We also define there how a handler list look like... As the Wireless
 * Extension space is quite dense, we use a simple array, which is faster
 * (that's the perfect hash table ;-).
 */

/*
 * Meta data about the request passed to the iw_handler.
 * Most handlers can safely ignore what's in there.
 * The 'cmd' field might come handy if you want to use the same handler
 * for multiple command...
 * This struct is also my long term insurance. I can add new fields here
 * without breaking the prototype of iw_handler...
 */
struct iw_request_info
{
	__u16		cmd;		/* Wireless Extension command */
	__u16		flags;		/* More to come ;-) */
};

struct net_device;

/*
 * This is how a function handling a Wireless Extension should look
 * like (both get and set, standard and private).
 */
typedef int (*iw_handler)(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra);

/*
 * This define all the handler that the driver export.
 * As you need only one per driver type, please use a static const
 * shared by all driver instances... Same for the members...
 * This will be linked from net_device in <linux/netdevice.h>
 */
struct iw_handler_def
{
	/* Number of handlers defined (more precisely, index of the
	 * last defined handler + 1) */
	__u16			num_standard;
	__u16			num_private;
	/* Number of private arg description */
	__u16			num_private_args;

	/* Array of handlers for standard ioctls
	 * We will call dev->wireless_handlers->standard[ioctl - SIOCSIWCOMMIT]
	 */
	const iw_handler *	standard;

	/* Array of handlers for private ioctls
	 * Will call dev->wireless_handlers->private[ioctl - SIOCIWFIRSTPRIV]
	 */
	const iw_handler *	private;

	/* Arguments of private handler. This one is just a list, so you
	 * can put it in any order you want and should not leave holes...
	 * We will automatically export that to user space... */
	const struct iw_priv_args *	private_args;

	/* New location of get_wireless_stats, to de-bloat struct net_device.
	 * The old pointer in struct net_device will be gradually phased
	 * out, and drivers are encouraged to use this one... */
	struct iw_statistics*	(*get_wireless_stats)(struct net_device *dev);
};


/************************* INLINE FUNTIONS *************************/
/*
 * Function that are so simple that it's more efficient inlining them
 */

static inline int iwe_stream_lcp_len(struct iw_request_info *info)
{
#ifdef CONFIG_COMPAT
	if (info->flags & IW_REQUEST_FLAG_COMPAT)
		return IW_EV_COMPAT_LCP_LEN;
#endif
	return IW_EV_LCP_LEN;
}

static inline int iwe_stream_point_len(struct iw_request_info *info)
{
#ifdef CONFIG_COMPAT
	if (info->flags & IW_REQUEST_FLAG_COMPAT)
		return IW_EV_COMPAT_POINT_LEN;
#endif
	return IW_EV_POINT_LEN;
}

static inline int iwe_stream_event_len_adjust(struct iw_request_info *info,
					      int event_len)
{
#ifdef CONFIG_COMPAT
	if (info->flags & IW_REQUEST_FLAG_COMPAT) {
		event_len -= IW_EV_LCP_LEN;
		event_len += IW_EV_COMPAT_LCP_LEN;
	}
#endif

	return event_len;
}

/*------------------------------------------------------------------*/
/*
 * Wrapper to add an Wireless Event to a stream of events.
 */
static inline char *
iwe_stream_add_event(struct iw_request_info *info, char *stream, char *ends,
		     struct iw_event *iwe, int event_len)
{
	int lcp_len = iwe_stream_lcp_len(info);

	event_len = iwe_stream_event_len_adjust(info, event_len);

	/* Check if it's possible */
	if(likely((stream + event_len) < ends)) {
		iwe->len = event_len;
		/* Beware of alignement issues on 64 bits */
		memcpy(stream, (char *) iwe, IW_EV_LCP_PK_LEN);
		memcpy(stream + lcp_len, &iwe->u,
		       event_len - lcp_len);
		stream += event_len;
	}
	return stream;
}

/*------------------------------------------------------------------*/
/*
 * Wrapper to add an short Wireless Event containing a pointer to a
 * stream of events.
 */
static inline char *
iwe_stream_add_point(struct iw_request_info *info, char *stream, char *ends,
		     struct iw_event *iwe, char *extra)
{
	int event_len = iwe_stream_point_len(info) + iwe->u.data.length;
	int point_len = iwe_stream_point_len(info);
	int lcp_len   = iwe_stream_lcp_len(info);

	/* Check if it's possible */
	if(likely((stream + event_len) < ends)) {
		iwe->len = event_len;
		memcpy(stream, (char *) iwe, IW_EV_LCP_PK_LEN);
		memcpy(stream + lcp_len,
		       ((char *) &iwe->u) + IW_EV_POINT_OFF,
		       IW_EV_POINT_PK_LEN - IW_EV_LCP_PK_LEN);
		memcpy(stream + point_len, extra, iwe->u.data.length);
		stream += event_len;
	}
	return stream;
}

/*------------------------------------------------------------------*/
/*
 * Wrapper to add a value to a Wireless Event in a stream of events.
 * Be careful, this one is tricky to use properly :
 * At the first run, you need to have (value = event + IW_EV_LCP_LEN).
 */
static inline char *
iwe_stream_add_value(struct iw_request_info *info, char *event, char *value,
		     char *ends, struct iw_event *iwe, int event_len)
{
	int lcp_len = iwe_stream_lcp_len(info);

	/* Don't duplicate LCP */
	event_len -= IW_EV_LCP_LEN;

	/* Check if it's possible */
	if(likely((value + event_len) < ends)) {
		/* Add new value */
		memcpy(value, &iwe->u, event_len);
		value += event_len;
		/* Patch LCP */
		iwe->len = value - event;
		memcpy(event, (char *) iwe, lcp_len);
	}
	return value;
}













#endif
