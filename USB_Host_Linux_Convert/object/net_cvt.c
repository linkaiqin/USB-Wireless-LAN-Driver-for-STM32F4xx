#define USBH_DEBUG_LEVEL USBH_DEBUG_TRACE

#include "os.h"
#include "misc_cvt.h"
#include "net_cvt.h"
#include "memory.h"
#include "string.h"
#include "usbh_config.h"
#include "errno-base.h"
#include "wlan_ethernetif.h"


void free_netdev(struct net_device *dev)
{
	kfree(dev);
}


struct net_device *alloc_etherdev(int sizeof_priv)
{
    struct net_device *dev;
	int alloc_size;

	/* ensure 32-byte alignment of the private area */
	alloc_size = sizeof (*dev) + sizeof_priv + 31;

	dev = (struct net_device *)kmalloc(alloc_size, GFP_KERNEL);
	if (dev == NULL) {
		USBH_DBG("alloc_etherdev: Unable to allocate device memory.\n");
		return NULL;
	}

	memset(dev, 0, alloc_size);
    memset(dev->broadcast, 0xFF, ETH_ALEN);
    dev->type		= ARPHRD_ETHER;
    
	if (sizeof_priv)
		dev->ml_priv = (void *)(((long)(dev + 1) + 31) & ~31);

//	setup(dev);
	strcpy(dev->name, "");

	return dev;


//    return alloc_netdev_mq(sizeof_priv, "eth%d", ether_setup, queue_count);
    
}


//void ether_setup(struct net_device *dev)
//{
//	dev->header_ops		= &eth_header_ops;
//#ifdef CONFIG_COMPAT_NET_DEV_OPS
//	dev->change_mtu		= eth_change_mtu;
//	dev->set_mac_address 	= eth_mac_addr;
//	dev->validate_addr	= eth_validate_addr;
//#endif
//	dev->type		= ARPHRD_ETHER;
//	dev->hard_header_len 	= ETH_HLEN;
//	dev->mtu		= ETH_DATA_LEN;
//	dev->addr_len		= ETH_ALEN;
//	dev->tx_queue_len	= 1000;	/* Ethernet wants good queues */
//	dev->flags		= IFF_BROADCAST|IFF_MULTICAST;

//	memset(dev->broadcast, 0xFF, ETH_ALEN);

//}


struct net_device *dev_get_by_name(struct net_device *dev, const char *name)
{
	if (!strncmp(dev->name, name, IFNAMSIZ))
	{
        dev_hold(dev);
		return dev;
	}
	else
	    return NULL;
}


int netif_rx(struct sk_buff *skb);



/**
 *	netif_start_queue - allow transmit
 *	@dev: network device
 *
 *	Allow upper layers to call the device hard_start_xmit routine.
 */
void netif_start_queue(struct net_device *dev)
{
//    clear_bit(__QUEUE_STATE_XOFF, &dev_queue->state);
    
//	netif_tx_start_queue(netdev_get_tx_queue(dev, 0));
}

/**
 *	netif_stop_queue - stop transmitted packets
 *	@dev: network device
 *
 *	Stop upper layers calling the device hard_start_xmit routine.
 *	Used for flow control when transmit resources are unavailable.
 */
void netif_stop_queue(struct net_device *dev)
{
//   set_bit(__QUEUE_STATE_XOFF, &dev_queue->state);

   
//	netif_tx_stop_queue(netdev_get_tx_queue(dev, 0));
}






/**
 *	netif_wake_queue - restart transmit
 *	@dev: network device
 *
 *	Allow upper layers to call the device hard_start_xmit routine.
 *	Used for flow control when transmit resources are available.
 */
void netif_wake_queue(struct net_device *dev)
{
//    	if (test_and_clear_bit(__QUEUE_STATE_XOFF, &dev_queue->state))
//		__netif_schedule(dev_queue->qdisc);

        
//	netif_tx_wake_queue(netdev_get_tx_queue(dev, 0));
}


/**
 *	netif_carrier_on - set carrier
 *	@dev: network device
 *
 * Device has detected that carrier.
 */
void netif_carrier_on(struct net_device *dev)
{
//	if (test_and_clear_bit(__LINK_STATE_NOCARRIER, &dev->state)) {
//		if (dev->reg_state == NETREG_UNINITIALIZED)
//			return;
//		linkwatch_fire_event(dev);
//		if (netif_running(dev))
//			__netdev_watchdog_up(dev);
//	}
}

/**
 *	netif_carrier_off - clear carrier
 *	@dev: network device
 *
 * Device has detected loss of carrier.
 */
void netif_carrier_off(struct net_device *dev)
{
//	if (!test_and_set_bit(__LINK_STATE_NOCARRIER, &dev->state)) {
//		if (dev->reg_state == NETREG_UNINITIALIZED)
//			return;
//		linkwatch_fire_event(dev);
//	}
}

struct net_device *_pnet_device = NULL;
extern struct netif wireless_netif;

int register_netdev(struct net_device *dev)
{    
	/* When net_device's are persistent, this will be fatal. */
	BUG_ON(dev->reg_state != NETREG_UNINITIALIZED);    
    dev->reg_state = NETREG_REGISTERED;
    dev->netif = &wireless_netif;
    dev->dev_addr = wireless_netif.hwaddr;   
    dev->status = 0;

//    netif_set_link_up(&wireless_netif);
//    netif_set_up(&wireless_netif);
        
    _pnet_device = dev; 
    return 0;
}

void unregister_netdev(struct net_device *dev)
{
    OS_ERR err;
    CPU_SR cpu_sr;
    
	/* Some devices call without registering for initialization unwind. */
	if (dev->reg_state == NETREG_UNINITIALIZED) {
		USBH_DBG("unregister_netdevice: device %s/%p never "
				  "was registered\r\n", dev->name, dev);

		WARN_ON(1);
		return;
	}

	BUG_ON(dev->reg_state != NETREG_REGISTERED);

	/* If device is running, close it first. */
//	dev_close(dev);
	/* And unlink it from device chain. */
//	unlist_netdevice(dev);

	dev->reg_state = NETREG_UNREGISTERING;  


//    netif_set_link_down(&wireless_netif);
//    netif_set_down(&wireless_netif);

    CPU_CRITICAL_ENTER();
    while(dev->status)
    {
        CPU_CRITICAL_EXIT();
        //wait for NET_IOCTL_RUNNING or NET_XIMT_RUNNING
        OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
        CPU_CRITICAL_ENTER();
    }
    _pnet_device = NULL;
    CPU_CRITICAL_EXIT();    
}



int netif_rx(struct sk_buff *skb)
{

//    USBH_TRACE("netif_rx len:%d  pkt_type:%d  protocol:%d\r\n",skb->len, skb->pkt_type, htons(skb->protocol));


    skb_push(skb, ETH_HLEN);
    wlan_ethernetif_input(skb->dev->netif, skb->data, skb->len);
    
    dev_kfree_skb_any(skb);
    return 0;
}


#define SKB_DATA_ALIGN(X)	(((X) + (4 - 1)) & \
				 ~(4 - 1))


unsigned int skbTotalCtr = 0;

struct sk_buff *__dev_alloc_skb(unsigned int size,gfp_t gfp_mask)
{
    struct sk_buff * skb;
    unsigned char *data;

    skb = kmalloc(sizeof(struct sk_buff),0);
    if(!skb)
    {
        USBH_DBG("dev_alloc_skb kmalloc skb Failed\r\n");        
        return NULL;
    }

    size = SKB_DATA_ALIGN(size);
    data = kmalloc(size ,0);
    if(!data)
    {
        USBH_DBG("dev_alloc_skb kmalloc data Failed\r\n");
        kfree(skb);
        return NULL;
    }
    
    skbTotalCtr++;
    
    skb->truesize = size + sizeof(struct sk_buff);
    skb->len = 0;
    skb->data = data;
    skb->head = data;
    skb->tail = skb->data;
	skb->end = skb->tail + size;
    
    return skb;
}




void dev_kfree_skb_any(struct sk_buff *skb)
{
//	if (in_irq() || irqs_disabled())
//		dev_kfree_skb_irq(skb);
//	else
//		dev_kfree_skb(skb);
    
    skbTotalCtr--;
    
    kfree(skb);
    kfree(skb->head);
}


/**
 *	skb_over_panic	- 	private function
 *	@skb: buffer
 *	@sz: size
 *	@here: address
 *
 *	Out of line support code for skb_put(). Not user callable.
 */
void skb_over_panic(struct sk_buff *skb, int sz, void *here)
{
	printk(KERN_EMERG "skb_over_panic: text:%p len:%d put:%d head:%p "
			  "data:%p tail:%#lx end:%#lx dev:%s\n",
	       here, skb->len, sz, skb->head, skb->data,
	       (unsigned long)skb->tail, (unsigned long)skb->end,
	       skb->dev ? skb->dev->name : "<NULL>");
	BUG();
}
/**
 *	skb_under_panic	- 	private function
 *	@skb: buffer
 *	@sz: size
 *	@here: address
 *
 *	Out of line support code for skb_push(). Not user callable.
 */

void skb_under_panic(struct sk_buff *skb, int sz, void *here)
{
	printk(KERN_EMERG "skb_under_panic: text:%p len:%d put:%d head:%p "
			  "data:%p tail:%#lx end:%#lx dev:%s\n",
	       here, skb->len, sz, skb->head, skb->data,
	       (unsigned long)skb->tail, (unsigned long)skb->end,
	       skb->dev ? skb->dev->name : "<NULL>");
	BUG();
}

/**
 *	skb_put - add data to a buffer
 *	@skb: buffer to use
 *	@len: amount of data to add
 *
 *	This function extends the used data area of the buffer. If this would
 *	exceed the total buffer size the kernel will panic. A pointer to the
 *	first byte of the extra data is returned.
 */
unsigned char *skb_put(struct sk_buff *skb, unsigned int len)
{
	unsigned char *tmp = skb->tail;
	skb->tail += len;
	skb->len  += len;
	if (skb->tail > skb->end)
		skb_over_panic(skb, len, __builtin_return_address(0));
	return tmp;
}

/**
 *	skb_push - add data to the start of a buffer
 *	@skb: buffer to use
 *	@len: amount of data to add
 *
 *	This function extends the used data area of the buffer at the buffer
 *	start. If this would exceed the total buffer headroom the kernel will
 *	panic. A pointer to the first byte of the extra data is returned.
 */
unsigned char *skb_push(struct sk_buff *skb, unsigned int len)
{
	skb->data -= len;
	skb->len  += len;
	if (skb->data<skb->head)
		skb_under_panic(skb, len, __builtin_return_address(0));
	return skb->data;
}


static inline unsigned char *__skb_pull(struct sk_buff *skb, unsigned int len)
{
	skb->len -= len;
//	BUG_ON(skb->len < skb->data_len);
	return skb->data += len;
}

/**
 *	skb_pull - remove data from the start of a buffer
 *	@skb: buffer to use
 *	@len: amount of data to remove
 *
 *	This function removes data from the start of a buffer, returning
 *	the memory to the headroom. A pointer to the next data in the buffer
 *	is returned. Once the data has been pulled future pushes will overwrite
 *	the old data.
 */
unsigned char *skb_pull(struct sk_buff *skb, unsigned int len)
{
	return unlikely(len > skb->len) ? NULL : __skb_pull(skb, len);
}

static inline void __skb_trim(struct sk_buff *skb, unsigned int len)
{
	skb->len = len;
    skb->tail = skb->data + len;
}

/**
 *	skb_trim - remove end from a buffer
 *	@skb: buffer to alter
 *	@len: new length
 *
 *	Cut the length of a buffer down by removing data from the tail. If
 *	the buffer is already under the length specified it is not modified.
 *	The skb must be linear.
 */
void skb_trim(struct sk_buff *skb, unsigned int len)
{
	if (skb->len > len)
		__skb_trim(skb, len);
}

static void skb_release_data(struct sk_buff *skb)
{
    kfree(skb->head);
}

/**
 *	pskb_expand_head - reallocate header of &sk_buff
 *	@skb: buffer to reallocate
 *	@nhead: room to add at head
 *	@ntail: room to add at tail
 *	@gfp_mask: allocation priority
 *
 *	Expands (or creates identical copy, if &nhead and &ntail are zero)
 *	header of skb. &sk_buff itself is not changed. &sk_buff MUST have
 *	reference count of 1. Returns zero in the case of success or error,
 *	if expansion failed. In the last case, &sk_buff is not changed.
 *
 *	All the pointers pointing into skb header may change and must be
 *	reloaded after call to this function.
 */

int pskb_expand_head(struct sk_buff *skb, int nhead, int ntail,
		     gfp_t gfp_mask)
{
//	int i;
	u8 *data;
	int size = nhead + (skb->end - skb->head) + ntail;
	long off;

    USBH_TRACE("pskb_expand_head handler\r\n");
	BUG_ON(nhead < 0);


	size = SKB_DATA_ALIGN(size);

	data = kmalloc(size, gfp_mask);
	if (!data)
        return -ENOMEM;

	/* Copy only real data... and, alas, header. This should be
	 * optimized for the cases when header is void. */

	memcpy(data + nhead, skb->head, skb->tail - skb->head);


	skb_release_data(skb);

	off = (data + nhead) - skb->head;

	skb->head     = data;
	skb->data    += off;

	skb->end      = skb->head + size;

	/* {transport,network,mac}_header and tail are relative to skb->head */
	skb->tail	      += off;
//	skb->transport_header += off;
//	skb->network_header   += off;
	skb->mac_header	      += off;
//	skb->csum_start       += nhead;
//	skb->cloned   = 0;
//	skb->hdr_len  = 0;
//	skb->nohdr    = 0;
//	atomic_set(&skb_shinfo(skb)->dataref, 1);
	return 0;
}

static void copy_skb_header(struct sk_buff *new, const struct sk_buff *old)
{
//	new->tstamp		= old->tstamp;
	new->dev		= old->dev;
//	new->transport_header	= old->transport_header;
//	new->network_header	= old->network_header;
	new->mac_header		= old->mac_header;
//	new->dst		= dst_clone(old->dst);
//#ifdef CONFIG_XFRM
//	new->sp			= secpath_get(old->sp);
//#endif
	memcpy(new->cb, old->cb, sizeof(old->cb));
//	new->csum_start		= old->csum_start;
//	new->csum_offset	= old->csum_offset;
	new->local_df		= old->local_df;
	new->pkt_type		= old->pkt_type;
	new->ip_summed		= old->ip_summed;
//	skb_copy_queue_mapping(new, old);
//	new->priority		= old->priority;
//#if defined(CONFIG_IP_VS) || defined(CONFIG_IP_VS_MODULE)
//	new->ipvs_property	= old->ipvs_property;
//#endif
//	new->protocol		= old->protocol;
//	new->mark		= old->mark;
//	__nf_copy(new, old);
//#if defined(CONFIG_NETFILTER_XT_TARGET_TRACE) || \
//    defined(CONFIG_NETFILTER_XT_TARGET_TRACE_MODULE)
//	new->nf_trace		= old->nf_trace;
//#endif
//#ifdef CONFIG_NET_SCHED
//	new->tc_index		= old->tc_index;
//#ifdef CONFIG_NET_CLS_ACT
//	new->tc_verd		= old->tc_verd;
//#endif
//#endif
//	new->vlan_tci		= old->vlan_tci;

//	skb_copy_secmark(new, old);
}


static inline void skb_copy_from_linear_data_offset(const struct sk_buff *skb,
						    const int offset, void *to,
						    const unsigned int len)
{
	memcpy(to, skb->data + offset, len);
}

static inline unsigned int skb_headlen(const struct sk_buff *skb)
{
	return skb->len;
}


/* Copy some data bits from skb to kernel buffer. */

int skb_copy_bits(const struct sk_buff *skb, int offset, unsigned char *to, int len)
{
	int copy;
	int start = skb_headlen(skb);

	if (offset > (int)skb->len - len)
		goto fault;

	/* Copy header. */
	if ((copy = start - offset) > 0) {
		if (copy > len)
			copy = len;
		skb_copy_from_linear_data_offset(skb, offset, to, copy);
		if ((len -= copy) == 0)
			return 0;
		offset += copy;
		to     += copy;
	}

	if (!len)
		return 0;

fault:
	return -EFAULT;
}



/**
 *	skb_copy_expand	-	copy and expand sk_buff
 *	@skb: buffer to copy
 *	@newheadroom: new free bytes at head
 *	@newtailroom: new free bytes at tail
 *	@gfp_mask: allocation priority
 *
 *	Make a copy of both an &sk_buff and its data and while doing so
 *	allocate additional space.
 *
 *	This is used when the caller wishes to modify the data and needs a
 *	private copy of the data to alter as well as more space for new fields.
 *	Returns %NULL on failure or the pointer to the buffer
 *	on success. The returned buffer has a reference count of 1.
 *
 *	You must pass %GFP_ATOMIC as the allocation priority if this function
 *	is called from an interrupt.
 */
struct sk_buff *skb_copy_expand(const struct sk_buff *skb, int newheadroom, int
	newtailroom, gfp_t gfp_mask)
{
	/*
	 *	Allocate the copy buffer
	 */
	struct sk_buff *n = dev_alloc_skb(newheadroom + skb->len + newtailroom);//, gfp_mask);
	int oldheadroom = skb_headroom(skb);
	int head_copy_len, head_copy_off;
//	int off;

	if (!n)
		return NULL;

	skb_reserve(n, newheadroom);

	/* Set the tail pointer and length */
	skb_put(n, skb->len);

	head_copy_len = oldheadroom;
	head_copy_off = 0;
	if (newheadroom <= head_copy_len)
		head_copy_len = newheadroom;
	else
		head_copy_off = newheadroom - head_copy_len;

	/* Copy the linear header and data. */
	if (skb_copy_bits(skb,  - head_copy_len, n->head + head_copy_off, skb->len +
	head_copy_len))
		BUG();

	copy_skb_header(n, skb);

//	off = newheadroom - oldheadroom;
//	if (n->ip_summed == CHECKSUM_PARTIAL)
//		n->cs.cs_w.csum_start += off;
//	#ifdef NET_SKBUFF_DATA_USES_OFFSET
//	n->transport_header += off;
//	n->network_header += off;
//	if (skb_mac_header_was_set(skb))
//		n->mac_header += off;
//	#endif 

	return n;
} 





/*
 *	This is an Ethernet frame header.
 */

struct ethhdr {
	unsigned char	h_dest[ETH_ALEN];	/* destination eth addr	*/
	unsigned char	h_source[ETH_ALEN];	/* source ether addr	*/
	__be16		h_proto;		/* packet type ID field	*/
} __attribute__((packed));


static inline struct ethhdr *eth_hdr(const struct sk_buff *skb)
{
	return (struct ethhdr *)skb->mac_header;
}

/**
 * is_multicast_ether_addr - Determine if the Ethernet address is a multicast.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is a multicast address.
 * By definition the broadcast address is also a multicast address.
 */
static inline int is_multicast_ether_addr(const __u8 *addr)
{
	return (0x01 & addr[0]);
}
/**
 * compare_ether_addr - Compare two Ethernet addresses
 * @addr1: Pointer to a six-byte array containing the Ethernet address
 * @addr2: Pointer other six-byte array containing the Ethernet address
 *
 * Compare two ethernet addresses, returns 0 if equal
 */
static inline unsigned compare_ether_addr(const __u8 *addr1, const __u8 *addr2)
{
	const __u16 *a = (const __u16 *) addr1;
	const __u16 *b = (const __u16 *) addr2;

//	BUILD_BUG_ON(ETH_ALEN != 6);
	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2])) != 0;
}


static inline unsigned compare_ether_addr_64bits(const __u8 addr1[6+2],
						 const __u8 addr2[6+2])
{

	return compare_ether_addr(addr1, addr2);
}


/**
 * eth_type_trans - determine the packet's protocol ID.
 * @skb: received socket data
 * @dev: receiving network device
 *
 * The rule here is that we
 * assume 802.3 if the type field is short enough to be a length.
 * This is normal practice and works for any 'now in use' protocol.
 */
__be16 eth_type_trans(struct sk_buff *skb, struct net_device *dev)
{
	struct ethhdr *eth;
	unsigned char *rawp;

	skb->dev = dev;
	skb_reset_mac_header(skb);
	skb_pull(skb, ETH_HLEN);
	eth = eth_hdr(skb);

	if (unlikely(is_multicast_ether_addr(eth->h_dest))) {
		if (!compare_ether_addr_64bits(eth->h_dest, dev->broadcast))
			skb->pkt_type = PACKET_BROADCAST;
		else
			skb->pkt_type = PACKET_MULTICAST;
	}

	/*
	 *      This ALLMULTI check should be redundant by 1.4
	 *      so don't forget to remove it.
	 *
	 *      Seems, you forgot to remove it. All silly devices
	 *      seems to set IFF_PROMISC.
	 */

	else if (1 /*dev->flags&IFF_PROMISC */ ) {
		if (unlikely(compare_ether_addr_64bits(eth->h_dest, dev->dev_addr)))
			skb->pkt_type = PACKET_OTHERHOST;
	}

	/*
	 * Some variants of DSA tagging don't have an ethertype field
	 * at all, so we check here whether one of those tagging
	 * variants has been configured on the receiving interface,
	 * and if so, set skb->protocol without looking at the packet.
	 */
//	if (netdev_uses_dsa_tags(dev))
//		return htons(ETH_P_DSA);
//	if (netdev_uses_trailer_tags(dev))
//		return htons(ETH_P_TRAILER);

	if (ntohs(eth->h_proto) >= 1536)
		return eth->h_proto;

	rawp = skb->data;

	/*
	 *      This is a magic hack to spot IPX packets. Older Novell breaks
	 *      the protocol design and runs IPX over 802.3 without an 802.2 LLC
	 *      layer. We look for FFFF which isn't a used 802.2 SSAP/DSAP. This
	 *      won't work for fault tolerant netware but does for the rest.
	 */
	if (*(unsigned short *)rawp == 0xFFFF)
		return htons(ETH_P_802_3);

	/*
	 *      Real 802.2 LLC
	 */
	return htons(ETH_P_802_2);
}



