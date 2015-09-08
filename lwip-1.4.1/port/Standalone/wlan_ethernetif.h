#ifndef __WLAN_ETHERNETIF_H__
#define __WLAN_ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

err_t wlan_ethernetif_init(struct netif *netif);
void wlan_ethernetif_input(struct netif *netif, unsigned char *data, u16_t len);

#endif
