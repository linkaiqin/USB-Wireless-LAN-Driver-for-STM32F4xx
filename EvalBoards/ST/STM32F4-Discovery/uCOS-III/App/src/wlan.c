#include "os.h"
#include "rt_config.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/netifapi.h"
#include "wlan_ethernetif.h"
#include "shell.h"
#include "wlan.h"

 
/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   165
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   1  



struct netif wireless_netif;
extern struct net_device *_pnet_device;




void wireless_send_event(struct net_device *	dev,
			 unsigned int		cmd,
			 union iwreq_data *	wrqu,
			 char *			extra)
{
    
    printf("wireless_send_event[0x%x]",cmd);


    if(cmd == IWEVCUSTOM)
    {
        printf("[0x%x] %s\r\n",wrqu->data.flags,extra);
        switch (wrqu->data.flags) 
        {
            case IW_STA_LINKUP_EVENT_FLAG:
                netifapi_netif_common(&wireless_netif, netif_set_link_up, NULL);
#if LWIP_DHCP
                printf("dhcp_start...\r\n");
                netifapi_netif_common(&wireless_netif, NULL, dhcp_start);
#endif 
                break;
            case IW_STA_LINKDOWN_EVENT_FLAG:
                netifapi_netif_common(&wireless_netif, netif_set_link_down, NULL);
#if LWIP_DHCP
                netifapi_netif_common(&wireless_netif, dhcp_stop, NULL);
                netifapi_netif_common(&wireless_netif, netif_set_down, NULL);
                netifapi_netif_set_addr(&wireless_netif, IP_ADDR_ANY, IP_ADDR_ANY, IP_ADDR_ANY);
#endif                 
                break;                  
        }

    }   
}




int wireless_exec_cmd(char *cmd)
{
    SHELL_ERR err;
    CPU_SR cpu_sr;
    struct net_device *net_dev;
    char s[45];

    CPU_CRITICAL_ENTER();
    if(!_pnet_device)
    {
        CPU_CRITICAL_EXIT();
        return -1;
    }    
    net_dev = _pnet_device;
    net_dev->status |= NET_IOCTL_RUNNING;
    CPU_CRITICAL_EXIT();
    
    strncpy(s,cmd,sizeof(s));
    Shell_Exec(s, NULL, NULL, &err);   

    CPU_CRITICAL_ENTER();
    net_dev->status &= ~NET_IOCTL_RUNNING;
    CPU_CRITICAL_EXIT();
    return 0; 
}




#if LWIP_DHCP   
static void netif_status_callback(struct netif *netif)
{
    ip_addr_t addr;

    if(netif_is_up(netif) && (netif->dhcp))
    {     
        addr = netif->ip_addr;
        printf("DHCP IP:%d.%d.%d.%d\r\n", ip4_addr1(&addr), ip4_addr2(&addr),ip4_addr3(&addr),ip4_addr4(&addr));

        addr = netif->gw;
        printf("DHCP GW:%d.%d.%d.%d\r\n", ip4_addr1(&addr), ip4_addr2(&addr),ip4_addr3(&addr),ip4_addr4(&addr));

        addr = netif->netmask;
        printf("DHCP MASK:%d.%d.%d.%d\r\n", ip4_addr1(&addr), ip4_addr2(&addr),ip4_addr3(&addr),ip4_addr4(&addr));
    }
}
#endif  


/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
void Netif_Config(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw; 


#if LWIP_DHCP
        ipaddr.addr = 0;
        netmask.addr = 0;
        gw.addr = 0;
#else
        IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
        IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
        IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
    
    
        printf("STATIC IP:%d.%d.%d.%d\r\n",ip4_addr1(&ipaddr), ip4_addr2(&ipaddr),ip4_addr3(&ipaddr),ip4_addr4(&ipaddr));
        printf("STATIC GW:%d.%d.%d.%d\r\n",ip4_addr1(&gw), ip4_addr2(&gw),ip4_addr3(&gw),ip4_addr4(&gw));
        printf("STATIC MASK:%d.%d.%d.%d\r\n",ip4_addr1(&netmask), ip4_addr2(&netmask),ip4_addr3(&netmask),ip4_addr4(&netmask));
#endif

        /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
                  struct ip_addr *netmask, struct ip_addr *gw,
                  void *state, err_t (* init)(struct netif *netif),
                  err_t (* input)(struct pbuf *p, struct netif *netif))
    
         Adds your network interface to the netif_list. Allocate a struct
        netif and pass a pointer to this structure as the first argument.
        Give pointers to cleared ip_addr structures when using DHCP,
        or fill them with sane numbers otherwise. The state pointer may be NULL.
    
        The init function pointer must point to a initialization function for
        your ethernet netif interface. The following code illustrates it's use.*/
        netif_add(&wireless_netif, &ipaddr, &netmask, &gw, NULL, &wlan_ethernetif_init, &tcpip_input);
 
        /*  Registers the default network interface.*/
       netifapi_netif_common(&wireless_netif, netif_set_default, NULL);
       
#if LWIP_DHCP
       netif_set_status_callback(&wireless_netif, netif_status_callback);
#else
       netifapi_netif_common(&wireless_netif, netif_set_up, NULL);
#endif

}





