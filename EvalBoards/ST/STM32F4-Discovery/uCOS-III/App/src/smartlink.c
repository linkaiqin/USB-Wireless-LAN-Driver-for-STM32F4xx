/* 
  Copyright (C) 2015 Jianguo Zhang <jgzhang.cross@foxmail.com>

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

#include "os.h"
#include "smartlink.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/netifapi.h"
#include "wlan_ethernetif.h"
#include "app_cfg.h"

#include "misc_cvt.h"
#include "net_cvt.h"

#define  LEN_CCM_MIC  8

#define SMARTLINK_GET_BIT(smart_link,bit)  ((smart_link.recv_data_bitmap[bit/32] >> (bit & (32 - 1u))) & 1)
#define SMARTLINK_SET_BIT(smart_link,bit)  (smart_link.recv_data_bitmap[bit/32] |= (1 << (bit & (32 - 1u))))

 
#define SMART_LINK_INIT         0
#define SMART_LINK_CONNECTING   1
#define SMART_LINK_CONNECTED    2


#define SMARTLINK_TLV_TYPE_SSID       0
#define SMARTLINK_TLV_TYP_WPA_KEY     1
#define SMARTLINK_TLV_TYP_IP_ADDRESS  2
#define SMARTLINK_TLV_TYP_WEP_KEY     3
#define SMARTLINK_TLV_TYP_PMK         4



__packed
struct pkt_header
{
    unsigned char  header1;
    unsigned char  header2;
    unsigned char  nonce[8];
    unsigned char  data[0];
};

typedef struct
{
    uint8_t type;
    uint8_t length;
    uint8_t data[1];
} tlv8_data_t;


struct
{
    RTMP_ADAPTER *pAd;
    RALINK_TIMER_STRUCT Timer;
    unsigned char state;
    OS_SEM lock;

    char ssid[32];
    char password[64];
    unsigned char PMK[LEN_PMK];

    int             recv_count;
    unsigned char   recv_data[255 - sizeof(struct pkt_header)];
    int             recv_data_bitmap[16];  //enough
    int          pkt_size_base;

    int scan_count;
    unsigned char          channel;
    unsigned char     channel_is_lock;
    OS_TICK     channel_lock_time;
    OS_TICK     channel_switch_time;
    OS_TICK     connecting_time;
} smart_link;



extern struct net_device *_pnet_device;
struct netif wireless_netif;

static const unsigned char data_packet_multicast_addr[] = {0x01, 0x00, 0x5e, 0x5d};
static const unsigned char beacon_packet_multicast_addr[] = {0x01, 0x00, 0x5e, 0x53, 0x00, 0x00};
 
 
 
//static void dump(unsigned char *data, int data_size)
//{
//    int i;
//    printf("data_size:%d\r\n",data_size);
//    for(i = 0; i < data_size; i += 4)
//    {
//        printf("0x%02x 0x%02x 0x%02x 0x%02x\r\n",data[i],data[i+1],data[i+2],data[i+3]);
//    }

//}
 
static void smartlink_set_state(int state)
{
    BOOLEAN Cancelled;
    
    smart_link.state = state;

//    printf("smartlink_set_state:%d\r\n",state);

    if(state == SMART_LINK_INIT)
    {
        smart_link.recv_count = 0;
        memset(smart_link.recv_data, 0, sizeof(smart_link.recv_data));
        memset(smart_link.recv_data_bitmap, 0, sizeof(smart_link.recv_data_bitmap));
        smart_link.pkt_size_base = 0;
        smart_link.scan_count = 0;
        smart_link.channel = 0;
        smart_link.channel_is_lock = 0;
        smart_link.channel_lock_time = 0;
        smart_link.channel_switch_time = 0;
        smart_link.connecting_time = 0;

        Set_NetworkType_Proc(smart_link.pAd, "Monitor");
        Set_Channel_Proc(smart_link.pAd,"1");
        
        printf("smartlink set STA to monitor mode\r\n");

        RTMPSetTimer(&smart_link.Timer, 20); //20ms
    }
    else if(state == SMART_LINK_CONNECTED)
    {
        RTMPCancelTimer(&smart_link.Timer, &Cancelled);
    }

}


void wireless_send_event(struct net_device *    dev,
                         unsigned int       cmd,
                         union iwreq_data * wrqu,
                         char *         extra)
{
    RTMP_ADAPTER *pAd;
    unsigned char match = 0;
    int i;


    GET_PAD_FROM_NET_DEV(pAd, dev);
    if (pAd == NULL)
    {
        return;
    }

    printf("wireless_send_event[0x%x]",cmd);

    if(cmd == IWEVCUSTOM)
    {
        printf("[0x%x] %s\r\n",wrqu->data.flags,extra);
        switch (wrqu->data.flags)
        {
            case IW_SCAN_COMPLETED_EVENT_FLAG:
                for (i = 0; i < pAd->ScanTab.BssNr; i++)
                {
                    if(!memcmp(pAd->ScanTab.BssEntry[i].Ssid, smart_link.ssid, pAd->ScanTab.BssEntry[i].SsidLen))
                    {
                        NDIS_802_11_AUTHENTICATION_MODE AuthMode = pAd->ScanTab.BssEntry[i].AuthMode;
                        NDIS_802_11_WEP_STATUS WepStatus = pAd->ScanTab.BssEntry[i].WepStatus;


                        if ((AuthMode != Ndis802_11AuthModeWPAPSK) &&
                            (AuthMode != Ndis802_11AuthModeWPA2PSK) &&
                            (AuthMode != Ndis802_11AuthModeOpen)
                           )
                            continue;

                        if(AuthMode == Ndis802_11AuthModeOpen)
                        {
                            match = 1;
                        }
                        else if(AuthMode == Ndis802_11AuthModeWPAPSK)
                        {
                            if(WepStatus == Ndis802_11Encryption2Enabled)//TKIP
                            {
                                Set_AuthMode_Proc(pAd,"WPAPSK");
                                Set_EncrypType_Proc(pAd,"TKIP");
                                match = 1;
                            }
                            else if(WepStatus == Ndis802_11Encryption3Enabled)//AES
                            {
                                Set_AuthMode_Proc(pAd,"WPAPSK");
                                Set_EncrypType_Proc(pAd,"AES");
                                match = 1;
                            }
                        }
                        else if(AuthMode == Ndis802_11AuthModeWPA2PSK)
                        {
                            if(WepStatus == Ndis802_11Encryption2Enabled)//TKIP
                            {
                                Set_AuthMode_Proc(pAd,"WPA2PSK");
                                Set_EncrypType_Proc(pAd,"TKIP");
                                match = 1;
                            }
                            else if(WepStatus == Ndis802_11Encryption3Enabled)//AES
                            {
                                Set_AuthMode_Proc(pAd,"WPA2PSK");
                                Set_EncrypType_Proc(pAd,"AES");
                                match = 1;
                            }
                        }
                    }
                }

                if(match)
                {
//                    printf("wireless_send_event  match ssid:%s\r\n",smart_link.ssid);
//                    Set_WPAPSK_Proc(pAd,smart_link.password);
                    memcpy(pAd->StaCfg.PMK, smart_link.PMK, LEN_PMK);    
                    memset(pAd->StaCfg.WpaPassPhrase, 0, 64);
                    memcpy(pAd->StaCfg.WpaPassPhrase, smart_link.password, strlen(smart_link.password));
                    pAd->StaCfg.WpaPassPhraseLen = strlen(smart_link.password);
                      
                    if(pAd->StaCfg.BssType == BSS_ADHOC &&
                       pAd->StaCfg.AuthMode == Ndis802_11AuthModeWPANone)
                    {
                        pAd->StaCfg.WpaState = SS_NOTUSE;     
                    }
                    else
                    {
                        /* Start STA supplicant state machine */
                        pAd->StaCfg.WpaState = SS_START;
                    } 
       
                }
                else if(++smart_link.scan_count >= 3)
                {
                    smartlink_set_state(SMART_LINK_INIT);   
                }

                break;
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
                netifapi_netif_set_addr(&wireless_netif, IP_ADDR_ANY, IP_ADDR_ANY, IP_ADDR_ANY);
#endif                 
                break;
            case IW_DEAUTH_EVENT_FLAG:
            case IW_DISASSOC_EVENT_FLAG:
            case IW_PAIRWISE_HS_TIMEOUT_EVENT_FLAG:
            case IW_GROUP_HS_TIMEOUT_EVENT_FLAG:
                smartlink_set_state(SMART_LINK_INIT);                          
                break;
        }

    }
}


static void smartlink_switch_channel(RTMP_ADAPTER *pAd, int channel)
{
    char buf[4];
    snprintf(buf,4,"%d",channel);
    Set_Channel_Proc(pAd,buf);
}


static int smartlink_lock()
{
    CPU_SR cpu_sr;    
    OS_ERR err;

    CPU_CRITICAL_ENTER();
    if(!_pnet_device)
    {
        CPU_CRITICAL_EXIT();
        return -1;
    }
    _pnet_device->status |= NET_SMARTLINK_RUNNING;
    CPU_CRITICAL_EXIT();

    OSSemPend(&smart_link.lock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);

    return 0;
}



static void smartlink_unlock()
{
    CPU_SR cpu_sr;    
    OS_ERR err;

    OSSemPost(&smart_link.lock, OS_OPT_POST_1, &err);
    
    CPU_CRITICAL_ENTER();
    _pnet_device->status &= ~NET_SMARTLINK_RUNNING;
    CPU_CRITICAL_EXIT();
}



static void smartlink_period_exec()
{
    static int pre_recv_count = 0;
    struct pkt_header *header;
    OS_ERR err;
    int i, complete = 0;
    unsigned char nonce[13];
    unsigned char key[] = "123456   2345678";//decrypt key(16 bytes), must be same as encrypt key in phone. 
    unsigned int  content_length;
    tlv8_data_t *tlv;


    if(smart_link.state == SMART_LINK_INIT)
    {
        //Check if we receive all data.

        header = (struct pkt_header *)smart_link.recv_data;
        content_length = ((header->header1 & 0x0F ) << 8)  + header->header2;

        if (content_length && ( smart_link.recv_count >= content_length))
        {
            for (i = 0; i < content_length/2; ++i)
            {
                if (!SMARTLINK_GET_BIT(smart_link, i))
                {
                    printf("bit:%d hasn't set\r\n ",i);
                    break;
                }
            }
            if(i >= content_length/2)
                complete = 1;
        }


        if(!complete)
        {
            if(((OSTimeGet(&err) - smart_link.channel_lock_time) > 3*1000) && smart_link.channel_is_lock)
            {
                printf("Lock channel:%d out of time,restart\r\n",smart_link.channel);
                smartlink_set_state(SMART_LINK_INIT);
                return;
            }
            else if(((OSTimeGet(&err) - smart_link.channel_switch_time) > 100) && !smart_link.channel_is_lock)
            {
                smart_link.channel_switch_time = OSTimeGet(&err);

                smart_link.channel++;
                if(smart_link.channel > 14)
                    smart_link.channel = 1;
                
//                printf("Convert to channel:%d\r\n",smart_link.channel);
                /*scan next channel*/
                smartlink_switch_channel(smart_link.pAd, smart_link.channel);
            }


            if(smart_link.channel_is_lock)
            {
                if (smart_link.recv_count != pre_recv_count)
                {
                    pre_recv_count = smart_link.recv_count;

                    printf("%u of %u bytes received\n", smart_link.recv_count, header->header2);
//                    printf("bitmap[0]:0x%08x\r\n",smart_link.recv_data_bitmap[0]);
//                    printf("bitmap[1]:0x%08x\r\n",smart_link.recv_data_bitmap[1]);
//                    printf("bitmap[2]:0x%08x\r\n",smart_link.recv_data_bitmap[2]);
                }
            }

        }
        else
        {
            printf("All data received\r\n");


            memcpy( nonce, header->nonce, 8 );
            memcpy( &nonce[8], "blink", 5 );


            content_length =  (( header->header1 & 0x0F ) << 8) + header->header2 - 10;

            /* Decrypt the data */
            if (AES_CCM_Decrypt(&smart_link.recv_data[10],
                                content_length,
                                key,
                                16,
                                nonce,
                                13,
                                (unsigned char*) header,
                                10,
                                LEN_CCM_MIC,
                                &smart_link.recv_data[10],
                                &content_length) != 0)
            {
                printf("data decryption failed\n");
                smartlink_set_state(SMART_LINK_INIT);
                return;
            }



            tlv = (tlv8_data_t*) header->data;

            /* Process the mandatory SSID */
            if ((tlv->type != SMARTLINK_TLV_TYPE_SSID)
                    || (tlv->length > sizeof(smart_link.ssid)))
            {
                smartlink_set_state(SMART_LINK_INIT);
                return;
            }
            memset(smart_link.ssid,0,sizeof(smart_link.ssid));
            memcpy(smart_link.ssid, tlv->data, tlv->length);



            /* Process the mandatory security key */
            tlv = (tlv8_data_t*) ( &tlv->data[tlv->length] );
            if (((tlv->type != SMARTLINK_TLV_TYP_WPA_KEY) && (tlv->type != SMARTLINK_TLV_TYP_WEP_KEY))
                    || (tlv->length > sizeof(smart_link.password)))
            {
                smartlink_set_state(SMART_LINK_INIT);
                return;
            }
            memset(smart_link.password,0,sizeof(smart_link.password));
            memcpy(smart_link.password, tlv->data, tlv->length);


            //process PMK
            tlv = (tlv8_data_t*) ( &tlv->data[tlv->length] );
            if (tlv->type != SMARTLINK_TLV_TYP_PMK)
            {
                smartlink_set_state(SMART_LINK_INIT);
                return;
            }
            memset(smart_link.PMK, 0, LEN_PMK);
            memcpy(smart_link.PMK, tlv->data, LEN_PMK);


            printf("SSID: %s\n", smart_link.ssid);
            printf("PSK : %s\n", smart_link.password);
            printf("smartlink try to connect %s\r\n",smart_link.ssid);
//            printf("PMK:");
//            dump(smart_link.PMK, LEN_PMK);


            Set_NetworkType_Proc(smart_link.pAd, "Infra");
            Set_AuthMode_Proc(smart_link.pAd,"OPEN");
            Set_EncrypType_Proc(smart_link.pAd,"NONE");
            Set_SSID_Proc(smart_link.pAd, smart_link.ssid);

            smart_link.connecting_time = OSTimeGet(&err);
            smartlink_set_state(SMART_LINK_CONNECTING);
        }
    }
    else if(smart_link.state == SMART_LINK_CONNECTING)
    {
        if((OSTimeGet(&err) - smart_link.connecting_time) > 20*1000)
        {
            printf("smartlink connect %s timeout\r\n",smart_link.ssid);
            smartlink_set_state(SMART_LINK_INIT);
        }
    }

}


static void smartlink_recv_802_11_pkt(unsigned char *buffer, int size)
{
    PHEADER_802_11 header;
    OS_ERR err;
    unsigned char* data = NULL;
    int extra_offset = 0;
    int index; 

    if(smartlink_lock())
        return;

    if(smart_link.state != SMART_LINK_INIT)
        goto RET;


    header = (PHEADER_802_11) buffer;


    if ( memcmp( header->Addr3, beacon_packet_multicast_addr, 6 ) == 0 )
    {
        smart_link.channel_is_lock = 1;
        smart_link.channel_lock_time = OSTimeGet(&err);
        if (smart_link.pkt_size_base == 0 )
        {
            printf("smartlink setup packet detected\r\n");
            smart_link.pkt_size_base = size - 2;
        }
        goto RET;
    }
    else if ( memcmp( header->Addr1, beacon_packet_multicast_addr, 6 ) == 0 )
    {
        smart_link.channel_is_lock = 1;
        smart_link.channel_lock_time = OSTimeGet(&err);
        if (smart_link.pkt_size_base == 0 )
        {
            smart_link.pkt_size_base = size;
        }
        goto RET;
    }
    else if((memcmp( header->Addr1, data_packet_multicast_addr, 4 ) == 0 )
            ||(memcmp( header->Addr3, data_packet_multicast_addr, 4 ) == 0 ))
    {
        smart_link.channel_is_lock = 1;
        smart_link.channel_lock_time = OSTimeGet(&err);
    }


    /* Check if we have known the base size of packet */
    if ( smart_link.pkt_size_base == 0 )
    {
        goto RET;
    }

    /* data packets */
    if ( memcmp( header->Addr3, data_packet_multicast_addr, 4 ) == 0 )
    {
        data = &header->Addr3[4];
        extra_offset = 2;
    }
    else if ( memcmp( header->Addr1, data_packet_multicast_addr, 4 ) == 0 )
    {
        data = &header->Addr1[4];
    }
    else
    {
        goto RET;
    }

    index =  size - smart_link.pkt_size_base - extra_offset;
    if(index >= sizeof(smart_link.recv_data))
    {
        printf("index:%d out range reset\r\n ",index);
        smartlink_set_state(SMART_LINK_INIT);
        goto RET;
    }

    if (SMARTLINK_GET_BIT(smart_link, index/2) == 0)
    {
        smart_link.recv_data[index] = data[0];
        smart_link.recv_data[index + 1] = data[1];
        SMARTLINK_SET_BIT(smart_link, index/2);
        smart_link.recv_count += 2;
    }
    
RET:
    smartlink_unlock();
}



VOID SmartLinkTimeout(
	IN PVOID SystemSpecific1,
	IN PVOID FunctionContext,
	IN PVOID SystemSpecific2,
	IN PVOID SystemSpecific3)
{
    if(smartlink_lock())
        return;

    smartlink_period_exec();

    smartlink_unlock();       
}

BUILD_TIMER_FUNCTION(SmartLinkTimeout);



int smartlink_start(RTMP_ADAPTER *pAd)
{
    int ret;

    if(!pAd)
        return -1;

    ret = smartlink_lock();
    if(ret) 
        return ret;


    smart_link.pAd = pAd;
    RTMPInitTimer(pAd, &smart_link.Timer, GET_TIMER_FUNCTION(SmartLinkTimeout), pAd, TRUE);
    
    smartlink_set_state(SMART_LINK_INIT);
    
    
    smartlink_unlock();
    
    return 0;
}

int smartlink_stop(RTMP_ADAPTER *pAd)
{
    int ret;
    BOOLEAN Cancelled;

    ret = smartlink_lock();
    if(ret) 
        return ret;
   
    smart_link.pAd = NULL;
    RTMPCancelTimer(&smart_link.Timer, &Cancelled);

    smartlink_unlock();

    return 0;
}



int smartlink_init()
{
    OS_ERR err;
    
    memset(&smart_link, 0, sizeof(smart_link));
    OSSemCreate(&smart_link.lock, "smartlink lock", 1, &err);

    register_net_monitor_function(smartlink_recv_802_11_pkt);
   
    return 0;
}

static void netif_status_callback(struct netif *netif)
{
    ip_addr_t addr;

    if(netif_is_up(netif) && (netif->dhcp) && !ip4_addr_cmp(&netif->ip_addr, IP_ADDR_ANY))
    {
        addr = netif->ip_addr;
        printf("DHCP IP:%d.%d.%d.%d\r\n", ip4_addr1(&addr), ip4_addr2(&addr),ip4_addr3(&addr),ip4_addr4(&addr));

        addr = netif->gw;
        printf("DHCP GW:%d.%d.%d.%d\r\n", ip4_addr1(&addr), ip4_addr2(&addr),ip4_addr3(&addr),ip4_addr4(&addr));

        addr = netif->netmask;
        printf("DHCP MASK:%d.%d.%d.%d\r\n", ip4_addr1(&addr), ip4_addr2(&addr),ip4_addr3(&addr),ip4_addr4(&addr));

       if(!smartlink_lock())
       {
            smartlink_set_state(SMART_LINK_CONNECTED);
            smartlink_unlock();
       }
    }
    else
    {
        printf("netif_status_callback dhcp request failed\r\n");
        if(!smartlink_lock())
        {
            smartlink_set_state(SMART_LINK_INIT);
            smartlink_unlock();
        }           
    }
}


/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
void Netif_Config(void)
{

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
    netif_add(&wireless_netif, IP_ADDR_ANY, IP_ADDR_ANY, IP_ADDR_ANY, NULL, &wlan_ethernetif_init, &tcpip_input);

    /*  Registers the default network interface.*/
    netifapi_netif_common(&wireless_netif, netif_set_default, NULL);

    netifapi_netif_common(&wireless_netif, netif_set_up, NULL);

    netif_set_status_callback(&wireless_netif, netif_status_callback);
}






