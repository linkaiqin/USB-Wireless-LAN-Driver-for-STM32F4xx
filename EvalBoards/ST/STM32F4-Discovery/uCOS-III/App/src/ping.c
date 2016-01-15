#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "os.h"
#include "lwip/opt.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/inet.h"
#include "netif/etharp.h"
#include "lwip/ip.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "stdio.h"
#include "ping.h"


/******************************************************
 *                      Macros
 ******************************************************/



#define PING_RCV_TIMEO       (2000)    /* ping receive timeout - in ms */
#define PING_ID              (0xAFAF)
#define PING_MAX_PAYLOAD_SIZE ( 10000 ) /* ping max size */


/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void ping_prepare_echo( char *buf, uint16_t len );
static void ping_recv( int socket_hnd, char *buf, int size);
static void print_stats(void);



/******************************************************
 *               Variable Definitions
 ******************************************************/
#define MAX_DUP_CHK  (8 * 32)

#define BYTE(bit)	G.rcvd_tbl[(bit)>>3]
#define MASK(bit)	(1 << ((bit) & 7))
#define SET(bit)	(BYTE(bit) |= MASK(bit))
#define CLR(bit)	(BYTE(bit) &= (~MASK(bit)))
#define TST(bit)	(BYTE(bit) & MASK(bit))

struct globals
{
    unsigned long ntransmitted, nreceived, nrepeats;
    unsigned tmin, tmax; /* in us */
    unsigned long long tsum; /* in us, sum of all times */
    unsigned char rcvd_tbl[MAX_DUP_CHK / 8];
    const char *hostname;
};
static struct globals G;



/*!
 ******************************************************************************
 * Sends an ICMP ping to the indicated host or IP address
 *
 * @return  0 for success, otherwise error
 */
CPU_INT16S ping (CPU_INT16U argc,
                 CPU_CHAR *argv[],
                 SHELL_OUT_FNCT out_fnct,
                 SHELL_CMD_PARAM *pcmd_param)
{
    CPU_CHAR    *pstr_next;
    struct hostent * host;
    struct sockaddr_in host_addr;
    int socket_hnd;
    struct timeval recv_timeout = {PING_RCV_TIMEO, 0};
    int ret;

    if ( argc == 1 )
    {
        return -1;
    }

    int i        = 0;
    int len      = 56;
    int num      = 3;
    int interval = 1000;
    uint8_t continuous = 0;

    memset(&G, 0, sizeof(struct globals));
    G.hostname = argv[1];
    G.tmin = 0xFFFFFFFF;

    if(strcmp(argv[1], "-h") == 0)
    {
        printf("Usage: ping <destination> [-i <interval in ms>] [-n <number>] [-l <length>]\r\n");
        return 0;      
    }

    for (i = 2; i < argc; i++ )
    {
        switch (argv[i][1])
        {
            case 'i':
                interval = Str_ParseNbr_Int32S(argv[i+1], &pstr_next, 10);
                if ( interval < 0 )
                {
                    printf("min interval 0\n\r");
                    return -4;
                }
                printf("interval: %d milliseconds\n\r", interval);
                i++;
                break;

            case 'l':
                len = Str_ParseNbr_Int32S(argv[i+1], &pstr_next, 10);
                if ( ( len > PING_MAX_PAYLOAD_SIZE ) || ( len < 0 ) )
                {
                    printf("max ping length: %d, min: 0\n\r", PING_MAX_PAYLOAD_SIZE);
                    return -5;
                }
                printf("length: %d\n\r", len);
                i++;
                break;

            case 'n':
                num = Str_ParseNbr_Int32S(argv[i+1], &pstr_next, 10);
                if ( num < 1 )
                {
                    printf("min number of packets 1\n\r");
                    return -6;
                }
                printf("number : %d\n\r", num);
                i++;
                break;

            case 't':
                continuous = 1;
                printf("continuous...\n\r");
                break;
            case 'h':
                printf("Usage: ping <destination> [-i <interval in ms>] [-n <number>] [-l <length>]\r\n");
                return 0;
            default:
                printf("Not supported, ignoring: %s\n\r", argv[i]);
                break;
        }
    }
    char *buf;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + len;
    size_t buf_size = ping_size + sizeof(struct ip_hdr);
    
    host = gethostbyname( argv[1] );

    if ( host == NULL )
    {
        printf( "Could not find host %s\n", argv[1] );
        return -2;
    }

    host_addr.sin_addr.s_addr = *((uint32_t*) host->h_addr_list[0]);
    host_addr.sin_len = sizeof( host_addr );
    host_addr.sin_family = AF_INET;

    /* Open a local socket for pinging */
    if ( ( socket_hnd = lwip_socket( AF_INET, SOCK_RAW, IP_PROTO_ICMP ) ) < 0 )
    {
        printf( "unable to create socket for Ping\r\n" );
        return -3;
    }

    /* Set the receive timeout on local socket so pings will time out. */
    if(lwip_setsockopt( socket_hnd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof( recv_timeout ) ) < 0)
    {
        printf( "unable to set socket option for ping\r\n" );
        return -3;        
    }


    printf ("PING %s (%u.%u.%u.%u)", G.hostname, (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 24 ) & 0xff ),
                      (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 16 ) & 0xff ),
                      (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  8 ) & 0xff ),
                      (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  0 ) & 0xff ) );

    printf(": %d data bytes\r\n", len);


    /* Allocate memory for packet */
    if ( ( buf = mem_malloc(buf_size) ) == NULL)
    {
        return ERR_MEM;
    }


    while (( num > 0 ) || ( continuous == 1 ) )
    {
        OS_ERR err;
        /* Construct ping request */
        ping_prepare_echo( buf, ping_size);

        if ( (ret = lwip_sendto( socket_hnd, buf, ping_size, 0, (struct sockaddr*) &host_addr, host_addr.sin_len )) > 0 )
        {
            ping_recv( socket_hnd, buf, buf_size);
        }
        else
        {
            printf("Ping error:%d\r\n",ret);
        }

        num--;
        if ( ( num > 0 ) || ( continuous == 1 ) )
        {
            OSTimeDlyHMSM(0u, 0u, 0u, interval,
                          OS_OPT_TIME_HMSM_NON_STRICT,
                          &err); // This is simple and should probably wait for a residual period
        }
    }

    // Free the packet
    mem_free( buf );

    print_stats();

    if ( 0 != lwip_close( socket_hnd ) )
    {
        printf("Could not close ping socket\r\n");
        return -7;
    }

    return 0;
}



/**
 *  Prepare the contents of an echo ICMP request packet
 *
 *  @param iecho  : Pointer to an icmp_echo_hdr structure in which the ICMP packet will be constructed
 *  @param len    : The length in bytes of the packet buffer passed to the iecho parameter
 *
 */

static void ping_prepare_echo( char *buf, uint16_t len )
{
    int i, payload_size;
    OS_ERR err;;
    struct icmp_echo_hdr *iecho = (struct icmp_echo_hdr *)buf;
    
    payload_size = len - sizeof(struct icmp_echo_hdr);

    ICMPH_TYPE_SET( iecho, ICMP_ECHO );
    ICMPH_CODE_SET( iecho, 0 );
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = htons(G.ntransmitted);

    /* fill the additional data buffer with some data */
    for ( i = 0; i < payload_size; i++ )
    {
        ( (char*) iecho )[sizeof(struct icmp_echo_hdr) + i] = i;
    }

    *(OS_TICK *)((char*) iecho + sizeof(struct icmp_echo_hdr)) = OSTimeGet(&err);

    iecho->chksum = inet_chksum( iecho, len );

    CLR((uint16_t)G.ntransmitted % MAX_DUP_CHK);
	G.ntransmitted++;
}


static const char *icmp_type_name(int id)
{
    switch (id)
    {
        case ICMP_ER:
            return "Echo Reply";
        case ICMP_DUR:
            return "Destination Unreachable";
        case ICMP_SQ:
            return "Source Quench";
        case ICMP_RD:
            return "Redirect (change route)";
        case ICMP_ECHO:
            return "Echo Request";
        case ICMP_TE:
            return "Time Exceeded";
        case ICMP_PP:
            return "Parameter Problem";
        case ICMP_TS:
            return "Timestamp Request";
        case ICMP_TSR:
            return "Timestamp Reply";
        case ICMP_IRQ:
            return "Information Request";
        case ICMP_IR:
            return "Information Reply";
//         case ICMP_AM:
//             return "Address Mask Request";
//         case ICMP_AMR:
//             return "Address Mask Reply";
        default:
            return "unknown ICMP type";
    }
}
#define ICMP_MINLEN 8               /* abs minimum */

static void unpack_tail(int sz, uint32_t *tp,
                        const char *from_str,
                        uint16_t recv_seq, int ttl)
{
    OS_ERR err;
    unsigned char *b, m;
    const char *dupmsg = " (DUP!)";
    unsigned triptime = triptime; /* for gcc */

    if (tp)
    {
        /* (int32_t) cast is for hypothetical 64-bit unsigned */
        /* (doesn't hurt 32-bit real-world anyway) */
        triptime = (int32_t) ((uint32_t)OSTimeGet(&err) - *tp);
        G.tsum += triptime;
        if (triptime < G.tmin)
            G.tmin = triptime;
        if (triptime > G.tmax)
            G.tmax = triptime;
    }

    b = &BYTE(recv_seq % MAX_DUP_CHK);
    m = MASK(recv_seq % MAX_DUP_CHK);
    /*if TST(recv_seq % MAX_DUP_CHK):*/
    if (*b & m)
    {
        ++G.nrepeats;
    }
    else
    {
        /*SET(recv_seq % MAX_DUP_CHK):*/
        *b |= m;
        ++G.nreceived;
        dupmsg += 7;
    }


    printf("%d bytes from %s: seq=%u ttl=%d", sz,
           from_str, recv_seq, ttl);
    if (tp)
        printf(" time=%u ms", triptime);
    puts(dupmsg);
}

static void unpack4(char *buf, int sz, struct sockaddr_in *from)
{
    struct icmp_echo_hdr *icmppkt;
    struct ip_hdr *iphdr;
    int hlen;

    /* discard if too short */
//    if (sz < (datalen + ICMP_MINLEN))
//        return;

    /* check IP header */
    iphdr = (struct ip_hdr *) buf;
    hlen = IPH_HL( iphdr ) * 4;
    sz -= hlen;
    icmppkt = (struct icmp_echo_hdr *) (buf + hlen);
    if (icmppkt->id != PING_ID)
        return;             /* not our ping */

    if (icmppkt->type == ICMP_ER)
    {
        uint16_t recv_seq = ntohs(icmppkt->seqno);
        uint32_t *tp = NULL;

        if (sz >= ICMP_MINLEN + sizeof(uint32_t))
            tp = (uint32_t *) (buf + hlen + sizeof(struct icmp_echo_hdr));
        unpack_tail(sz, tp,
                    inet_ntoa(*(struct in_addr *) &from->sin_addr.s_addr),
                    recv_seq, IPH_TTL(iphdr));
    }
    else if (icmppkt->type != ICMP_ECHO)
    {
        printf("warning: got ICMP %d (%s)",
               icmppkt->type,
               icmp_type_name(icmppkt->type));
    }
}

static void print_stats()
{
	unsigned long ul;
	unsigned long nrecv;

	nrecv = G.nreceived;
	printf("\n--- %s ping statistics ---\n"
		"%lu packets transmitted, "
		"%lu packets received, ",
		G.hostname, G.ntransmitted, nrecv
	);
	if (G.nrepeats)
		printf("%lu duplicates, ", G.nrepeats);
	ul = G.ntransmitted;
	if (ul != 0)
		ul = (ul - nrecv) * 100 / ul;
	printf("%lu%% packet loss\n", ul);
	if (G.tmin != 0xFFFFFFFF) {
		unsigned tavg = G.tsum / (nrecv + G.nrepeats);
		printf("round-trip min/avg/max = %u/%u/%u ms\n",
			G.tmin ,
			tavg ,
			G.tmax );
	}
}


static void ping_recv( int socket_hnd, char *buf, int size)
{
    int fromlen, len;
    struct sockaddr_in from;

    len = lwip_recvfrom( socket_hnd, buf, size, 0, (struct sockaddr*) &from, (socklen_t*) &fromlen );
    
    if(len < 0)
    {
        printf("Ping timeout\r\n");
    }
    else if ( len >= (int) ( sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr) ) )
    {

        unpack4(buf, len, &from);

    }
}

