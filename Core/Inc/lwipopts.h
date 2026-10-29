#ifndef REMOTEDAQ_LWIPOPTS_H
#define REMOTEDAQ_LWIPOPTS_H
/* Bare-metal raw API. All stack calls run in main; no RTOS or libc heap. */
#define NO_SYS 1
#define SYS_LIGHTWEIGHT_PROT 0
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0
#define LWIP_TCP 0
#define LWIP_RAW 0
#define LWIP_IPV6 0
#define LWIP_IPV4 1
#define LWIP_UDP 1
#define LWIP_ICMP 1
#define LWIP_ARP 1
#define LWIP_ETHERNET 1
#define LWIP_DHCP 0
#define LWIP_AUTOIP 0
#define LWIP_IGMP 0
#define IP_FORWARD 0
#define IP_REASSEMBLY 0
#define IP_FRAG 0
#define LWIP_NETIF_HOSTNAME 0
#define LWIP_NETIF_STATUS_CALLBACK 0
#define LWIP_NETIF_LINK_CALLBACK 0
#define LWIP_STATS 0
#define MEM_ALIGNMENT 4
#define MEM_SIZE 8192
#define MEMP_NUM_PBUF 8
#define MEMP_NUM_UDP_PCB 1
#define MEMP_NUM_SYS_TIMEOUT 4
#define LWIP_SUPPORT_CUSTOM_PBUF 1
#define PBUF_POOL_SIZE 8
#define PBUF_POOL_BUFSIZE 512
#define ARP_TABLE_SIZE 4
/* Real-time latest-only send: resolve peer first; never silently ARP-queue
 * an application frame whose SEQ would be reported as sent too early. */
#define ARP_QUEUEING 0
#define LWIP_ARP_FILTER_NETIF 0
#define LWIP_NETIF_TX_SINGLE_PBUF 1
#define CHECKSUM_GEN_IP 1
#define CHECKSUM_GEN_UDP 1
#define CHECKSUM_CHECK_IP 1
#define CHECKSUM_CHECK_UDP 1
#define LWIP_CHKSUM_ALGORITHM 2
#define LWIP_RAND() 0U
#endif
