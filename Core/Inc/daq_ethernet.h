#ifndef DAQ_ETHERNET_H
#define DAQ_ETHERNET_H
#include "lwip/netif.h"
#include "lwip/pbuf.h"
err_t DAQ_EthernetInit(struct netif *netif);
uint32_t DAQ_EthernetSuspend(struct netif *netif);
uint32_t DAQ_EthernetPoll(struct netif *netif);
uint32_t DAQ_NetClockMs(void);
uint32_t DAQ_NetClockCycles(void);
#endif
