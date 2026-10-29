#ifndef DAQ_NETWORK_CONFIG_H
#define DAQ_NETWORK_CONFIG_H

/* V1.1 candidate configuration. No DHCP, gateway or B source-port assumption.
 * Both boards must select the same transport before their joint test. */
#define DAQ_NET_A_IP     192,168,1,10
#define DAQ_NET_B_IP     192,168,1,20
#define DAQ_NET_NETMASK  255,255,255,0
#define DAQ_NET_LOCAL_PORT 5006U
#define DAQ_NET_PEER_PORT  5005U
#define DAQ_NET_MAC {0x02U,0x46U,0xDAU,0x00U,0x00U,0x0AU}
#define DAQ_NET_TX_STALL_MS 20U
#define DAQ_NET_RX_BUDGET 2U
#define DAQ_NET_CONTROL_CAPACITY 8U
#define DAQ_NET_LINK_PERIOD_MS 250U
#define DAQ_NET_ARP_PERIOD_MS 1000U

#endif
