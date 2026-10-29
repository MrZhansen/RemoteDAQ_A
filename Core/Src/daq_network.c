#include "daq_network.h"
#include "daq_network_config.h"
#include "daq_ethernet.h"
#include "lwip/init.h"
#include "lwip/udp.h"
#include "lwip/etharp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

volatile DAQ_NetDiagnostics daq_net;
static struct netif network_interface;
static struct udp_pcb *control_pcb;
static ip_addr_t peer_address;
static DAQ_ControlCommand controls[DAQ_NET_CONTROL_CAPACITY];
static uint32_t control_read, control_write, control_count, arp_tick;

u32_t sys_now(void) { return DAQ_NetClockMs(); }
void DAQ_NetAssert(const char *message)
{
  (void)message;
  daq_net.assertion_count++;
  daq_net.fault = 1U;
  daq_net.last_error = 1U;
  /* Never enter Error_Handler or stop acquisition on a network assertion. */
}
static void SetAddress(ip4_addr_t *address, uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{ IP4_ADDR(address, a,b,c,d); }

/* Called by ethernet_input in main, never by an ISR. Source port is not
 * restricted: V1.1 specifies destination ports but not the B source port. */
static void ReceiveControl(void *argument, struct udp_pcb *pcb,
    struct pbuf *packet, const ip_addr_t *address, u16_t port)
{
  uint8_t bytes[DAQ_CONTROL_FRAME_SIZE];
  DAQ_ControlCommand command;
  DAQ_ControlParseStatus status;
  (void)argument; (void)pcb; (void)port;
  if (packet == NULL) { return; }
  if ((daq_net.enabled == 0U) || !ip_addr_cmp(address, &peer_address))
  { daq_net.rx_wrong_peer++; }
  else if (packet->tot_len != DAQ_CONTROL_FRAME_SIZE)
  { daq_net.rx_bad_length++; }
  else if (pbuf_copy_partial(packet, bytes, sizeof(bytes), 0U) != sizeof(bytes))
  { daq_net.rx_dropped++; }
  else
  {
    status = DAQ_ParseControlFrame(bytes, sizeof(bytes), &command);
    if (status == DAQ_CONTROL_BAD_CRC) { daq_net.rx_crc_error++; }
    else if (status != DAQ_CONTROL_OK) { daq_net.rx_format_error++; }
    else if (control_count == DAQ_NET_CONTROL_CAPACITY)
    { daq_net.rx_queue_overflow++; }
    else
    {
      controls[control_write] = command;
      control_write = (control_write + 1U) % DAQ_NET_CONTROL_CAPACITY;
      control_count++; daq_net.rx_control_count++;
    }
  }
  pbuf_free(packet);
}

uint32_t DAQ_NetSetEnabled(uint32_t enabled)
{
  ip4_addr_t address, mask, gateway;
  uint32_t suspended;
  if (enabled > 1U) { return 0U; }
  if (enabled == 0U)
  {
    if (daq_net.initialized != 0U)
    {
      suspended = DAQ_EthernetSuspend(&network_interface);
      if (suspended != 1U) { return suspended; }
    }
    daq_net.enabled = 0U; daq_net.peer_ready = 0U;
    control_read = control_write = control_count = 0U;
    return 1U;
  }
  if (daq_net.fault != 0U) { return 0U; }
  if (daq_net.initialized == 0U)
  {
    daq_net.init_count++;
    lwip_init();
    SetAddress(&address, DAQ_NET_A_IP);
    SetAddress(&mask, DAQ_NET_NETMASK);
    SetAddress(&gateway, 0,0,0,0);
    SetAddress(ip_2_ip4(&peer_address), DAQ_NET_B_IP);
    if (netif_add(&network_interface, &address, &mask, &gateway, NULL,
                  DAQ_EthernetInit, ethernet_input) == NULL)
    {
      /* Preserve precise adapter errors, including identity refusal (9). */
      if (daq_net.last_error == 0U) { daq_net.last_error = 12U; }
      daq_net.init_error_count++; daq_net.fault = 1U; return 0U;
    }
    netif_set_default(&network_interface);
    control_pcb = udp_new();
    if (control_pcb == NULL)
    {
      /* Init has not started ETH DMA. Remove the unused LwIP interface. */
      netif_remove(&network_interface);
      daq_net.last_error = 10U; /* UDP PCB allocation failed. */
      daq_net.init_error_count++; daq_net.fault = 1U; return 0U;
    }
    if (udp_bind(control_pcb, IP_ADDR_ANY, DAQ_NET_LOCAL_PORT) != ERR_OK)
    {
      udp_remove(control_pcb); control_pcb = NULL;
      netif_remove(&network_interface);
      daq_net.last_error = 11U; /* UDP bind failed; no false ready state. */
      daq_net.init_error_count++; daq_net.fault = 1U; return 0U;
    }
    udp_recv(control_pcb, ReceiveControl, NULL);
    daq_net.initialized = 1U;
  }
  else if ((daq_net.enabled == 0U) &&
           (DAQ_EthernetInit(&network_interface) != ERR_OK))
  { daq_net.init_error_count++; daq_net.fault = 1U; return 0U; }
  daq_net.enabled = 1U;
  arp_tick = DAQ_NetClockMs() - DAQ_NET_ARP_PERIOD_MS;
  return 1U;
}
uint32_t DAQ_NetReady(void)
{
  return (daq_net.enabled != 0U) && (daq_net.fault == 0U) &&
         (daq_net.link_up != 0U) && (daq_net.peer_ready != 0U);
}
uint32_t DAQ_NetPoll(void)
{
  uint32_t start, elapsed, work, now;
  struct eth_addr *hardware_address;
  const ip4_addr_t *resolved;
  if ((daq_net.enabled == 0U) || (daq_net.fault != 0U)) { return 0U; }
  start = DAQ_NetClockCycles();
  work = DAQ_EthernetPoll(&network_interface);
  sys_check_timeouts();
  now = DAQ_NetClockMs();
  daq_net.peer_ready = (daq_net.link_up != 0U) &&
      (etharp_find_addr(&network_interface, ip_2_ip4(&peer_address),
                       &hardware_address, &resolved) >= 0);
  if ((daq_net.link_up != 0U) && (daq_net.peer_ready == 0U) &&
      ((uint32_t)(now - arp_tick) >= DAQ_NET_ARP_PERIOD_MS))
  {
    arp_tick = now;
    (void)etharp_request(&network_interface, ip_2_ip4(&peer_address));
    work = 1U;
  }
  elapsed = (uint32_t)(DAQ_NetClockCycles() - start);
  if (elapsed > daq_net.poll_max_cycles) { daq_net.poll_max_cycles = elapsed; }
  daq_net.poll_count++;
  return work;
}
uint32_t DAQ_NetTakeControl(DAQ_ControlCommand *command)
{
  if ((command == NULL) || (control_count == 0U) ||
      (daq_net.enabled == 0U) || (daq_net.fault != 0U)) { return 0U; }
  *command = controls[control_read];
  control_read = (control_read + 1U) % DAQ_NET_CONTROL_CAPACITY;
  control_count--;
  return 1U;
}
uint32_t DAQ_NetSendFrame(const uint8_t *frame, uint16_t length)
{
  struct pbuf *packet;
  err_t result;
  if ((frame == NULL) ||
      !(((length == DAQ_DATA_FRAME_SIZE) && (frame[2] == 1U)) ||
        ((length == DAQ_STATUS_FRAME_SIZE) && (frame[2] == 3U))) ||
      (frame[0] != 0xAAU) || (frame[1] != 0x55U))
  { daq_net.last_error = 2U; daq_net.tx_error++; return 0U; }
  if (DAQ_NetReady() == 0U) { daq_net.tx_busy++; return 0U; }
  packet = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
  if (packet == NULL) { daq_net.tx_busy++; return 0U; }
  result = pbuf_take(packet, frame, length);
  if (result == ERR_OK)
  { result = udp_sendto(control_pcb, packet, &peer_address, DAQ_NET_PEER_PORT); }
  pbuf_free(packet); /* linkoutput copied into an owned DMA TX slot. */
  if (result != ERR_OK)
  { daq_net.last_error = (uint32_t)(int32_t)result; daq_net.tx_error++; return 0U; }
  if (length == DAQ_DATA_FRAME_SIZE) { daq_net.data_submitted++; }
  else { daq_net.status_submitted++; }
  return 1U;
}
