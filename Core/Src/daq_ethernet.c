/* STM32F746G-DISCO / LAN8742A, modern CubeF7 HAL ETH API.
 * Main-context polling, no ETH IRQ, no TX-completion wait and no RTOS.
 * Initial/link HAL operations have finite timeouts and may skip groups, but
 * never mask acquisition IRQs. No runtime PHY reset pin is toggled. */
#include "main.h"
#include "daq_network.h"
#include "daq_network_config.h"
#include "daq_ethernet.h"
#include "lan8742.h"
#include "lwip/etharp.h"
#include <string.h>
#define NET_BUFFER_BYTES 1536U
#define NET_RX_SLOTS 8U
#define NET_TX_SLOTS ETH_TX_DESC_CNT

typedef struct {
  ETH_DMADescTypeDef rx[ETH_RX_DESC_CNT];
  ETH_DMADescTypeDef tx[ETH_TX_DESC_CNT];
  uint8_t descriptor_padding[192U]; /* Entire first 512 B is Device memory. */
  uint8_t rx_bytes[NET_RX_SLOTS][NET_BUFFER_BYTES];
  uint8_t tx_bytes[NET_TX_SLOTS][NET_BUFFER_BYTES];
} EthernetDMAStorage;
typedef char DescriptorRegionSizeCheck[(offsetof(EthernetDMAStorage,rx_bytes)==512U)?1:-1];
/* Explicit SRAM1 area in ICF; separate from ADC at 0x20010000. */
#pragma location=".eth_dma"
#pragma data_alignment=32
__no_init static EthernetDMAStorage ethernet_dma;
typedef struct { struct pbuf_custom custom; uint32_t used; } RxSlot;
static RxSlot rx_slots[NET_RX_SLOTS];
static uint32_t tx_used[NET_TX_SLOTS];
static uint32_t tx_started[NET_TX_SLOTS];
static ETH_HandleTypeDef network_eth;
static lan8742_Object_t network_phy;
static uint8_t mac_address[6] = DAQ_NET_MAC;
static uint32_t link_tick;
uint32_t DAQ_NetClockMs(void) { return HAL_GetTick(); }
uint32_t DAQ_NetClockCycles(void) { return DWT->CYCCNT; }

/* No NVIC enable: ReadData/ReleaseTxPacket execute in main. */
void HAL_ETH_MspInit(ETH_HandleTypeDef *handle)
{
  GPIO_InitTypeDef gpio = {0};
  if (handle->Instance != ETH) { return; }
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_ETH_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_AF_PP; gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF11_ETH;
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOG, &gpio);
}
static int32_t PhyRead(uint32_t address, uint32_t reg, uint32_t *value)
{ return (HAL_ETH_ReadPHYRegister(&network_eth,address,reg,value)==HAL_OK) ? 0 : -1; }
static int32_t PhyWrite(uint32_t address, uint32_t reg, uint32_t value)
{ return (HAL_ETH_WritePHYRegister(&network_eth,address,reg,value)==HAL_OK) ? 0 : -1; }
static int32_t PhyTick(void) { return (int32_t)HAL_GetTick(); }
static void RxFree(struct pbuf *packet)
{
  uint32_t i;
  for (i=0U; i<NET_RX_SLOTS; i++)
  { if (packet==&rx_slots[i].custom.pbuf) { rx_slots[i].used=0U; return; } }
  daq_net.fault=1U;
}
void HAL_ETH_RxAllocateCallback(uint8_t **buffer)
{
  uint32_t i;
  *buffer=NULL;
  if (daq_net.fault!=0U) { return; } /* Never rearm a fault-pinned pool. */
  for (i=0U; i<NET_RX_SLOTS; i++)
  {
    if (rx_slots[i].used==0U)
    {
      rx_slots[i].used=1U;
      rx_slots[i].custom.custom_free_function=RxFree;
      (void)pbuf_alloced_custom(PBUF_RAW,0U,PBUF_REF,&rx_slots[i].custom,
                               ethernet_dma.rx_bytes[i],NET_BUFFER_BYTES);
      *buffer=ethernet_dma.rx_bytes[i]; return;
    }
  }
}
void HAL_ETH_RxLinkCallback(void **start, void **end, uint8_t *buffer, uint16_t length)
{
  struct pbuf *packet, *cursor, *tail=NULL;
  uintptr_t base=(uintptr_t)&ethernet_dma.rx_bytes[0][0];
  uintptr_t address=(uintptr_t)buffer;
  uint32_t index, chain_count=0U, slot;
  if ((start==NULL) || (end==NULL)) { daq_net.fault=1U; return; }
  if (daq_net.fault!=0U) { goto invalid; }
  if ((address<base) || ((address-base)>=sizeof(ethernet_dma.rx_bytes)) ||
      (((address-base)%NET_BUFFER_BYTES)!=0U) || (length>NET_BUFFER_BYTES))
  { goto invalid; }
  index=(uint32_t)((address-base)/NET_BUFFER_BYTES);
  if (rx_slots[index].used!=1U) { goto invalid; }
  packet=&rx_slots[index].custom.pbuf;
  /* Validate the prefix before any mutation. A repeated buffer would make a
   * self-cycle; corrupt prefixes must not enter pbuf_free/ethernet_input.
   * Every dereferenced node must be one of our currently allocated slots. */
  cursor=(struct pbuf *)*start;
  while (cursor!=NULL)
  {
    if ((chain_count>=NET_RX_SLOTS-1U) || (cursor==packet)) { goto invalid; }
    for (slot=0U; slot<NET_RX_SLOTS; slot++)
    { if (cursor==&rx_slots[slot].custom.pbuf) { break; } }
    if ((slot==NET_RX_SLOTS) || (rx_slots[slot].used!=1U)) { goto invalid; }
    if ((cursor->len>NET_BUFFER_BYTES) ||
        (cursor->tot_len>(NET_RX_SLOTS*NET_BUFFER_BYTES-length))) { goto invalid; }
    tail=cursor; cursor=cursor->next; chain_count++;
  }
  if ((chain_count!=0U) && (tail!=(struct pbuf *)*end)) { goto invalid; }
  /* HAL resets pRxStart but may leave pRxEnd stale between packets.
   * An empty prefix deliberately ignores that stale end pointer. */
  cursor=(struct pbuf *)*start;
  for (slot=0U; slot<chain_count; slot++)
  { cursor->tot_len=(u16_t)(cursor->tot_len+length); cursor=cursor->next; }
  packet->next=NULL; packet->len=length; packet->tot_len=length;
  if (tail==NULL) { *start=packet; }
  else { tail->next=packet; }
  *end=packet;
  return;
invalid:
  /* Pin the finite pool until reset. Clearing only the HAL list roots keeps
   * an untrusted/cyclic list away from later traversal and release paths. */
  daq_net.fault=1U; *start=NULL; *end=NULL;
}
void HAL_ETH_TxFreeCallback(uint32_t *token)
{
  uint32_t i;
  for (i=0U; i<NET_TX_SLOTS; i++)
  {
    if (token==&tx_used[i])
    {
      uint32_t descriptor=network_eth.TxDescList.releaseIndex;
      if (descriptor>=ETH_TX_DESC_CNT) { daq_net.fault=1U; return; }
      if ((ethernet_dma.tx[descriptor].DESC0 & ETH_DMATXDESC_ES)!=0U)
      { daq_net.tx_dma_error++; }
      else { daq_net.tx_completed++; }
      tx_used[i]=0U; return;
    }
  }
  daq_net.fault=1U;
}
static err_t LinkOutput(struct netif *interface, struct pbuf *packet)
{
  uint32_t i;
  ETH_BufferTypeDef buffer={0};
  ETH_TxPacketConfig config={0};
  (void)interface;
  if ((daq_net.fault!=0U) || (daq_net.link_up==0U)) { return ERR_IF; }
  if ((packet==NULL) || (packet->tot_len>NET_BUFFER_BYTES)) { return ERR_BUF; }
  (void)HAL_ETH_ReleaseTxPacket(&network_eth);
  /* Completion callbacks may latch an ownership/integrity fault. */
  if (daq_net.fault!=0U) { return ERR_IF; }
  for (i=0U; i<NET_TX_SLOTS; i++) { if (tx_used[i]==0U) { break; } }
  if (i==NET_TX_SLOTS) { daq_net.tx_busy++; return ERR_MEM; }
  if (pbuf_copy_partial(packet,ethernet_dma.tx_bytes[i],packet->tot_len,0U)!=packet->tot_len)
  { return ERR_BUF; }
  tx_used[i]=1U;
  buffer.buffer=ethernet_dma.tx_bytes[i]; buffer.len=packet->tot_len;
  /* HAL initializes CIC to full offload. Explicitly bypass it per packet:
   * LwIP has already generated IPv4/UDP checksums in software. */
  config.Attributes=ETH_TX_PACKETS_FEATURES_CRCPAD|ETH_TX_PACKETS_FEATURES_CSUM;
  config.ChecksumCtrl=ETH_CHECKSUM_DISABLE;
  config.CRCPadCtrl=ETH_CRC_PAD_INSERT;
  config.Length=packet->tot_len; config.TxBuffer=&buffer; config.pData=&tx_used[i];
  __DMB();
  daq_net.last_hal_status=(uint32_t)HAL_ETH_Transmit_IT(&network_eth,&config);
  if (daq_net.last_hal_status!=(uint32_t)HAL_OK)
  { tx_used[i]=0U; daq_net.hal_error=network_eth.ErrorCode; return ERR_MEM; }
  tx_started[i]=HAL_GetTick();
  daq_net.tx_submitted++; return ERR_OK;
}
/* RM0385 Rev9 40.6.1: DEV_ID 0x449 identifies F74/F75, not a package.
 * ES0290 Rev7 p1 lists Z/1 (0x1001). RM0385 also names A (0x1000),
 * but that revision is not covered by ES0290 Rev7's applicability table.
 * ES0334 2.21.6 is F76/F77 only; do not infer it applies to this F746. */
static DAQ_NetIdentityStatus SiliconIdentityStatus(uint32_t idcode)
{
  uint32_t device=(idcode & DBGMCU_IDCODE_DEV_ID_Msk) >> DBGMCU_IDCODE_DEV_ID_Pos;
  uint32_t revision=(idcode & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos;
  if (device!=0x449U) { return DAQ_NET_ID_WRONG_DEVICE; }
  if (revision==0x1001U) { return DAQ_NET_ID_ES0290_LISTED; }
  if (revision==0x1000U) { return DAQ_NET_ID_REV_A_UNQUALIFIED; }
  return DAQ_NET_ID_UNKNOWN_REVISION;
}
err_t DAQ_EthernetInit(struct netif *interface)
{
  lan8742_IOCtx_t io={NULL,NULL,PhyWrite,PhyRead,PhyTick};
  uint32_t id1,id2;
  ETH_MACConfigTypeDef mac;
  MPU_Region_InitTypeDef region={0};
  uint32_t idcode=DBGMCU->IDCODE; /* One real MMIO read; no default revision. */
  daq_net.identity_status=(uint32_t)DAQ_NET_ID_UNREAD;
  daq_net.cache_ccr=SCB->CCR;
  daq_net.device_idcode=idcode;
  daq_net.device_id=(idcode & DBGMCU_IDCODE_DEV_ID_Msk) >> DBGMCU_IDCODE_DEV_ID_Pos;
  daq_net.revision=(idcode & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos;
  __DMB();
  daq_net.identity_status=(uint32_t)SiliconIdentityStatus(idcode);
  /* Reject unqualified/unknown revisions and wrong families before GPIO,
   * ETH or MPU setup. Keep Rev A closed pending F746-specific evidence. */
  if (daq_net.identity_status!=(uint32_t)DAQ_NET_ID_ES0290_LISTED)
  { daq_net.last_error=9U; return ERR_IF; }
  if ((SCB->CCR & SCB_CCR_DC_Msk)!=0U)
  { daq_net.last_error=3U; return ERR_IF; }
  /* Official same-board example uses Device attributes for descriptors.
   * Region 0 stays intact; no acquisition IRQ accesses region 1. */
  region.Enable=MPU_REGION_ENABLE; region.Number=MPU_REGION_NUMBER1;
  region.BaseAddress=0x20018000UL; region.Size=MPU_REGION_SIZE_512B;
  region.AccessPermission=MPU_REGION_FULL_ACCESS;
  region.DisableExec=MPU_INSTRUCTION_ACCESS_DISABLE;
  region.TypeExtField=MPU_TEX_LEVEL0;
  region.IsShareable=MPU_ACCESS_SHAREABLE;
  region.IsCacheable=MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable=MPU_ACCESS_BUFFERABLE;
  __DMB(); HAL_MPU_ConfigRegion(&region); __DSB(); __ISB();
  daq_net.descriptor_mpu_base=MPU->RBAR;
  daq_net.descriptor_mpu_attributes=MPU->RASR;
  if (((MPU->RBAR & 0xFFFFFFE0UL)!=0x20018000UL) ||
      (MPU->RASR!=0x13050011UL)) /* XN/AP=3/S/B/512B/enable. */
  { daq_net.last_error=8U; return ERR_IF; }
  memset(&ethernet_dma,0,sizeof(ethernet_dma));
  memset(rx_slots,0,sizeof(rx_slots)); memset(tx_used,0,sizeof(tx_used));
  network_eth.Instance=ETH; network_eth.Init.MACAddr=mac_address;
  network_eth.Init.MediaInterface=HAL_ETH_RMII_MODE;
  network_eth.Init.RxDesc=ethernet_dma.rx; network_eth.Init.TxDesc=ethernet_dma.tx;
  network_eth.Init.RxBuffLen=NET_BUFFER_BYTES;
  daq_net.last_hal_status=(uint32_t)HAL_ETH_Init(&network_eth);
  if (daq_net.last_hal_status!=(uint32_t)HAL_OK)
  { daq_net.hal_error=network_eth.ErrorCode; return ERR_IF; }
  HAL_ETH_SetMDIOClockRange(&network_eth);
  if ((LAN8742_RegisterBusIO(&network_phy,&io)!=LAN8742_STATUS_OK) ||
      (LAN8742_Init(&network_phy)!=LAN8742_STATUS_OK)) { return ERR_IF; }
  daq_net.phy_address=network_phy.DevAddr;
  if ((PhyRead(network_phy.DevAddr,LAN8742_PHYI1R,&id1)<0) ||
      (PhyRead(network_phy.DevAddr,LAN8742_PHYI2R,&id2)<0)) { return ERR_IF; }
  daq_net.phy_id=(id1<<16U)|id2;
  if ((daq_net.phy_id & 0xFFFFFFF0UL)!=0x0007C130UL) { return ERR_IF; }
  if (HAL_ETH_GetMACConfig(&network_eth,&mac)!=HAL_OK) { return ERR_IF; }
  mac.ChecksumOffload=DISABLE;
  if (HAL_ETH_SetMACConfig(&network_eth,&mac)!=HAL_OK) { return ERR_IF; }
  interface->name[0]='e'; interface->name[1]='0'; interface->hwaddr_len=6U;
  memcpy(interface->hwaddr,mac_address,sizeof(mac_address));
  interface->mtu=1500U; interface->flags=NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP;
  interface->output=etharp_output; interface->linkoutput=LinkOutput;
  link_tick=HAL_GetTick()-DAQ_NET_LINK_PERIOD_MS;
  return ERR_OK;
}
uint32_t DAQ_EthernetPoll(struct netif *interface)
{
  uint32_t now=HAL_GetTick(), work=0U, i;
  ETH_MACConfigTypeDef mac;
  struct pbuf *packet;
  if ((uint32_t)(now-link_tick)>=DAQ_NET_LINK_PERIOD_MS)
  {
    link_tick=now; daq_net.phy_state=LAN8742_GetLinkState(&network_phy);
    if (daq_net.phy_state<0)
    { daq_net.fault=1U; daq_net.last_error=4U; return 1U; }
    if ((daq_net.phy_state>=LAN8742_STATUS_100MBITS_FULLDUPLEX) &&
        (daq_net.phy_state<=LAN8742_STATUS_10MBITS_HALFDUPLEX))
    {
      if (daq_net.link_up==0U)
      {
        if (HAL_ETH_GetMACConfig(&network_eth,&mac)!=HAL_OK)
        { daq_net.fault=1U; return 1U; }
        mac.Speed=(daq_net.phy_state<=LAN8742_STATUS_100MBITS_HALFDUPLEX)?ETH_SPEED_100M:ETH_SPEED_10M;
        mac.DuplexMode=((daq_net.phy_state==LAN8742_STATUS_100MBITS_FULLDUPLEX) ||
            (daq_net.phy_state==LAN8742_STATUS_10MBITS_FULLDUPLEX))?ETH_FULLDUPLEX_MODE:ETH_HALFDUPLEX_MODE;
        if ((HAL_ETH_SetMACConfig(&network_eth,&mac)!=HAL_OK) ||
            (HAL_ETH_Start(&network_eth)!=HAL_OK)) { daq_net.fault=1U; return 1U; }
        network_eth.Instance->DMAIER=0U;
        netif_set_up(interface); netif_set_link_up(interface); daq_net.link_up=1U;
      }
    }
    else if (daq_net.link_up!=0U)
    {
      netif_set_link_down(interface); netif_set_down(interface); daq_net.link_up=0U;
      /* Latch failure; Suspend will quiesce DMA before UART resumes.
       * Do not HAL_Stop/flush an uncertain TxFIFO (ES0290 2.14.3). */
      daq_net.fault=1U; daq_net.last_error=5U; return 1U;
    }
  }
  if (daq_net.link_up==0U) { return work; }
  if ((network_eth.Instance->DMASR & ETH_DMASR_FBES)!=0U)
  { daq_net.fault=1U; daq_net.last_error=6U; return 1U; }
  (void)HAL_ETH_ReleaseTxPacket(&network_eth);
  if (daq_net.fault!=0U) { return 1U; }
  for (i=0U; i<NET_TX_SLOTS; i++)
  {
    if ((tx_used[i]!=0U) && ((uint32_t)(now-tx_started[i])>DAQ_NET_TX_STALL_MS))
    { daq_net.tx_stall_count++; daq_net.fault=1U; daq_net.last_error=7U; return 1U; }
  }
  for (i=0U; (i<DAQ_NET_RX_BUDGET) && (daq_net.fault==0U); i++)
  {
    packet=NULL;
    if (HAL_ETH_ReadData(&network_eth,(void **)&packet)!=HAL_OK) { break; }
    __DMB(); /* OWN release observed before reading uncached packet bytes. */
    work=1U;
    if (packet!=NULL)
    {
      uint32_t errors=0U;
      (void)HAL_ETH_GetRxDataErrorCode(&network_eth,&errors);
      if ((errors!=0U) || (daq_net.fault!=0U))
      { daq_net.rx_dropped++; pbuf_free(packet); }
      else
      {
        daq_net.rx_packets++;
        if (interface->input(packet,interface)!=ERR_OK)
        { pbuf_free(packet); daq_net.rx_dropped++; }
      }
    }
  }
  /* Leave TBUS/RBUS to the HAL descriptor rearm paths. In particular,
   * Transmit_IT only issues DMATPDR when TBUS is still asserted; clearing
   * it here can strand an OWN descriptor while DMA remains suspended. */
  network_eth.Instance->DMASR=ETH_DMASR_NIS|ETH_DMASR_AIS|ETH_DMASR_RS|ETH_DMASR_TS;
  return work;
}

/* Main-only, before rearming UART or permitting finite offline acquisition. */
uint32_t DAQ_EthernetSuspend(struct netif *interface)
{
  uint32_t i, now=HAL_GetTick();
  if (daq_net.fault!=0U)
  {
    /* Fault-only exit. RM0385 pp1623/1624/1627/1628: ST/SR clear is
     * only a request; TPS/RPS must both report Stopped before returning 1.
     * Do not flush, reset descriptors, free/reuse slots or claim completion.
     * Network fault stays latched, so these pinned buffers cannot restart. */
    netif_set_link_down(interface); netif_set_down(interface);
    if ((network_eth.Instance->DMAOMR & (ETH_DMAOMR_ST|ETH_DMAOMR_SR))!=0U)
    {
      link_tick=now; /* Reuse PHY timestamp only after permanent fault. */
      network_eth.Instance->DMAOMR &= ~(ETH_DMAOMR_ST|ETH_DMAOMR_SR);
    }
    __DSB();
    if ((network_eth.Instance->DMAOMR & (ETH_DMAOMR_ST|ETH_DMAOMR_SR))!=0U)
    { return 0U; }
    if ((network_eth.Instance->DMASR & (ETH_DMASR_TPS|ETH_DMASR_RPS))!=0U)
    {
      return ((uint32_t)(now-link_tick)<=DAQ_NET_TX_STALL_MS) ? 2U : 0U;
    }
    daq_net.link_up=0U; return 1U;
  }
  (void)HAL_ETH_ReleaseTxPacket(&network_eth);
  for (i=0U; i<NET_TX_SLOTS; i++)
  {
    if (tx_used[i]!=0U)
    {
      if ((uint32_t)(now-tx_started[i])<=DAQ_NET_TX_STALL_MS) { return 2U; }
      daq_net.tx_stall_count++; daq_net.fault=1U; daq_net.last_error=7U;
      /* Next main poll enters the bounded fault-only quiesce above.
       * Returning pending does not claim UART is safe yet. */
      return 2U;
    }
  }
  netif_set_link_down(interface); netif_set_down(interface);
  if (network_eth.gState == HAL_ETH_STATE_STARTED)
  {
    daq_net.last_hal_status=(uint32_t)HAL_ETH_Stop(&network_eth);
    if (daq_net.last_hal_status!=(uint32_t)HAL_OK)
    { daq_net.fault=1U; return 0U; }
  }
  __DSB();
  if ((network_eth.Instance->DMAOMR & (ETH_DMAOMR_ST|ETH_DMAOMR_SR))!=0U)
  { daq_net.fault=1U; return 0U; }
  if ((network_eth.Instance->DMASR & (ETH_DMASR_TPS|ETH_DMASR_RPS))!=0U)
  { daq_net.fault=1U; return 0U; }
  daq_net.link_up=0U; return 1U;
}
