/* Real Ethernet adapter, real official LAN8742 driver and LwIP. HAL/register
 * stubs validate ownership and error paths; they do not model physical DMA. */
#include "NetworkHAL/main.h"
#include "daq_network.h"
#include "daq_ethernet.h"
#include "lan8742.h"
#include "lwip/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t checks, tick, failure, gpio_mask, mpu_calls;
static uint32_t link_state=1U, init_calls, stop_calls, submitted, completed;
static uint32_t rx_error, release_fault_armed;
static uint8_t tx_capture[1536];
static uint16_t tx_length;
static uint32_t *pending[4];
static uint32_t pending_count;
TestSCB test_scb; TestDWT test_dwt; TestETH test_eth; TestMPU test_mpu;
TestDBGMCU test_dbgmcu={0x10010449U}; /* Host fixture only, never real silicon. */
/* Integration fixtures link the real network service instead. */
#ifndef DAQ_TEST_REAL_NETWORK_SERVICE
volatile DAQ_NetDiagnostics daq_net;
#endif
#define CHECK(c) do { checks++; if(!(c)) { fprintf(stderr,"FAIL Ethernet line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
#ifndef DAQ_TEST_REAL_NETWORK_SERVICE
void DAQ_NetAssert(const char *message) { (void)message; CHECK(0); }
u32_t sys_now(void) { return tick; }
#endif
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *region)
{
  mpu_calls++;
  CHECK(region->Enable==1U && region->Number==1U && region->BaseAddress==0x20018000U);
  CHECK(region->Size==8U && region->AccessPermission==3U && region->DisableExec==1U);
  CHECK(region->IsShareable==1U && region->IsCacheable==0U && region->IsBufferable==1U);
  CHECK(region->TypeExtField==0U && region->SubRegionDisable==0U);
  test_mpu.RBAR=region->BaseAddress; test_mpu.RASR=(failure==12U)?0U:0x13050011U;
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_Init(uint32_t port,GPIO_InitTypeDef *gpio)
{
  CHECK(gpio->Alternate==11U && gpio->Mode==GPIO_MODE_AF_PP && gpio->Pull==GPIO_NOPULL);
  if(port==GPIOA) { CHECK(gpio->Pin==(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7)); }
  else if(port==GPIOC) { CHECK(gpio->Pin==(GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5)); }
  else { CHECK(port==GPIOG && gpio->Pin==(GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14)); }
  gpio_mask|=1U<<port;
}
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *h)
{
  init_calls++; HAL_ETH_MspInit(h);
  CHECK(h->Instance==ETH && h->Init.MediaInterface==HAL_ETH_RMII_MODE);
  CHECK(h->Init.RxBuffLen==1536U && h->Init.RxDesc!=h->Init.TxDesc);
  /* Match real HAL ETH_DMATxDescListInit: default CIC is full offload. */
  { uint32_t i; for(i=0U;i<4U;i++) { h->Init.TxDesc[i].DESC0=ETH_DMATXDESC_CIC; } }
  h->gState=HAL_ETH_STATE_READY; return (failure==1U)?HAL_ERROR:HAL_OK;
}
void HAL_ETH_SetMDIOClockRange(ETH_HandleTypeDef *h) { CHECK(h->Instance==ETH); }
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *h,uint32_t address,uint32_t reg,uint32_t *value)
{
  CHECK(h->Instance==ETH);
  if(failure==2U) { return HAL_ERROR; }
  if(address!=0U) { *value=65535U; return HAL_OK; }
  switch(reg) {
  case LAN8742_SMR: *value=0U; break;
  case LAN8742_PHYI1R: *value=7U; break;
  case LAN8742_PHYI2R: *value=(failure==3U)?0U:0xC131U; break;
  case LAN8742_BSR: *value=link_state?LAN8742_BSR_LINK_STATUS:0U; break;
  case LAN8742_BCR: *value=LAN8742_BCR_SPEED_SELECT|LAN8742_BCR_DUPLEX_MODE; break;
  default: *value=0U; break;
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_ETH_WritePHYRegister(const ETH_HandleTypeDef *h,uint32_t a,uint32_t r,uint32_t v)
{ (void)h; (void)a; (void)r; (void)v; return HAL_OK; }
HAL_StatusTypeDef HAL_ETH_GetMACConfig(ETH_HandleTypeDef *h,ETH_MACConfigTypeDef *c)
{ (void)h; memset(c,0,sizeof(*c)); return (failure==4U)?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef HAL_ETH_SetMACConfig(ETH_HandleTypeDef *h,ETH_MACConfigTypeDef *c)
{ (void)h; CHECK(c->ChecksumOffload==0U); return (failure==5U)?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef HAL_ETH_Start(ETH_HandleTypeDef *h)
{
  h->gState=HAL_ETH_STATE_STARTED; h->Instance->DMAOMR=ETH_DMAOMR_ST|ETH_DMAOMR_SR;
  return (failure==6U)?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *h)
{
  stop_calls++;
  if(failure==7U) { return HAL_ERROR; }
  h->gState=HAL_ETH_STATE_READY; h->Instance->DMAOMR=0U; return HAL_OK;
}
HAL_StatusTypeDef HAL_ETH_ReleaseTxPacket(ETH_HandleTypeDef *h)
{
  (void)h;
  if(release_fault_armed!=0U) { release_fault_armed=0U; daq_net.fault=1U; }
  return HAL_OK;
} /* Test chooses when DMA ownership is released; callback may latch fault. */
HAL_StatusTypeDef HAL_ETH_Transmit_IT(ETH_HandleTypeDef *h,ETH_TxPacketConfig *config)
{
  CHECK(h->Instance==ETH && config->TxBuffer->next==NULL);
  uint32_t descriptor=submitted%4U;
  CHECK(config->Attributes==(ETH_TX_PACKETS_FEATURES_CRCPAD|ETH_TX_PACKETS_FEATURES_CSUM));
  CHECK(config->ChecksumCtrl==ETH_CHECKSUM_DISABLE);
  /* Match real ETH_Prepare_Tx_Descriptors conditional CIC update. */
  if ((config->Attributes & ETH_TX_PACKETS_FEATURES_CSUM)!=0U)
  { h->Init.TxDesc[descriptor].DESC0=(h->Init.TxDesc[descriptor].DESC0 & ~ETH_DMATXDESC_CIC)|config->ChecksumCtrl; }
  CHECK(config->CRCPadCtrl==ETH_CRC_PAD_INSERT && config->pData!=NULL);
  CHECK(config->Length==config->TxBuffer->len && config->Length<=1536U);
  if(failure==8U) { return HAL_BUSY; }
  CHECK(pending_count<4U); pending[pending_count++]=(uint32_t *)config->pData;
  tx_length=(uint16_t)config->Length; memcpy(tx_capture,config->TxBuffer->buffer,tx_length);
  h->Init.TxDesc[descriptor].DESC0 |= ETH_DMATXDESC_OWN;
  submitted++; return HAL_OK;
}
#ifndef DAQ_TEST_EXTERNAL_RX
HAL_StatusTypeDef HAL_ETH_ReadData(ETH_HandleTypeDef *h,void **packet)
{ (void)h; (void)packet; return HAL_ERROR; }
#endif
HAL_StatusTypeDef HAL_ETH_GetRxDataErrorCode(ETH_HandleTypeDef *h,uint32_t *error)
{ (void)h; *error=rx_error; return HAL_OK; }
#include "../Core/Src/daq_ethernet.c"
/* Every refusal exercises the production initializer, not a second guard.
 * Reserved IDCODE[15:12] is masked in parser tests; this does not assert
 * that nonzero reserved bits are a documented hardware device identity. */
static void TestIdentityGuards(void)
{
  static const struct { uint32_t raw; DAQ_NetIdentityStatus status; } cases[]={
    {0x10010449U,DAQ_NET_ID_ES0290_LISTED},
    {0x1001F449U,DAQ_NET_ID_ES0290_LISTED},
    {0x10000449U,DAQ_NET_ID_REV_A_UNQUALIFIED},
    {0x1000F449U,DAQ_NET_ID_REV_A_UNQUALIFIED},
    {0x00000449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x00010449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x0FFF0449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x10020449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x10030449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x10070449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x20000449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x20010449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0xFFFF0449U,DAQ_NET_ID_UNKNOWN_REVISION},
    {0x10010000U,DAQ_NET_ID_WRONG_DEVICE},
    {0x10010001U,DAQ_NET_ID_WRONG_DEVICE},
    {0x10010448U,DAQ_NET_ID_WRONG_DEVICE},
    {0x1001044AU,DAQ_NET_ID_WRONG_DEVICE},
    {0x10010451U,DAQ_NET_ID_WRONG_DEVICE},
    {0x10010FFFU,DAQ_NET_ID_WRONG_DEVICE},
    {0x10000451U,DAQ_NET_ID_WRONG_DEVICE},
    {0x00000000U,DAQ_NET_ID_WRONG_DEVICE},
    {0xFFFFFFFFU,DAQ_NET_ID_WRONG_DEVICE}
  };
  struct netif interface={0};
  uint32_t i;
  CHECK(daq_net.identity_status==DAQ_NET_ID_UNREAD);
  for(i=0U;i<sizeof(cases)/sizeof(cases[0]);i++)
  {
    CHECK(SiliconIdentityStatus(cases[i].raw)==cases[i].status);
    if(cases[i].status==DAQ_NET_ID_ES0290_LISTED) { continue; }
    test_dbgmcu.IDCODE=cases[i].raw;
    CHECK(DAQ_EthernetInit(&interface)==ERR_IF);
    CHECK(daq_net.device_idcode==cases[i].raw);
    CHECK(daq_net.device_id==(cases[i].raw & 0xFFFU));
    CHECK(daq_net.revision==(cases[i].raw >> 16U));
    CHECK(daq_net.identity_status==(uint32_t)cases[i].status);
    CHECK(daq_net.last_error==9U);
    CHECK(init_calls==0U && mpu_calls==0U && gpio_mask==0U);
  }
  /* Reject unknown revision even with D-cache on; cache remains a separate
   * guard for qualified silicon (scenario 9). No hardware init occurred. */
  test_scb.CCR=SCB_CCR_DC_Msk; test_dbgmcu.IDCODE=0x10020449U;
  CHECK(DAQ_EthernetInit(&interface)==ERR_IF);
  CHECK(daq_net.identity_status==DAQ_NET_ID_UNKNOWN_REVISION && daq_net.last_error==9U);
  CHECK(init_calls==0U && mpu_calls==0U && gpio_mask==0U);
  memset((void *)&daq_net,0,sizeof(daq_net));
  test_scb.CCR=0U; test_dbgmcu.IDCODE=0x10010449U;
  printf("PASS identity matrix: %lu identities; real MMIO not accessed\n",
         (unsigned long)(sizeof(cases)/sizeof(cases[0])));
}
static void Complete(void)
{
  uint32_t i;
  for(i=0U;i<pending_count;i++) { network_eth.Init.TxDesc[i].DESC0 &= ~ETH_DMATXDESC_OWN; network_eth.TxDescList.releaseIndex=i; HAL_ETH_TxFreeCallback(pending[i]); completed++; }
  pending_count=0U;
}
int main(int argc,char **argv)
{
  struct netif interface={0}; struct pbuf *p;
  uint8_t bytes[302],*buffers[9]; uint32_t i;
  if(argc>1) { failure=(uint32_t)strtoul(argv[1],NULL,10); }
  if(failure==0U) { TestIdentityGuards(); }
  if(failure==9U) { test_scb.CCR=SCB_CCR_DC_Msk; }
  if(failure==10U) { test_dbgmcu.IDCODE=0x10000449U; }
  lwip_init();
  if ((failure>=1U && failure<=5U) || failure==9U || failure==10U || failure==12U)
  {
    CHECK(DAQ_EthernetInit(&interface)==ERR_IF);
    if(failure==9U) {
      CHECK(init_calls==0U && mpu_calls==0U && gpio_mask==0U && daq_net.last_error==3U);
      CHECK(daq_net.identity_status==DAQ_NET_ID_ES0290_LISTED);
    }
    if(failure==10U) {
      CHECK(init_calls==0U && mpu_calls==0U && gpio_mask==0U && daq_net.last_error==9U);
      CHECK(daq_net.identity_status==DAQ_NET_ID_REV_A_UNQUALIFIED);
    }
    if(failure==12U) { CHECK(init_calls==0U && daq_net.last_error==8U); }
    printf("PASS Ethernet init failure %lu: %lu checks\n",(unsigned long)failure,(unsigned long)checks); return 0;
  }
  CHECK(DAQ_EthernetInit(&interface)==ERR_OK && gpio_mask==14U);
  CHECK(daq_net.identity_status==DAQ_NET_ID_ES0290_LISTED);
  CHECK(daq_net.device_idcode==0x10010449U && daq_net.device_id==0x449U);
  CHECK(daq_net.revision==0x1001U);
  CHECK(daq_net.phy_id==0x0007C131U && daq_net.phy_address==0U);
  (void)DAQ_EthernetPoll(&interface);
  if(failure==6U) { CHECK(daq_net.fault==1U); printf("PASS Ethernet start failure: %lu checks\n",(unsigned long)checks); return 0; }
  CHECK(daq_net.link_up==1U && test_eth.DMAIER==0U);
  memset(bytes,0x5A,sizeof(bytes));
  p=pbuf_alloc(PBUF_RAW,sizeof(bytes),PBUF_RAM); CHECK(p!=NULL);
  CHECK(pbuf_take(p,bytes,sizeof(bytes))==ERR_OK);
  if(failure==8U)
  { CHECK(LinkOutput(&interface,p)==ERR_MEM && tx_used[0]==0U); pbuf_free(p); printf("PASS Ethernet TX busy: %lu checks\n",(unsigned long)checks); return 0; }
  for(i=0U;i<4U;i++) { CHECK(LinkOutput(&interface,p)==ERR_OK); }
  CHECK(LinkOutput(&interface,p)==ERR_MEM && daq_net.tx_busy==1U);
  CHECK(tx_length==302U && memcmp(tx_capture,bytes,302U)==0);
  pbuf_free(p); CHECK(memcmp(ethernet_dma.tx_bytes[0],bytes,302U)==0);
  if(failure==13U)
  {
    link_state=0U; tick=250U; (void)DAQ_EthernetPoll(&interface);
    CHECK(daq_net.fault==1U && daq_net.last_error==5U && stop_calls==0U);
    CHECK(tx_used[0]==1U && (test_eth.DMAOMR & (ETH_DMAOMR_ST|ETH_DMAOMR_SR))!=0U);
    CHECK(DAQ_EthernetSuspend(&interface)==1U);
    CHECK((test_eth.DMAOMR & (ETH_DMAOMR_ST|ETH_DMAOMR_SR))==0U);
    CHECK(daq_net.tx_completed==0U && daq_net.fault==1U && stop_calls==0U);
    for(i=0U;i<4U;i++) { CHECK(tx_used[i]==1U && (network_eth.Init.TxDesc[i].DESC0 & ETH_DMATXDESC_OWN)!=0U); }
    CHECK(DAQ_EthernetSuspend(&interface)==1U && LinkOutput(&interface,NULL)==ERR_IF);
    printf("PASS fault-stopped link UART recovery/pinned TX: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(failure==14U)
  {
    for(i=0U;i<4U;i++) { CHECK((network_eth.Init.TxDesc[i].DESC0 & ETH_DMATXDESC_CIC)==0U); }
    CHECK(memcmp(ethernet_dma.tx_bytes[0],bytes,302U)==0);
    printf("PASS explicit software-checksum CIC bypass: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(failure>=15U && failure<=18U)
  {
    uint32_t began;
    daq_net.fault=1U; daq_net.last_error=6U;
    test_eth.DMASR=(failure==18U)?ETH_DMASR_RPS:ETH_DMASR_TPS;
    tick=(failure==17U)?UINT32_MAX-5U:100U; began=tick;
    CHECK(DAQ_EthernetSuspend(&interface)==2U && stop_calls==0U);
    CHECK((test_eth.DMAOMR & (ETH_DMAOMR_ST|ETH_DMAOMR_SR))==0U);
    CHECK(tx_used[0]==1U && daq_net.tx_completed==0U);
    if(failure==16U || failure==18U)
    {
      tick=began+1U; test_eth.DMASR=0U;
      CHECK(DAQ_EthernetSuspend(&interface)==1U);
      CHECK(daq_net.fault==1U && tx_used[0]==1U && stop_calls==0U);
    }
    else
    {
      tick=began+20U; CHECK(DAQ_EthernetSuspend(&interface)==2U);
      tick=began+21U; CHECK(DAQ_EthernetSuspend(&interface)==0U);
      CHECK(daq_net.fault==1U && tx_used[0]==1U && daq_net.last_error==6U);
    }
    printf("PASS fault quiesce boundary/wrap %lu: %lu checks\n",(unsigned long)failure,(unsigned long)checks); return 0;
  }
  if(failure==19U)
  {
    tick=20U; CHECK(DAQ_EthernetSuspend(&interface)==2U && daq_net.fault==0U);
    tick=21U; CHECK(DAQ_EthernetSuspend(&interface)==2U && daq_net.fault==1U);
    CHECK(daq_net.tx_stall_count==1U && daq_net.last_error==7U);
    CHECK(DAQ_EthernetSuspend(&interface)==1U && stop_calls==0U);
    CHECK(tx_used[0]==1U && daq_net.tx_completed==0U);
    printf("PASS graceful drain timeout to bounded fault quiesce: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(failure==11U)
  {
    tick=20U; (void)DAQ_EthernetPoll(&interface); CHECK(daq_net.fault==0U);
    tick=21U; (void)DAQ_EthernetPoll(&interface);
    CHECK(daq_net.fault==1U && daq_net.tx_stall_count==1U && daq_net.last_error==7U);
    printf("PASS Ethernet TX stall: %lu checks\n",(unsigned long)checks); return 0;
  }
  CHECK(DAQ_EthernetSuspend(&interface)==2U && stop_calls==0U);
  Complete(); CHECK(submitted==4U && completed==4U && daq_net.tx_completed==4U);
  for(i=0U;i<8U;i++) { HAL_ETH_RxAllocateCallback(&buffers[i]); CHECK(buffers[i]!=NULL); }
  HAL_ETH_RxAllocateCallback(&buffers[8]); CHECK(buffers[8]==NULL);
  for(i=0U;i<8U;i++) { RxFree(&rx_slots[i].custom.pbuf); }
  HAL_ETH_RxAllocateCallback(&buffers[0]); CHECK(buffers[0]!=NULL);
  { void *start=NULL,*end=NULL; HAL_ETH_RxLinkCallback(&start,&end,buffers[0],51U);
    CHECK(start==end && ((struct pbuf *)start)->len==51U);
    pbuf_free((struct pbuf *)start); CHECK(rx_slots[0].used==0U); }
  if(failure==7U) { CHECK(DAQ_EthernetSuspend(&interface)==0U && daq_net.fault==1U); printf("PASS Ethernet stop failure: %lu checks\n",(unsigned long)checks); return 0; }
  CHECK(DAQ_EthernetSuspend(&interface)==1U && stop_calls==1U && daq_net.link_up==0U);
  CHECK(DAQ_EthernetInit(&interface)==ERR_OK); (void)DAQ_EthernetPoll(&interface);
  link_state=0U; tick+=250U; (void)DAQ_EthernetPoll(&interface);
  CHECK(daq_net.fault==1U && daq_net.last_error==5U && daq_net.link_up==0U);
  printf("PASS Ethernet ownership/link lifecycle: %lu checks\n",(unsigned long)checks);
  return 0;
}
