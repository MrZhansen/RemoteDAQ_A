/* Actual LwIP IPv4/ARP/UDP + DAQ service, fake Ethernet boundary only.
 * No socket, ST-LINK or physical interface is opened. */
#include "daq_network.h"
#include "daq_network_config.h"
#include "daq_ethernet.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t checks, tick, cycles, fake_link=1U, output_calls, fail_output;
static uint32_t init_calls, suspend_calls, init_failure;
static uint8_t output[1600];
static uint16_t output_size;
static struct netif *active_interface;
#define CHECK(c) do { checks++; if (!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
uint32_t DAQ_NetClockMs(void) { return tick; }
uint32_t DAQ_NetClockCycles(void) { cycles+=17U; return cycles; }
static err_t Output(struct netif *interface, struct pbuf *packet)
{
  (void)interface;
  if (fail_output) { return ERR_MEM; }
  CHECK(packet->tot_len<=sizeof(output));
  output_size=pbuf_copy_partial(packet,output,packet->tot_len,0U);
  CHECK(output_size==packet->tot_len); output_calls++; return ERR_OK;
}
err_t DAQ_EthernetInit(struct netif *interface)
{
  static const uint8_t mac[6]=DAQ_NET_MAC;
  init_calls++; active_interface=interface;
  if(init_failure!=0U) { if(init_failure==2U) { daq_net.last_error=9U; } return ERR_IF; }
  interface->name[0]='t'; interface->name[1]='0'; interface->hwaddr_len=6U;
  memcpy(interface->hwaddr,mac,6U); interface->mtu=1500U;
  interface->flags=NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP;
  interface->output=etharp_output; interface->linkoutput=Output;
  return ERR_OK;
}
uint32_t DAQ_EthernetSuspend(struct netif *interface)
{
  suspend_calls++; netif_set_down(interface); netif_set_link_down(interface);
  daq_net.link_up=0U; return 1U;
}
uint32_t DAQ_EthernetPoll(struct netif *interface)
{
  if (fake_link) { netif_set_up(interface); netif_set_link_up(interface); daq_net.link_up=1U; }
  else { netif_set_link_down(interface); daq_net.link_up=0U; }
  return 0U;
}
static void Feed(const uint8_t *bytes, uint16_t length)
{
  struct pbuf *p=pbuf_alloc(PBUF_RAW,length,PBUF_POOL);
  CHECK(p!=NULL); CHECK(pbuf_take(p,bytes,length)==ERR_OK);
  CHECK(ethernet_input(p,active_interface)==ERR_OK);
}
static void ArpReply(void)
{
  uint8_t arp[42]={0};
  uint8_t a_mac[6]=DAQ_NET_MAC, b_mac[6]={0,0x80,0xE1,0,0,0x20};
  memcpy(arp,a_mac,6); memcpy(arp+6,b_mac,6); arp[12]=8; arp[13]=6;
  arp[15]=1; arp[16]=8; arp[18]=6; arp[19]=4; arp[21]=2;
  memcpy(arp+22,b_mac,6); arp[28]=192; arp[29]=168; arp[30]=1; arp[31]=20;
  memcpy(arp+32,a_mac,6); arp[38]=192; arp[39]=168; arp[40]=1; arp[41]=10;
  Feed(arp,sizeof(arp));
}
static void MakeControl(uint8_t *bytes, uint8_t cmd, uint16_t parameter)
{
  uint16_t crc;
  bytes[0]=0xAA; bytes[1]=0x55; bytes[2]=2; bytes[3]=cmd;
  bytes[4]=(uint8_t)parameter; bytes[5]=(uint8_t)(parameter>>8); bytes[6]=0;
  crc=DAQ_CRC16_CCITT_FALSE(bytes,7U); bytes[7]=(uint8_t)crc; bytes[8]=(uint8_t)(crc>>8);
}
static void Datagram(const uint8_t *payload, uint16_t size, uint8_t source_last, uint16_t source_port)
{
  uint8_t bytes[1600]={0};
  uint16_t length=(uint16_t)(28U+size), checksum;
  uint32_t sum=0U, i;
  uint8_t a_mac[6]=DAQ_NET_MAC;
  CHECK(size<1400U); memcpy(bytes,a_mac,6); bytes[6]=0; bytes[7]=0x80; bytes[8]=0xE1; bytes[11]=0x20;
  bytes[12]=8; bytes[13]=0;
  bytes[14]=0x45; bytes[16]=(uint8_t)(length>>8); bytes[17]=(uint8_t)length;
  bytes[22]=64; bytes[23]=17;
  bytes[26]=192; bytes[27]=168; bytes[28]=1; bytes[29]=source_last;
  bytes[30]=192; bytes[31]=168; bytes[32]=1; bytes[33]=10;
  for(i=14U;i<34U;i+=2U) { sum+=((uint32_t)bytes[i]<<8U)|bytes[i+1U]; }
  while(sum>>16U) { sum=(sum&65535U)+(sum>>16U); }
  checksum=(uint16_t)~sum; bytes[24]=(uint8_t)(checksum>>8); bytes[25]=(uint8_t)checksum;
  bytes[34]=(uint8_t)(source_port>>8); bytes[35]=(uint8_t)source_port;
  bytes[36]=(uint8_t)(DAQ_NET_LOCAL_PORT>>8); bytes[37]=(uint8_t)DAQ_NET_LOCAL_PORT;
  length=(uint16_t)(size+8U); bytes[38]=(uint8_t)(length>>8); bytes[39]=(uint8_t)length;
  memcpy(bytes+42,payload,size); Feed(bytes,(uint16_t)(42U+size));
}
static uint32_t SumWireWords(const uint8_t *bytes,uint32_t size)
{
  uint32_t sum=0U,i;
  for(i=0U;i+1U<size;i+=2U) { sum+=((uint32_t)bytes[i]<<8U)|bytes[i+1U]; }
  if((size & 1U)!=0U) { sum+=(uint32_t)bytes[size-1U]<<8U; }
  return sum;
}
static void CheckWireChecksums(void)
{
  uint32_t sum,udp_size,total;
  CHECK(output_size>=42U && output[14]==0x45U && output[23]==17U);
  total=((uint32_t)output[16]<<8U)|output[17];
  udp_size=((uint32_t)output[38]<<8U)|output[39];
  CHECK(total+14U==output_size && udp_size+20U==total);
  sum=SumWireWords(output+14U,20U);
  while((sum>>16U)!=0U) { sum=(sum & 65535U)+(sum>>16U); }
  CHECK(sum==65535U);
  CHECK(output[40]!=0U || output[41]!=0U); /* Software UDP checksum enabled. */
  sum=SumWireWords(output+26U,8U)+17U+udp_size;
  sum+=SumWireWords(output+34U,udp_size);
  while((sum>>16U)!=0U) { sum=(sum & 65535U)+(sum>>16U); }
  CHECK(sum==65535U);
}
int main(void)
{
  uint8_t command[10]={0}, frame[260], saved[260];
  uint16_t ch1[50],ch2[50]; uint8_t dig[50];
  DAQ_ControlCommand decoded={99U,99U};
  DAQ_StatusState state={0U,31U,0U,0U,0U,1000U};
  uint32_t i, count;
  CHECK(DAQ_NetReady()==0U && daq_net.enabled==0U);
  CHECK(DAQ_NetSetEnabled(2U)==0U && init_calls==0U);
  CHECK(DAQ_NetSetEnabled(0U)==1U && init_calls==0U);
  CHECK(DAQ_NetSetEnabled(1U)==1U && init_calls==1U);
  CHECK(DAQ_NetSetEnabled(1U)==1U && init_calls==1U);
  CHECK(DAQ_NetReady()==0U);
  (void)DAQ_NetPoll(); CHECK(output_calls>=1U && daq_net.peer_ready==0U);
  CHECK(output[12]==8U && output[13]==6U); /* ARP before application TX. */
  ArpReply(); (void)DAQ_NetPoll(); CHECK(DAQ_NetReady()!=0U);
  MakeControl(command,6U,0U); Datagram(command,9U,20U,40000U);
  CHECK(DAQ_NetTakeControl(&decoded)==1U && decoded.command==6U && decoded.parameter==0U);
  CHECK(DAQ_NetTakeControl(&decoded)==0U);
  Datagram(command,9U,20U,5005U); CHECK(DAQ_NetTakeControl(&decoded)==1U);
  Datagram(command,9U,21U,5005U); CHECK(daq_net.rx_wrong_peer==1U);
  Datagram(command,8U,20U,5005U); Datagram(command,10U,20U,5005U);
  CHECK(daq_net.rx_bad_length==2U);
  command[8]^=1U; Datagram(command,9U,20U,5005U); CHECK(daq_net.rx_crc_error==1U);
  MakeControl(command,2U,32U); Datagram(command,9U,20U,5005U);
  CHECK(daq_net.rx_format_error==1U && DAQ_NetTakeControl(&decoded)==0U);
  for(i=0U;i<32U;i++)
  {
    MakeControl(command,2U,(uint16_t)i); Datagram(command,9U,20U,5005U);
    CHECK(DAQ_NetTakeControl(&decoded)==1U && decoded.command==2U && decoded.parameter==i);
  }
  for(i=0U;i<11U;i++) { MakeControl(command,6U,0U); Datagram(command,9U,20U,5005U); }
  CHECK(daq_net.rx_queue_overflow==3U);
  for(i=0U;i<8U;i++) { CHECK(DAQ_NetTakeControl(&decoded)==1U); }
  CHECK(DAQ_NetTakeControl(&decoded)==0U);
  for(i=0U;i<50U;i++) { ch1[i]=(uint16_t)i; ch2[i]=(uint16_t)(4095U-i); dig[i]=(uint8_t)(i&7U); }
  CHECK(DAQ_BuildSelectedFrameForGear(ch1,ch2,dig,0U,31U,65535U,frame,sizeof(frame))==DAQ_FRAME_OK);
  memcpy(saved,frame,sizeof(saved)); CHECK(DAQ_NetSendFrame(frame,260U)==1U);
  CHECK(output_size==302U && output[36]==(DAQ_NET_PEER_PORT>>8) && output[37]==(DAQ_NET_PEER_PORT&255U));
  CHECK(output[34]==(DAQ_NET_LOCAL_PORT>>8) && output[35]==(DAQ_NET_LOCAL_PORT&255U));
  CHECK(memcmp(output+42,saved,260U)==0 && memcmp(frame,saved,260U)==0);
  CHECK(daq_net.data_submitted==1U); CheckWireChecksums();
  CHECK(DAQ_BuildStatusFrame(&state,frame,sizeof(frame))==DAQ_STATUS_OK);
  CHECK(DAQ_NetSendFrame(frame,12U)==1U && output_size==54U && daq_net.status_submitted==1U);
  CHECK(memcmp(output+42,frame,12U)==0); CheckWireChecksums();
  count=output_calls; CHECK(DAQ_NetSendFrame(frame,9U)==0U && output_calls==count);
  fail_output=1U; CHECK(DAQ_NetSendFrame(frame,12U)==0U && daq_net.status_submitted==1U);
  fail_output=0U; fake_link=0U; (void)DAQ_NetPoll(); CHECK(DAQ_NetReady()==0U);
  CHECK(DAQ_NetSendFrame(frame,12U)==0U);
  fake_link=1U; tick=UINT32_MAX-2U; cycles=UINT32_MAX-10U; (void)DAQ_NetPoll();
  tick=3U; (void)DAQ_NetPoll(); CHECK(daq_net.poll_count>=5U && daq_net.poll_max_cycles==17U);
  CHECK(DAQ_NetSetEnabled(0U)==1U && suspend_calls==1U && DAQ_NetReady()==0U);
  CHECK(DAQ_NetSetEnabled(1U)==1U && init_calls==2U);
  DAQ_NetAssert("injected"); CHECK(daq_net.fault==1U && daq_net.assertion_count==1U);
  CHECK(DAQ_NetPoll()==0U && DAQ_NetSetEnabled(1U)==0U);
  CHECK(DAQ_NetSetEnabled(0U)==1U);
  printf("PASS network real-LwIP/fake-Ethernet: %lu checks\n",(unsigned long)checks);
  return 0;
}
