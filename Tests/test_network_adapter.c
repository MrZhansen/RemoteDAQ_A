/* Real DAQ network + real Ethernet adapter/LwIP, HAL/register fixtures only.
 * IDs below are host inputs; no production gate or device register is changed. */
#define DAQ_TEST_REAL_NETWORK_SERVICE
#define DAQ_TEST_EXTERNAL_RX
#define main adapter_fixture_main
#include "test_ethernet.c"
#undef main
#include "netif/ethernet.h"

static struct pbuf *rx_queue[4];
static uint32_t rx_queue_error[4], rx_read, rx_write, read_calls, corrupt_read;

HAL_StatusTypeDef HAL_ETH_ReadData(ETH_HandleTypeDef *h,void **packet)
{
  CHECK(h==&network_eth && packet!=NULL);
  read_calls++;
  if(rx_read==rx_write) { return HAL_ERROR; }
  *packet=rx_queue[rx_read%4U]; rx_error=rx_queue_error[rx_read%4U];
  rx_read++;
  if(corrupt_read!=0U)
  {
    struct pbuf *p=(struct pbuf *)*packet;
    void *start=p,*end=p;
    HAL_ETH_RxLinkCallback(&start,&end,(uint8_t *)p->payload,1U);
    *packet=start; corrupt_read=0U;
  }
  return HAL_OK;
}
static uint32_t LiveSlots(void)
{
  uint32_t i,n=0U;
  for(i=0U;i<NET_RX_SLOTS;i++) { n+=rx_slots[i].used; }
  return n;
}
static void Control(uint8_t *bytes,uint8_t cmd,uint16_t parameter)
{
  uint16_t crc;
  bytes[0]=0xAA; bytes[1]=0x55; bytes[2]=2U; bytes[3]=cmd;
  bytes[4]=(uint8_t)parameter; bytes[5]=(uint8_t)(parameter>>8U); bytes[6]=0U;
  crc=DAQ_CRC16_CCITT_FALSE(bytes,7U);
  bytes[7]=(uint8_t)crc; bytes[8]=(uint8_t)(crc>>8U);
}
static void Wire(uint8_t *wire,const uint8_t *control)
{
  uint32_t i,sum=0U;
  uint16_t checksum;
  static const uint8_t mac[6]=DAQ_NET_MAC;
  memset(wire,0,51U); memcpy(wire,mac,6U); wire[6]=0U; wire[7]=0x80U; wire[8]=0xE1U; wire[11]=0x20U;
  wire[12]=8U; wire[14]=0x45U; wire[17]=37U;
  wire[22]=64U; wire[23]=17U;
  wire[26]=192U; wire[27]=168U; wire[28]=1U; wire[29]=20U;
  wire[30]=192U; wire[31]=168U; wire[32]=1U; wire[33]=10U;
  for(i=14U;i<34U;i+=2U) { sum+=((uint32_t)wire[i]<<8U)|wire[i+1U]; }
  while(sum>>16U) { sum=(sum & 65535U)+(sum>>16U); }
  checksum=(uint16_t)~sum;
  wire[24]=(uint8_t)(checksum>>8U); wire[25]=(uint8_t)checksum;
  wire[34]=0x13U; wire[35]=0x8DU; /* B source 5005, not restricted by A. */
  wire[36]=(uint8_t)(DAQ_NET_LOCAL_PORT>>8U); wire[37]=(uint8_t)DAQ_NET_LOCAL_PORT;
  wire[39]=17U; /* IPv4 permits zero UDP checksum; application CRC is real. */
  memcpy(wire+42U,control,9U);
}
static struct pbuf *Packet(const uint8_t *wire,uint32_t split)
{
  uint8_t *one,*two;
  void *start=NULL,*end=(void *)(uintptr_t)1U;
  HAL_ETH_RxAllocateCallback(&one); CHECK(one!=NULL);
  if(split!=0U)
  {
    HAL_ETH_RxAllocateCallback(&two); CHECK(two!=NULL);
    memcpy(one,wire,46U); memcpy(two,wire+46U,5U);
    HAL_ETH_RxLinkCallback(&start,&end,one,46U);
    HAL_ETH_RxLinkCallback(&start,&end,two,5U);
    CHECK(start!=end && ((struct pbuf *)start)->tot_len==51U);
    CHECK(((struct pbuf *)start)->len==46U && ((struct pbuf *)start)->next->tot_len==5U);
  }
  else
  {
    memcpy(one,wire,51U);
    /* Real HAL leaves stale pRxEnd when pRxStart is reset. Ignore it for
     * an empty prefix; do not introduce a false fault on the next packet. */
    HAL_ETH_RxLinkCallback(&start,&end,one,51U);
    CHECK(start==end && ((struct pbuf *)start)->tot_len==51U);
  }
  CHECK(daq_net.fault==0U);
  return (struct pbuf *)start;
}
static void Queue(const uint8_t *wire,uint32_t error,uint32_t split)
{
  CHECK(rx_write-rx_read<4U);
  rx_queue[rx_write%4U]=Packet(wire,split);
  rx_queue_error[rx_write%4U]=error; rx_write++;
}
static void Poll(void) { (void)DAQ_NetPoll(); Complete(); }
static void Setup(void)
{
  static const uint8_t confirmed_b_mac[6]={0U,0x80U,0xE1U,0U,0U,0x20U};
  static const uint8_t expected_a_mac[6]={2U,0x46U,0xDAU,0U,0U,0x0AU};
  CHECK(memcmp(mac_address,expected_a_mac,6U)==0);
  CHECK(memcmp(mac_address,confirmed_b_mac,6U)!=0 && (mac_address[0]&3U)==2U);
  CHECK(DAQ_NetSetEnabled(1U)==1U);
  Poll();
  CHECK(daq_net.link_up==1U && daq_net.fault==0U && LiveSlots()==0U);
}
static err_t RejectInput(struct pbuf *packet,struct netif *interface)
{ (void)interface; CHECK(packet!=NULL); return ERR_IF; }

static err_t FaultInput(struct pbuf *packet,struct netif *interface)
{
  (void)interface; pbuf_free(packet); DAQ_NetAssert("injected input fault");
  return ERR_OK; /* Ownership consumed; caller must not free twice. */
}

int main(int argc,char **argv)
{
  const char *mode=(argc>1)?argv[1]:"rx";
  DAQ_ControlCommand decoded;
  uint8_t control[9],wire[51],*one,*two;
  uint32_t i,before,calls;
  void *start=NULL,*end=NULL;
  if(strcmp(mode,"gate")==0)
  {
    uint32_t raw=(argc>2)?(uint32_t)strtoul(argv[2],NULL,0):0x10000449U;
    uint32_t allowed=(raw==0x10010449U);
    test_dbgmcu.IDCODE=raw;
    CHECK(daq_net.identity_status==DAQ_NET_ID_UNREAD);
    CHECK(DAQ_NetSetEnabled(1U)==allowed);
    CHECK(daq_net.device_idcode==raw && daq_net.device_id==(raw & 0xFFFU));
    CHECK(daq_net.revision==(raw>>16U));
    if(!allowed)
    {
      CHECK(init_calls==0U && gpio_mask==0U && mpu_calls==0U);
      CHECK(daq_net.enabled==0U && daq_net.initialized==0U && daq_net.fault==1U);
      CHECK(daq_net.last_error==9U && DAQ_NetReady()==0U && DAQ_NetPoll()==0U);
      CHECK(DAQ_NetSetEnabled(1U)==0U && daq_net.init_count==1U);
      CHECK(DAQ_NetTakeControl(&decoded)==0U && submitted==0U);
    }
    else { CHECK(init_calls==1U && daq_net.enabled==1U && daq_net.identity_status==DAQ_NET_ID_ES0290_LISTED); }
    printf("PASS real service identity gate %08lX: %lu checks; simulated MMIO only\n",
           (unsigned long)raw,(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"unowned")==0)
  {
    HAL_ETH_RxLinkCallback(&start,&end,ethernet_dma.rx_bytes[0],1U);
    CHECK(daq_net.fault==1U && start==NULL && end==NULL);
    printf("PASS RX unowned slot refusal: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"duplicate")==0 || strcmp(mode,"cycle")==0)
  {
    HAL_ETH_RxAllocateCallback(&one); CHECK(one!=NULL);
    struct pbuf *saved;
    HAL_ETH_RxLinkCallback(&start,&end,one,2U); CHECK(start==end); saved=(struct pbuf *)start;
    if(strcmp(mode,"cycle")==0)
    {
      ((struct pbuf *)start)->next=(struct pbuf *)start;
      HAL_ETH_RxAllocateCallback(&two); CHECK(two!=NULL);
      HAL_ETH_RxLinkCallback(&start,&end,two,1U);
      CHECK(daq_net.fault==1U && start==NULL && end==NULL && saved->len==2U);
      saved->next=NULL; /* Host-only cleanup after injected corruption. */
      pbuf_free(&rx_slots[(uint32_t)(two-&ethernet_dma.rx_bytes[0][0])/NET_BUFFER_BYTES].custom.pbuf);
    }
    else
    {
      HAL_ETH_RxLinkCallback(&start,&end,one,1U);
      CHECK(daq_net.fault==1U && start==NULL && end==NULL && saved->next==NULL);
      CHECK(saved->len==2U && saved->tot_len==2U);
    }
    pbuf_free(saved); CHECK(LiveSlots()==0U);
    printf("PASS RX bounded %s rejection: %lu checks\n",mode,(unsigned long)checks); return 0;
  }
  Setup();
  Control(control,DAQ_CMD_REQUEST_STATUS,0U); Wire(wire,control);
  if(strcmp(mode,"padding")==0)
  {
    uint8_t padded[64];
    for(i=60U;i<=64U;i+=4U)
    {
      memset(padded,0xA5,sizeof(padded)); memcpy(padded,wire,51U);
      HAL_ETH_RxAllocateCallback(&one); CHECK(one!=NULL); memcpy(one,padded,i);
      start=NULL; end=NULL; HAL_ETH_RxLinkCallback(&start,&end,one,(uint16_t)i);
      CHECK(start!=NULL && ((struct pbuf *)start)->tot_len==i);
      rx_queue[rx_write%4U]=(struct pbuf *)start; rx_queue_error[rx_write%4U]=0U; rx_write++;
      before=daq_net.rx_control_count; Poll();
      CHECK(daq_net.rx_control_count==before+1U && LiveSlots()==0U);
      CHECK(DAQ_NetTakeControl(&decoded)==1U && decoded.command==6U && decoded.parameter==0U);
    }
    printf("PASS Ethernet padding/tail trimmed before exact 9-byte control parse: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"send_frames")==0)
  {
    uint8_t arp[51]={0},frame[260],digital[50];
    uint16_t ch1[50],ch2[50];
    DAQ_StatusState state={2U,31U,0U,0U,0U,1000U};
    static const uint8_t a_mac[6]=DAQ_NET_MAC;
    memcpy(arp,a_mac,6U); arp[6]=0U; arp[7]=0x80U; arp[8]=0xE1U; arp[11]=0x20U; arp[12]=8U; arp[13]=6U;
    arp[15]=1U; arp[16]=8U; arp[18]=6U; arp[19]=4U; arp[21]=2U;
    arp[22]=0U; arp[23]=0x80U; arp[24]=0xE1U; arp[27]=0x20U; arp[28]=192U; arp[29]=168U; arp[30]=1U; arp[31]=20U;
    memcpy(arp+32U,a_mac,6U); arp[38]=192U; arp[39]=168U; arp[40]=1U; arp[41]=10U;
    Queue(arp,0U,0U); Poll(); CHECK(DAQ_NetReady()!=0U && LiveSlots()==0U);
    for(i=0U;i<50U;i++) { ch1[i]=(uint16_t)i;ch2[i]=(uint16_t)(4095U-i);digital[i]=(uint8_t)(i&7U); }
    CHECK(DAQ_BuildUDPGEAR2CandidateFrame(ch1,ch2,digital,31U,65535U,frame,sizeof(frame))==DAQ_FRAME_OK);
    CHECK(DAQ_NetSendFrame(frame,260U)==1U && tx_length==302U);
    CHECK(memcmp(tx_capture+42U,frame,260U)==0 && tx_capture[47]==2U && tx_capture[48]==31U);
    CHECK(tx_capture[34]==0x13U && tx_capture[35]==0x8EU && tx_capture[36]==0x13U && tx_capture[37]==0x8DU);
    CHECK(tx_capture[0]==0U && tx_capture[1]==0x80U && tx_capture[2]==0xE1U && tx_capture[5]==0x20U && memcmp(tx_capture+6U,a_mac,6U)==0);
    CHECK(daq_net.data_submitted==1U && daq_net.tx_completed==1U); /* ARP completed; data merely submitted. */
    Complete(); CHECK(daq_net.tx_completed==2U);
    CHECK(DAQ_BuildStatusFrame(&state,frame,sizeof(frame))==DAQ_STATUS_OK);
    CHECK(DAQ_NetSendFrame(frame,12U)==1U && tx_length==54U);
    CHECK(memcmp(tx_capture+42U,frame,12U)==0 && daq_net.status_submitted==1U); Complete();
    CHECK(DAQ_BuildUDPGEAR2CandidateFrame(ch1,ch2,digital,31U,0U,frame,sizeof(frame))==DAQ_FRAME_OK);
    before=submitted; release_fault_armed=1U;
    CHECK(DAQ_NetSendFrame(frame,260U)==0U && submitted==before);
    CHECK(daq_net.data_submitted==1U && daq_net.tx_error==1U && daq_net.fault==1U);
    CHECK(DAQ_NetSetEnabled(0U)==1U);
    printf("PASS real LwIP/adapter 260/12-byte send/completion/fault: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"tx_release_fault")==0)
  {
    struct pbuf *packet=pbuf_alloc(PBUF_RAW,42U,PBUF_RAM);
    CHECK(packet!=NULL); before=submitted; release_fault_armed=1U;
    CHECK(LinkOutput(NULL,packet)==ERR_IF);
    CHECK(daq_net.fault==1U && submitted==before && tx_used[0]==0U);
    pbuf_free(packet);
    printf("PASS no TX after completion callback fault: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"poll_release_fault")==0)
  {
    Queue(wire,0U,0U); calls=read_calls; release_fault_armed=1U; Poll();
    CHECK(daq_net.fault==1U && read_calls==calls && LiveSlots()==1U);
    CHECK(daq_net.rx_control_count==0U && DAQ_NetSetEnabled(0U)==1U);
    printf("PASS no RX after completion callback fault: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"rx_input_fault")==0)
  {
    struct netif rejected={0}; rejected.input=FaultInput;
    Queue(wire,0U,0U); Queue(wire,0U,0U); calls=read_calls;
    (void)DAQ_EthernetPoll(&rejected);
    CHECK(daq_net.fault==1U && read_calls==calls+1U && LiveSlots()==1U);
    CHECK(DAQ_NetSetEnabled(0U)==1U);
    printf("PASS bounded RX stops after consumed-input fault: %lu checks\n",(unsigned long)checks); return 0;
  }
  if(strcmp(mode,"rx_fault")==0)
  {
    Queue(wire,0U,0U); corrupt_read=1U; Poll();
    CHECK(daq_net.fault==1U && LiveSlots()==1U && daq_net.rx_control_count==0U);
    CHECK(DAQ_NetReady()==0U && DAQ_NetTakeControl(&decoded)==0U);
    HAL_ETH_RxAllocateCallback(&one); CHECK(one==NULL);
    CHECK(DAQ_NetSetEnabled(0U)==1U && LiveSlots()==1U);
    CHECK(DAQ_NetSetEnabled(1U)==0U);
    printf("PASS RX corrupt prefix isolated from stack/free/rearm: %lu checks\n",(unsigned long)checks); return 0;
  }
  before=daq_net.rx_control_count;
  Queue(wire,0U,0U); Queue(wire,0U,1U); Queue(wire,0U,0U);
  CHECK(LiveSlots()==4U); calls=read_calls; Poll();
  CHECK(read_calls==calls+2U && rx_read==2U && LiveSlots()==1U);
  CHECK(daq_net.rx_control_count==before+2U);
  CHECK(DAQ_NetTakeControl(&decoded)==1U && decoded.command==6U);
  CHECK(DAQ_NetTakeControl(&decoded)==1U && DAQ_NetTakeControl(&decoded)==0U);
  Poll(); CHECK(LiveSlots()==0U && DAQ_NetTakeControl(&decoded)==1U);
  before=daq_net.rx_dropped; Queue(wire,1U,0U); Poll();
  CHECK(daq_net.rx_dropped==before+1U && LiveSlots()==0U && DAQ_NetTakeControl(&decoded)==0U);
  before=daq_net.rx_crc_error; control[8]^=1U; Wire(wire,control); Queue(wire,0U,0U); Poll();
  CHECK(daq_net.rx_crc_error==before+1U && LiveSlots()==0U);
  before=daq_net.rx_format_error; Control(control,DAQ_CMD_SET_CHANNEL_MASK,32U);
  Wire(wire,control); Queue(wire,0U,0U); Poll();
  CHECK(daq_net.rx_format_error==before+1U && DAQ_NetTakeControl(&decoded)==0U && LiveSlots()==0U);
  Control(control,DAQ_CMD_REQUEST_STATUS,0U); Wire(wire,control);
  before=daq_net.rx_control_count; wire[40]=0x12U; wire[41]=0x34U;
  Queue(wire,0U,0U); Poll();
  CHECK(daq_net.rx_control_count==before && LiveSlots()==0U);
  Wire(wire,control); before=daq_net.rx_dropped;
  /* DAQ service netif is private; the next test injects a rejected input via
   * direct adapter interface, without changing the service's actual netif. */
  {
    struct netif rejected={0};
    rejected.input=RejectInput;
    Queue(wire,0U,0U);
    (void)DAQ_EthernetPoll(&rejected);
    CHECK(daq_net.rx_dropped==before+1U && LiveSlots()==0U);
  }
  before=daq_net.rx_control_count;
  for(i=0U;i<1000U;i++)
  {
    Control(control,(i & 1U)?DAQ_CMD_SET_CHANNEL_MASK:DAQ_CMD_REQUEST_STATUS,(uint16_t)((i & 1U)?(i & 31U):0U));
    Wire(wire,control); Queue(wire,0U,i & 1U); Poll();
    CHECK(DAQ_NetTakeControl(&decoded)==1U);
    CHECK(decoded.command==control[3] && decoded.parameter==(uint16_t)(control[4]|((uint16_t)control[5]<<8U)));
    CHECK(LiveSlots()==0U && daq_net.fault==0U);
  }
  CHECK(daq_net.rx_control_count==before+1000U);
  CHECK(DAQ_NetSetEnabled(0U)==1U && DAQ_NetReady()==0U && LiveSlots()==0U);
  printf("PASS real service/adapter RX ownership: %lu checks; 1000 scripted packets, not throughput\n",(unsigned long)checks);
  return 0;
}
