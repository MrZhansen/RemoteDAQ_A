/* Real LwIP/DAQ initialization error paths; fake Ethernet only. No socket. */
#define main network_baseline_main
#include "test_network.c"
#undef main
#include "lwip/udp.h"
static uint32_t failure_mode, new_calls, bind_calls, init_assertion;
extern const char * volatile daq_net_assert_message;
void __real_lwip_init(void);
void __wrap_lwip_init(void)
{
  __real_lwip_init();
  if (init_assertion != 0U) { DAQ_NetAssert("injected stack init assertion"); }
}
struct udp_pcb *__real_udp_new(void);
err_t __real_udp_bind(struct udp_pcb *,const ip_addr_t *,u16_t);
struct udp_pcb *__wrap_udp_new(void)
{
  new_calls++;
  return (failure_mode==1U)?NULL:__real_udp_new();
}
err_t __wrap_udp_bind(struct udp_pcb *pcb,const ip_addr_t *ip,u16_t port)
{
  bind_calls++;
  CHECK(port==5006U);
  if(failure_mode==2U) { return ERR_USE; }
  return __real_udp_bind(pcb,ip,port);
}
int main(int argc,char **argv)
{
  const char *mode=(argc>1)?argv[1]:"bind";
  struct udp_pcb *probe;
  uint8_t control[9],frame[12]; DAQ_ControlCommand result={99U,99U};
  DAQ_StatusState state={0U,31U,0U,0U,0U,1000U};
  if(strcmp(mode,"new")==0) { failure_mode=1U; }
  else if(strcmp(mode,"bind")==0) { failure_mode=2U; }
  else if(strcmp(mode,"init")==0) { init_failure=1U; }
  else if(strcmp(mode,"guard")==0) { init_failure=2U; }
  else if(strcmp(mode,"lwip_assert")==0) { init_assertion=1U; }
  if(failure_mode || init_failure || init_assertion)
  {
    CHECK(DAQ_NetSetEnabled(1U)==0U);
    CHECK(daq_net.fault==1U && daq_net.init_error_count==1U && daq_net.enabled==0U && daq_net.initialized==0U);
    if(failure_mode)
    {
      probe=__real_udp_new(); CHECK(probe!=NULL); /* Failed startup must not pin the only PCB. */
      udp_remove(probe);
      CHECK(netif_list==NULL && netif_default==NULL);
      CHECK(daq_net.last_error==((failure_mode==1U)?10U:11U));
    }
    else if(init_assertion)
    {
      CHECK(new_calls==0U && bind_calls==0U && init_calls==0U);
      CHECK(netif_list==NULL && netif_default==NULL);
      CHECK(daq_net.last_error==1U && daq_net.assertion_count==1U);
      CHECK(daq_net_assert_message!=NULL && strcmp(daq_net_assert_message,"injected stack init assertion")==0);
      DAQ_NetAssert("later assertion");
      CHECK(daq_net.assertion_count==2U && strcmp(daq_net_assert_message,"injected stack init assertion")==0);
    }
    else
    {
      CHECK(new_calls==0U && bind_calls==0U);
      CHECK(daq_net.last_error==((init_failure==2U)?9U:12U));
    }
    CHECK(DAQ_NetSetEnabled(1U)==0U && daq_net.init_count==1U);
    CHECK(DAQ_NetPoll()==0U && DAQ_NetTakeControl(&result)==0U);
    CHECK(output_calls==0U && DAQ_NetSetEnabled(0U)==1U);
  }
  else
  {
    CHECK(DAQ_NetSetEnabled(1U)==1U); (void)DAQ_NetPoll(); ArpReply(); (void)DAQ_NetPoll();
    CHECK(DAQ_NetReady()!=0U && new_calls==1U && bind_calls==1U);
    MakeControl(control,6U,0U); Datagram(control,9U,20U,5005U);
    if(strcmp(mode,"fault")==0)
    {
      uint32_t calls=output_calls;
      DAQ_NetAssert("injected after valid handshake queued");
      CHECK(DAQ_NetTakeControl(&result)==0U && result.command==99U);
      CHECK(DAQ_BuildStatusFrame(&state,frame,sizeof(frame))==DAQ_STATUS_OK);
      CHECK(DAQ_NetSendFrame(frame,12U)==0U && output_calls==calls);
      CHECK(DAQ_NetPoll()==0U && DAQ_NetSetEnabled(1U)==0U);
      CHECK(DAQ_NetSetEnabled(0U)==1U && daq_net.fault==1U);
      CHECK(DAQ_NetSetEnabled(1U)==0U); /* CMD06 is not a fault-reset command. */
    }
    else
    { CHECK(DAQ_NetTakeControl(&result)==1U && result.command==6U && result.parameter==0U); }
  }
  printf("PASS network init/handshake lifecycle %s: %lu checks\n",mode,(unsigned long)checks);
  return 0;
}
