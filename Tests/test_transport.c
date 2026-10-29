/* Actual main-context transport manager + real legacy UART command path;
 * only network DMA submission and acquisition hardware are simulated. */
#define UART_CONTROL_MAIN transport_legacy_main
#include "test_uart6_control.c"
#include "offline_uart_under_test.inc"
#include "daq_network.h"
#include "daq_udp_trial.h"
volatile DAQ_UDPTrialDiagnostics udp_trial;
#include "transport_state.inc"
static uint32_t net_enable=0U, net_ready=1U, net_enable_result=1U;
static uint32_t net_send_result=1U, udp_calls, udp_status, udp_data, disable_pending;
static uint32_t pending_command, received_command, received_parameter;
static uint8_t udp_last[260];
uint32_t DAQ_NetSetEnabled(uint32_t enabled)
{ if(net_enable_result==0U) { return 0U; }
  if(enabled==0U && disable_pending) { disable_pending--; return 2U; }
  net_enable=enabled; return 1U; }
uint32_t DAQ_NetPoll(void) { CHECK(net_enable==1U); return 0U; }
uint32_t DAQ_NetReady(void) { return net_enable && net_ready; }
uint32_t DAQ_NetTakeControl(DAQ_ControlCommand *command)
{
  if (!pending_command) { return 0U; }
  command->command=(uint8_t)received_command; command->parameter=(uint16_t)received_parameter;
  pending_command=0U; return 1U;
}
uint32_t DAQ_NetSendFrame(const uint8_t *bytes,uint16_t length)
{
  uint16_t crc;
  CHECK(net_enable==1U && uart6_rx_active==0U && daq_transport_active==1U);
  CHECK(length==12U || length==260U);
  CHECK(bytes[0]==0xAAU && bytes[1]==0x55U && bytes[2]==((length==12U)?3U:1U));
  crc=DAQ_CRC16_CCITT_FALSE(bytes,length-2U);
  CHECK(crc==(uint16_t)(bytes[length-2U]|((uint16_t)bytes[length-1U]<<8U)));
  udp_calls++; memcpy(udp_last,bytes,length);
  if (net_send_result) { if(length==12U) { udp_status++; } else { udp_data++; } }
  return net_send_result;
}
#include "transport_under_test.inc"
static void Request(uint32_t cmd,uint32_t parameter)
{ pending_command=1U; received_command=cmd; received_parameter=parameter; }
#ifndef TRANSPORT_TEST_MAIN
#define TRANSPORT_TEST_MAIN main
#endif
int TRANSPORT_TEST_MAIN(void)
{
  uint32_t calls,seq,errors;
  reset_control();
  CHECK(daq_transport_active==0U && daq_transport_request==0U);
  CHECK(DAQ_TransportPoll()==0U && net_enable==0U && uart6_rx_active==1U);
  drain(); set_group(0U,123U); drain(); seq=ch1_uart_next_seq;
  daq_transport_request=2U; CHECK(DAQ_TransportPoll()==1U && daq_transport_active==0U);
  CHECK(daq_transport_request==0U && daq_transport_result==1U);
  net_enable_result=0U; daq_transport_request=1U;
  CHECK(DAQ_TransportPoll()==1U && daq_transport_active==0U && uart6_rx_active==1U);
  net_enable_result=1U; daq_transport_request=1U; calls=wire_calls;
  CHECK(DAQ_TransportPoll()==1U && daq_transport_active==1U && uart6_rx_active==0U);
  CHECK(ch1_uart_next_seq==seq && wire_calls==calls);
  (void)DAQ_TransportPoll(); CHECK(udp_status==1U && udp_last[3]==0U && udp_last[4]==31U);
  set_group(1U,321U); (void)DAQ_TransportPoll();
  CHECK(udp_data==1U && ch1_uart_next_seq==(uint16_t)(seq+1U));
  CHECK(udp_last[3]==(uint8_t)seq && udp_last[4]==(uint8_t)(seq>>8U));
  calls=udp_calls; (void)DAQ_TransportPoll(); CHECK(udp_calls==calls);
  net_send_result=0U; set_group(2U,555U); seq=ch1_uart_next_seq;
  errors=daq_udp_data_error_count; (void)DAQ_TransportPoll();
  CHECK(daq_udp_data_error_count==errors+1U && ch1_uart_next_seq==seq);
  calls=udp_calls; (void)DAQ_TransportPoll(); CHECK(udp_calls==calls);
  net_send_result=1U; ch1_uart_next_seq=65535U; set_group(3U,123U);
  (void)DAQ_TransportPoll(); CHECK(ch1_uart_next_seq==0U && daq_udp_last_seq==65535U);
  Request(2U,1U); (void)DAQ_TransportPoll();
  CHECK(daq_active_channel_mask==1U && udp_last[4]==1U && udp_last[2]==3U);
  set_group(4U,123U); (void)DAQ_TransportPoll();
  CHECK(udp_last[6]==1U && udp_last[10]==0U && udp_last[11]==0U && udp_last[12]==0U);
  Request(6U,0U); calls=udp_status; (void)DAQ_TransportPoll(); CHECK(udp_status==calls+1U);
  Request(1U,1U); calls=udp_status; (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==0U && acquisition_request_pending==1U && udp_status==calls);
  Acquisition_GearPoll(); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==1U && udp_last[3]==1U && udp_status==calls+1U);
  Request(1U,2U); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==1U && uart6_gear_unsupported_count>0U);
  Request(1U,0U); (void)DAQ_TransportPoll(); Acquisition_GearPoll(); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==0U);
  /* GEAR2 remains unavailable through CMD01, but an internal candidate
   * snapshot can be encoded and submitted only through UDP, with real state. */
  udp_trial.state=UDP_TRIAL_RUNNING; acquisition_active_gear=2U;
  set_group(50U,345U); source.gear=2U; (void)DAQ_TransportPoll();
  CHECK(udp_last[5]==2U && udp_last[6]==1U);
  Request(6U,0U); (void)DAQ_TransportPoll();
  CHECK(udp_last[2]==3U && udp_last[3]==2U);
  CHECK(UART6_GetStatus(&(DAQ_StatusState){0})==0U);
  udp_trial.state=UDP_TRIAL_DONE; acquisition_active_gear=0U; source.gear=0U;
  daq_transport_request=0U; seq=ch1_uart_next_seq;
  disable_pending=1U; calls=udp_calls;
  CHECK(DAQ_TransportPoll()==0U && daq_transport_active==1U && uart6_rx_active==0U);
  CHECK(daq_transport_request==0U && daq_transport_result==6U && udp_calls==calls);
  CHECK(DAQ_TransportPoll()==1U && daq_transport_active==0U && net_enable==0U);
  CHECK(uart6_rx_active==1U && ch1_uart_next_seq==seq);
  calls=wire_calls; drain(); CHECK(wire_calls==calls+1U);
  set_group(5U,987U); drain(); CHECK(ch1_uart_next_seq==(uint16_t)(seq+1U));
  CHECK(ch1_uart_last_seq==seq && ch1_uart_last_group==5U);
  CHECK(adc_error==0U && dma_error==0U && digital_sync.error==0U);
  printf("PASS selected UART/UDP transport integration: %u checks\n",checks);
  return 0;
}
