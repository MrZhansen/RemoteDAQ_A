/* Real command/status/frame/transport functions; HAL acquisition is stubbed.
 * test_remote_gear2.c separately executes the actual register manager. */
#define TRANSPORT_TEST_MAIN transport_fixture_main
#include "test_transport.c"
int main(void)
{
  uint32_t count,seq,i;
  reset_control();
  UART6_DispatchControl(&(DAQ_ControlCommand){DAQ_CMD_SET_GEAR,2U});
  CHECK(acquisition_request_pending==0U && acquisition_active_gear==0U);
  CHECK(uart6_gear_last_result==UART6_GEAR_UNSUPPORTED);
  daq_transport_request=1U;
  CHECK(DAQ_TransportPoll()==1U && daq_transport_active==1U);
  (void)DAQ_TransportPoll();
  count=udp_status; Request(1U,2U); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==0U && acquisition_request_pending==1U);
  CHECK(uart6_gear_pending==1U && udp_status==count);
  Acquisition_GearPoll(); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==2U && acquisition_adc_active_cycles==144U);
  CHECK(uart6_gear_last_result==UART6_GEAR_OK && udp_last[2]==3U && udp_last[3]==2U);
  CHECK(udp_trial.state==UDP_TRIAL_IDLE);
  ch1_uart_next_seq=65535U;
  set_group(100U,123U); source.gear=2U; (void)DAQ_TransportPoll();
  CHECK(udp_last[5]==2U && udp_last[6]==31U && udp_last[7]==50U && ch1_uart_next_seq==0U);
  count=udp_data; (void)DAQ_TransportPoll(); CHECK(udp_data==count);
  net_send_result=0U; set_group(101U,234U); source.gear=2U;
  (void)DAQ_TransportPoll(); CHECK(ch1_uart_next_seq==0U);
  net_send_result=1U; set_group(102U,345U); source.gear=2U;
  (void)DAQ_TransportPoll(); CHECK(ch1_uart_next_seq==1U);
  Request(6U,0U); count=udp_status; (void)DAQ_TransportPoll();
  CHECK(udp_status==count+1U && udp_last[3]==2U);
  Request(2U,1U); (void)DAQ_TransportPoll();
  set_group(103U,345U); source.gear=2U; (void)DAQ_TransportPoll();
  CHECK(udp_last[6]==1U && udp_last[10]==0U && udp_last[11]==0U && udp_last[12]==0U);
  Request(2U,31U); (void)DAQ_TransportPoll();
  for(count=0U;(count<8U)&&(uart6_status_pending!=0U);count++) { (void)DAQ_TransportPoll(); }
  CHECK(uart6_status_pending==0U);
  for(i=3U;i<=4U;i++)
  {
    Request(1U,i); (void)DAQ_TransportPoll();
    CHECK(acquisition_request_pending==1U && acquisition_active_gear==i-1U);
    Acquisition_GearPoll(); (void)DAQ_TransportPoll();
    CHECK(acquisition_active_gear==i && acquisition_adc_active_cycles==
          ((i==3U)?28U:3U));
    CHECK(udp_last[2]==3U && udp_last[3]==i);
    set_group(200U+i,456U); source.gear=i; (void)DAQ_TransportPoll();
    CHECK(udp_last[2]==1U && udp_last[5]==i && udp_last[6]==31U && udp_last[7]==50U);
  }
  seq=ch1_uart_next_seq; Request(1U,1U); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==4U); Acquisition_GearPoll(); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==1U && acquisition_adc_active_cycles==480U && udp_last[3]==1U);
  CHECK(ch1_uart_next_seq==seq);
  Request(1U,2U); (void)DAQ_TransportPoll(); Acquisition_GearPoll(); (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==2U);
  daq_transport_request=0U; Acquisition_UDPPolicyPoll(); Acquisition_GearPoll();
  (void)DAQ_TransportPoll();
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==480U);
  CHECK(daq_transport_active==0U && uart6_rx_active==1U && ch1_uart_next_seq==seq);
  printf("PASS remote UDP CMD01 GEAR2..4/status/mask/SEQ integration: %u checks (host only)\n",checks);
  return 0;
}
