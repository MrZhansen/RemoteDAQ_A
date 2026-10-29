/* Actual main transport/command/status service, simulated network readiness.
 * 0x06 is V1.1 request handshake/status, not a separate wire protocol. */
#define TRANSPORT_TEST_MAIN transport_baseline_main
#include "test_transport.c"
int main(void)
{
  uint32_t seq,uart_calls,statuses,pending,i;
  reset_control();
  adc_snapshot_valid=0U;
  net_ready=0U; daq_transport_request=1U;
  CHECK(DAQ_TransportPoll()==1U && daq_transport_active==1U && uart6_rx_active==0U);
  seq=ch1_uart_next_seq; uart_calls=wire_calls;
  pending=uart6_status_pending;
  CHECK(pending>=1U); /* Transport-entry state notification. */
  Request(DAQ_CMD_REQUEST_STATUS,0U);
  (void)DAQ_TransportPoll();
  CHECK(uart6_status_pending==pending+1U && udp_status==0U && udp_data==0U);
  CHECK(acquisition_active_gear==0U && daq_active_channel_mask==31U);
  (void)DAQ_TransportPoll(); (void)DAQ_TransportPoll();
  CHECK(uart6_status_pending==pending+1U && ch1_uart_next_seq==seq && wire_calls==uart_calls);
  net_ready=1U;
  for(i=0U;i<16U && uart6_status_pending!=0U;i++) { (void)DAQ_TransportPoll(); }
  CHECK(uart6_status_pending==0U);
  CHECK(udp_status==pending+1U && udp_last[2]==3U && udp_last[3]==0U && udp_last[4]==31U);
  CHECK(udp_last[5]==0U && udp_last[6]==0U && udp_last[7]==0U);
  CHECK((uint16_t)(udp_last[8]|((uint16_t)udp_last[9]<<8U))==1000U);
  CHECK(ch1_uart_next_seq==seq && wire_calls==uart_calls && udp_data==0U);
  statuses=udp_status; (void)DAQ_TransportPoll(); CHECK(udp_status==statuses);
  Request(DAQ_CMD_REQUEST_STATUS,0U); (void)DAQ_TransportPoll();
  CHECK(udp_status==statuses+1U && ch1_uart_next_seq==seq);
  printf("PASS CMD06 handshake/status waits for ARP-ready without false ACK or SEQ: %u checks\n",checks);
  return 0;
}
