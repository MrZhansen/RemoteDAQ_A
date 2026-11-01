/* Actual parser, command dispatcher, status and UART/UDP frame services.
 * PWM HAL is modeled; test_square_wave_control tests the actual TIM3 APIs. */
#define TRANSPORT_TEST_MAIN transport_fixture_main
#include "test_transport.c"
int main(void)
{
  uint8_t command[9];
  DAQ_ControlCommand parsed;
  uint32_t i,stops,frames,seq,submits,frequency;
  const uint16_t targets[]={100U,1000U,5000U,10000U,1000U};
  const uint16_t invalid[]={0U,99U,10001U,65535U};
  reset_control(); drain();
  CHECK(square_wave_actual_hz==1000U && wave_stop_calls==0U);
  for(i=0U;i<4U;i++)
  {
    seq=ch1_uart_next_seq;
    make_control(command,7U,targets[i]); inject_bytes(command,9U);
    CHECK(square_wave_actual_hz==((i==0U)?1000U:targets[i-1U])); /* ISR buffers only. */
    drain();
    CHECK(square_wave_actual_hz==targets[i] && square_wave_running==1U);
    CHECK(wire_last[2]==3U && wire_last[8]==(uint8_t)targets[i]);
    CHECK(wire_last[9]==(uint8_t)(targets[i]>>8U));
    CHECK(ch1_uart_next_seq==seq && acquisition_active_gear==0U);
    CHECK(DAQ_CRC16_CCITT_FALSE(wire_last,10U)==(uint16_t)(wire_last[10]|((uint16_t)wire_last[11]<<8U)));
  }
  frequency=square_wave_actual_hz; stops=wave_stop_calls;
  for(i=0U;i<4U;i++)
  { make_control(command,7U,invalid[i]); inject_bytes(command,9U); drain(); }
  for(i=0U;i<4U;i++)
  {
    make_control(command,7U,5000U);
    if(i==0U)command[8]^=1U;
    if(i==1U)command[0]=0U;
    if(i==2U)command[2]=1U;
    if(i==3U)command[6]=1U;
    CHECK(DAQ_ParseControlFrame(command,9U,&parsed)!=DAQ_CONTROL_OK);
    inject_bytes(command,9U); drain();
  }
  CHECK(square_wave_actual_hz==frequency && wave_stop_calls==stops);
  make_control(command,7U,5000U);
  CHECK(DAQ_ParseControlFrame(command,8U,&parsed)!=DAQ_CONTROL_OK);
  frames=wire_calls; inject_bytes(command,4U); drain();
  CHECK(square_wave_actual_hz==frequency && wire_calls==frames);
  inject_bytes(command+4U,5U); drain(); CHECK(square_wave_actual_hz==5000U);
  daq_transport_request=1U; (void)DAQ_TransportPoll(); (void)DAQ_TransportPoll();
  CHECK(daq_transport_active==1U && uart6_rx_active==0U);
  for(i=0U;i<5U;i++)
  {
    Request(1U,i); (void)DAQ_TransportPoll(); Acquisition_GearPoll(); (void)DAQ_TransportPoll();
    CHECK(acquisition_active_gear==i); submits=gear_request_calls;
    stops=wave_stop_calls; seq=ch1_uart_next_seq;
    Request(7U,targets[i]); (void)DAQ_TransportPoll();
    CHECK(square_wave_actual_hz==targets[i] && square_wave_last_set_status==HAL_OK);
    CHECK(gear_request_calls==submits && acquisition_active_gear==i);
    CHECK(udp_last[2]==3U && udp_last[8]==(uint8_t)targets[i] && udp_last[9]==(uint8_t)(targets[i]>>8U));
    CHECK(ch1_uart_next_seq==seq && wave_stop_calls==stops+1U);
    Request(6U,0U); frames=udp_status; (void)DAQ_TransportPoll(); CHECK(udp_status==frames+1U);
    set_group(100U+i,123U); source.gear=i; (void)DAQ_TransportPoll();
    CHECK(udp_last[2]==1U && udp_last[5]==i && udp_last[6]==31U && udp_last[7]==50U);
    CHECK(ch1_uart_next_seq==(uint16_t)(seq+1U) && adc_running==1U && digital_sync.error==0U);
  }
  printf("PASS CMD07 UART/UDP/status/G0-G4 integration: %u checks (host simulation, no board)\n",checks);
  return 0;
}
