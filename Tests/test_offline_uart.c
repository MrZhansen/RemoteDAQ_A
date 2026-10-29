/* Real RX pause/resume bridge with existing UART HAL stubs. */
#define UART_CONTROL_MAIN offline_uart_legacy_main
#include "test_uart6_control.c"
#include "offline_uart_under_test.inc"
int main(void)
{
  uint8_t request[9]; uint32_t seq,calls,parsed;
  reset_control(); drain(); seq=ch1_uart_next_seq; calls=wire_calls;
  make_control(request,6U,0U); inject_bytes(request,4U);
  CHECK(Offline_UARTPause()==HAL_OK && uart6_rx_active==0U && rx_dma.CR==0U);
  CHECK((irq_mask&24U)==0U && uart6_status_pending==0U && rx_uart.idle_enable==0U);
  CHECK(wire_calls==calls && ch1_uart_next_seq==seq);
  CHECK(Offline_UARTResume()==HAL_OK && uart6_rx_active==1U && rx_dma.NDTR==2048U);
  CHECK((irq_mask&24U)==24U && uart6_status_pending==0U);
  parsed=uart6_control_stream.frame_count;
  inject_bytes(request+4U,5U); drain();
  CHECK(uart6_control_stream.frame_count==parsed); /* Partial pre-pause frame discarded. */
  inject_bytes(request,9U); drain();
  CHECK(uart6_control_stream.frame_count==parsed+1U && wire_calls==calls+1U);
  CHECK(wire_lengths[wire_calls-1U]==12U);
  reset_control(); uart6_transmit_busy=1U;
  CHECK(Offline_UARTPause()==HAL_BUSY && uart6_rx_active==1U && rx_dma.CR==1U);
  uart6_transmit_busy=0U; abort_result=HAL_TIMEOUT;
  CHECK(Offline_UARTPause()==HAL_TIMEOUT && (irq_mask&24U)==0U);
  abort_result=HAL_OK;
  reset_control(); CHECK(Offline_UARTPause()==HAL_OK); arm_result=HAL_ERROR;
  CHECK(Offline_UARTResume()==HAL_ERROR && uart6_rx_active==0U && (irq_mask&24U)==0U);
  CHECK(rx_dma.CR==0U);
  printf("PASS offline UART isolation: %u checks; bounded abort/rearm, partial discard, no TX/SEQ pollution and CMD06 recovery; simulated only.\n",checks);
  return 0;
}
