/* Real firmware RX DMA/service functions with HAL/register stubs.
 * No serial device is opened. IRQ timing and wire behavior are not measured. */
#include "daq_control_stream.h"
typedef struct { uint32_t NDTR, CR; } FakeRxDma;
typedef struct {
  uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment;
  uint32_t MemDataAlignment, Mode, Priority, FIFOMode;
} FakeRxInit;
typedef struct { FakeRxDma *Instance; FakeRxInit Init; void *Parent; } DMA_HandleTypeDef;
typedef struct { uint32_t idle, idle_enable; } FakeUartRegs;
#define UART_CONTROL_TEST
#define main uart_legacy_main
#include "test_ch1_uart.c"
#undef main
#include "uart6_control_defines.inc"
#include "gear_defines.inc"

#define DMA_SxCR_EN 1U
#define DMA_CHANNEL_5 5U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_PINC_DISABLE 0U
#define DMA_MINC_ENABLE 1U
#define DMA_PDATAALIGN_BYTE 0U
#define DMA_MDATAALIGN_BYTE 0U
#define DMA_CIRCULAR 1U
#define DMA_PRIORITY_LOW 0U
#define DMA_FIFOMODE_DISABLE 0U
#define DMA2_Stream1_IRQn 3U
#define USART6_IRQn 4U
#define DMA_FLAG_TCIF1_5 1U
#define UART_FLAG_IDLE 1U
#define UART_IT_IDLE 1U
#define RESET 0U
#define __HAL_RCC_DMA2_CLK_ENABLE() ((void)0)
#define __HAL_LINKDMA(u,member,d) do { (u)->member=&(d); (d).Parent=(u); } while(0)
#define __HAL_UART_GET_FLAG(u,f) ((u)->Instance->idle)
#define __HAL_UART_GET_IT_SOURCE(u,f) ((u)->Instance->idle_enable)
#define __HAL_UART_CLEAR_IDLEFLAG(u) ((u)->Instance->idle=0U)
#define __HAL_UART_ENABLE_IT(u,f) ((u)->Instance->idle_enable=1U)
#define __HAL_UART_DISABLE_IT(u,f) ((u)->Instance->idle_enable=0U)
#define __HAL_DMA_GET_COUNTER(h) ((h)->Instance->NDTR)
#define __HAL_DMA_GET_FLAG(h,f) (rx_tc_pending)
static FakeRxDma rx_dma;
static FakeUartRegs rx_uart;
#define DMA2_Stream1 (&rx_dma)
static uint32_t rx_tc_pending, irq_mask, init_calls, arm_calls, abort_calls;
static uint32_t hw_error, wire_calls, wire_bytes, nesting, inject_on_data;
static uint32_t copy_overrun_countdown, copy_error_countdown;
static uint32_t inject_mask_on_data=UINT32_MAX;
static uint32_t wire_lengths[2048], wire_types[2048];
static uint8_t wire_last[260];
static uint8_t *dma_target;
static uint32_t dma_length, dma_index;
static HAL_StatusTypeDef dma_init_result=HAL_OK, arm_result=HAL_OK;
static HAL_StatusTypeDef abort_result=HAL_OK, dma_abort_result=HAL_OK;
static HAL_StatusTypeDef next_tx_result=HAL_OK;
static uint32_t acquisition_active_gear=0U, acquisition_switching;
static uint32_t acquisition_fault_latched, adc_running=1U;
static uint32_t acquisition_requested_gear, acquisition_request_pending;
static uint32_t acquisition_last_switch_result, adc_error, dma_error;
static uint32_t acquisition_udp_gear2_available, acquisition_udp_gear2_active;
static uint32_t acquisition_adc_active_cycles=480U;
static uint32_t gear_apply_failure, gear_request_calls;
static HAL_StatusTypeDef gear_request_status=HAL_OK;
static HAL_StatusTypeDef Acquisition_RequestGear(uint32_t gear)
{
  gear_request_calls++;
  if (gear_request_status!=HAL_OK) { return gear_request_status; }
  acquisition_requested_gear=gear; acquisition_request_pending=1U;
  return HAL_OK;
}
/* Main-loop hardware manager stand-in; test_cmd01.c uses the real manager. */
static void Acquisition_GearPoll(void)
{
  if (acquisition_request_pending==0U) { return; }
  acquisition_request_pending=0U;
  acquisition_last_switch_result=gear_apply_failure;
  if (gear_apply_failure!=0U)
  { adc_running=0U; acquisition_fault_latched=1U; return; }
  acquisition_active_gear=acquisition_requested_gear;
  acquisition_udp_gear2_active=(acquisition_active_gear==2U)?1U:0U;
  acquisition_adc_active_cycles=(acquisition_active_gear==2U)?144U:480U;
  adc_snapshot_valid=0U;
}
static struct { uint32_t error; } digital_sync;
static uint32_t square_wave_running=1U, square_wave_actual_hz=1000U;
static void inject_bytes(const uint8_t *bytes, uint32_t count);
static HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *dma_handle)
{
  init_calls++;
  CHECK(dma_handle->Instance==DMA2_Stream1);
  CHECK(dma_handle->Init.Channel==5U && dma_handle->Init.Mode==DMA_CIRCULAR);
  CHECK(dma_handle->Init.Priority==DMA_PRIORITY_LOW);
  CHECK(dma_handle->Init.PeriphDataAlignment==DMA_PDATAALIGN_BYTE);
  CHECK(dma_handle->Init.MemDataAlignment==DMA_MDATAALIGN_BYTE);
  CHECK(dma_handle->Init.MemInc==DMA_MINC_ENABLE);
  return dma_init_result;
}
static void HAL_NVIC_SetPriority(uint32_t n,uint32_t p,uint32_t sub)
{ CHECK((n==3U || n==4U) && p==2U && sub==0U); }
static void HAL_NVIC_ClearPendingIRQ(uint32_t n) { CHECK(n==3U || n==4U); rx_tc_pending=0U; }
static void HAL_NVIC_EnableIRQ(uint32_t n) { irq_mask |= 1U<<n; }
static void HAL_NVIC_DisableIRQ(uint32_t n) { irq_mask &= ~(1U<<n); }
static HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u,uint8_t *b,uint16_t n)
{
  CHECK(u==&huart6 && n==2048U && u->hdmarx!=NULL);
  arm_calls++; dma_target=b; dma_length=n; dma_index=0U;
  rx_dma.NDTR=n; rx_dma.CR=(arm_result==HAL_OK)?1U:0U;
  return arm_result;
}
static HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{
  CHECK(u==&huart6 && (irq_mask & 24U)==0U);
  abort_calls++; rx_dma.CR=0U; u->error=0U;
  return abort_result;
}
static HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *d)
{ CHECK(d==huart6.hdmarx); d->Instance->CR=0U; return dma_abort_result; }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u);
static void HAL_UART_IRQHandler(UART_HandleTypeDef *u)
{
  if(hw_error!=0U) { u->error=hw_error; hw_error=0U; HAL_UART_ErrorCallback(u); }
}
#include "uart6_control_state.inc"
#include "uart6_control_under_test.inc"

static void control_barrier(void)
{
  uint8_t noise[3000];
  if(copy_overrun_countdown!=0U && --copy_overrun_countdown==0U)
  {
    memset(noise,0x33,sizeof(noise)); inject_bytes(noise,sizeof(noise));
  }
  if(copy_error_countdown!=0U && --copy_error_countdown==0U)
  {
    hw_error=8U; UART6_ReceiveIRQ();
  }
}
static void make_control(uint8_t *b,uint8_t cmd,uint16_t param)
{
  uint16_t crc;
  b[0]=0xAAU; b[1]=0x55U; b[2]=2U; b[3]=cmd;
  b[4]=(uint8_t)param; b[5]=(uint8_t)(param>>8U); b[6]=0U;
  crc=DAQ_CRC16_CCITT_FALSE(b,7U); b[7]=(uint8_t)crc; b[8]=(uint8_t)(crc>>8U);
}
static void inject_bytes(const uint8_t *b,uint32_t n)
{
  uint32_t i;
  CHECK(uart6_rx_active!=0U && dma_target!=NULL);
  for(i=0U;i<n;i++)
  {
    dma_target[dma_index++]=b[i];
    if(dma_index==dma_length/2U) { HAL_UART_RxHalfCpltCallback(&huart6); }
    if(dma_index==dma_length)
    { dma_index=0U; HAL_UART_RxCpltCallback(&huart6); }
    rx_dma.NDTR=dma_length-dma_index;
  }
  rx_uart.idle=1U; UART6_ReceiveIRQ();
}
static HAL_StatusTypeDef control_transmit(UART_HandleTypeDef *u,
    const uint8_t *b,uint16_t n,uint32_t timeout)
{
  uint8_t saved[260], request[9];
  uint16_t crc;
  CHECK(u==&huart6 && nesting==0U && uart6_transmit_busy==1U);
  CHECK((n==260U && timeout==expected_timeout) || (n==12U && timeout==2U));
  CHECK(b[0]==0xAAU && b[1]==0x55U);
  CHECK(b[2]==((n==12U)?3U:1U));
  crc=DAQ_CRC16_CCITT_FALSE(b,n-2U);
  CHECK(crc==(uint16_t)(b[n-2U]|((uint16_t)b[n-1U]<<8U)));
  memcpy(saved,b,n); memcpy(wire_last,b,n);
  if(n==260U) { memcpy(captured_frame,b,n); }
  CHECK(wire_calls<2048U); wire_lengths[wire_calls]=n; wire_types[wire_calls]=b[2];
  wire_calls++; wire_bytes+=n; nesting++;
  /* A receive IRQ can run while TX polls. It never transmits or parses. */
  if(n==260U && inject_on_data!=0U)
  {
    uint32_t calls=wire_calls;
    make_control(request,6U,0U); inject_bytes(request,9U); inject_on_data=0U;
    CHECK(UART6_ServicePoll()==0U && wire_calls==calls);
    CHECK(UART6_TransmitFrame(request,9U,1U)==HAL_BUSY && wire_calls==calls);
  }
  if(n==260U && inject_mask_on_data<=0x1FU)
  {
    uint32_t old_mask=daq_active_channel_mask;
    make_control(request,2U,(uint16_t)inject_mask_on_data);
    inject_mask_on_data=UINT32_MAX; inject_bytes(request,9U);
    CHECK(daq_active_channel_mask==old_mask);
    CHECK(UART6_ServicePoll()==0U && daq_active_channel_mask==old_mask);
  }
  CHECK(memcmp(saved,b,n)==0); nesting--;
  fake_tick+=(next_tx_result==HAL_TIMEOUT)?timeout:((n==260U)?3U:1U);
  return next_tx_result;
}
static void drain(void)
{
  uint32_t i;
  for(i=0U;i<20U;i++) { Acquisition_GearPoll(); if(UART6_ServicePoll()==0U) { return; } }
  CHECK(0); /* Bounded fixture, not a production loop. */
}
static void reset_control(void)
{
  acquisition_udp_gear2_available=0U; acquisition_udp_gear2_active=0U;
  acquisition_adc_active_cycles=480U;
  memset(&huart6,0,sizeof(huart6)); huart6.Instance=&rx_uart;
  uart6_control_started=0U; uart6_status_pending=0U; uart6_rx_recovery_pending=0U;
  uart6_initialized=1U; dma_init_result=HAL_OK; arm_result=HAL_OK;
  abort_result=HAL_OK; dma_abort_result=HAL_OK; next_tx_result=HAL_OK;
  adc_running=1U; acquisition_active_gear=0U; acquisition_switching=0U;
  acquisition_fault_latched=0U; digital_sync.error=0U;
  acquisition_request_pending=0U; acquisition_requested_gear=0U;
  acquisition_last_switch_result=0U; gear_apply_failure=0U;
  gear_request_status=HAL_OK; adc_error=0U; dma_error=0U;
  uart6_gear_pending=0U;
  daq_active_channel_mask=0x1FU; uart6_mask_apply_count=0U;
  inject_mask_on_data=UINT32_MAX;
  square_wave_running=1U; square_wave_actual_hz=1000U;
  adc_snapshot_valid=0U; uart6_transmit_busy=0U; irq_mask=7U;
  UART6_ControlStart();
}
#ifndef UART_CONTROL_MAIN
#define UART_CONTROL_MAIN main
#endif
int UART_CONTROL_MAIN(void)
{
  uint8_t request[9], bad[9], noise[4096];
  uint32_t before, errors, seq, i, parsed, calls;
  DAQ_ControlStream stream;
  DAQ_ControlCommand command;
  DAQ_StatusState state;
  reset_control();
  CHECK(uart6_rx_active==1U && rx_dma.NDTR==2048U && irq_mask==31U);
  CHECK(uart6_status_pending==1U);
  drain(); CHECK(uart6_status_tx_count==1U && wire_lengths[0]==12U);
  CHECK(wire_last[3]==0U && wire_last[4]==0x1FU);
  CHECK(wire_last[5]==0U && wire_last[6]==0U && wire_last[7]==0U);
  CHECK(wire_last[8]==0xE8U && wire_last[9]==3U);
  before=wire_calls; UART6_ControlStart(); drain(); CHECK(wire_calls==before);
  CHECK(uart6_status_last_valid==1U && (uart6_status_snapshot_seq&1U)==0U);
  CHECK(memcmp(wire_last,(const void *)uart6_status_last_frame,12U)==0);

  make_control(request,6U,0U);
  inject_bytes(request,4U); drain(); CHECK(wire_calls==before);
  inject_bytes(request+4U,5U); drain(); CHECK(wire_calls==before+1U);
  CHECK(uart6_control_stream.frame_count==1U && uart6_control_stream.last_command==6U);
  CHECK(uart6_control_stream.last_parameter==0U && uart6_rx_byte_count==9U);
  before=wire_calls; inject_bytes(request,9U); inject_bytes(request,9U); drain();
  CHECK(wire_calls==before+2U && uart6_control_stream.frame_count==3U);
  /* A valid frame plus one extra byte remains one frame, not a ten-byte command. */
  before=wire_calls; noise[0]=0x13U;
  inject_bytes(request,9U); inject_bytes(noise,1U); drain(); CHECK(wire_calls==before+1U);

  memcpy(bad,request,9U); bad[8]^=1U;
  errors=uart6_control_stream.crc_error_count; before=wire_calls;
  inject_bytes(bad,9U); drain(); CHECK(wire_calls==before);
  CHECK(uart6_control_stream.crc_error_count==errors+1U);
  memcpy(bad,request,9U); bad[0]=0U; inject_bytes(bad,9U); drain(); CHECK(wire_calls==before);
  memcpy(bad,request,9U); bad[2]=1U; inject_bytes(bad,9U); drain(); CHECK(wire_calls==before);
  /* Wrong lengths: short never executes, extra byte is noise, not a second frame. */
  inject_bytes(request,8U); drain(); CHECK(wire_calls==before);
  fake_tick+=51U; drain(); CHECK(uart6_control_stream.partial_timeout_count!=0U);
  noise[0]=0x42U; noise[1]=0xAAU; noise[2]=0xAAU;
  inject_bytes(noise,3U); inject_bytes(request,9U); drain(); CHECK(wire_calls==before+1U);
  before=wire_calls;
  /* A byte lost in a candidate: sliding synchronization finds the following frame. */
  inject_bytes(request,4U); inject_bytes(request+5U,4U);
  inject_bytes(request,9U); drain(); CHECK(wire_calls==before+1U);

  for(i=3U;i<=7U;i++)
  {
    uint16_t param=(i==7U)?2500U:((i==3U)?5U:((i==4U)?2U:((i==5U)?255U:1U)));
    if(i==6U) { continue; }
    before=wire_calls; calls=uart6_control_unsupported_count;
    make_control(bad,(uint8_t)i,param); inject_bytes(bad,9U); drain();
    CHECK(wire_calls==before && uart6_control_unsupported_count==calls+1U);
    CHECK(acquisition_active_gear==0U && square_wave_actual_hz==1000U);
  }
  /* Legal protocol GEAR2 can be parsed but cannot change hardware. */
  make_control(bad,1U,2U); inject_bytes(bad,9U); drain(); CHECK(acquisition_active_gear==0U);
  make_control(bad,6U,1U); before=wire_calls; inject_bytes(bad,9U); drain(); CHECK(wire_calls==before);
  make_control(bad,0x77U,0U); inject_bytes(bad,9U); drain(); CHECK(wire_calls==before);
  make_control(bad,6U,0U); bad[6]=1U;
  { uint16_t crc=DAQ_CRC16_CCITT_FALSE(bad,7U); bad[7]=(uint8_t)crc; bad[8]=(uint8_t)(crc>>8U); }
  inject_bytes(bad,9U); drain(); CHECK(wire_calls==before);

  /* Hardware ORE/FE invalidates the partial stream; main restarts RX only. */
  inject_bytes(request,4U); drain(); parsed=uart6_control_stream.frame_count;
  errors=uart6_rx_hardware_error_count; hw_error=0x0CU; UART6_ReceiveIRQ();
  CHECK(uart6_rx_recovery_pending==1U && uart6_rx_active==0U);
  CHECK(uart6_rx_hardware_error_count==errors+1U && uart6_rx_last_error==0x0CU);
  drain(); CHECK(uart6_rx_active==1U && uart6_rx_recovery_count==1U);
  CHECK(uart6_control_stream.used==0U && adc_running==1U && irq_mask==31U);
  inject_bytes(request+4U,5U); drain(); CHECK(uart6_control_stream.frame_count==parsed);
  inject_bytes(request,9U); drain(); CHECK(uart6_control_stream.frame_count==parsed+1U);

  /* RX continues during a whole 260-byte TX; the status appears after it. */
  before=wire_calls; seq=ch1_uart_next_seq; inject_on_data=1U;
  set_group(1U,0xABCU); drain();
  CHECK(wire_calls==before+2U && wire_lengths[before]==260U && wire_lengths[before+1U]==12U);
  CHECK(ch1_uart_tx_count==1U && ch1_uart_next_seq==(uint16_t)(seq+1U));
  before=wire_calls; drain(); CHECK(wire_calls==before);
  adc_snapshot_valid=0U;
  acquisition_active_gear=1U; square_wave_actual_hz=2500U;
  inject_bytes(request,9U); drain(); CHECK(wire_last[3]==1U && wire_last[8]==0xC4U && wire_last[9]==9U);
  CHECK(ch1_uart_next_seq==(uint16_t)(seq+1U));
  CHECK(UART6_GetStatus(&state)==1U && state.channel_mask==0x1FU);
  square_wave_running=0U; before=wire_calls; inject_bytes(request,9U); drain();
  CHECK(wire_calls==before && uart6_status_unavailable_count==1U); square_wave_running=1U;
  acquisition_active_gear=2U; CHECK(UART6_GetStatus(&state)==0U); acquisition_active_gear=0U;

  for(i=HAL_ERROR;i<=HAL_TIMEOUT;i++)
  {
    next_tx_result=(HAL_StatusTypeDef)i; before=uart6_status_tx_error_count;
    inject_bytes(request,9U); drain();
    CHECK(uart6_status_tx_error_count==before+1U && uart6_status_last_tx_status==i);
    CHECK(adc_running==1U && uart6_status_pending==0U);
  }
  next_tx_result=HAL_OK;
  /* Ring wrap/HT/TC and bounded overflow recovery. */
  memset(noise,0x17,sizeof(noise));
  for(i=0U;i<12U;i++) { inject_bytes(noise,300U); drain(); }
  CHECK(uart6_rx_full_count>=1U && uart6_rx_half_count>=2U);
  before=uart6_rx_overflow_count; inject_bytes(noise,3000U); drain();
  CHECK(uart6_rx_overflow_count==before+1U && uart6_control_stream.used==0U);
  inject_bytes(request,9U); drain(); CHECK(uart6_control_stream.last_command==6U);
  rx_tc_pending=1U; before=uart6_rx_consumed;
  CHECK(UART6_ReceivePoll()==0U && uart6_rx_consumed==before); rx_tc_pending=0U;
  /* DMA overtakes the source DURING private copying: never decode that copy. */
  parsed=uart6_control_stream.frame_count; before=uart6_rx_overflow_count;
  inject_bytes(request,9U); copy_overrun_countdown=2U;
  CHECK(UART6_ReceivePoll()==1U);
  CHECK(uart6_rx_overflow_count==before+1U && uart6_control_stream.frame_count==parsed);
  inject_bytes(request,9U); copy_error_countdown=2U;
  CHECK(UART6_ReceivePoll()==1U && uart6_rx_recovery_pending==1U);
  CHECK(uart6_control_stream.frame_count==parsed); drain();
  CHECK(uart6_rx_active==1U && uart6_control_stream.used==0U);
  /* Monotonic DMA producer index and unsigned accounting survive uint32 wrap. */
  uart6_rx_wraps=UINT32_MAX/2048U; rx_dma.NDTR=1U;
  CHECK(UART6_ReceivePosition(&before)==1U && before==UINT32_MAX);
  uart6_rx_wraps++; rx_dma.NDTR=2048U;
  CHECK(UART6_ReceivePosition(&before)==1U && before==0U);
  CHECK((uint32_t)(before-UINT32_MAX)==1U);
  /* Queue is intentionally bounded under command floods. */
  uart6_status_pending=0U; before=uart6_status_queue_overflow_count;
  for(i=0U;i<20U;i++) { UART6_QueueStatus(); }
  CHECK(uart6_status_pending==16U && uart6_status_queue_overflow_count==before+4U);
  uart6_status_pending=0U;

  /* Parser timeout wrap, every split boundary, all noise octets, CRC recovery. */
  for(i=0U;i<256U;i++)
  {
    uint32_t j;
    DAQ_ControlStreamInit(&stream);
    for(j=0U;j<9U;j++) { (void)DAQ_ControlStreamFeed(&stream,(uint8_t)i,0U,&command); }
    parsed=stream.frame_count;
    for(j=0U;j<9U;j++) { (void)DAQ_ControlStreamFeed(&stream,request[j],1U,&command); }
    CHECK(stream.frame_count==parsed+1U && command.command==6U && stream.used<9U);
  }
  for(i=1U;i<9U;i++)
  {
    uint32_t j;
    DAQ_ControlStreamInit(&stream);
    for(j=0U;j<i;j++) { CHECK(DAQ_ControlStreamFeed(&stream,request[j],UINT32_MAX-10U,&command)==0U); }
    DAQ_ControlStreamExpire(&stream,5U); CHECK(stream.used==i);
    for(j=i;j<9U;j++) { CHECK(DAQ_ControlStreamFeed(&stream,request[j],5U,&command)==((j==8U)?1U:0U)); }
    CHECK(stream.frame_count==1U && stream.used==0U);
  }
  DAQ_ControlStreamInit(&stream);
  (void)DAQ_ControlStreamFeed(&stream,0xAAU,UINT32_MAX-10U,&command);
  DAQ_ControlStreamExpire(&stream,41U); CHECK(stream.used==0U && stream.partial_timeout_count==1U);

  /* Failure is bounded: no automatic infinite re-arm/retry, acquisition lives. */
  uart6_rx_recovery_error_count=0U;
  reset_control(); drain(); arm_result=HAL_ERROR; hw_error=8U; UART6_ReceiveIRQ(); drain();
  CHECK(uart6_rx_active==0U && uart6_rx_recovery_error_count==1U && adc_running==1U);
  calls=arm_calls; drain(); CHECK(arm_calls==calls);
  reset_control(); drain(); abort_result=HAL_TIMEOUT; hw_error=4U; UART6_ReceiveIRQ(); drain();
  CHECK(uart6_rx_active==0U && uart6_rx_recovery_error_count==2U && adc_running==1U);
  reset_control(); drain();
  /* Cold-start DMA initialization/arm failures remain isolated from ADC. */
  uart6_control_started=0U; uart6_rx_active=0U; dma_init_result=HAL_ERROR;
  calls=arm_calls; UART6_ControlStart();
  CHECK(arm_calls==calls && uart6_rx_active==0U && uart6_rx_start_status==HAL_ERROR);
  CHECK(adc_running==1U); reset_control(); drain();
  uart6_control_started=0U; uart6_rx_active=0U; arm_result=HAL_BUSY;
  UART6_ControlStart(); CHECK(uart6_rx_active==0U && uart6_rx_start_status==HAL_BUSY);
  CHECK(adc_running==1U); reset_control(); drain();
  /* Actual DMA stream -> parser -> deferred CMD01 -> single TX manager. */
  reset_control(); drain();
  before=wire_calls; calls=gear_request_calls; seq=ch1_uart_next_seq;
  make_control(request,1U,1U); inject_bytes(request,9U);
  CHECK(UART6_ServicePoll()==1U && uart6_gear_pending==1U);
  CHECK(gear_request_calls==calls+1U && acquisition_active_gear==0U);
  CHECK(wire_calls==before && ch1_uart_next_seq==seq);
  CHECK(uart6_gear_last_result==UART6_GEAR_PENDING);
  /* Busy second request never overwrites the submitted first request. */
  make_control(bad,1U,0U); inject_bytes(bad,9U);
  CHECK(UART6_ServicePoll()==1U && acquisition_requested_gear==1U);
  CHECK(uart6_gear_busy_count==1U && wire_calls==before);
  Acquisition_GearPoll(); drain();
  CHECK(acquisition_active_gear==1U && uart6_gear_pending==0U);
  CHECK(uart6_gear_last_result==UART6_GEAR_OK);
  CHECK(wire_last[2]==3U && wire_last[3]==1U && ch1_uart_next_seq==seq);
  set_group(700U,123U); source.gear=1U; drain();
  CHECK(captured_frame[5]==1U && captured_frame[6]==0x1FU);
  CHECK(ch1_uart_next_seq==(uint16_t)(seq+1U));
  make_control(request,6U,0U); inject_bytes(request,9U); drain();
  CHECK(wire_last[2]==3U && wire_last[3]==1U);
  make_control(request,1U,0U); inject_bytes(request,9U); drain();
  CHECK(acquisition_active_gear==0U && wire_last[3]==0U);
  before=uart6_gear_success_count;
  inject_bytes(request,9U); drain();
  CHECK(acquisition_active_gear==0U && uart6_gear_success_count==before+1U);
  for(i=2U;i<=4U;i++)
  {
    calls=gear_request_calls; before=uart6_gear_unsupported_count;
    make_control(request,1U,(uint16_t)i); inject_bytes(request,9U); drain();
    CHECK(gear_request_calls==calls && acquisition_active_gear==0U);
    CHECK(uart6_gear_unsupported_count==before+1U && wire_last[3]==0U);
  }
  /* CRC and invalid single-byte parameter never reach execution. */
  calls=gear_request_calls; make_control(request,1U,1U); request[8]^=1U;
  inject_bytes(request,9U); drain(); CHECK(gear_request_calls==calls);
  make_control(request,1U,0x100U); inject_bytes(request,9U); drain();
  CHECK(gear_request_calls==calls);
  make_control(request,1U,1U); gear_request_status=HAL_ERROR;
  before=uart6_gear_error_count; inject_bytes(request,9U); drain();
  CHECK(uart6_gear_error_count==before+1U && uart6_gear_pending==0U);
  CHECK(adc_running==1U && wire_last[3]==0U);
  gear_request_status=HAL_OK; gear_apply_failure=ACQ_SWITCH_ADC_START;
  before=uart6_gear_success_count; calls=uart6_status_tx_count;
  inject_bytes(request,9U); drain();
  CHECK(uart6_gear_last_result==UART6_GEAR_FAILED && adc_running==0U);
  CHECK(uart6_gear_success_count==before && uart6_status_tx_count==calls);
  CHECK(uart6_gear_pending==0U && acquisition_request_pending==0U);

  printf("PASS UART control RX/status integration: %u checks (simulated DMA/HAL, no board)\n",checks);
  return 0;
}
