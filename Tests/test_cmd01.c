/* Real parser/dispatcher/request/gear manager/snapshots/frame code with HAL
 * stubs. No serial port, ST-LINK or development board is accessed. */
#include "daq_control_stream.h"
#define ADC_SAMPLING_TEST
#include "test_gear.c"
#undef ADC_SAMPLING_TEST

static uint32_t uart6_control_unsupported_count;
static uint32_t pending_status, status_count, status_unavailable;
static uint32_t square_wave_running=1U, square_wave_actual_hz=1000U;
static uint8_t status_frame[12];
static void UART6_QueueStatus(void) { pending_status++; }
#include "uart6_gear_state.inc"
/* This suite isolates CMD01; CMD07's real executor is tested separately. */
static HAL_StatusTypeDef SquareWave_ChangeFrequency(uint32_t hz)
{ (void)hz; return HAL_OK; }
static void Trigger_Reset(void) { }
#include "uart6_gear_under_test.inc"
#include "uart6_status_source.inc"

static void reset_command(void)
{
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  pending_status=0U; status_count=0U; status_unavailable=0U;
  uart6_control_unsupported_count=0U;
  uart6_gear_request_count=0U; uart6_gear_submit_count=0U;
  uart6_gear_success_count=0U; uart6_gear_error_count=0U;
  uart6_gear_unsupported_count=0U; uart6_gear_busy_count=0U;
  uart6_gear_pending=0U; uart6_gear_requested=UINT32_MAX;
  uart6_gear_last_request_status=UINT32_MAX;
  uart6_gear_last_switch_result=UINT32_MAX;
  uart6_gear_last_result=UART6_GEAR_IDLE;
  memset(status_frame,0xA5,sizeof(status_frame));
}

/* A whole input frame is parsed by the real streaming parser before dispatch.
 * corruption: 1=CRC, 2=header, 3=TYPE, 4=reserved (with a recomputed CRC). */
static void receive_command(uint8_t cmd,uint16_t param,uint32_t corruption)
{
  uint8_t b[9]={0xAAU,0x55U,2U,0U,0U,0U,0U,0U,0U};
  DAQ_ControlStream stream;
  DAQ_ControlCommand command;
  uint16_t crc;
  uint32_t i;
  b[3]=cmd; b[4]=(uint8_t)param; b[5]=(uint8_t)(param>>8U);
  if(corruption==2U) { b[0]=0U; }
  if(corruption==3U) { b[2]=3U; }
  if(corruption==4U) { b[6]=1U; }
  crc=DAQ_CRC16_CCITT_FALSE(b,7U); b[7]=(uint8_t)crc; b[8]=(uint8_t)(crc>>8U);
  if(corruption==1U) { b[8]^=1U; }
  DAQ_ControlStreamInit(&stream);
  for(i=0U;i<9U;i++)
  { if(DAQ_ControlStreamFeed(&stream,b[i],0U,&command)) { UART6_DispatchControl(&command); } }
}

static void reply(void)
{
  DAQ_StatusState state;
  if(pending_status==0U || uart6_gear_pending!=0U) { return; }
  pending_status--;
  if(!UART6_GetStatus(&state)) { status_unavailable++; return; }
  CHECK(DAQ_BuildStatusFrame(&state,status_frame,sizeof(status_frame))==DAQ_STATUS_OK);
  CHECK(status_frame[3]==acquisition_active_gear && status_frame[4]==0x1FU);
  CHECK(reference_crc(status_frame,10U)==
        (uint16_t)(status_frame[10]|((uint16_t)status_frame[11]<<8U)));
  status_count++;
}

static void execute_command(void)
{
  Acquisition_GearPoll();
  (void)UART6_GearControlPoll();
  reply();
}

int main(void)
{
  uint32_t i, before, group, configs, samples, seq;
  Full_Snapshot snapshot;
  reset_command();
  complete_and_send(0U,0U); group=adc_group_count;
  receive_command(1U,1U,0U);
  CHECK(uart6_gear_request_count==1U && uart6_gear_submit_count==1U);
  CHECK(uart6_gear_pending==1U && acquisition_request_pending==1U);
  CHECK(acquisition_active_gear==0U && uart6_gear_success_count==0U);
  CHECK(Full_GetSnapshot(&snapshot)==HAL_OK && snapshot.gear==0U);
  CHECK(UART6_GearControlPoll()==1U && pending_status==0U);
  reply(); CHECK(status_count==0U);
  execute_command();
  CHECK(uart6_gear_last_result==UART6_GEAR_OK && status_frame[3]==1U);
  CHECK(uart6_gear_success_count==1U && status_count==1U);
  CHECK(adc_group_count==group && ch1_uart_next_seq==1U);
  CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U);
  CHECK(Full_GetSnapshot(&snapshot)==HAL_BUSY && CH1_FrameTransmitPoll()==0U);
  complete_and_send(1U,1U);
  CHECK(adc_group_count==group+1U && sent[5]==1U);
  receive_command(1U,0U,0U); execute_command();
  CHECK(status_frame[3]==0U && uart6_gear_success_count==2U);
  complete_and_send(0U,2U);
  /* Repeating the current gear confirms it without restarting healthy DMA. */
  configs=config_count; group=adc_group_count;
  receive_command(1U,0U,0U); execute_command();
  CHECK(config_count==configs && adc_group_count==group && full_snapshot_valid==1U);
  CHECK(uart6_gear_success_count==3U && CH1_FrameTransmitPoll()==0U);

  for(i=2U;i<=4U;i++)
  {
    before=status_count; seq=ch1_uart_next_seq; configs=config_count;
    receive_command(1U,(uint16_t)i,0U); execute_command();
    CHECK(status_count==before+1U && status_frame[3]==0U);
    CHECK(acquisition_requested_gear==0U && acquisition_active_gear==0U);
    CHECK(config_count==configs && adc_running==1U && ch1_uart_next_seq==seq);
    CHECK(uart6_gear_pending==0U && uart6_gear_last_result==UART6_GEAR_UNSUPPORTED);
  }
  CHECK(uart6_gear_unsupported_count==3U && uart6_control_unsupported_count==3U);
  before=uart6_gear_request_count;
  for(i=1U;i<=4U;i++) { receive_command(1U,1U,i); execute_command(); }
  receive_command(1U,0x100U,0U); receive_command(1U,5U,0U);
  CHECK(uart6_gear_request_count==before && acquisition_active_gear==0U);

  /* Requests during a pending command cannot replace its target. CMD06 is
   * queued normally, but the TX-manager test checks its deferred wire send. */
  receive_command(1U,1U,0U); receive_command(1U,0U,0U);
  CHECK(uart6_gear_busy_count==1U && acquisition_requested_gear==1U);
  receive_command(6U,0U,0U); execute_command(); reply(); reply();
  CHECK(status_frame[3]==1U && uart6_gear_success_count==4U);

  /* Repeated remote changes at partial DMA offsets discard only incomplete
   * points; new complete frames carry the new gear and a continuous SEQ. */
  reset_command();
  for(i=0U;i<20U;i++)
  {
    uint32_t gear=(i&1U)?0U:1U;
    timer2.CNT=1U; generate(17U,0U); group=adc_group_count;
    samples=digital_sync.sample_count;
    receive_command(1U,(uint16_t)gear,0U); execute_command();
    CHECK(status_frame[3]==gear && adc_group_count==group);
    CHECK(digital_sync.sample_count==samples && full_snapshot_valid==0U);
    complete_and_send(gear,(uint16_t)i);
  }
  CHECK(uart6_gear_success_count==20U && uart6_gear_error_count==0U);
  ch1_uart_next_seq=0xFFFFU;
  receive_command(1U,1U,0U); execute_command(); complete_and_send(1U,0xFFFFU);
  receive_command(1U,0U,0U); execute_command(); complete_and_send(0U,0U);

  /* Every existing HAL stop/configure/start failure remains latched, gives no
   * healthy status response, and never retries itself. */
  for(i=ACQ_SWITCH_TIM2_STOP;i<=ACQ_SWITCH_TIM2_START;i++)
  {
    reset_command(); failure=i;
    receive_command(1U,1U,0U); execute_command();
    CHECK(uart6_gear_last_result==UART6_GEAR_FAILED && uart6_gear_error_count==1U);
    CHECK(uart6_gear_last_switch_result==i && uart6_gear_success_count==0U);
    CHECK(adc_running==0U && acquisition_fault_latched==1U && full_snapshot_valid==0U);
    CHECK(status_count==0U && status_unavailable==1U && uart6_gear_pending==0U);
    CHECK(CH1_FrameTransmitPoll()==0U);
    configs=config_count; execute_command(); execute_command();
    CHECK(config_count==configs && uart6_gear_error_count==1U);
  }
  reset_command(); adc_config_fail_rank=2U;
  receive_command(1U,1U,0U); execute_command();
  CHECK(uart6_gear_last_switch_result==ACQ_SWITCH_ADC_CONFIG && status_count==0U);

  /* ADC144 remains GEAR0-only: request rejection leaves acquisition alive. */
  reset_command();
  CHECK(Acquisition_RequestADCSampling(144U)==HAL_OK); Acquisition_GearPoll();
  CHECK(acquisition_adc_active_cycles==144U && adc_running==1U);
  configs=config_count; receive_command(1U,1U,0U); execute_command();
  CHECK(uart6_gear_last_result==UART6_GEAR_REJECTED && uart6_gear_pending==0U);
  CHECK(config_count==configs && adc_running==1U && acquisition_active_gear==0U);
  CHECK(uart6_gear_error_count==1U && status_frame[3]==0U);
  CHECK(uart6_gear_last_switch_result==ACQ_SWITCH_ADC_CONFLICT);
  printf("PASS CMD01 real-manager integration: %u checks (HAL/register simulation, no board)\n",checks);
  return 0;
}
