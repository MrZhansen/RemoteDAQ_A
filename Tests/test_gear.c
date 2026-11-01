/* Actual firmware gear/ISR/snapshot/frame/UART services with HAL/register stubs.
 * This validates logic, not MCU interrupt latency or serial wire timing. */
#include "daq_udp_trial.h"
#define GEAR_TEST
#define main full_sync_legacy_main
#include "test_full_sync.c"
#undef main
#include "ch1_service_defines.inc"

#define TIM_CR1_CEN 1U
#define TIM_CR2_MMS 0x70U
#define TIM_TRGO_ENABLE 0x10U
#define TIM_TRGO_UPDATE 0x20U
#define ADC_CR2_ADON 1U
#define ADC_SAMPLETIME_480CYCLES 7U
#define ADC_SAMPLETIME_144CYCLES 6U
#define ADC_SAMPLETIME_28CYCLES 2U
#define ADC_SAMPLETIME_3CYCLES 0U
#define ADC_CHANNEL_0 0U
#define ADC_CHANNEL_8 8U
#define ADC_REGULAR_RANK_1 1U
#define ADC_REGULAR_RANK_2 2U
#define ADC_SMPR2_SMP0 7U
#define ADC_SMPR2_SMP8 (7U << 24U)
#define ADC_SQR3_SQ1 31U
#define ADC_SQR3_SQ2 (31U << 5U)
typedef struct { uint32_t Channel, Rank, SamplingTime, Offset; } ADC_ChannelConfTypeDef;
#define DMA_SxCR_EN 1U
#define ADC_FLAG_EOC 2U
#define ADC_FLAG_OVR 32U
#define ADC_FLAG_STRT 16U
#define HAL_ADC_ERROR_NONE 0U
#define HAL_DMA_ERROR_NONE 0U
#define HAL_UART_ERROR_NONE 0U
#define TIM_FLAG_UPDATE 1U
#define TIM2_IRQn 0U
#define DMA2_Stream0_IRQn 1U
#define ADC_IRQn 2U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0U
#define TIM_SLAVEMODE_EXTERNAL1 7U
#define TIM_TS_ITR0 0U
#define RCC_DCKCFGR1_TIMPRE 0x100U
#define CH1_DMA_SAMPLES 200U
#define CLEAR_BIT(reg,mask) ((reg) &= ~(mask))
#define MODIFY_REG(reg,mask,value) ((reg) = ((reg) & ~(mask)) | (value))
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->Instance->CNT=(v))
#define __HAL_TIM_CLEAR_FLAG(h,v) ((h)->Instance->SR &= ~(v))
#define __HAL_DBGMCU_FREEZE_TIM5() ((void)0)
#define __HAL_RCC_TIM5_CLK_ENABLE() ((void)0)
#define __HAL_DMA_GET_TC_FLAG_INDEX(h) 1U
#define __HAL_DMA_GET_HT_FLAG_INDEX(h) 2U
#define __HAL_DMA_GET_TE_FLAG_INDEX(h) 4U
#define __HAL_DMA_GET_DME_FLAG_INDEX(h) 8U
#define __HAL_DMA_GET_FE_FLAG_INDEX(h) 16U
#define __HAL_DMA_CLEAR_FLAG(h,v) (dma_pending &= ~(v))
#define __HAL_ADC_CLEAR_FLAG(h,v) ((h)->Instance->SR &= ~(v))
#define __DSB() ((void)0)
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_3 8U
#define GPIO_PIN_SET 1U

typedef struct { uint32_t SlaveMode, InputTrigger; } TIM_SlaveConfigTypeDef;
typedef struct { uint32_t error; } UART_HandleTypeDef;
static UART_HandleTypeDef huart6;
static struct { uint32_t DCKCFGR1; } rcc;
#define RCC (&rcc)
static uint32_t primask, irq_enable, irq_pending, dma_pending;
static uint32_t failure, counter_init_failure, counter_slave_failure;
static uint32_t fail_once, adc_config_fail_once;
static uint32_t fail_model(uint32_t code)
{
  if(failure!=code) { return 0U; }
  if(fail_once!=0U) { failure=0U; fail_once=0U; }
  return 1U;
}
static uint32_t pclk=54000000U, hclk=216000000U;
static uint32_t config_count, ug_count, transmitted;
static uint32_t adc_config_count, adc_start_count, adc_config_fail_rank;
static uint32_t adc_bad_sample_rank, adc_bad_rank, adc_last_sampling;
static uint32_t request_during_uart, stop_readback_failure, bad_arr, bad_ndtr;
static uint8_t sent[260];
static HAL_StatusTypeDef tx_status=HAL_OK;
static volatile uint32_t uart6_initialized=1U;
static volatile uint32_t adc_error, adc_hal_error, dma_error, adc_error_count;
static volatile uint32_t adc_dma_start_status, tim2_start_status, adc_startup_stage;
static volatile uint32_t digital_input_d1, digital_input_d2, digital_input_d3;
static uint32_t adc_debug_start;
#define ADC_ERROR_START 1U
#define ADC_ERROR_TIMER_START 2U
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask=1U; }
static void __set_PRIMASK(uint32_t p) { primask=p; }
static void HAL_NVIC_DisableIRQ(uint32_t n) { irq_enable &= ~(1U<<n); }
static void HAL_NVIC_EnableIRQ(uint32_t n) { irq_enable |= 1U<<n; }
static void HAL_NVIC_ClearPendingIRQ(uint32_t n) { irq_pending &= ~(1U<<n); }
static void HAL_NVIC_SetPriority(uint32_t n,uint32_t p,uint32_t sub)
{ CHECK(n==TIM2_IRQn && p==0U && sub==0U); }
static uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
static uint32_t HAL_RCC_GetHCLKFreq(void) { return hclk; }
static uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *h) { CHECK(h==&hadc3); return 0U; }
static uint32_t HAL_DMA_GetError(FakeDmaHandle *h) { CHECK(h==&hdma_adc3); return 0U; }
static void CH1_CaptureDebug(void *out,uint32_t p) { CHECK(out==&adc_debug_start && p==0U); }
static HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *h)
{
  CHECK(primask==0U && h==&htim2 && adc_running==0U && irq_enable==0U);
  CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U);
  h->Instance->CR1 &= ~1U; h->Instance->DIER &= ~1U;
  return (fail_model(ACQ_SWITCH_TIM2_STOP))?HAL_ERROR:HAL_OK;
}
static HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *h)
{
  CHECK(primask==0U && h==&hadc3 && (TIM2->CR1&1U)==0U);
  if(fail_model(ACQ_SWITCH_ADC_STOP)) { return HAL_TIMEOUT; }
  if(stop_readback_failure) { return HAL_OK; }
  h->Instance->CR2=0U; dma.CR=0U;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *h)
{
  CHECK(primask==0U && h==&htim5 && (TIM2->CR1&1U)==0U);
  h->Instance->CR1=0U;
  return (fail_model(ACQ_SWITCH_TIM5_STOP))?HAL_BUSY:HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *h)
{
  CHECK(primask==0U && (TIM2->CR1&1U)==0U && adc3.CR2==0U && dma.CR==0U);
  if(h==&htim2)
  {
    CHECK((TIM2->CR2&TIM_CR2_MMS)==TIM_TRGO_ENABLE);
    CHECK(htim5.Instance==NULL || (TIM5->CR1&1U)==0U);
    config_count++;
    if(fail_model(ACQ_SWITCH_TIM2_CONFIG)) { return HAL_ERROR; }
    ug_count++; /* This UG is gated; no ADC/TIM5 sees a trigger. */
  }
  else
  {
    CHECK(h==&htim5);
    /* ADC is already armed for TIM5 init, so the generic assertion above
     * is handled separately by the counter wrapper below. */
  }
  h->Instance->PSC=h->Init.Prescaler;
  h->Instance->ARR=h->Init.Period + bad_arr;
  h->Instance->CNT=0U; h->Instance->SR=1U;
  return HAL_OK;
}
/* Real DigitalCounter_Init runs after ADC arm, with no TIM2 trigger yet. */
static HAL_StatusTypeDef Counter_Base_Init(TIM_HandleTypeDef *h)
{
  CHECK(h==&htim5 && (TIM2->CR1&1U)==0U);
  if(counter_init_failure) { return HAL_ERROR; }
  h->Instance->PSC=h->Init.Prescaler; h->Instance->ARR=h->Init.Period;
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *h,TIM_SlaveConfigTypeDef *s)
{
  CHECK(h==&htim5 && s->SlaveMode==7U && s->InputTrigger==0U);
  return counter_slave_failure?HAL_ERROR:HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *h)
{
  CHECK(h==&htim5 && (TIM2->CR1&1U)==0U && dma.NDTR==200U);
  if(fail_model(ACQ_SWITCH_TIM5_START)) { return HAL_ERROR; }
  h->Instance->CR1=1U; return HAL_OK;
}
static HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{
  CHECK(primask==0U && h==&hadc3 && adc_running==0U && irq_enable==0U);
  CHECK((TIM2->CR1&1U)==0U && adc3.CR2==0U && dma.CR==0U);
  CHECK(htim5.Instance==NULL || (TIM5->CR1&1U)==0U);
  CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U);
  CHECK(c->Rank==1U || c->Rank==2U);
  CHECK(c->Channel==((c->Rank==1U)?0U:8U));
  CHECK(c->SamplingTime==0U || c->SamplingTime==2U ||
        c->SamplingTime==6U || c->SamplingTime==7U);
  if(c->Rank==1U) { adc_last_sampling=c->SamplingTime; }
  CHECK(c->SamplingTime==adc_last_sampling && c->Offset==0U);
  adc_config_count++;
  if(adc_config_fail_rank==c->Rank)
  {
    if(adc_config_fail_once) { adc_config_fail_rank=0U; adc_config_fail_once=0U; }
    return (c->Rank==1U)?HAL_BUSY:HAL_ERROR;
  }
  {
    uint32_t shift=c->Channel*3U, rank_shift=(c->Rank-1U)*5U;
    uint32_t sample=(adc_bad_sample_rank==c->Rank)?0U:c->SamplingTime;
    uint32_t channel=(adc_bad_rank==c->Rank)?1U:c->Channel;
    adc3.SMPR2=(adc3.SMPR2&~(7U<<shift))|(sample<<shift);
    adc3.SQR3=(adc3.SQR3&~(31U<<rank_shift))|(channel<<rank_shift);
  }
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h,uint32_t *b,uint32_t n)
{
  CHECK(primask==0U && h==&hadc3 && b==(uint32_t *)(void *)adc_dma_buffer && n==200U);
  CHECK((TIM2->CR1&1U)==0U && (TIM5->CR1&1U)==0U && irq_enable==0U);
  CHECK(irq_pending==0U && dma_pending==0U && timer2.SR==0U);
  CHECK((adc3.SMPR2&7U)==adc_last_sampling && ((adc3.SMPR2>>24U)&7U)==adc_last_sampling);
  CHECK((adc3.SQR3&31U)==0U && ((adc3.SQR3>>5U)&31U)==8U);
  adc_start_count++;
  pair_index=0U; serial=0U;
  adc3.CR2=1U; dma.CR=1U; dma.NDTR=200U-bad_ndtr;
  return (fail_model(ACQ_SWITCH_ADC_START))?HAL_BUSY:HAL_OK;
}
static HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h)
{
  CHECK(h==&htim2 && adc_running==1U && acquisition_switching==0U);
  CHECK(irq_enable==7U && timer5.CNT==0U && digital_sync.next_index==0U);
  CHECK(digital_sync.completed_groups==adc_group_count && dma.NDTR==200U);
  CHECK((TIM2->CR2&TIM_CR2_MMS)==TIM_TRGO_UPDATE && TIM2->CNT==0U && TIM2->SR==0U);
  h->Instance->DIER=1U; h->Instance->CR1=1U;
  return (fail_model(ACQ_SWITCH_TIM2_START))?HAL_ERROR:HAL_OK;
}
static HAL_StatusTypeDef CH1_GetSnapshot(CH1_Snapshot *p) { (void)p; return HAL_BUSY; }
static HAL_StatusTypeDef Analog_GetSnapshot(Analog_Snapshot *p) { (void)p; return HAL_BUSY; }
static uint32_t HAL_UART_GetError(UART_HandleTypeDef *h) { return h->error; }
static uint32_t HAL_GPIO_ReadPin(FakeGpio *port,uint32_t pin) { return (port->IDR&pin)?1U:0U; }
static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,const uint8_t *b,uint16_t n,uint32_t timeout)
{
  CHECK(h==&huart6 && n==260U && timeout==10U && primask==0U);
  CHECK(b[5]==acquisition_active_gear && b[6]==0x1FU && b[7]==50U);
  CHECK(reference_crc(b,258U)==(uint16_t)(b[258]|((uint16_t)b[259]<<8U)));
  memcpy(sent,b,n); transmitted++;
  if(request_during_uart) { acquisition_requested_gear=1U; acquisition_request_pending=1U; request_during_uart=0U; }
  return tx_status;
}
#include "ch1_preview_state.inc"
#include "ch1_uart_state.inc"
#include "analog_frame_builder.inc"
#include "trigger_config_state.inc"
#ifndef TRIGGER_UART_STUBS_DEFINED
#define TRIGGER_UART_STUBS_DEFINED
static uint32_t Trigger_ProcessNewSnapshot(void) { return 0U; }
static uint32_t Trigger_TakeReady(Full_Snapshot *snapshot)
{ (void)snapshot; return 0U; }
static uint32_t Trigger_BuildFramePreview(uint16_t sequence,
                                          const Full_Snapshot *snapshot)
{ (void)sequence; (void)snapshot; return 0U; }
#endif
#include "ch1_uart_under_test.inc"
/* Route only the counter's HAL init call to its phase-specific stub. */
#define HAL_TIM_Base_Init Counter_Base_Init
#include "gear_counter.inc"
#undef HAL_TIM_Base_Init
#include "gear_under_test.inc"

static void setup(void)
{
  reset_model();
  memset((void *)&offline,0,sizeof(offline));
  offline_request_groups=0U; offline_stop_request=0U; model_tick=100U; fake_dwt.CTRL=1U;
  memset(&timer2,0,sizeof(timer2)); memset(&timer5,0,sizeof(timer5));
  memset(&dma,0,sizeof(dma)); memset(&adc3,0,sizeof(adc3));
  memset(&htim5,0,sizeof(htim5));
  adc_running=0U; adc_snapshot_valid=0U; full_snapshot_valid=0U;
  full_group_publish_count=0U; adc_group_count=0U;
  acquisition_active_gear=ACQ_GEAR_INVALID; acquisition_requested_gear=0U;
  acquisition_request_pending=0U; acquisition_last_request=0U;
  acquisition_fault_latched=0U; acquisition_switching=0U;
  acquisition_udp_gear2_available=0U; acquisition_udp_gear2_active=0U;
  acquisition_udp_gear2_starting=0U;
  acquisition_udp_gear2_return_count=0U; acquisition_udp_gear2_fault_count=0U;
  acquisition_switch_ok_count=0U; acquisition_switch_fail_count=0U;
  acquisition_last_switch_result=ACQ_GEAR_INVALID;
  acquisition_adc_active_cycles=ACQ_GEAR_INVALID; acquisition_adc_requested_cycles=480U;
  acquisition_adc_request_pending=0U; acquisition_adc_test_mode=0U; acquisition_adc_last_request=480U;
  acquisition_adc_last_switch_result=ACQ_GEAR_INVALID;
  acquisition_adc_switch_ok_count=0U; acquisition_adc_switch_fail_count=0U;
  acquisition_adc_rank1_status=UINT32_MAX; acquisition_adc_rank2_status=UINT32_MAX;
  acquisition_fast_isr_active=0U; acquisition_fast_irq_last_cycles=0U;
  acquisition_fast_irq_max_cycles=0U; acquisition_fast_irq_budget_cycles=0U;
  acquisition_fast_phase_last_cycles=0U; acquisition_fast_phase_max_cycles=0U;
  acquisition_fast_budget_fail_count=0U;
  ch1_preview_have_group=0U; ch1_preview_last_group=0U;
  ch1_frame_build_count=0U; ch1_uart_tx_count=0U; ch1_uart_tx_error_count=0U;
  ch1_uart_next_seq=0U; ch1_skipped_group_count=0U;
  ch1_frame_preview_valid=0U; ch1_uart_last_valid=0U;
  failure=0U; fail_once=0U; adc_config_fail_once=0U;
  counter_init_failure=0U; counter_slave_failure=0U;
  adc_error=0U; adc_hal_error=0U; dma_error=0U; adc_error_count=0U;
  irq_enable=7U; irq_pending=7U; dma_pending=31U; primask=0U;
  pclk=54000000U; hclk=216000000U; RCC->DCKCFGR1=0U;
  config_count=0U; ug_count=0U; transmitted=0U; tx_status=HAL_OK;
  request_during_uart=0U; stop_readback_failure=0U; bad_arr=0U; bad_ndtr=0U;
  adc_config_count=0U; adc_start_count=0U; adc_config_fail_rank=0U;
  adc_bad_sample_rank=0U; adc_bad_rank=0U; adc_last_sampling=7U;
  pattern_mode=2U; timer2.CNT=5U;
}
static void switch_to(uint32_t gear)
{
  acquisition_requested_gear=gear; Acquisition_GearPoll();
  CHECK(acquisition_active_gear==gear && adc_running==1U);
  CHECK(timer2.PSC==107U && timer2.ARR==((gear==0U)?199U:99U));
  CHECK(acquisition_sample_interval_us==((gear==0U)?200U:100U));
  CHECK(digital_sync.timing.sample_interval_us==acquisition_sample_interval_us);
  CHECK(digital_sync.timing.max_phase_us==20U && digital_sync.timing.max_body_us==20U);
  CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U && ch1_frame_preview_valid==0U);
  CHECK((full_snapshot_seq&1U)==0U && (adc_snapshot_seq&1U)==0U);
  CHECK(digital_sync.next_index==0U && digital_sync.error==0U);
  CHECK(timer5.CNT==0U && dma.NDTR==200U && digital_sync.last_trigger==0U);
}
static void complete_and_send(uint32_t gear,uint16_t expected_seq)
{
  Full_Snapshot s;
  uint32_t i, before=ch1_uart_tx_count;
  timer2.CNT=1U;
  generate(49U,1U);
  CHECK(full_snapshot_valid==0U && Full_GetSnapshot(&s)==HAL_BUSY);
  CHECK(CH1_FrameTransmitPoll()==0U && ch1_uart_tx_count==before);
  generate(1U,1U);
  CHECK(Full_GetSnapshot(&s)==HAL_OK && s.gear==gear);
  CHECK(s.group_count==adc_group_count && s.group_count==digital_sync.completed_groups);
  for(i=0U;i<50U;i++) { CHECK(s.ch1[i]==i+1U && s.ch2[i]==i+1001U && s.dig[i]==pattern(i)); }
  CHECK(CH1_FrameTransmitPoll()==1U && ch1_uart_tx_count==before+1U);
  CHECK(sent[5]==gear && ch1_frame_gear==gear);
  CHECK((uint16_t)(sent[3]|((uint16_t)sent[4]<<8U))==expected_seq);
  CHECK(ch1_uart_next_seq==(uint16_t)(expected_seq+1U));
  CHECK(CH1_FrameTransmitPoll()==0U && ch1_uart_tx_count==before+1U);
}
#ifdef ADC_SAMPLING_TEST
#define GEAR_TEST_MAIN gear_legacy_main
#else
#define GEAR_TEST_MAIN main
#endif
int GEAR_TEST_MAIN(void)
{
  uint32_t i, n, before, base_group, old_samples;
  Full_Snapshot s, saved;
  uint8_t frame[260], unchanged[260];
  setup();
  CHECK(acquisition_requested_gear==0U);
  CHECK(Acquisition_ApplyGear(0U)==HAL_OK && acquisition_active_gear==0U);
  CHECK(acquisition_sample_interval_us==200U && timer2.PSC==107U && timer2.ARR==199U);
  CHECK(digital_sync.timing.sample_interval_us==200U && digital_sync.timing.max_phase_us==20U);
  complete_and_send(0U,0U);
  CHECK(Acquisition_RequestGear(1U)==HAL_OK);
  /* A request alone never relabels the already published old group. */
  CHECK(Full_GetSnapshot(&s)==HAL_OK && s.gear==0U);
  CHECK(DAQ_BuildFullFrameForGear(s.ch1,s.ch2,s.dig,s.gear,13U,frame,260U)==DAQ_FRAME_OK && frame[5]==0U);
  old_samples=digital_sync.sample_count; base_group=adc_group_count;
  generate(17U,0U); old_samples+=17U;
  switch_to(1U);
  CHECK(adc_group_count==base_group && digital_sync.completed_groups==base_group);
  CHECK(digital_sync.sample_count==old_samples && ch1_uart_next_seq==1U);
  memset(&s,0x5A,sizeof(s)); memcpy(&saved,&s,sizeof(s));
  CHECK(Full_GetSnapshot(&s)==HAL_BUSY && memcmp(&s,&saved,sizeof(s))==0);
  Full_PublishGroup(0U); /* A stale IRQ/partial half cannot publish. */
  CHECK(full_snapshot_valid==0U && digital_sync.error!=0U);
  /* Stale callbacks are prevented by NVIC/flags; reset this deliberate misuse. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); complete_and_send(0U,0U);
  generate(17U,0U); switch_to(1U); complete_and_send(1U,1U);
  switch_to(0U); complete_and_send(0U,2U);
  for(n=0U;n<20U;n++) { switch_to((n&1U)?0U:1U); complete_and_send((n&1U)?0U:1U,(uint16_t)(n+3U)); }
  CHECK(acquisition_switch_ok_count==22U && acquisition_switch_fail_count==0U);
  before=config_count;
  CHECK(Acquisition_RequestGear(2U)==HAL_ERROR && config_count==before && adc_running==1U);
  acquisition_requested_gear=99U; Acquisition_GearPoll();
  CHECK(acquisition_switch_fail_count==2U && acquisition_last_switch_result==ACQ_SWITCH_BAD_GEAR);
  Acquisition_GearPoll(); CHECK(acquisition_switch_fail_count==2U && config_count==before);
  CHECK(Acquisition_RequestGear(0U)==HAL_OK); Acquisition_GearPoll(); CHECK(config_count==before);
  ch1_uart_next_seq=0xFFFFU;
  switch_to(1U); complete_and_send(1U,0xFFFFU);
  switch_to(0U); complete_and_send(0U,0U);
  switch_to(1U); timer2.CNT=1U; generate(50U,1U); tx_status=HAL_TIMEOUT;
  CHECK(CH1_FrameTransmitPoll()==1U && ch1_uart_next_seq==1U && ch1_uart_tx_error_count==1U);
  CHECK(CH1_FrameTransmitPoll()==0U); tx_status=HAL_OK;
  switch_to(0U); complete_and_send(0U,1U);
  CH1_BackgroundPoll();
  /* Snapshot gear remains authoritative even if an unrelated variable changes. */
  CHECK(Full_GetSnapshot(&s)==HAL_OK); acquisition_active_gear=1U;
  CHECK(DAQ_BuildFullFrameForGear(s.ch1,s.ch2,s.dig,s.gear,7U,frame,260U)==DAQ_FRAME_OK && frame[5]==0U);
  memset(frame,0x5A,sizeof(frame)); memcpy(unchanged,frame,sizeof(frame));
  CHECK(DAQ_BuildFullFrameForGear(s.ch1,s.ch2,s.dig,2U,7U,frame,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(memcmp(frame,unchanged,260U)==0);
  /* All HAL stop/configure/start failures stop publication; no auto retry. */
  for(i=ACQ_SWITCH_TIM2_STOP;i<=ACQ_SWITCH_TIM2_START;i++)
  {
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); complete_and_send(0U,0U);
    failure=i; acquisition_requested_gear=1U; Acquisition_GearPoll();
    CHECK(acquisition_fault_latched==1U && acquisition_last_switch_result==i);
    CHECK(adc_running==0U && acquisition_active_gear==ACQ_GEAR_INVALID && (timer2.CR1&1U)==0U);
    CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U && ch1_frame_preview_valid==0U);
    CHECK(irq_enable==0U && acquisition_switch_fail_count==1U && ch1_uart_next_seq==1U);
    before=transmitted; CHECK(CH1_FrameTransmitPoll()==0U && transmitted==before);
    Full_PublishGroup(0U); CHECK(full_snapshot_valid==0U);
    Acquisition_GearPoll(); CHECK(acquisition_switch_fail_count==1U);
  }
  for(i=0U;i<2U;i++)
  {
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
    counter_init_failure=(i==0U); counter_slave_failure=(i==1U);
    acquisition_requested_gear=1U; Acquisition_GearPoll();
    CHECK(acquisition_last_switch_result==ACQ_SWITCH_TIM5_START && adc_running==0U);
  }
  setup(); pclk=53123456U;
  CHECK(Acquisition_ApplyGear(0U)==HAL_ERROR && acquisition_last_switch_result==ACQ_SWITCH_CLOCK);
  CHECK(adc_running==0U && full_snapshot_valid==0U);
  /* Group wrap remains separate from ring-half parity and protocol SEQ. */
  setup(); adc_group_count=0xFFFFFFFFU; CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  complete_and_send(0U,0U); CHECK(adc_group_count==0U);
  switch_to(1U); complete_and_send(1U,1U); CHECK(adc_group_count==1U);
  CHECK(ug_count==config_count && digital_sync.error==0U);
  /* Restart from every relevant half boundary, with stale IRQ flags set. */
  {
    static const uint32_t offsets[]={0U,1U,49U,50U,51U,99U,100U,149U};
    for(i=0U;i<sizeof(offsets)/sizeof(offsets[0]);i++)
    {
      setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
      timer2.CNT=1U; generate(offsets[i],1U);
      base_group=adc_group_count; irq_pending=7U; dma_pending=31U;
      switch_to(1U); CHECK(adc_group_count==base_group);
      complete_and_send(1U,0U);
      CHECK(adc_group_count==base_group+1U && digital_sync.error==0U);
    }
  }
  /* Stop after digital point 50, before its ADC pair and DMA half finish. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  timer2.CNT=1U; generate(49U,0U); timer5.CNT++;
  timer2.SR=1U; TIM2_IRQHandler();
  CHECK(digital_sync.completed_groups==1U && adc_group_count==0U);
  switch_to(1U); CHECK(digital_sync.completed_groups==0U);
  complete_and_send(1U,0U);
  /* A Watch request during a bounded UART call takes effect next poll only. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_during_uart=1U;
  complete_and_send(0U,0U);
  CHECK(acquisition_requested_gear==1U && acquisition_active_gear==0U && sent[5]==0U);
  Acquisition_GearPoll(); CHECK(acquisition_active_gear==1U && ch1_uart_next_seq==1U);
  complete_and_send(1U,1U);
  /* HAL_OK alone is insufficient: check ADC stop, ARR and DMA NDTR readback. */
  for(i=0U;i<3U;i++)
  {
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
    stop_readback_failure=(i==0U); bad_arr=(i==1U); bad_ndtr=(i==2U);
    acquisition_requested_gear=1U; Acquisition_GearPoll();
    CHECK(acquisition_last_switch_result==((i==0U)?ACQ_SWITCH_ADC_STOP:
        ((i==1U)?ACQ_SWITCH_TIM2_CONFIG:ACQ_SWITCH_ADC_START)));
    CHECK(adc_running==0U && full_snapshot_valid==0U && acquisition_fault_latched==1U);
  }
  printf("PASS gear integration: %u checks; 0/1 startup/switching, discarded partial groups, coherent gear/data, HAL failures, SEQ/CRC/UART continuity, no duplicate sends.\n",checks);
  return 0;
}
