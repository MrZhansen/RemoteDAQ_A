/* Extracted production state machine + real IRQ/publisher/snapshot code.
 * HAL/register stubs and scripted time only; no hardware timing assertion. */
#define ADC_SAMPLING_TEST
#include "test_gear.c"
#undef ADC_SAMPLING_TEST
#include "offline_private_state.inc"
static uint32_t uart6_gear_pending, uart6_rx_active=1U;
static uint32_t pause_calls, resume_calls, pclk2=108000000U;
static HAL_StatusTypeDef pause_status=HAL_OK, resume_status=HAL_OK;
static struct { uint32_t CCR; } adc_common={0x10000U};
#define ADC (&adc_common)
#define ADC_CCR_ADCPRE 0x30000U
#define ADC_CLOCK_SYNC_PCLK_DIV4 0x10000U
#define DWT_CTRL_CYCCNTENA_Msk 1U
static uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2; }
static HAL_StatusTypeDef Offline_UARTPause(void)
{ pause_calls++; uart6_rx_active=0U; return pause_status; }
static HAL_StatusTypeDef Offline_UARTResume(void)
{ resume_calls++; uart6_rx_active=(resume_status==HAL_OK)?1U:0U; return resume_status; }
#include "offline_under_test.inc"
static void off_setup(void)
{
  setup(); uart6_gear_pending=0U; uart6_rx_active=1U; uart6_initialized=1U;
  pause_calls=0U; resume_calls=0U; pause_status=HAL_OK; resume_status=HAL_OK;
  pclk2=108000000U; adc_common.CCR=0x10000U; SystemCoreClock=216000000U;
  CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
}
static void begin(uint32_t groups)
{
  CHECK(Acquisition_RequestOfflineGEAR2(groups)==HAL_OK);
  CHECK(acquisition_active_gear==0U || acquisition_active_gear==1U);
  CHECK(Acquisition_OfflinePoll()==1U && offline.state==OFFLINE_RUNNING);
  CHECK(offline.start_ok==1U && offline.start_fail==0U && uart6_rx_active==0U);
  CHECK(acquisition_active_gear==2U && acquisition_adc_active_cycles==144U);
  CHECK(timer2.PSC==107U && timer2.ARR==19U && dma.NDTR==200U);
  CHECK((adc3.SMPR2&7U)==6U && ((adc3.SMPR2>>24U)&7U)==6U);
  CHECK(digital_sync.timing.max_phase_us==4U && digital_sync.timing.max_body_us==4U);
  CHECK(digital_sync.next_index==0U && full_snapshot_valid==0U);
  timer2.CNT=1U;
}
static void points(uint32_t n)
{
  uint32_t i;
  for(i=0U;i<n;i++)
  {
    generate(1U,0U); fake_dwt.CYCCNT+=4320U;
    if(pair_index==50U || pair_index==0U)
    {
      fake_dma2.LISR=(pair_index==50U)?DMA_LISR_HTIF0:DMA_LISR_TCIF0;
      DMA2_Stream0_IRQHandler();
    }
  }
  model_tick+=(n*20U)/1000U;
}
static void restored(uint32_t result)
{
  CHECK(offline.state==OFFLINE_DONE && offline.result==result);
  CHECK(offline.restore_ok==1U && offline.restore_fail==0U && uart6_rx_active==1U);
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==480U);
  CHECK(adc_running==1U && acquisition_fault_latched==0U && timer2.ARR==199U);
  CHECK((offline.adc_smpr2_after_restore&7U)==7U);
  CHECK(((offline.adc_smpr2_after_restore>>24U)&7U)==7U);
  CHECK(offline.adc_sqr3_after_restore==(8U<<5U));
  CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U);
  CHECK(acquisition_requested_gear==0U && acquisition_adc_requested_cycles==480U);
}

#ifndef OFFLINE_TEST_MAIN
#define OFFLINE_TEST_MAIN main
#endif
int OFFLINE_TEST_MAIN(void)
{
  uint32_t i,j,seq,starts,configs,group;
  const DigitalSyncTimingConfig *cfg=DigitalSync_GetOfflineTimingConfig();
  CHECK(cfg!=NULL && cfg->sample_interval_us==20U && cfg->max_phase_us==4U && cfg->max_body_us==4U);
  CHECK(DigitalSync_GetTimingConfigForADC(2U,144U)==DigitalSync_GetTimingConfig(2U));
  CHECK(sizeof(offline_samples)==32U*sizeof(Full_Snapshot));
  off_setup(); CHECK(offline_request_groups==0U && offline.state==OFFLINE_IDLE);
  configs=config_count;
  CHECK(Acquisition_OfflinePoll()==0U && config_count==configs);
  CHECK(Acquisition_RequestGear(2U)==HAL_ERROR);
  CHECK(Acquisition_ApplyConfiguration(2U,144U)==HAL_ERROR && config_count==configs);
  CHECK(Acquisition_RequestOfflineGEAR2(0U)==HAL_ERROR);
  CHECK(Acquisition_RequestOfflineGEAR2(1001U)==HAL_ERROR);
  CHECK(Acquisition_RequestOfflineGEAR2(UINT32_MAX)==HAL_ERROR);
  offline_request_groups=1001U; CHECK(Acquisition_OfflinePoll()==0U && config_count==configs);
  CHECK(offline.rejected==4U && acquisition_active_gear==0U);
  for(i=0U;i<8U;i++)
  {
    off_setup();
    if(i==0U) { adc_error=1U; }
    if(i==1U) { digital_sync.error=1U; }
    if(i==2U) { dma_error=1U; }
    if(i==3U) { uart6_rx_active=0U; }
    if(i==4U) { uart6_gear_pending=1U; }
    if(i==5U) { acquisition_adc_request_pending=1U; }
    if(i==6U) { acquisition_request_pending=1U; }
    if(i==7U) { acquisition_adc_active_cycles=144U; }
    CHECK(Acquisition_RequestOfflineGEAR2(10U)==HAL_BUSY && config_count==1U);
  }
  off_setup(); pclk2=54000000U; CHECK(Acquisition_RequestOfflineGEAR2(1U)==HAL_OK);
  CHECK(Acquisition_OfflinePoll()==1U && offline.result==OFFLINE_CLOCK && adc_running==1U && pause_calls==0U);
  for(i=0U;i<2U;i++)
  {
    off_setup(); if(i) { switch_to(1U); }
    ch1_uart_next_seq=0xFFFFU; seq=ch1_uart_next_seq; begin(40U);
    CHECK(Acquisition_RequestGear(0U)==HAL_BUSY && Acquisition_RequestADCSampling(480U)==HAL_BUSY);
    CHECK(Acquisition_RequestOfflineGEAR2(1U)==HAL_BUSY);
    points(49U); CHECK(Acquisition_OfflinePoll()==1U && offline.consumed==0U && full_snapshot_valid==0U);
    points(1U); CHECK(Acquisition_OfflinePoll()==1U && offline.consumed==1U);
    CHECK(offline_samples[0].gear==2U && offline_samples[0].group_count==1U);
    for(j=0U;j<50U;j++)
    {
      CHECK(offline_samples[0].ch1[j]==j+1U && offline_samples[0].ch2[j]==j+1001U);
      CHECK(offline_samples[0].dig[j]==pattern(j));
    }
    CHECK(Acquisition_OfflinePoll()==1U && offline.consumed==1U);
    for(j=1U;j<40U;j++) { points(50U); CHECK(Acquisition_OfflinePoll()==1U); }
    restored(OFFLINE_OK);
    CHECK(offline.published==40U && offline.analog_published==40U && offline.consumed==40U);
    CHECK(offline.half_callbacks==20U && offline.full_callbacks==20U && offline.dma_irqs==40U);
    CHECK(offline.retained==32U && offline.retention_dropped==8U && offline.unconsumed==0U);
    /* Scripted DMA dispatch adds 40 cycles per group in this stub. */
    CHECK(offline.skipped==0U && offline.nominal_rate_groups==999U);
    CHECK(offline.elapsed_cycles==8641600U && (offline.data_seq&1U)==0U);
    CHECK(ch1_uart_next_seq==seq && transmitted==0U && ch1_uart_tx_count==0U);
    CHECK(offline_samples[0].ch1[0]==1U && offline_samples[31].ch1[0]==1551U);
    CHECK(Acquisition_OfflinePoll()==0U); complete_and_send(0U,(uint16_t)seq);
    switch_to(1U); complete_and_send(1U,0U);
  }
  /* Latest-only overwrites and 32-bit group/publication/callback rollover. */
  off_setup(); adc_group_count=UINT32_MAX-1U; digital_sync.completed_groups=adc_group_count;
  full_group_publish_count=UINT32_MAX-1U;
  adc_half_callback_count=UINT32_MAX; adc_full_callback_count=UINT32_MAX; dma2_stream0_irq_count=UINT32_MAX;
  begin(4U); points(150U); CHECK(Acquisition_OfflinePoll()==1U);
  CHECK(offline.consumed==1U && offline.skipped==2U && offline.last_group==1U);
  points(50U); CHECK(Acquisition_OfflinePoll()==1U); restored(OFFLINE_OK);
  CHECK(offline.published==4U && offline.consumed==2U && offline.unconsumed==2U);
  CHECK(offline.first_group==1U && offline.last_group==2U && offline.dma_irqs==4U);
  /* Reject concurrent read; do not use stale/partial data. */
  off_setup(); begin(3U); points(50U); full_snapshot_seq++;
  CHECK(Acquisition_OfflinePoll()==1U && offline.snapshot_busy==1U && offline.retained==0U);
  full_snapshot_seq++; CHECK(Acquisition_OfflinePoll()==1U && offline.consumed==1U);
  offline_seen_publish--; CHECK(Acquisition_OfflinePoll()==1U && offline.duplicate_reads==1U && offline.consumed==1U);
  offline_stop_request=1U; CHECK(Acquisition_OfflinePoll()==1U); restored(OFFLINE_CANCELLED);
  /* Timeout is bounded, unsigned tick/cycle subtraction survives wrap. */
  off_setup(); model_tick=UINT32_MAX-4U; fake_dwt.CYCCNT=UINT32_MAX-100U; begin(1U);
  model_tick+=251U; fake_dwt.CYCCNT+=216000U;
  CHECK(Acquisition_OfflinePoll()==1U); restored(OFFLINE_TIMEOUT);
  CHECK(offline.elapsed_ms==251U && offline.elapsed_cycles==216000U && offline.published==0U);
  /* The maximum legal window remains finite and does not grow retained RAM. */
  off_setup(); begin(OFFLINE_MAX_GROUPS);
  for(i=0U;i<OFFLINE_MAX_GROUPS;i++)
  { points(50U); CHECK(Acquisition_OfflinePoll()==1U); }
  restored(OFFLINE_OK);
  CHECK(offline.published==1000U && offline.consumed==1000U && offline.retained==32U);
  CHECK(offline.retention_dropped==968U && offline.skipped==0U && offline.unconsumed==0U);
  CHECK(offline.half_callbacks==500U && offline.full_callbacks==500U && transmitted==0U);
  /* Read phase/NDTR/TIM5 protections remain strict; candidate faults saved. */
  for(i=0U;i<5U;i++)
  {
    off_setup(); begin(3U);
    if(i==0U) { timer2.CNT=5U; points(1U); }
    if(i==1U) { timer5.CNT++; points(1U); }
    if(i==2U) { dma_error=4U; }
    if(i==3U) { adc_error=4U; }
    if(i==4U) { adc_running=0U; }
    CHECK(Acquisition_OfflinePoll()==1U); restored(OFFLINE_RUNTIME);
    CHECK(offline.published==0U && offline.retained==0U && ch1_uart_next_seq==0U);
    if(i<2U) { CHECK(offline.digital_error!=0U); }
  }
  off_setup(); begin(2U); timer2.CNT=4U; points(50U); CHECK(digital_sync.error==0U);
  CHECK(Acquisition_OfflinePoll()==1U && offline.consumed==1U);
  inject_copy=1U; points(50U); CHECK(Acquisition_OfflinePoll()==1U); restored(OFFLINE_RUNTIME);
  CHECK((offline.digital_error&DIGITAL_SYNC_ERROR_REUSE)!=0U);
  /* One-shot rank failure recovers; persistent bad readback must latch. */
  for(i=1U;i<=2U;i++)
  {
    off_setup(); adc_config_fail_rank=i; adc_config_fail_once=1U;
    CHECK(Acquisition_RequestOfflineGEAR2(3U)==HAL_OK && Acquisition_OfflinePoll()==1U);
    restored(OFFLINE_START); CHECK(offline.start_fail==1U && offline.start_ok==0U);
    CHECK(offline.start_switch_result==ACQ_SWITCH_ADC_CONFIG && ch1_uart_next_seq==0U);
    off_setup(); adc_bad_sample_rank=i;
    CHECK(Acquisition_RequestOfflineGEAR2(3U)==HAL_OK && Acquisition_OfflinePoll()==1U);
    CHECK(offline.state==OFFLINE_FAULT && offline.result==OFFLINE_RESTORE && adc_running==0U);
  }
  for(i=1U;i<=2U;i++)
  {
    off_setup(); adc_bad_rank=i;
    CHECK(Acquisition_RequestOfflineGEAR2(1U)==HAL_OK && Acquisition_OfflinePoll()==1U);
    CHECK(offline.state==OFFLINE_FAULT && offline.start_switch_result==ACQ_SWITCH_ADC_CONFIG);
    CHECK(offline.restore_ok==0U && adc_running==0U);
  }
  /* All start/stop/timer/counter failures: restoration once or latched stop. */
  for(i=ACQ_SWITCH_TIM2_STOP;i<=ACQ_SWITCH_TIM2_START;i++)
  {
    off_setup(); failure=i; fail_once=1U;
    CHECK(Acquisition_RequestOfflineGEAR2(2U)==HAL_OK && Acquisition_OfflinePoll()==1U);
    CHECK(offline.start_fail==1U && offline.start_switch_result==i);
    if(i<=ACQ_SWITCH_TIM5_STOP) { CHECK(offline.state==OFFLINE_FAULT && adc_running==0U); }
    else { restored(OFFLINE_START); }
    configs=adc_config_count; starts=adc_start_count;
    CHECK(Acquisition_OfflinePoll()==(offline.state==OFFLINE_FAULT?1U:0U));
    CHECK(adc_config_count==configs && adc_start_count==starts);
  }
  for(i=ACQ_SWITCH_TIM2_STOP;i<=ACQ_SWITCH_TIM2_START;i++)
  {
    off_setup(); begin(1U); points(50U); failure=i;
    CHECK(Acquisition_OfflinePoll()==1U && offline.state==OFFLINE_FAULT);
    CHECK(adc_running==0U && acquisition_fault_latched==1U && offline.restore_ok==0U);
    configs=adc_config_count; starts=adc_start_count;
    for(j=0U;j<3U;j++) { CHECK(Acquisition_OfflinePoll()==1U); }
    CHECK(adc_config_count==configs && adc_start_count==starts && transmitted==0U);
  }
  off_setup(); pause_status=HAL_TIMEOUT;
  CHECK(Acquisition_RequestOfflineGEAR2(1U)==HAL_OK && Acquisition_OfflinePoll()==1U);
  CHECK(offline.state==OFFLINE_FAULT && offline.result==OFFLINE_UART_PAUSE && adc_running==0U);
  off_setup(); begin(1U); points(50U); resume_status=HAL_ERROR;
  CHECK(Acquisition_OfflinePoll()==1U && offline.state==OFFLINE_FAULT && adc_running==0U);
  CHECK(offline.result==OFFLINE_UART_RESUME && offline.restore_ok==0U && ch1_uart_next_seq==0U);
  /* A stale pending DMA flag cannot be ignored by the isolated arm. */
  off_setup(); fake_dma2.LISR=DMA_LISR_HTIF0;
  CHECK(Acquisition_RequestOfflineGEAR2(1U)==HAL_OK && Acquisition_OfflinePoll()==1U);
  CHECK(offline.state==OFFLINE_FAULT && offline.start_switch_result==ACQ_SWITCH_CLEANUP);
  /* Repeated experiments retain explicit, bounded ownership and fresh data. */
  off_setup();
  for(i=0U;i<3U;i++)
  {
    group=adc_group_count; CHECK(Acquisition_RequestOfflineGEAR2(1U)==HAL_OK);
    CHECK(Acquisition_OfflinePoll()==1U); timer2.CNT=1U; points(50U);
    CHECK(Acquisition_OfflinePoll()==1U && offline.state==OFFLINE_DONE);
    CHECK(offline.published==1U && offline.retained==1U && offline_samples[0].group_count==group+1U);
  }
  CHECK(offline.start_ok==3U && offline.restore_ok==3U);
  printf("PASS offline GEAR2: %u checks; finite isolation, both ranks, coherent halves, retention, wraps, failures, recovery480, SEQ and production regressions; simulated only.\n",checks);
  return 0;
}
