/* Actual continuous owner, stopped reconfiguration, IRQs, snapshots and
 * shared encoder. Register/time models only, no physical timing or network. */
#define OFFLINE_TEST_MAIN udp_offline_legacy_main
#include "test_offline_gear2.c"
#include "daq_network.h"
volatile DAQ_NetDiagnostics daq_net;
static uint32_t daq_transport_active,daq_transport_request,ready=1U,queued_status;
uint32_t DAQ_NetReady(void) { return ready; }
static void UART6_QueueStatus(void) { queued_status++; }
#include "udp_trial_under_test.inc"
static void trial_setup(void)
{
  memset((void *)&udp_trial,0,sizeof(udp_trial));
  memset((void *)&daq_net,0,sizeof(daq_net));
  off_setup(); daq_transport_active=1U; daq_transport_request=1U;
  uart6_rx_active=0U; ready=1U; queued_status=0U;
}
static void trial_start(void)
{
  udp_trial.arm=UDP_TRIAL_ARM_MAGIC; udp_trial.request=1U;
  DAQ_UDPTrialPoll();
  CHECK(udp_trial.state==UDP_TRIAL_RUNNING && udp_trial.start_ok==1U);
  CHECK(acquisition_active_gear==2U && acquisition_adc_active_cycles==144U);
  CHECK(timer2.PSC==107U && timer2.ARR==19U);
  CHECK((adc3.SMPR2&7U)==6U && ((adc3.SMPR2>>24U)&7U)==6U);
  CHECK(full_snapshot_valid==0U && digital_sync.timing.max_phase_us==4U);
  CHECK(digital_sync.timing.max_body_us==4U && offline.state==OFFLINE_IDLE);
  timer2.CNT=1U;
}
static void trial_restored(void)
{
  CHECK(udp_trial.state==UDP_TRIAL_DONE && udp_trial.restore_ok==1U);
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==480U && adc_running==1U);
  CHECK(timer2.PSC==107U && timer2.ARR==199U);
  CHECK((adc3.SMPR2&7U)==7U && ((adc3.SMPR2>>24U)&7U)==7U);
  CHECK(adc3.SQR3==(8U<<5U) && digital_sync.error==0U);
  CHECK(full_snapshot_valid==0U && udp_trial.arm==0U && udp_trial.request==0U);
}
#ifndef UDP_TRIAL_TEST_MAIN
#define UDP_TRIAL_TEST_MAIN main
#endif
int UDP_TRIAL_TEST_MAIN(void)
{
  uint32_t i, mask, seq, configs, uart_calls, group, old;
  uint8_t frame[260],saved[260]; Full_Snapshot snapshot;
  static const uint32_t failures[]={ACQ_SWITCH_TIM2_STOP,ACQ_SWITCH_ADC_STOP,ACQ_SWITCH_TIM5_STOP,
      ACQ_SWITCH_TIM2_CONFIG,ACQ_SWITCH_ADC_START,ACQ_SWITCH_TIM5_START,ACQ_SWITCH_TIM2_START};
  trial_setup(); configs=config_count; DAQ_UDPTrialPoll();
  CHECK(udp_trial.state==0U && config_count==configs && acquisition_active_gear==0U);
  CHECK(Acquisition_RequestGear(2U)==HAL_ERROR);
  CHECK(DigitalSync_GetTimingConfig(2U)!=NULL &&
      DigitalSync_GetTimingConfig(2U)->adc_sample_cycles==144U);
  CHECK(Acquisition_ApplyConfiguration(2U,144U)==HAL_ERROR);
  for(i=0U;i<5U;i++)
  {
    trial_setup(); udp_trial.request=1U; udp_trial.arm=UDP_TRIAL_ARM_MAGIC;
    if(i==0U) { udp_trial.arm=0U; }
    if(i==1U) { ready=0U; }
    if(i==2U) { daq_transport_active=0U; }
    if(i==3U) { uart6_rx_active=1U; }
    if(i==4U) { pclk2=54000000U; }
    configs=config_count; DAQ_UDPTrialPoll();
    CHECK(udp_trial.rejected==1U && udp_trial.request==0U && config_count==configs);
    CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==480U);
  }
  trial_setup(); trial_start();
  CHECK(Acquisition_RequestGear(0U)==HAL_BUSY && Acquisition_RequestGear(2U)==HAL_BUSY);
  CHECK(Acquisition_RequestADCSampling(480U)==HAL_BUSY);
  CHECK(Acquisition_RequestOfflineGEAR2(10U)==HAL_BUSY);
  configs=config_count; acquisition_requested_gear=1U; Acquisition_GearPoll();
  CHECK(config_count==configs && acquisition_active_gear==2U); acquisition_requested_gear=0U;
  points(49U); CHECK(Full_GetSnapshot(&snapshot)==HAL_BUSY);
  points(1U); CHECK(Full_GetSnapshot(&snapshot)==HAL_OK && snapshot.gear==2U);
  group=snapshot.group_count;
  for(mask=0U;mask<32U;mask++)
  {
    CHECK(DAQ_BuildUDPGEAR2CandidateFrame(snapshot.ch1,snapshot.ch2,snapshot.dig,
        mask,65535U,frame,sizeof(frame))==DAQ_FRAME_OK);
    CHECK(frame[5]==2U && frame[6]==mask && frame[7]==50U && frame[3]==255U && frame[4]==255U);
    for(i=0U;i<50U;i++)
    {
      uint32_t o=8U+5U*i;
      uint16_t a=(mask&1U)?(uint16_t)(snapshot.ch1[i]<<4U):0U;
      uint16_t b=(mask&2U)?(uint16_t)(snapshot.ch2[i]<<4U):0U;
      CHECK(frame[o]==(uint8_t)a && frame[o+1U]==(uint8_t)(a>>8U));
      CHECK(frame[o+2U]==(uint8_t)b && frame[o+3U]==(uint8_t)(b>>8U));
      CHECK(frame[o+4U]==(snapshot.dig[i]&((mask>>2U)&7U)));
    }
    CHECK(reference_crc(frame,258U)==(uint16_t)(frame[258]|((uint16_t)frame[259]<<8U)));
  }
  memcpy(saved,frame,260U); snapshot.ch1[7]=65535U;
  CHECK(DAQ_BuildUDPGEAR2CandidateFrame(snapshot.ch1,snapshot.ch2,snapshot.dig,31U,0U,frame,260U)==DAQ_FRAME_ADC_OUT_OF_RANGE);
  CHECK(memcmp(saved,frame,260U)==0); snapshot.ch1[7]=0U;
  CHECK(DAQ_BuildSelectedFrameForGear(snapshot.ch1,snapshot.ch2,snapshot.dig,2U,31U,0U,frame,260U)==DAQ_FRAME_OK && frame[5]==2U);
  seq=ch1_uart_next_seq; uart_calls=transmitted;
  CHECK(CH1_FrameTransmitPoll()==1U && transmitted==uart_calls && ch1_uart_next_seq==seq);
  CHECK(ch1_frame_gear==2U && ch1_preview_work[5]==2U);
  CHECK(CH1_FrameTransmitPoll()==0U);
  points(100U); DAQ_UDPTrialPoll();
  CHECK(udp_trial.published==3U && udp_trial.analog_published==3U);
  CHECK(udp_trial.half_callbacks==2U && udp_trial.full_callbacks==1U);
  CHECK(Full_GetSnapshot(&snapshot)==HAL_OK && snapshot.group_count==group+2U);
  /* Same producer path, latest group and modulo statistics; never infer
   * physical 1000 Hz from scripted timestamps. */
  udp_trial.base_published=UINT32_MAX-1U; full_group_publish_count=1U;
  daq_net.data_submitted=UINT32_MAX; udp_trial.base_submitted=UINT32_MAX-2U;
  udp_trial.start_tick=UINT32_MAX-499U; udp_trial.rate_tick=UINT32_MAX-499U;
  model_tick=500U; DAQ_UDPTrialPoll();
  CHECK(udp_trial.published==3U && udp_trial.submitted==2U && udp_trial.elapsed_ms==1000U);
  CHECK(udp_trial.rate_groups==3U && udp_trial.rate_frames==2U && udp_trial.rate_elapsed_ms==1000U);
  udp_trial.request=0U; DAQ_UDPTrialPoll(); trial_restored();
  CHECK(ch1_uart_next_seq==seq && udp_trial.result==7U && offline.state==OFFLINE_IDLE);
  trial_setup(); trial_start(); ready=0U; DAQ_UDPTrialPoll(); trial_restored(); CHECK(udp_trial.result==8U);
  trial_setup(); trial_start(); digital_sync.error=1U; DAQ_UDPTrialPoll();
  trial_restored(); CHECK(udp_trial.digital_error==1U && udp_trial.result==4U);
  for(i=0U;i<sizeof(failures)/sizeof(failures[0]);i++)
  {
    trial_setup(); failure=failures[i]; fail_once=1U;
    udp_trial.arm=UDP_TRIAL_ARM_MAGIC; udp_trial.request=1U; DAQ_UDPTrialPoll();
    CHECK(udp_trial.start_fail==1U && udp_trial.start_ok==0U);
    if(failures[i]<=ACQ_SWITCH_TIM5_STOP)
    { CHECK(udp_trial.state==UDP_TRIAL_FAULT && acquisition_fault_latched==1U && adc_running==0U); }
    else { trial_restored(); CHECK(udp_trial.result==3U); }
    old=config_count; DAQ_UDPTrialPoll(); CHECK(config_count==old);
  }
  for(i=1U;i<=2U;i++)
  {
    trial_setup(); adc_config_fail_rank=i; adc_config_fail_once=1U;
    udp_trial.arm=UDP_TRIAL_ARM_MAGIC; udp_trial.request=1U; DAQ_UDPTrialPoll();
    CHECK(udp_trial.start_fail==1U); trial_restored();
  }
  /* 1000 complete groups through actual TIM2/DMA publisher and coherent
   * reader. Scripted time is deliberately not physical rate evidence. */
  trial_setup(); points(17U); trial_start();
  for(i=0U;i<1000U;i++)
  {
    points(50U);
    CHECK(Full_GetSnapshot(&snapshot)==HAL_OK && snapshot.group_count==i+1U && snapshot.gear==2U);
    CHECK(Analog_BuildFramePreview((uint16_t)i,2U)==1U);
    CHECK(Analog_BuildFramePreview((uint16_t)i,2U)==0U);
    CHECK(ch1_preview_work[5]==2U && ch1_preview_work[6]==31U);
    CHECK(reference_crc(ch1_preview_work,258U)==(uint16_t)(ch1_preview_work[258]|((uint16_t)ch1_preview_work[259]<<8U)));
    DAQ_UDPTrialPoll();
  }
  CHECK(udp_trial.published==1000U && udp_trial.analog_published==1000U);
  CHECK(udp_trial.half_callbacks==500U && udp_trial.full_callbacks==500U);
  CHECK(udp_trial.consumed==1000U && udp_trial.skipped==0U && udp_trial.submitted==0U);
  CHECK(ch1_uart_next_seq==0U && ch1_uart_tx_count==0U);
  udp_trial.request=0U; DAQ_UDPTrialPoll(); trial_restored();
  CHECK(Analog_BuildFramePreview(0U,2U)==0U); /* Invalidated old candidate. */
  switch_to(1U); complete_and_send(1U,0U); switch_to(0U);
  trial_setup(); trial_start(); failure=ACQ_SWITCH_ADC_STOP; fail_once=1U;
  udp_trial.request=0U; DAQ_UDPTrialPoll();
  CHECK(udp_trial.state==UDP_TRIAL_FAULT && udp_trial.result==6U);
  trial_setup(); adc_bad_sample_rank=2U;
  udp_trial.arm=UDP_TRIAL_ARM_MAGIC; udp_trial.request=1U; DAQ_UDPTrialPoll();
  CHECK(udp_trial.start_ok==0U && udp_trial.start_fail==1U);
  CHECK(udp_trial.state==UDP_TRIAL_FAULT && adc_running==0U);
  trial_setup(); trial_start(); adc_config_fail_rank=2U; udp_trial.request=0U;
  DAQ_UDPTrialPoll(); CHECK(udp_trial.state==UDP_TRIAL_FAULT && udp_trial.restore_fail==1U);
  CHECK(adc_running==0U && acquisition_fault_latched==1U);
  old=adc_config_count; DAQ_UDPTrialPoll(); CHECK(adc_config_count==old);
  printf("PASS continuous UDP GEAR2 candidate: %u checks (simulated registers/time only)\n",checks);
  return 0;
}
