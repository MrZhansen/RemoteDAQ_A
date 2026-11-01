/* Controlled ADC sample-time mode: actual acquisition/IRQ/snapshot/UART code
 * with HAL stubs. These checks do not measure ADC accuracy or MCU latency. */
#define ADC_SAMPLING_TEST
#include "test_gear.c"

static void check_adc_mode(uint32_t cycles)
{
  uint32_t encoding=(cycles==144U)?6U:7U;
  CHECK(acquisition_adc_active_cycles==cycles && adc_running==1U);
  CHECK(acquisition_adc_test_mode==((cycles==144U)?1U:0U));
  CHECK((adc3.SMPR2&7U)==encoding && ((adc3.SMPR2>>24U)&7U)==encoding);
  CHECK((adc3.SQR3&31U)==0U && ((adc3.SQR3>>5U)&31U)==8U);
  CHECK(digital_sync.timing.max_phase_us==((cycles==144U)?4U:20U));
  CHECK(digital_sync.timing.sample_interval_us==((acquisition_active_gear==0U)?200U:100U));
  CHECK(timer2.PSC==107U && timer2.ARR==((acquisition_active_gear==0U)?199U:99U));
}
static void request_adc(uint32_t cycles)
{
  uint32_t previous=acquisition_adc_active_cycles;
  CHECK(Acquisition_RequestADCSampling(cycles)==HAL_OK);
  CHECK(acquisition_adc_active_cycles==previous); /* Deferred until main service. */
  Acquisition_GearPoll();
  CHECK(acquisition_adc_last_switch_result==ACQ_SWITCH_OK);
  CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U && ch1_frame_preview_valid==0U);
  CHECK(digital_sync.next_index==0U && digital_sync.last_trigger==0U && dma.NDTR==200U);
  check_adc_mode(cycles);
}
int main(void)
{
  uint32_t i,j,before,starts,base_group,samples,failures;
  Full_Snapshot out,unchanged;
  const DigitalSyncTimingConfig *fast=DigitalSync_GetTimingConfigForADC(0U,144U);
  static const uint32_t invalid[]={0U,143U,145U,479U,481U,UINT32_MAX};
  CHECK(fast!=NULL && fast->sample_interval_us==200U && fast->max_phase_us==4U);
  CHECK(fast->max_body_us==20U);
  CHECK(DigitalSync_GetTimingConfigForADC(1U,144U)==NULL);
  CHECK(DigitalSync_GetTimingConfigForADC(2U,144U)==DigitalSync_GetTimingConfig(2U));
  CHECK(DigitalSync_GetTimingConfigForADC(2U,480U)==NULL);
  CHECK(DigitalSync_GetTimingConfigForADC(3U,28U)==DigitalSync_GetTimingConfig(3U));
  CHECK(DigitalSync_GetTimingConfigForADC(4U,3U)==DigitalSync_GetTimingConfig(4U));
  CHECK(DigitalSync_GetTimingConfigForADC(0U,480U)==DigitalSync_GetTimingConfig(0U));
  CHECK(DigitalSync_GetTimingConfigForADC(1U,480U)==DigitalSync_GetTimingConfig(1U));
  setup();
  CHECK(acquisition_adc_requested_cycles==480U && acquisition_adc_test_mode==0U);
  CHECK(acquisition_adc_request_pending==0U && adc_config_count==0U);
  CHECK(Acquisition_ApplyGear(0U)==HAL_OK); check_adc_mode(480U);
  CHECK(adc_config_count==2U && adc_start_count==1U);
  CHECK(acquisition_adc_switch_ok_count==0U && acquisition_adc_switch_fail_count==0U);
  complete_and_send(0U,0U);
  CHECK(Full_GetSnapshot(&out)==HAL_OK);
  generate(17U,0U); samples=digital_sync.sample_count; base_group=adc_group_count;
  request_adc(144U);
  CHECK(acquisition_adc_switch_ok_count==1U && acquisition_adc_switch_fail_count==0U);
  CHECK(adc_group_count==base_group && digital_sync.sample_count==samples && ch1_uart_next_seq==1U);
  memset(&out,0x5A,sizeof(out)); memcpy(&unchanged,&out,sizeof(out));
  CHECK(Full_GetSnapshot(&out)==HAL_BUSY && memcmp(&out,&unchanged,sizeof(out))==0);
  complete_and_send(0U,1U); /* New point zero; old 17 points cannot leak in. */
  CHECK(adc_group_count==base_group+1U && digital_sync.sample_count==samples+50U);
  generate(49U,0U); request_adc(480U);
  CHECK(acquisition_adc_switch_ok_count==2U && ch1_uart_next_seq==2U);
  complete_and_send(0U,2U);
  switch_to(1U); check_adc_mode(480U); complete_and_send(1U,3U);
  switch_to(0U); check_adc_mode(480U); complete_and_send(0U,4U);
  CHECK(ug_count==config_count);
  /* Healthy acquisition cannot be reconfigured through the low-level helper. */
  for(i=0U;i<4U;i++)
  {
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
    CHECK(Acquisition_Stop()==ACQ_SWITCH_OK);
    if(i==0U) { adc_running=1U; }
    if(i==1U) { timer2.CR1=1U; }
    if(i==2U) { adc3.CR2=1U; }
    if(i==3U) { dma.CR=1U; }
    before=adc_config_count;
    CHECK(Acquisition_ConfigureADC(144U)==HAL_BUSY && adc_config_count==before);
  }
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_adc(144U);
  before=config_count;
  CHECK(Acquisition_RequestGear(1U)==HAL_ERROR);
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_ADC_CONFLICT && config_count==before);
  acquisition_requested_gear=1U; Acquisition_GearPoll(); /* Watch bypass is also checked. */
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_ADC_CONFLICT && config_count==before);
  check_adc_mode(144U);
  CHECK(Acquisition_RequestGear(0U)==HAL_OK); Acquisition_GearPoll();
  request_adc(480U); switch_to(1U); check_adc_mode(480U);
  before=config_count;
  CHECK(Acquisition_RequestADCSampling(144U)==HAL_ERROR);
  CHECK(acquisition_adc_last_switch_result==ACQ_SWITCH_ADC_CONFLICT && config_count==before);
  acquisition_adc_requested_cycles=144U; acquisition_adc_request_pending=1U; Acquisition_GearPoll();
  CHECK(acquisition_adc_last_switch_result==ACQ_SWITCH_ADC_CONFLICT && config_count==before);
  switch_to(0U); check_adc_mode(480U); /* A rejected request never retries on its own. */
  request_adc(144U);
  /* Conflicting simultaneous Watch requests preserve normal GEAR1/480. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  acquisition_adc_requested_cycles=144U; acquisition_adc_request_pending=1U;
  acquisition_requested_gear=1U; acquisition_request_pending=1U; Acquisition_GearPoll();
  CHECK(acquisition_active_gear==1U && acquisition_adc_last_switch_result==ACQ_SWITCH_ADC_CONFLICT);
  check_adc_mode(480U);
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  CHECK(Acquisition_RequestADCSampling(144U)==HAL_OK);
  CHECK(Acquisition_RequestGear(1U)==HAL_ERROR); Acquisition_GearPoll(); check_adc_mode(144U);
  /* Input validation and no automatic repeated failure attempts. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  before=adc_config_count;
  for(i=0U;i<sizeof(invalid)/sizeof(invalid[0]);i++)
  {
    CHECK(Acquisition_RequestADCSampling(invalid[i])==HAL_ERROR);
    acquisition_adc_requested_cycles=invalid[i]; acquisition_adc_request_pending=1U;
    Acquisition_GearPoll(); failures=acquisition_adc_switch_fail_count;
    CHECK(acquisition_adc_last_switch_result==ACQ_SWITCH_ADC_CYCLES && adc_config_count==before);
    Acquisition_GearPoll(); CHECK(acquisition_adc_switch_fail_count==failures);
    check_adc_mode(480U);
  }
  CHECK(Acquisition_RequestGear(2U)==HAL_ERROR);
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_UDP_REQUIRED && adc_config_count==before);
  CHECK(Acquisition_ApplyConfiguration(2U,144U)==HAL_ERROR && adc_config_count==before);
  /* The short check accepts CNT 4 and rejects 5; point/trigger tests stay strict. */
  for(i=0U;i<3U;i++)
  {
    static const uint32_t phase[]={4U,5U,20U};
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_adc(144U);
    timer2.CNT=phase[i]; generate(1U,0U);
    CHECK((digital_sync.error==0U)==(i==0U));
    CHECK(digital_sync.sample_count==((i==0U)?1U:0U));
    if(i!=0U) { CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_PHASE); }
  }
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_adc(144U);
  timer2.CNT=1U; timer5.CNT=1U; dma.NDTR=198U; timer2.SR=1U; TIM2_IRQHandler();
  CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_NDTR_INDEX && digital_sync.sample_count==0U);
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_adc(144U);
  timer2.CNT=1U; timer5.CNT=2U; timer2.SR=1U; TIM2_IRQHandler();
  CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_TRIGGER && digital_sync.sample_count==0U);
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_adc(144U);
  timing_phases[0]=198U; timing_phases[1]=199U; timing_phases[2]=0U; timing_phases[3]=1U;
  timing_phase_count=4U; generate(1U,0U);
  CHECK(digital_sync.fault_detail==DIGITAL_SYNC_DETAIL_CNT_WRAP && digital_sync.sample_count==0U);
  /* Both halves keep analog/digital order and protocol SEQ/CRC in test mode. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); request_adc(144U); timer2.CNT=1U;
  for(i=0U;i<4U;i++)
  {
    generate(50U,1U); CHECK(Full_GetSnapshot(&out)==HAL_OK && out.group_count==i+1U && out.gear==0U);
    CHECK(out.group_count==digital_sync.completed_groups && digital_sync.error==0U);
    for(j=0U;j<50U;j++)
    {
      CHECK(out.ch1[j]==i*50U+j+1U && out.ch2[j]==i*50U+j+1001U);
      CHECK(out.dig[j]==pattern(i*50U+j));
    }
    CHECK(CH1_FrameTransmitPoll()==1U && ch1_uart_last_seq==i && ch1_uart_next_seq==i+1U);
    CHECK(CH1_FrameTransmitPoll()==0U);
  }
  CHECK(adc_half_callback_count==2U && adc_full_callback_count==2U);
  /* HAL failure on either rank, or incorrect register readback, must not arm. */
  for(i=0U;i<3U;i++) for(j=1U;j<=2U;j++)
  {
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); complete_and_send(0U,0U);
    starts=adc_start_count;
    adc_config_fail_rank=(i==0U)?j:0U;
    adc_bad_sample_rank=(i==1U)?j:0U; adc_bad_rank=(i==2U)?j:0U;
    CHECK(Acquisition_RequestADCSampling(144U)==HAL_OK); Acquisition_GearPoll();
    CHECK(acquisition_adc_last_switch_result==ACQ_SWITCH_ADC_CONFIG);
    CHECK(acquisition_adc_switch_fail_count==1U && acquisition_adc_switch_ok_count==0U);
    CHECK(adc_running==0U && acquisition_fault_latched==1U && irq_enable==0U);
    CHECK(acquisition_adc_active_cycles==ACQ_GEAR_INVALID && acquisition_adc_test_mode==0U);
    CHECK(full_snapshot_valid==0U && adc_snapshot_valid==0U && ch1_frame_preview_valid==0U);
    CHECK(adc_start_count==starts && (timer2.CR1&1U)==0U && dma.CR==0U && adc3.CR2==0U);
    CHECK(ch1_uart_next_seq==1U && CH1_FrameTransmitPoll()==0U);
    if(i==0U) { CHECK(((j==1U)?acquisition_adc_rank1_status:acquisition_adc_rank2_status)!=HAL_OK); }
    before=adc_config_count; Acquisition_GearPoll(); CHECK(adc_config_count==before);
  }
  /* Every existing stop/config/start failure is also safe on an ADC request. */
  for(i=ACQ_SWITCH_TIM2_STOP;i<=ACQ_SWITCH_TIM2_START;i++)
  {
    setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK); complete_and_send(0U,0U);
    failure=i;
    CHECK(Acquisition_RequestADCSampling(144U)==HAL_OK); Acquisition_GearPoll();
    CHECK(acquisition_adc_last_switch_result==i && acquisition_adc_switch_fail_count==1U);
    CHECK(adc_running==0U && acquisition_fault_latched==1U && irq_enable==0U);
    CHECK(full_snapshot_valid==0U && acquisition_adc_active_cycles==ACQ_GEAR_INVALID);
    CHECK(CH1_FrameTransmitPoll()==0U && ch1_uart_next_seq==1U);
    before=adc_config_count; starts=adc_start_count; Acquisition_GearPoll();
    CHECK(adc_config_count==before && adc_start_count==starts);
  }
  printf("PASS ADC sampling experiment: %u checks; explicit 480/144/480, stopped pair configuration, readback/HAL failures, short phase boundary, group/SEQ/CRC/UART continuity, conflicts and no retries.\n",checks);
  return 0;
}
