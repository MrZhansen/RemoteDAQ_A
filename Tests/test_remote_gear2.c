/* Actual production policy + stopped HAL configuration, using register/time
 * fixtures. No board, serial connection or physical timing assertion. */
#define UDP_TRIAL_TEST_MAIN udp_trial_fixture_main
#include "test_udp_trial.c"
#include "udp_production_policy.inc"

static void remote_setup(void)
{
  trial_setup();
  daq_net.identity_status=DAQ_NET_ID_ES0290_LISTED;
  Acquisition_UDPPolicyPoll();
  CHECK(acquisition_udp_gear2_available==1U);
}
static void remote_enter(void)
{
  uint32_t configs=config_count;
  CHECK(Acquisition_RequestGear(2U)==HAL_OK);
  CHECK(acquisition_active_gear<2U && config_count==configs);
  Acquisition_UDPPolicyPoll(); Acquisition_GearPoll();
  CHECK(acquisition_active_gear==2U && acquisition_udp_gear2_active==1U);
  CHECK(acquisition_adc_active_cycles==144U && adc_running==1U);
  CHECK(timer2.PSC==107U && timer2.ARR==19U && adc3.SQR3==(8U<<5U));
  CHECK((adc3.SMPR2&7U)==6U && ((adc3.SMPR2>>24U)&7U)==6U);
  CHECK(digital_sync.timing.sample_interval_us==20U);
  CHECK(digital_sync.timing.max_phase_us==4U && digital_sync.timing.max_body_us==4U);
  CHECK(full_snapshot_valid==0U && udp_trial.state==UDP_TRIAL_IDLE && offline.state==OFFLINE_IDLE);
}
int main(void)
{
  uint32_t before,from,to,i;
  Full_Snapshot snap;
  uint8_t frame[260];
  const uint32_t failures[]={ACQ_SWITCH_TIM2_STOP,ACQ_SWITCH_ADC_STOP,
    ACQ_SWITCH_TIM5_STOP,ACQ_SWITCH_TIM2_CONFIG,ACQ_SWITCH_ADC_START,
    ACQ_SWITCH_TIM5_START,ACQ_SWITCH_TIM2_START};
  for(from=0U;from<2U;from++) for(to=0U;to<2U;to++)
  {
    remote_setup();
    CHECK(Acquisition_RequestGear(from)==HAL_OK); Acquisition_GearPoll();
    ch1_uart_next_seq=65535U;
    remote_enter();
    CHECK(ch1_uart_next_seq==65535U);
    CHECK(Acquisition_RequestADCSampling(480U)==HAL_BUSY);
    points(49U); CHECK(Full_GetSnapshot(&snap)==HAL_BUSY);
    points(1U); CHECK(Full_GetSnapshot(&snap)==HAL_OK && snap.gear==2U);
    CHECK(DAQ_BuildUDPGEAR2CandidateFrame(snap.ch1,snap.ch2,snap.dig,31U,
      65535U,frame,sizeof(frame))==DAQ_FRAME_OK);
    CHECK(frame[5]==2U && frame[6]==31U && frame[7]==50U);
    CHECK(reference_crc(frame,258U)==(uint16_t)(frame[258]|((uint16_t)frame[259]<<8U)));
    before=config_count;
    CHECK(Acquisition_RequestGear(2U)==HAL_OK); Acquisition_GearPoll();
    CHECK(config_count==before && adc_running==1U);
    CHECK(Acquisition_RequestGear(to)==HAL_OK); Acquisition_GearPoll();
    CHECK(acquisition_active_gear==to && acquisition_adc_active_cycles==480U);
    CHECK(acquisition_udp_gear2_active==0U && full_snapshot_valid==0U);
    CHECK((adc3.SMPR2&7U)==7U && ((adc3.SMPR2>>24U)&7U)==7U);
    CHECK(timer2.ARR==((to==0U)?199U:99U));
    CHECK(ch1_uart_next_seq==65535U);
  }
  for(i=0U;i<12U;i++)
  {
    remote_setup(); before=config_count;
    if(i==0U) daq_transport_active=0U;
    if(i==1U) daq_transport_request=0U;
    if(i==2U) ready=0U;
    if(i==3U) daq_net.identity_status=DAQ_NET_ID_REV_A_UNQUALIFIED;
    if(i==4U) daq_net.identity_status=DAQ_NET_ID_UNKNOWN_REVISION;
    if(i==5U) daq_net.identity_status=DAQ_NET_ID_WRONG_DEVICE;
    if(i==6U) adc_running=0U;
    if(i==7U) acquisition_fault_latched=1U;
    if(i==8U) adc_error=1U;
    if(i==9U) adc_hal_error=1U;
    if(i==10U) dma_error=1U;
    if(i==11U) digital_sync.error=DIGITAL_SYNC_ERROR_ALIGN;
    Acquisition_UDPPolicyPoll();
    CHECK(acquisition_udp_gear2_available==0U);
    CHECK(Acquisition_RequestGear(2U)==HAL_ERROR && config_count==before);
    CHECK(acquisition_active_gear==0U && adc_running==((i==6U)?0U:1U));
  }
  remote_setup(); before=config_count;
  CHECK(Acquisition_RequestGear(2U)==HAL_OK); pclk2=54000000U;
  Acquisition_UDPPolicyPoll(); Acquisition_GearPoll();
  CHECK(acquisition_active_gear==0U && config_count==before && adc_running==1U);
  remote_setup(); CHECK(Acquisition_RequestADCSampling(144U)==HAL_OK);
  Acquisition_ADCSamplingPoll();
  CHECK(Acquisition_RequestGear(2U)==HAL_ERROR && acquisition_active_gear==0U);
  remote_setup(); remote_enter(); ready=0U;
  Acquisition_UDPPolicyPoll(); Acquisition_UDPPolicyPoll();
  CHECK(acquisition_udp_gear2_return_count==1U && acquisition_request_pending==1U);
  Acquisition_GearPoll();
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==480U && adc_running==1U);
  remote_setup(); remote_enter(); daq_transport_request=0U;
  Acquisition_UDPPolicyPoll(); Acquisition_GearPoll();
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==480U);
  remote_setup(); remote_enter(); digital_sync.error=DIGITAL_SYNC_ERROR_ALIGN;
  Acquisition_UDPPolicyPoll(); before=config_count; Acquisition_UDPPolicyPoll();
  CHECK(acquisition_fault_latched==1U && adc_running==0U && config_count==before);
  CHECK(acquisition_udp_gear2_fault_count==1U);
  for(i=0U;i<sizeof(failures)/sizeof(failures[0]);i++)
  {
    remote_setup(); failure=failures[i];
    CHECK(Acquisition_RequestGear(2U)==HAL_OK);
    Acquisition_UDPPolicyPoll(); Acquisition_GearPoll();
    CHECK(adc_running==0U && acquisition_fault_latched==1U);
    CHECK(acquisition_udp_gear2_active==0U && full_snapshot_valid==0U);
  }
  remote_setup(); remote_enter(); adc_config_fail_rank=2U;
  CHECK(Acquisition_RequestGear(0U)==HAL_OK); Acquisition_GearPoll();
  CHECK(adc_running==0U && acquisition_fault_latched==1U && acquisition_udp_gear2_active==0U);
  printf("PASS actual UDP-only production GEAR2 policy/config/restore: %u checks (host only)\n",checks);
  return 0;
}
