/* Real gear/ADC/DMA/ISR/snapshot/frame code with HAL/register stubs.
 * This verifies host logic and scripted cycle accounting, not board timing. */
#define ADC_SAMPLING_TEST
#include "test_gear.c"

static uint32_t uart_status_calls;
static uint32_t square_wave_running=1U, square_wave_actual_hz=1000U;
static uint32_t daq_transport_active=1U;
static uint32_t UART6_GetStatus(DAQ_StatusState *state)
{
  uart_status_calls++;
  state->gear=acquisition_active_gear;
  state->channel_mask=daq_active_channel_mask;
  state->trigger_source=daq_trigger_source;
  state->trigger_mode=daq_trigger_mode;
  state->trigger_level=daq_trigger_level;
  state->wave_frequency_hz=square_wave_actual_hz;
  return 1U;
}
#include "gear34_status_source.inc"

static void udp_setup(void)
{
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  acquisition_udp_gear2_available=1U;
  uart_status_calls=0U;
  square_wave_running=1U; square_wave_actual_hz=1000U;
  daq_transport_active=1U;
}

static void expect_high_gear(uint32_t gear)
{
  uint32_t expected_cycles=(gear==3U)?28U:3U;
  uint32_t expected_arr=(gear==3U)?9U:1U;
  uint32_t expected_smpr=(gear==3U)?2U:0U;
  uint32_t expected_phase=(gear==3U)?540U:216U;
  uint32_t expected_body=(gear==3U)?972U:390U;
  CHECK(acquisition_active_gear==gear && acquisition_adc_active_cycles==expected_cycles);
  CHECK(acquisition_sample_interval_us==((gear==3U)?10U:2U));
  CHECK(timer2.PSC==107U && timer2.ARR==expected_arr);
  CHECK((adc3.SMPR2&7U)==expected_smpr && ((adc3.SMPR2>>24U)&7U)==expected_smpr);
  CHECK((adc3.SQR3&31U)==ADC_CHANNEL_0 && ((adc3.SQR3>>5U)&31U)==ADC_CHANNEL_8);
  CHECK(acquisition_fast_isr_active==1U);
  CHECK(acquisition_fast_irq_budget_cycles==expected_body);
  CHECK(digital_sync.timing.fast_isr==1U && digital_sync.timing.udp_required==1U);
  CHECK(digital_sync.timing.max_phase_cycles==expected_phase);
  CHECK(digital_sync.timing.max_body_cycles==expected_body);
  CHECK(digital_sync.next_index==0U && digital_sync.error==0U);
  CHECK(dma.NDTR==200U && full_snapshot_valid==0U && adc_snapshot_valid==0U);
}

static void enter_high_gear(uint32_t gear)
{
  uint32_t configs=config_count;
  CHECK(Acquisition_RequestGear(gear)==HAL_OK);
  CHECK(acquisition_request_pending==1U && config_count==configs);
  Acquisition_GearPoll();
  CHECK(config_count==configs+1U);
  expect_high_gear(gear);
}

static void validate_high_frame(uint32_t gear,uint32_t expected_samples,uint16_t sequence)
{
  Full_Snapshot snapshot;
  uint8_t frame[DAQ_DATA_FRAME_SIZE];
  uint16_t crc;
  uint32_t i;
  generate(50U,1U);
  CHECK(Full_GetSnapshot(&snapshot)==HAL_OK && snapshot.gear==gear);
  CHECK(snapshot.group_count==adc_group_count);
  CHECK(digital_sync.sample_count==expected_samples);
  for(i=0U;i<DAQ_ANALOG_SAMPLE_COUNT;i++)
  {
    CHECK(snapshot.ch1[i]==i+1U && snapshot.ch2[i]==i+1001U);
    CHECK(snapshot.dig[i]==pattern(i) && snapshot.dig[i]<8U);
  }
  CHECK(DAQ_BuildSelectedFrameForGear(snapshot.ch1,snapshot.ch2,snapshot.dig,
        gear,0x1FU,sequence,frame,sizeof(frame))==DAQ_FRAME_OK);
  CHECK(frame[0]==0xAAU && frame[1]==0x55U && frame[2]==1U);
  CHECK(frame[3]==(uint8_t)sequence && frame[4]==(uint8_t)(sequence>>8U));
  CHECK(frame[5]==gear && frame[6]==0x1FU && frame[7]==50U);
  crc=reference_crc(frame,DAQ_DATA_CRC_OFFSET);
  CHECK(crc==(uint16_t)(frame[DAQ_DATA_CRC_OFFSET]|
        ((uint16_t)frame[DAQ_DATA_CRC_OFFSET+1U]<<8U)));
}

static void check_fast_budget(uint32_t gear,uint32_t phase,uint32_t body,
                              uint32_t expected_error,uint32_t expected_detail)
{
  udp_setup(); enter_high_gear(gear);
  timing_cycles[0]=100U;
  timing_cycles[1]=100U+phase;
  timing_cycles[2]=100U+body;
  timing_cycle_count=3U;
  generate(1U,0U);
  CHECK(acquisition_fast_phase_last_cycles==phase);
  CHECK(acquisition_fast_irq_last_cycles==body);
  CHECK(digital_sync.error==expected_error);
  if(expected_error!=0U) { CHECK(digital_sync.fault_detail==expected_detail); }
  CHECK(timer2.DIER==((expected_error==0U)?TIM_DIER_UIE:0U));
}

int main(void)
{
  DAQ_StatusState status;
  uint32_t i,groups,samples,configs;
  static const uint32_t gears[]={3U,4U};

  /* High-rate gears cannot start without a healthy live UDP owner. */
  setup(); CHECK(Acquisition_ApplyGear(0U)==HAL_OK);
  configs=config_count;
  for(i=3U;i<=4U;i++)
  {
    CHECK(Acquisition_RequestGear(i)==HAL_ERROR);
    CHECK(acquisition_last_switch_result==ACQ_SWITCH_UDP_REQUIRED);
    CHECK(acquisition_active_gear==0U && config_count==configs && adc_running==1U);
  }
  CHECK(Acquisition_RequestGear(5U)==HAL_ERROR);
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_BAD_GEAR);
  CHECK(acquisition_active_gear==0U && config_count==configs);

  for(i=0U;i<sizeof(gears)/sizeof(gears[0]);i++)
  {
    udp_setup(); enter_high_gear(gears[i]);
    CHECK(DAQ_TransportGetStatus(&status)==1U && status.gear==gears[i]);
    CHECK(status.channel_mask==0x1FU && status.wave_frequency_hz==1000U);
    validate_high_frame(gears[i],50U,(uint16_t)(0x1200U+i));

    /* A repeated request is acknowledged without resetting healthy DMA. */
    configs=config_count; groups=adc_group_count; samples=digital_sync.sample_count;
    CHECK(Acquisition_RequestGear(gears[i])==HAL_OK); Acquisition_GearPoll();
    CHECK(config_count==configs && adc_group_count==groups);
    CHECK(digital_sync.sample_count==samples && adc_running==1U);

    /* Status must use the active profile, not the former GEAR2 144-cycle
     * constant, and must fail closed after an ISR budget violation. */
    acquisition_adc_active_cycles=144U;
    CHECK(DAQ_TransportGetStatus(&status)==0U);
    acquisition_adc_active_cycles=(gears[i]==3U)?28U:3U;
    acquisition_fast_budget_fail_count=1U;
    CHECK(DAQ_TransportGetStatus(&status)==0U);
  }

  /* Switching G3 -> G4 discards a partial digital half and preserves the
   * completed group/SEQ baseline. No stale publication is manufactured. */
  udp_setup(); enter_high_gear(3U);
  generate(17U,0U); groups=adc_group_count; samples=digital_sync.sample_count;
  CHECK(digital_sync.next_index==17U && full_snapshot_valid==0U);
  CHECK(Acquisition_RequestGear(4U)==HAL_OK); Acquisition_GearPoll();
  expect_high_gear(4U);
  CHECK(adc_group_count==groups && digital_sync.sample_count==samples);
  CHECK(digital_sync.completed_groups==groups && full_snapshot_valid==0U);
  validate_high_frame(4U,samples+50U,0x34U);

  /* Restore sequence G4 -> G3 -> G2 -> G1 -> G0 and verify exact rows. */
  {
    static const uint32_t restore[]={3U,2U,1U,0U};
    for(i=0U;i<sizeof(restore)/sizeof(restore[0]);i++)
    {
      uint32_t gear=restore[i];
      CHECK(Acquisition_RequestGear(gear)==HAL_OK); Acquisition_GearPoll();
      CHECK(acquisition_active_gear==gear && adc_running==1U);
      CHECK(acquisition_adc_active_cycles==
            DigitalSync_GetTimingConfig(gear)->adc_sample_cycles);
      CHECK(timer2.PSC==107U &&
            timer2.ARR==(DigitalSync_GetTimingConfig(gear)->sample_interval_us-1U));
    }
  }
  CHECK(acquisition_udp_gear2_active==0U && acquisition_requested_gear==0U);
  CHECK(acquisition_adc_requested_cycles==480U && acquisition_adc_active_cycles==480U);
  CHECK(adc_error==0U && dma_error==0U && digital_sync.error==0U);

  /* The old high-gear admission rule required active/requested ADC cycles to
   * be 480 whenever udp_gear2_active was clear. A healthy high gear is at
   * 28/3 cycles, so only the GEAR0/144 experiment may block a high gear. */
  udp_setup(); enter_high_gear(3U);
  acquisition_udp_gear2_active=0U;
  CHECK(Acquisition_RequestGear(4U)==HAL_OK && acquisition_request_pending==1U);
  Acquisition_GearPoll();
  expect_high_gear(4U);
  CHECK(acquisition_udp_gear2_active==1U && acquisition_switch_fail_count==0U);

  udp_setup(); enter_high_gear(4U);
  acquisition_udp_gear2_active=0U;
  CHECK(Acquisition_RequestGear(3U)==HAL_OK && acquisition_request_pending==1U);
  Acquisition_GearPoll();
  expect_high_gear(3U);
  CHECK(acquisition_udp_gear2_active==1U && acquisition_switch_fail_count==0U);

  /* A repeated same-gear request must not be misreported as ADC conflict. */
  udp_setup(); enter_high_gear(3U);
  acquisition_udp_gear2_active=0U;
  configs=config_count;
  CHECK(Acquisition_RequestGear(3U)==HAL_OK);
  Acquisition_GearPoll();
  CHECK(config_count==configs && acquisition_active_gear==3U);
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_OK);
  CHECK(acquisition_switch_fail_count==0U);

  /* GEAR0/144 remains mutually exclusive with GEAR3/GEAR4. */
  udp_setup();
  CHECK(Acquisition_RequestADCSampling(144U)==HAL_OK);
  Acquisition_GearPoll();
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==144U);
  groups=adc_group_count;
  CHECK(Acquisition_RequestGear(3U)==HAL_ERROR);
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_ADC_CONFLICT);
  CHECK(acquisition_active_gear==0U && acquisition_adc_active_cycles==144U);
  CHECK(adc_group_count==groups);

  udp_setup();
  acquisition_adc_requested_cycles=144U;
  acquisition_adc_request_pending=1U;
  CHECK(Acquisition_RequestGear(4U)==HAL_ERROR);
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_ADC_CONFLICT);

  /* The ADC fix must not bypass a real fault latch. */
  udp_setup(); enter_high_gear(3U);
  acquisition_fault_latched=1U;
  CHECK(Acquisition_RequestGear(4U)==HAL_OK);
  Acquisition_GearPoll();
  CHECK(acquisition_last_switch_result==ACQ_SWITCH_LATCHED);
  CHECK(acquisition_active_gear==3U && adc_running==1U);
  CHECK(acquisition_switch_fail_count==1U);

  /* GEAR3/4 cycle budgets are inclusive phase limits and strict whole-body
   * limits. Equality on the body budget is a failure, never rounded away. */
  check_fast_budget(3U,540U,971U,0U,0U);
  check_fast_budget(3U,541U,971U,DIGITAL_SYNC_ERROR_LATE,DIGITAL_SYNC_DETAIL_PHASE);
  check_fast_budget(3U,540U,972U,DIGITAL_SYNC_ERROR_LATE,DIGITAL_SYNC_DETAIL_BODY_BUDGET);
  check_fast_budget(4U,216U,389U,0U,0U);
  check_fast_budget(4U,217U,389U,DIGITAL_SYNC_ERROR_LATE,DIGITAL_SYNC_DETAIL_PHASE);
  check_fast_budget(4U,216U,390U,DIGITAL_SYNC_ERROR_LATE,DIGITAL_SYNC_DETAIL_BODY_BUDGET);

  printf("PASS GEAR3/GEAR4 manager/isr/status: %u checks; exact ADC/TIM2 rows, "
         "UDP gate, partial discard, G4->G0 restore, frame CRC/GEAR, 50-point "
         "data, phase/body budget boundaries.\n",
         checks);
  return 0;
}
