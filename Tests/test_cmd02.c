/* Real DMA receiver, streaming parser, dispatcher, frame builder and TX arbiter
 * with HAL/register stubs. No serial port or development board is accessed. */
#define UART_CONTROL_MAIN uart_control_legacy_main
#include "test_uart6_control.c"

static uint16_t mask_reference_crc(const uint8_t *bytes,uint32_t length)
{
  uint32_t value=0xFFFFU, i, bit;
  for(i=0U;i<length;i++)
  {
    for(bit=0U;bit<8U;bit++)
    {
      uint32_t feedback=((value>>15U) ^ (bytes[i]>>(7U-bit))) & 1U;
      value=(value<<1U) & 0xFFFFU;
      if(feedback) { value^=0x1021U; }
    }
  }
  return (uint16_t)value;
}

static void check_mask_frame(const uint8_t *frame,const uint16_t *ch1,
    const uint16_t *ch2,const uint8_t *dig,uint32_t mask,uint32_t gear,uint16_t seq)
{
  uint32_t i;
  CHECK(frame[0]==0xAAU && frame[1]==0x55U && frame[2]==1U);
  CHECK(frame[3]==(uint8_t)seq && frame[4]==(uint8_t)(seq>>8U));
  CHECK(frame[5]==gear && frame[6]==mask && frame[7]==50U);
  for(i=0U;i<50U;i++)
  {
    uint32_t o=8U+5U*i;
    uint16_t first=(mask&1U)?(uint16_t)(ch1[i]<<4U):0U;
    uint16_t second=(mask&2U)?(uint16_t)(ch2[i]<<4U):0U;
    uint8_t bits=(mask&0x1CU)?(uint8_t)(dig[i]&((mask>>2U)&7U)):0U;
    CHECK(frame[o]==(uint8_t)first && frame[o+1U]==(uint8_t)(first>>8U));
    CHECK(frame[o+2U]==(uint8_t)second && frame[o+3U]==(uint8_t)(second>>8U));
    CHECK(frame[o+4U]==bits && (frame[o+4U]&0xF8U)==0U);
  }
  CHECK(mask_reference_crc(frame,258U)==(uint16_t)(frame[258]|((uint16_t)frame[259]<<8U)));
}

static void protocol_masks(void)
{
  uint16_t first[50], second[50];
  uint8_t bits[50], guarded[262], legacy[260], saved[262];
  uint32_t gear, mask, i, s;
  static const uint16_t sequences[]={0U,1U,0x1234U,0xFFFFU};
  CHECK(mask_reference_crc((const uint8_t *)"123456789",9U)==0x29B1U);
  for(i=0U;i<50U;i++)
  {
    first[i]=(uint16_t)((71U*i+3U)&0xFFFU);
    second[i]=(uint16_t)((4095U-53U*i)&0xFFFU);
    bits[i]=(uint8_t)(i&7U); /* All eight combinations and independent bits. */
  }
  first[0]=0U; first[1]=4095U; second[0]=4095U; second[1]=0U;
  for(gear=0U;gear<2U;gear++)
  {
    for(mask=0U;mask<32U;mask++)
    {
      for(s=0U;s<sizeof(sequences)/sizeof(sequences[0]);s++)
      {
        memset(guarded,0xA5,sizeof(guarded));
        CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,gear,mask,
            sequences[s],guarded+1U,260U)==DAQ_FRAME_OK);
        CHECK(guarded[0]==0xA5U && guarded[261]==0xA5U);
        check_mask_frame(guarded+1U,first,second,bits,mask,gear,sequences[s]);
      }
    }
  }
  /* Old public interfaces keep byte-for-byte output compatibility. */
  CHECK(DAQ_BuildCH1Frame(first,7U,legacy,260U)==DAQ_FRAME_OK);
  CHECK(DAQ_BuildSelectedFrameForGear(first,NULL,NULL,0U,1U,7U,guarded,260U)==DAQ_FRAME_OK);
  CHECK(memcmp(legacy,guarded,260U)==0);
  CHECK(DAQ_BuildAnalogFrame(first,second,7U,legacy,260U)==DAQ_FRAME_OK);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,NULL,0U,3U,7U,guarded,260U)==DAQ_FRAME_OK);
  CHECK(memcmp(legacy,guarded,260U)==0);
  CHECK(DAQ_BuildFullFrameForGear(first,second,bits,1U,7U,legacy,260U)==DAQ_FRAME_OK);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,1U,31U,7U,guarded,260U)==DAQ_FRAME_OK);
  CHECK(memcmp(legacy,guarded,260U)==0);
  CHECK(DAQ_BuildSelectedFrameForGear(NULL,NULL,NULL,0U,0U,7U,guarded,260U)==DAQ_FRAME_OK);
  check_mask_frame(guarded,NULL,NULL,NULL,0U,0U,7U);

  memset(guarded,0x5A,sizeof(guarded)); memcpy(saved,guarded,sizeof(saved));
  for(mask=32U;mask<=255U;mask++)
  {
    CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,mask,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
    CHECK(memcmp(guarded,saved,sizeof(saved))==0);
  }
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,0x100U,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,UINT32_MAX,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,2U,31U,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,31U,7U,guarded,259U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildSelectedFrameForGear(NULL,second,bits,0U,1U,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildSelectedFrameForGear(first,NULL,bits,0U,2U,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,NULL,0U,4U,7U,guarded,260U)==DAQ_FRAME_BAD_ARGUMENT);
  CHECK(memcmp(guarded,saved,sizeof(saved))==0);
  first[0]=0xFFFFU;
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,1U,7U,guarded,260U)==DAQ_FRAME_ADC_OUT_OF_RANGE);
  CHECK(memcmp(guarded,saved,sizeof(saved))==0);
  /* Disabled fields never consume invalid raw samples or NULL pointers. */
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,2U,7U,guarded,260U)==DAQ_FRAME_OK);
  first[0]=0U; second[0]=0x1000U; memset(guarded,0x5A,sizeof(guarded));
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,2U,7U,guarded,260U)==DAQ_FRAME_ADC_OUT_OF_RANGE);
  CHECK(memcmp(guarded,saved,sizeof(saved))==0);
  second[0]=4095U; bits[0]=0x80U;
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,4U,7U,guarded,260U)==DAQ_FRAME_DIG_OUT_OF_RANGE);
  CHECK(memcmp(guarded,saved,sizeof(saved))==0);
  CHECK(DAQ_BuildSelectedFrameForGear(first,second,bits,0U,3U,7U,guarded,260U)==DAQ_FRAME_OK);
}

static void send_mask(uint32_t mask)
{
  uint8_t request[9];
  uint32_t count=uart6_mask_apply_count, replies=uart6_status_tx_count;
  uint32_t gear_calls=gear_request_calls;
  uint16_t seq=ch1_uart_next_seq;
  make_control(request,2U,(uint16_t)mask); inject_bytes(request,9U); drain();
  CHECK(daq_active_channel_mask==mask && uart6_mask_apply_count==count+1U);
  CHECK(uart6_status_tx_count==replies+1U && uart6_status_last_frame[4]==mask);
  CHECK(gear_request_calls==gear_calls && adc_running==1U);
  CHECK(ch1_uart_next_seq==seq); /* Status response never consumes a data SEQ. */
}

static void send_group(uint32_t group)
{
  uint32_t i, count=ch1_uart_tx_count, calls=wire_calls;
  uint16_t seq=ch1_uart_next_seq;
  Full_Snapshot raw;
  set_group(group,0x321U); source.gear=acquisition_active_gear;
  for(i=0U;i<50U;i++) { source.dig[i]=(uint8_t)(i&7U); }
  raw=source; drain();
  CHECK(ch1_uart_tx_count==count+1U && ch1_uart_next_seq==(uint16_t)(seq+1U));
  CHECK(wire_calls==calls+1U && wire_lengths[calls]==260U && wire_types[calls]==1U);
  check_mask_frame(captured_frame,source.ch1,source.ch2,source.dig,
      daq_active_channel_mask,acquisition_active_gear,seq);
  CHECK(ch1_frame_channel_mask==daq_active_channel_mask);
  CHECK(memcmp(&source,&raw,sizeof(raw))==0); /* Raw snapshot remains intact. */
  count=wire_calls; drain(); CHECK(wire_calls==count);
}

int main(void)
{
  uint8_t request[10], bad[10], previous[260];
  uint32_t mask, gear, i, j, count, replies, calls, group=1000U;
  uint16_t seq;
  DAQ_ControlCommand command, unchanged;
  protocol_masks();
  reset_control(); drain();
  CHECK(daq_active_channel_mask==31U && uart6_status_last_frame[4]==31U);
  for(gear=0U;gear<2U;gear++)
  {
    make_control(request,1U,(uint16_t)gear); inject_bytes(request,9U); drain();
    CHECK(acquisition_active_gear==gear && uart6_status_last_frame[3]==gear);
    for(mask=0U;mask<32U;mask++) { send_mask(mask); send_group(group++); }
    /* Re-enable in reverse order to cover data restoration as well. */
    for(mask=32U;mask>0U;mask--) { send_mask(mask-1U); send_group(group++); }
  }
  send_mask(0x15U);
  make_control(request,6U,0U); replies=uart6_status_tx_count;
  inject_bytes(request,9U); drain();
  CHECK(uart6_status_tx_count==replies+1U && uart6_status_last_frame[4]==0x15U);
  make_control(request,1U,0U); inject_bytes(request,9U); drain();
  CHECK(acquisition_active_gear==0U && daq_active_channel_mask==0x15U);
  CHECK(uart6_status_last_frame[3]==0U && uart6_status_last_frame[4]==0x15U);
  send_group(group++);

  count=uart6_mask_apply_count; calls=wire_calls;
  /* Invalid protocol values, header/type, CRC and reserved fields. */
  for(i=0U;i<9U;i++)
  {
    make_control(bad,2U,31U);
    if(i==0U) { bad[0]=0U; }
    if(i==1U) { bad[2]=1U; }
    if(i==2U) { bad[6]=1U; }
    if(i==3U) { bad[4]=32U; }
    if(i==4U) { bad[4]=0xFFU; }
    if(i==5U) { bad[5]=1U; }
    if(i==6U) { bad[4]=0xFFU; bad[5]=0xFFU; }
    if(i<=6U)
    {
      uint16_t crc=mask_reference_crc(bad,7U);
      bad[7]=(uint8_t)crc; bad[8]=(uint8_t)(crc>>8U);
    }
    if(i==7U) { bad[7]^=1U; }
    if(i==8U) { bad[8]^=0x80U; }
    command.command=0x66U; command.parameter=0x1234U; unchanged=command;
    CHECK(DAQ_ParseControlFrame(bad,9U,&command)!=DAQ_CONTROL_OK);
    CHECK(command.command==unchanged.command && command.parameter==unchanged.parameter);
    inject_bytes(bad,9U); drain(); fake_tick+=51U; drain();
    CHECK(daq_active_channel_mask==0x15U && uart6_mask_apply_count==count);
  }
  CHECK(wire_calls==calls);
  make_control(request,2U,0U); request[9]=0x33U;
  CHECK(DAQ_ParseControlFrame(request,8U,&command)==DAQ_CONTROL_BAD_LENGTH);
  CHECK(DAQ_ParseControlFrame(request,10U,&command)==DAQ_CONTROL_BAD_LENGTH);
  inject_bytes(request,8U); drain(); CHECK(uart6_mask_apply_count==count);
  fake_tick+=51U; drain(); CHECK(daq_active_channel_mask==0x15U);
  /* Every split point preserves the command until the ninth byte arrives. */
  for(j=1U;j<9U;j++)
  {
    make_control(request,2U,(uint16_t)j); count=uart6_mask_apply_count;
    inject_bytes(request,j); drain(); CHECK(uart6_mask_apply_count==count);
    inject_bytes(request+j,9U-j); drain();
    CHECK(daq_active_channel_mask==j && uart6_mask_apply_count==count+1U);
  }

  /* Command bytes arriving during data TX cannot modify that frame or start
   * nested TX. The next service boundary applies the mask and replies first. */
  send_mask(31U);
  set_group(group++,0x456U); source.gear=acquisition_active_gear;
  inject_mask_on_data=0U; count=uart6_mask_apply_count;
  CHECK(UART6_ServicePoll()==1U);
  CHECK(captured_frame[6]==31U && daq_active_channel_mask==31U);
  memcpy(previous,captured_frame,260U); replies=uart6_status_tx_count;
  CHECK(UART6_ServicePoll()==1U);
  CHECK(daq_active_channel_mask==0U && uart6_mask_apply_count==count+1U);
  CHECK(uart6_status_tx_count==replies+1U && uart6_status_last_frame[4]==0U);
  CHECK(memcmp(previous,captured_frame,260U)==0); /* No old-group resend. */
  send_group(group++);

  /* Two complete controls in one DMA batch are applied in order; status
   * reports the actual final mask, not a fabricated intermediate state. */
  count=uart6_mask_apply_count; replies=uart6_status_tx_count;
  make_control(request,2U,1U); make_control(bad,2U,0x1EU);
  inject_bytes(request,9U); inject_bytes(bad,9U); drain();
  CHECK(uart6_mask_apply_count==count+2U && daq_active_channel_mask==0x1EU);
  CHECK(uart6_status_tx_count==replies+2U && uart6_status_last_frame[4]==0x1EU);
  send_group(group++);
  send_mask(0U); ch1_uart_next_seq=0xFFFFU;
  send_group(group++); CHECK(ch1_uart_next_seq==0U);
  send_group(group++); CHECK(ch1_uart_next_seq==1U);
  send_mask(31U); seq=ch1_uart_next_seq; count=ch1_uart_tx_error_count;
  set_group(group++,0x789U); source.gear=acquisition_active_gear;
  next_tx_result=HAL_TIMEOUT; CHECK(UART6_ServicePoll()==1U);
  CHECK(ch1_uart_tx_error_count==count+1U && ch1_uart_next_seq==seq);
  calls=wire_calls; drain(); CHECK(wire_calls==calls); /* No retry of old group. */
  next_tx_result=HAL_OK; send_group(group++);
  CHECK(adc_running==1U && adc_error==0U && dma_error==0U && digital_sync.error==0U);
  printf("PASS CMD02 masks/RX/TX integration: %u checks (all 32 masks, HAL/DMA simulation, no board)\n",checks);
  return 0;
}
