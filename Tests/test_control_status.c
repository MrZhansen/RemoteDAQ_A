/* Protocol-only host checks: no UART, HAL or board access. */
#include "daq_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int checks;
#define CHECK(c) do { checks++; if (!(c)) { \
  fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); exit(1); } } while (0)

/* Independent polynomial division, not the production 16-bit shift register. */
static uint16_t reference_crc(const uint8_t *p, size_t n)
{
  uint32_t r=0xFFFFU;
  size_t i;
  int bit;
  for(i=0U;i<n;i++)
  {
    r=(r<<8U)^((uint32_t)p[i]<<16U);
    for(bit=23;bit>=16;bit--)
    { if((r&((uint32_t)1U<<bit))!=0U) { r^=(uint32_t)0x11021U<<(bit-16); } }
  }
  return (uint16_t)r;
}
static void control_crc(uint8_t frame[9])
{
  uint16_t crc=reference_crc(frame,7U);
  frame[7]=(uint8_t)(crc%256U); frame[8]=(uint8_t)(crc/256U);
}
static void make_control(uint8_t frame[9], uint8_t cmd, uint16_t param)
{
  frame[0]=0xAAU; frame[1]=0x55U; frame[2]=2U; frame[3]=cmd;
  frame[4]=(uint8_t)(param%256U); frame[5]=(uint8_t)(param/256U); frame[6]=0U;
  control_crc(frame);
}
static void expect_control(const uint8_t *frame, size_t length, DAQ_ControlParseStatus expected)
{
  DAQ_ControlCommand out,saved;
  uint8_t input[9];
  memset(&out,0xA5,sizeof(out)); memcpy(&saved,&out,sizeof(out)); memcpy(input,frame,9U);
  CHECK(DAQ_ParseControlFrame(frame,length,&out)==expected);
  CHECK(memcmp(frame,input,9U)==0);
  if(expected==DAQ_CONTROL_OK)
  {
    CHECK(out.command==frame[3]);
    CHECK(out.parameter==(uint16_t)(frame[4]+256U*frame[5]));
  }
  else { CHECK(memcmp(&out,&saved,sizeof(out))==0); }
}
static void check_status(const DAQ_StatusState *state)
{
  uint8_t storage[16],expected[12];
  uint16_t crc;
  memset(storage,0xA5,sizeof(storage));
  CHECK(DAQ_BuildStatusFrame(state,&storage[2],14U)==DAQ_STATUS_OK);
  expected[0]=0xAAU; expected[1]=0x55U; expected[2]=3U;
  expected[3]=(uint8_t)state->gear; expected[4]=(uint8_t)state->channel_mask;
  expected[5]=(uint8_t)state->trigger_source; expected[6]=(uint8_t)state->trigger_mode;
  expected[7]=(uint8_t)state->trigger_level;
  expected[8]=(uint8_t)(state->wave_frequency_hz%256U);
  expected[9]=(uint8_t)(state->wave_frequency_hz/256U);
  crc=reference_crc(expected,10U); expected[10]=(uint8_t)(crc%256U); expected[11]=(uint8_t)(crc/256U);
  CHECK(memcmp(&storage[2],expected,12U)==0);
  CHECK(storage[0]==0xA5U && storage[1]==0xA5U && storage[14]==0xA5U && storage[15]==0xA5U);
  CHECK(DAQ_CRC16_CCITT_FALSE(&storage[2],10U)==crc);
}
static void reject_status(const DAQ_StatusState *state, size_t capacity, DAQ_StatusBuildStatus result)
{
  uint8_t frame[16],saved[16];
  DAQ_StatusState valid={0U,0x1FU,0U,0U,0U,1000U};
  memset(frame,0xA5,sizeof(frame));
  CHECK(DAQ_BuildStatusFrame(&valid,frame,sizeof(frame))==DAQ_STATUS_OK);
  memcpy(saved,frame,sizeof(frame));
  CHECK(DAQ_BuildStatusFrame(state,frame,capacity)==result);
  CHECK(memcmp(frame,saved,sizeof(frame))==0);
}
int main(void)
{
  static const uint8_t vector[]="123456789";
  static const uint32_t limits[]={4U,31U,5U,2U,255U,0U};
  static const uint32_t frequencies[]={100U,255U,256U,1000U,9999U,10000U};
  static const uint32_t levels[]={0U,1U,127U,128U,254U,255U};
  uint8_t control[32],body[9],status[12];
  DAQ_ControlCommand command;
  DAQ_StatusState state={0U,0x1FU,0U,0U,0U,1000U},bad;
  uint32_t cmd,p,hi,g,m,s,t,i,bit;
  uint16_t crc;
  CHECK(DAQ_CONTROL_FRAME_SIZE==9U && DAQ_CONTROL_CRC_OFFSET==7U);
  CHECK(DAQ_STATUS_FRAME_SIZE==12U && DAQ_STATUS_CRC_OFFSET==10U);
  CHECK(DAQ_CRC16_CCITT_FALSE(vector,9U)==0x29B1U && reference_crc(vector,9U)==0x29B1U);
  make_control(control,DAQ_CMD_SET_GEAR,2U); expect_control(control,9U,DAQ_CONTROL_OK);
  /* Protocol validity is independent of the current GEAR0/1 execution limit. */
  for(cmd=1U;cmd<=6U;cmd++)
  {
    for(p=0U;p<=255U;p++)
    {
      make_control(control,(uint8_t)cmd,(uint16_t)p);
      expect_control(control,9U,(p<=limits[cmd-1U])?DAQ_CONTROL_OK:DAQ_CONTROL_PARAMETER_OUT_OF_RANGE);
    }
    for(hi=1U;hi<=255U;hi++)
    {
      make_control(control,(uint8_t)cmd,(uint16_t)(hi*256U));
      expect_control(control,9U,DAQ_CONTROL_BAD_PARAMETER_HIGH);
    }
  }
  /* Every 16-bit frequency, including both legal endpoints. */
  for(p=0U;p<=65535U;p++)
  {
    make_control(control,DAQ_CMD_SET_WAVE_FREQUENCY,(uint16_t)p);
    expect_control(control,9U,((p>=100U)&&(p<=10000U))?DAQ_CONTROL_OK:DAQ_CONTROL_PARAMETER_OUT_OF_RANGE);
  }
  for(cmd=0U;cmd<=255U;cmd++)
  {
    if(cmd>=1U && cmd<=7U) { continue; }
    make_control(control,(uint8_t)cmd,0U); expect_control(control,9U,DAQ_CONTROL_UNKNOWN_COMMAND);
  }
  make_control(control,DAQ_CMD_REQUEST_STATUS,0U);
  for(i=0U;i<=32U;i++) { if(i!=9U) { expect_control(control,i,DAQ_CONTROL_BAD_LENGTH); } }
  CHECK(DAQ_ParseControlFrame(NULL,9U,&command)==DAQ_CONTROL_BAD_ARGUMENT);
  CHECK(DAQ_ParseControlFrame(control,9U,NULL)==DAQ_CONTROL_BAD_ARGUMENT);
  for(i=0U;i<256U;i++)
  {
    make_control(control,DAQ_CMD_REQUEST_STATUS,0U);
    if(i!=0xAAU) { control[0]=(uint8_t)i; control_crc(control); expect_control(control,9U,DAQ_CONTROL_BAD_HEADER); }
    make_control(control,DAQ_CMD_REQUEST_STATUS,0U);
    if(i!=0x55U) { control[1]=(uint8_t)i; control_crc(control); expect_control(control,9U,DAQ_CONTROL_BAD_HEADER); }
    make_control(control,DAQ_CMD_REQUEST_STATUS,0U);
    if(i!=2U) { control[2]=(uint8_t)i; control_crc(control); expect_control(control,9U,DAQ_CONTROL_BAD_TYPE); }
    if(i!=0U)
    {
      make_control(control,DAQ_CMD_REQUEST_STATUS,0U); control[6]=(uint8_t)i; control_crc(control);
      expect_control(control,9U,DAQ_CONTROL_BAD_RESERVED);
    }
  }
  make_control(body,DAQ_CMD_SET_WAVE_FREQUENCY,1000U);
  CHECK(body[4]==0xE8U && body[5]==0x03U);
  for(i=0U;i<9U;i++) for(bit=0U;bit<8U;bit++)
  {
    memcpy(control,body,9U); control[i]^=(uint8_t)(1U<<bit);
    expect_control(control,9U,(i<2U)?DAQ_CONTROL_BAD_HEADER:((i==2U)?DAQ_CONTROL_BAD_TYPE:DAQ_CONTROL_BAD_CRC));
  }
  memcpy(control,body,9U); CHECK(control[7]!=control[8]);
  control[7]=body[8]; control[8]=body[7]; expect_control(control,9U,DAQ_CONTROL_BAD_CRC);
  /* Cross-products accept all independent V1.1 fields, including mask=0 and
   * disabled/digital sources with retained mode/level. No trigger policy here. */
  for(g=0U;g<=4U;g++) for(m=0U;m<=31U;m++) for(s=0U;s<=5U;s++) for(t=0U;t<=2U;t++)
  {
    state.gear=g; state.channel_mask=m; state.trigger_source=s; state.trigger_mode=t;
    state.trigger_level=levels[(g+m+s+t)%6U]; state.wave_frequency_hz=frequencies[(g+m+s+t)%6U];
    check_status(&state);
  }
  state.gear=0U; state.channel_mask=31U; state.trigger_source=0U; state.trigger_mode=0U; state.trigger_level=0U;
  for(p=0U;p<=65535U;p++)
  {
    state.wave_frequency_hz=p;
    if(p>=100U && p<=10000U) { check_status(&state); }
    else { reject_status(&state,12U,DAQ_STATUS_PARAMETER_OUT_OF_RANGE); }
  }
  state.wave_frequency_hz=1000U;
  for(i=0U;i<12U;i++) { reject_status(&state,i,DAQ_STATUS_BUFFER_TOO_SMALL); }
  reject_status(NULL,12U,DAQ_STATUS_BAD_ARGUMENT);
  CHECK(DAQ_BuildStatusFrame(&state,NULL,12U)==DAQ_STATUS_BAD_ARGUMENT);
  for(i=0U;i<6U;i++)
  {
    bad=state;
    switch(i)
    {
      case 0U: bad.gear=5U; break;
      case 1U: bad.channel_mask=32U; break;
      case 2U: bad.trigger_source=6U; break;
      case 3U: bad.trigger_mode=3U; break;
      case 4U: bad.trigger_level=256U; break;
      default: bad.wave_frequency_hz=65536U; break;
    }
    reject_status(&bad,12U,DAQ_STATUS_PARAMETER_OUT_OF_RANGE);
    switch(i)
    {
      case 0U: bad.gear=UINT32_MAX; break;
      case 1U: bad.channel_mask=UINT32_MAX; break;
      case 2U: bad.trigger_source=UINT32_MAX; break;
      case 3U: bad.trigger_mode=UINT32_MAX; break;
      case 4U: bad.trigger_level=UINT32_MAX; break;
      default: bad.wave_frequency_hz=UINT32_MAX; break;
    }
    reject_status(&bad,12U,DAQ_STATUS_PARAMETER_OUT_OF_RANGE);
  }
  CHECK(DAQ_BuildStatusFrame(&state,status,12U)==DAQ_STATUS_OK);
  CHECK(status[0]==0xAAU && status[1]==0x55U && status[2]==3U);
  CHECK(status[3]==0U && status[4]==31U && status[5]==0U && status[6]==0U && status[7]==0U);
  CHECK(status[8]==0xE8U && status[9]==3U);
  crc=reference_crc(status,10U);
  CHECK(status[10]==(uint8_t)(crc%256U) && status[11]==(uint8_t)(crc/256U));
  for(i=0U;i<10U;i++) for(bit=0U;bit<8U;bit++)
  {
    status[i]^=(uint8_t)(1U<<bit); CHECK(reference_crc(status,10U)!=crc); status[i]^=(uint8_t)(1U<<bit);
  }
  status[10]^=0xFFU; status[11]^=0xFFU;
  CHECK(DAQ_CRC16_CCITT_FALSE(status,10U)==crc); /* CRC excludes itself. */
  printf("PASS control/status protocol: %u checks; all commands, all 16-bit frequencies, strict frame/CRC/reserved/high-byte rules, status field cross-products, unchanged error outputs.\n",checks);
  return 0;
}
