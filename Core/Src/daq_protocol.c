#include "daq_protocol.h"

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0)
 * driven by a 256-entry table. The former bit-at-a-time loop cost ~69 us of
 * the ~92 us GEAR4 frame build on the 216 MHz F746; the table keeps the same
 * field values while removing that bottleneck. */
static const uint16_t crc16_ccitt_false_table[256] =
{
  0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
  0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
  0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
  0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
  0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
  0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
  0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
  0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
  0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
  0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
  0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
  0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
  0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
  0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
  0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
  0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
  0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
  0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
  0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
  0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
  0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
  0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
  0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
  0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
  0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
  0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
  0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
  0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
  0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
  0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
  0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
  0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U,
};

uint16_t DAQ_CRC16_CCITT_FALSE(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  size_t i;

  for (i = 0U; i < length; i++)
  {
    crc = (uint16_t)((uint16_t)(crc << 8U) ^
          crc16_ccitt_false_table[((crc >> 8U) ^ data[i]) & 0xFFU]);
  }
  return crc;
}

/* One shared encoder for fixed previews and CMD02-selected channels.
 * CH_MASK bits 2..4 gate the existing DIG bits 0..2 without remapping. */
static DAQ_FrameStatus BuildDataFrame(const uint16_t *ch1, const uint16_t *ch2,
                                     const uint8_t *dig,
                                     uint8_t mask, uint8_t gear, uint16_t sequence,
                                     uint8_t *frame, size_t frame_capacity)
{
  uint32_t i;
  uint16_t wire_value;
  uint16_t crc;
  size_t offset;

  if ((gear > 4U) || (mask > 0x1FU) ||
      (((mask & 0x01U) != 0U) && (ch1 == NULL)) ||
      (((mask & 0x02U) != 0U) && (ch2 == NULL)) ||
      (((mask & 0x1CU) != 0U) && (dig == NULL)) ||
      (frame == NULL) ||
      (frame_capacity < DAQ_DATA_FRAME_SIZE))
  {
    return DAQ_FRAME_BAD_ARGUMENT;
  }
  /* Validate the entire stable snapshot before changing any output bytes.
   * In particular, do not mask the startup sentinel 0xFFFF into valid data. */
  for (i = 0U; i < DAQ_ANALOG_SAMPLE_COUNT; i++)
  {
    if ((((mask & 0x01U) != 0U) && (ch1[i] > 0x0FFFU)) ||
        (((mask & 0x02U) != 0U) && (ch2[i] > 0x0FFFU)))
    {
      return DAQ_FRAME_ADC_OUT_OF_RANGE;
    }
    if (((mask & 0x1CU) != 0U) && ((dig[i] & 0xF8U) != 0U))
    {
      return DAQ_FRAME_DIG_OUT_OF_RANGE;
    }
  }

  frame[0] = 0xAAU;
  frame[1] = 0x55U;
  frame[2] = 0x01U;
  frame[3] = (uint8_t)(sequence & 0xFFU);
  frame[4] = (uint8_t)(sequence >> 8U);
  frame[5] = gear;
  frame[6] = mask;
  frame[7] = (uint8_t)DAQ_ANALOG_SAMPLE_COUNT;

  for (i = 0U; i < DAQ_ANALOG_SAMPLE_COUNT; i++)
  {
    offset = 8U + (5U * i);
    wire_value = ((mask & 0x01U) != 0U) ?
        (uint16_t)((uint32_t)ch1[i] << 4U) : (uint16_t)0U;
    frame[offset] = (uint8_t)(wire_value & 0xFFU);
    frame[offset + 1U] = (uint8_t)(wire_value >> 8U);
    wire_value = ((mask & 0x02U) != 0U) ?
        (uint16_t)((uint32_t)ch2[i] << 4U) : (uint16_t)0U;
    frame[offset + 2U] = (uint8_t)(wire_value & 0xFFU);
    frame[offset + 3U] = (uint8_t)(wire_value >> 8U);
    frame[offset + 4U] = ((mask & 0x1CU) != 0U) ?
        (uint8_t)(dig[i] & ((mask >> 2U) & 0x07U)) : (uint8_t)0U;
  }

  crc = DAQ_CRC16_CCITT_FALSE(frame, DAQ_DATA_CRC_OFFSET);
  frame[DAQ_DATA_CRC_OFFSET] = (uint8_t)(crc & 0xFFU);
  frame[DAQ_DATA_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8U);
  return DAQ_FRAME_OK;
}

#if defined(__ICCARM__)
__root
#endif
DAQ_FrameStatus DAQ_BuildCH1Frame(const uint16_t samples[DAQ_CH1_SAMPLE_COUNT],
                                uint16_t sequence, uint8_t *frame,
                                size_t frame_capacity)
{
  return BuildDataFrame(samples, NULL, NULL, 0x01U, 0U, sequence, frame, frame_capacity);
}

#if defined(__ICCARM__)
__root
#endif
DAQ_FrameStatus DAQ_BuildAnalogFrame(
    const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    uint16_t sequence, uint8_t *frame, size_t frame_capacity)
{
  return BuildDataFrame(ch1, ch2, NULL, 0x03U, 0U, sequence, frame, frame_capacity);
}

#if defined(__ICCARM__)
__root
#endif
DAQ_FrameStatus DAQ_BuildFullFrame(const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    const uint8_t dig[DAQ_ANALOG_SAMPLE_COUNT],
    uint16_t sequence, uint8_t *frame, size_t frame_capacity)
{
  return BuildDataFrame(ch1, ch2, dig, 0x1FU, 0U, sequence, frame, frame_capacity);
}

#if defined(__ICCARM__)
__root
#endif
DAQ_FrameStatus DAQ_BuildFullFrameForGear(
    const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    const uint8_t dig[DAQ_ANALOG_SAMPLE_COUNT], uint32_t gear,
    uint16_t sequence, uint8_t *frame, size_t frame_capacity)
{
  if (gear > 1U) { return DAQ_FRAME_BAD_ARGUMENT; }
  return BuildDataFrame(ch1, ch2, dig, 0x1FU, (uint8_t)gear, sequence, frame, frame_capacity);
}

#if defined(__ICCARM__)
__root
#endif
DAQ_FrameStatus DAQ_BuildSelectedFrameForGear(
    const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    const uint8_t dig[DAQ_ANALOG_SAMPLE_COUNT], uint32_t gear,
    uint32_t channel_mask, uint16_t sequence, uint8_t *frame,
    size_t frame_capacity)
{
  if ((gear > 4U) || (channel_mask > 0x1FU)) { return DAQ_FRAME_BAD_ARGUMENT; }
  return BuildDataFrame(ch1, ch2, dig, (uint8_t)channel_mask, (uint8_t)gear,
                        sequence, frame, frame_capacity);
}


/* Encoding capability only, not an acquisition qualification or CMD01 gate. */
DAQ_FrameStatus DAQ_BuildUDPGEAR2CandidateFrame(const uint16_t *ch1,
    const uint16_t *ch2, const uint8_t *dig, uint32_t channel_mask,
    uint16_t sequence, uint8_t *frame, size_t frame_capacity)
{
  if (channel_mask > 0x1FU) { return DAQ_FRAME_BAD_ARGUMENT; }
  return BuildDataFrame(ch1, ch2, dig, (uint8_t)channel_mask, 2U,
                        sequence, frame, frame_capacity);
}

#if defined(__ICCARM__)
__root
#endif
DAQ_ControlParseStatus DAQ_ParseControlFrame(const uint8_t *frame, size_t length,
                                            DAQ_ControlCommand *command)
{
  DAQ_ControlCommand candidate;
  uint16_t crc;
  uint16_t received_crc;
  if ((frame == NULL) || (command == NULL)) { return DAQ_CONTROL_BAD_ARGUMENT; }
  if (length != DAQ_CONTROL_FRAME_SIZE) { return DAQ_CONTROL_BAD_LENGTH; }
  if ((frame[0] != 0xAAU) || (frame[1] != 0x55U)) { return DAQ_CONTROL_BAD_HEADER; }
  if (frame[2] != 0x02U) { return DAQ_CONTROL_BAD_TYPE; }
  crc = DAQ_CRC16_CCITT_FALSE(frame, DAQ_CONTROL_CRC_OFFSET);
  received_crc = (uint16_t)((uint16_t)frame[7] | ((uint16_t)frame[8] << 8U));
  if (crc != received_crc) { return DAQ_CONTROL_BAD_CRC; }
  if (frame[6] != 0U) { return DAQ_CONTROL_BAD_RESERVED; }
  if ((frame[3] < DAQ_CMD_SET_GEAR) || (frame[3] > DAQ_CMD_SET_WAVE_FREQUENCY))
  { return DAQ_CONTROL_UNKNOWN_COMMAND; }
  if ((frame[3] != DAQ_CMD_SET_WAVE_FREQUENCY) && (frame[5] != 0U))
  { return DAQ_CONTROL_BAD_PARAMETER_HIGH; }
  candidate.command = frame[3];
  candidate.parameter = (uint16_t)((uint16_t)frame[4] | ((uint16_t)frame[5] << 8U));
  switch (candidate.command)
  {
    case DAQ_CMD_SET_GEAR:
      if (candidate.parameter > 4U) { return DAQ_CONTROL_PARAMETER_OUT_OF_RANGE; }
      break;
    case DAQ_CMD_SET_CHANNEL_MASK:
      if (candidate.parameter > 0x1FU) { return DAQ_CONTROL_PARAMETER_OUT_OF_RANGE; }
      break;
    case DAQ_CMD_SET_TRIGGER_SOURCE:
      if (candidate.parameter > 5U) { return DAQ_CONTROL_PARAMETER_OUT_OF_RANGE; }
      break;
    case DAQ_CMD_SET_TRIGGER_MODE:
      if (candidate.parameter > 2U) { return DAQ_CONTROL_PARAMETER_OUT_OF_RANGE; }
      break;
    case DAQ_CMD_SET_TRIGGER_LEVEL:
      break; /* All 0..255 values are valid, even for an inactive/digital source. */
    case DAQ_CMD_REQUEST_STATUS:
      if (candidate.parameter != 0U) { return DAQ_CONTROL_PARAMETER_OUT_OF_RANGE; }
      break;
    case DAQ_CMD_SET_WAVE_FREQUENCY:
      if ((candidate.parameter < 100U) || (candidate.parameter > 10000U))
      { return DAQ_CONTROL_PARAMETER_OUT_OF_RANGE; }
      break;
    default:
      return DAQ_CONTROL_UNKNOWN_COMMAND;
  }
  *command = candidate; /* Publish only after complete validation. */
  return DAQ_CONTROL_OK;
}

#if defined(__ICCARM__)
__root
#endif
DAQ_StatusBuildStatus DAQ_BuildStatusFrame(const DAQ_StatusState *state,
                                         uint8_t *frame, size_t frame_capacity)
{
  DAQ_StatusState value;
  uint16_t crc;
  if ((state == NULL) || (frame == NULL)) { return DAQ_STATUS_BAD_ARGUMENT; }
  if (frame_capacity < DAQ_STATUS_FRAME_SIZE) { return DAQ_STATUS_BUFFER_TOO_SMALL; }
  value = *state;
  if ((value.gear > 4U) || (value.channel_mask > 0x1FU) ||
      (value.trigger_source > 5U) || (value.trigger_mode > 2U) ||
      (value.trigger_level > 255U) || (value.wave_frequency_hz < 100U) ||
      (value.wave_frequency_hz > 10000U))
  { return DAQ_STATUS_PARAMETER_OUT_OF_RANGE; }
  frame[0] = 0xAAU;
  frame[1] = 0x55U;
  frame[2] = 0x03U;
  frame[3] = (uint8_t)value.gear;
  frame[4] = (uint8_t)value.channel_mask;
  frame[5] = (uint8_t)value.trigger_source;
  frame[6] = (uint8_t)value.trigger_mode;
  frame[7] = (uint8_t)value.trigger_level;
  frame[8] = (uint8_t)(value.wave_frequency_hz & 0xFFU);
  frame[9] = (uint8_t)(value.wave_frequency_hz >> 8U);
  crc = DAQ_CRC16_CCITT_FALSE(frame, DAQ_STATUS_CRC_OFFSET);
  frame[10] = (uint8_t)(crc & 0xFFU);
  frame[11] = (uint8_t)(crc >> 8U);
  return DAQ_STATUS_OK;
}
