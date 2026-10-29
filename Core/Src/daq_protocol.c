#include "daq_protocol.h"

uint16_t DAQ_CRC16_CCITT_FALSE(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  size_t i;
  uint32_t bit;

  for (i = 0U; i < length; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8U);
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)(((uint32_t)crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)((uint32_t)crc << 1U);
      }
    }
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

  if ((gear > 2U) || (mask > 0x1FU) ||
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
  if ((gear > 1U) || (channel_mask > 0x1FU)) { return DAQ_FRAME_BAD_ARGUMENT; }
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
