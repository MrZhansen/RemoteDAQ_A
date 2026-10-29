#include "daq_control_stream.h"
void DAQ_ControlStreamInit(DAQ_ControlStream *stream)
{
  DAQ_ControlStream empty = {0};
  *stream = empty;
}
void DAQ_ControlStreamReset(DAQ_ControlStream *stream) { stream->used = 0U; }
void DAQ_ControlStreamExpire(DAQ_ControlStream *stream, uint32_t now)
{
  if ((stream->used != 0U) &&
      ((uint32_t)(now - stream->last_tick) > DAQ_CONTROL_PARTIAL_TIMEOUT_MS))
  {
    stream->partial_timeout_count++;
    stream->format_error_count++;
    stream->used = 0U;
  }
}
uint32_t DAQ_ControlStreamFeed(DAQ_ControlStream *stream, uint8_t byte,
                             uint32_t now, DAQ_ControlCommand *command)
{
  uint32_t attempt, i;
  DAQ_ControlParseStatus status;
  DAQ_ControlStreamExpire(stream, now);
  stream->last_tick = now;
  stream->bytes[stream->used++] = byte;
  /* At most nine discarded positions per input byte; no unbounded scan. */
  for (attempt = 0U; attempt < DAQ_CONTROL_FRAME_SIZE; attempt++)
  {
    if (stream->used == 0U) { return 0U; }
    if ((stream->bytes[0] == 0xAAU) &&
        ((stream->used < 2U) || (stream->bytes[1] == 0x55U)) &&
        ((stream->used < 3U) || (stream->bytes[2] == 0x02U)))
    {
      if (stream->used < DAQ_CONTROL_FRAME_SIZE) { return 0U; }
      stream->candidate_count++;
      status = DAQ_ParseControlFrame(stream->bytes, DAQ_CONTROL_FRAME_SIZE, command);
      stream->last_parse_status = (uint32_t)status;
      if (status == DAQ_CONTROL_OK)
      {
        stream->frame_count++;
        stream->last_command = command->command;
        stream->last_parameter = command->parameter;
        stream->used = 0U;
        return 1U;
      }
      if (status == DAQ_CONTROL_BAD_CRC) { stream->crc_error_count++; }
      else { stream->format_error_count++; }
    }
    else
    {
      stream->noise_byte_count++;
      if (stream->bytes[0] == 0xAAU) { stream->format_error_count++; }
    }
    /* Slide one byte, retaining any embedded beginning of the next frame. */
    stream->used--;
    for (i = 0U; i < stream->used; i++) { stream->bytes[i] = stream->bytes[i + 1U]; }
  }
  return 0U;
}
