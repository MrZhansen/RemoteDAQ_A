#ifndef DAQ_CONTROL_STREAM_H
#define DAQ_CONTROL_STREAM_H
#include "daq_protocol.h"
#define DAQ_CONTROL_PARTIAL_TIMEOUT_MS 50U
/* Main-context only. UART DMA/IRQ producers never touch this parser. */
typedef struct
{
  uint8_t bytes[DAQ_CONTROL_FRAME_SIZE];
  uint32_t used, last_tick;
  uint32_t frame_count, candidate_count, crc_error_count, format_error_count;
  uint32_t noise_byte_count, partial_timeout_count;
  uint32_t last_parse_status, last_command, last_parameter;
} DAQ_ControlStream;
void DAQ_ControlStreamInit(DAQ_ControlStream *stream);
void DAQ_ControlStreamReset(DAQ_ControlStream *stream);
void DAQ_ControlStreamExpire(DAQ_ControlStream *stream, uint32_t now);
uint32_t DAQ_ControlStreamFeed(DAQ_ControlStream *stream, uint8_t byte,
                             uint32_t now, DAQ_ControlCommand *command);
#endif
