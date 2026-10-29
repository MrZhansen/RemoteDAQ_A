#ifndef DAQ_PROTOCOL_H
#define DAQ_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAQ_ANALOG_SAMPLE_COUNT 50U
#define DAQ_CH1_SAMPLE_COUNT    DAQ_ANALOG_SAMPLE_COUNT
#define DAQ_DATA_FRAME_SIZE     260U
#define DAQ_DATA_CRC_OFFSET     258U

typedef enum
{
  DAQ_FRAME_OK = 0,
  DAQ_FRAME_BAD_ARGUMENT,
  DAQ_FRAME_ADC_OUT_OF_RANGE,
  DAQ_FRAME_DIG_OUT_OF_RANGE
} DAQ_FrameStatus;

/* CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection, xorout=0.
 * data must address length readable bytes; NULL is allowed only for length=0. */
uint16_t DAQ_CRC16_CCITT_FALSE(const uint8_t *data, size_t length);

/* V1.1 data frame: GEAR=0, CH_MASK=1, N=50. No transport or SEQ mutation.
 * samples must hold 50 stable raw ADC values; it must not overlap frame.
 * Writes exactly 260 bytes on success. On any error, frame is unchanged.
 * A non-NULL frame must have frame_capacity writable bytes. */
DAQ_FrameStatus DAQ_BuildCH1Frame(const uint16_t samples[DAQ_CH1_SAMPLE_COUNT],
                                uint16_t sequence, uint8_t *frame,
                                size_t frame_capacity);

/* Same output/error contract, with CH_MASK=3. Both arrays must contain 50
 * stable values from the same sampling group; DIG remains zero. */
DAQ_FrameStatus DAQ_BuildAnalogFrame(
    const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    uint16_t sequence, uint8_t *frame, size_t frame_capacity);

/* All five channels enabled (mask=0x1F); dig contains 50 values in 0..7.
 * Same stable-input, no-overlap and unchanged-output-on-error contract. */
DAQ_FrameStatus DAQ_BuildFullFrame(const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    const uint8_t dig[DAQ_ANALOG_SAMPLE_COUNT],
    uint16_t sequence, uint8_t *frame, size_t frame_capacity);

/* GEAR 0/1 only. All data and gear must come from one stable snapshot.
 * The legacy entry points above remain GEAR 0. No SEQ mutation. */
DAQ_FrameStatus DAQ_BuildFullFrameForGear(
    const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    const uint8_t dig[DAQ_ANALOG_SAMPLE_COUNT], uint32_t gear,
    uint16_t sequence, uint8_t *frame, size_t frame_capacity);

/* GEAR 0/1, CH_MASK 0..0x1F. Disabled fields are zero; DIG preserves
 * bit0=D1, bit1=D2, bit2=D3 and only clears bits for disabled channels.
 * Only enabled inputs need non-NULL arrays with valid 50-point samples.
 * Mask zero accepts NULL inputs and still creates a complete 260-byte frame.
 * The existing stable-input/no-overlap/unchanged-output-on-error rules apply. */
DAQ_FrameStatus DAQ_BuildSelectedFrameForGear(
    const uint16_t ch1[DAQ_ANALOG_SAMPLE_COUNT],
    const uint16_t ch2[DAQ_ANALOG_SAMPLE_COUNT],
    const uint8_t dig[DAQ_ANALOG_SAMPLE_COUNT], uint32_t gear,
    uint32_t channel_mask, uint16_t sequence, uint8_t *frame,
    size_t frame_capacity);

/* Internal UDP qualification encoder, fixed GEAR=2. Existing public gear
 * builders still reject 2. Caller must own a coherent candidate snapshot;
 * this function neither enables hardware nor advertises CMD01 support. */
DAQ_FrameStatus DAQ_BuildUDPGEAR2CandidateFrame(const uint16_t *ch1,
    const uint16_t *ch2, const uint8_t *dig, uint32_t channel_mask,
    uint16_t sequence, uint8_t *frame, size_t frame_capacity);

/* V1.1 control/status protocol only. These interfaces neither execute commands
 * nor determine firmware/transport capabilities. A valid GEAR 2..4 request
 * must still be rejected by an execution layer that does not support it. */
#define DAQ_CONTROL_FRAME_SIZE 9U
#define DAQ_CONTROL_CRC_OFFSET 7U
#define DAQ_STATUS_FRAME_SIZE 12U
#define DAQ_STATUS_CRC_OFFSET 10U

typedef enum
{
  DAQ_CMD_SET_GEAR = 0x01,
  DAQ_CMD_SET_CHANNEL_MASK = 0x02,
  DAQ_CMD_SET_TRIGGER_SOURCE = 0x03,
  DAQ_CMD_SET_TRIGGER_MODE = 0x04,
  DAQ_CMD_SET_TRIGGER_LEVEL = 0x05,
  DAQ_CMD_REQUEST_STATUS = 0x06,
  DAQ_CMD_SET_WAVE_FREQUENCY = 0x07
} DAQ_CommandId;

typedef struct
{
  uint8_t command;
  uint16_t parameter; /* Little-endian wire value decoded to host integer. */
} DAQ_ControlCommand;

typedef enum
{
  DAQ_CONTROL_OK = 0,
  DAQ_CONTROL_BAD_ARGUMENT,
  DAQ_CONTROL_BAD_LENGTH,
  DAQ_CONTROL_BAD_HEADER,
  DAQ_CONTROL_BAD_TYPE,
  DAQ_CONTROL_BAD_CRC,
  DAQ_CONTROL_BAD_RESERVED,
  DAQ_CONTROL_UNKNOWN_COMMAND,
  DAQ_CONTROL_BAD_PARAMETER_HIGH,
  DAQ_CONTROL_PARAMETER_OUT_OF_RANGE
} DAQ_ControlParseStatus;

/* Exactly one stable 9-byte frame; not a UART stream/framing state machine.
 * Format errors (length/header/type/CRC/reserved), command errors and parameter
 * errors have distinct codes. On every failure, *command remains unchanged.
 * NULL arguments are rejected; non-NULL input must address length bytes. */
DAQ_ControlParseStatus DAQ_ParseControlFrame(const uint8_t *frame, size_t length,
                                            DAQ_ControlCommand *command);

/* Caller supplies a stable, truthful state; fields are deliberately wider
 * than their wire encoding so invalid values are rejected before narrowing.
 * No register reads, capability checks, trigger algorithm or UART access. */
typedef struct
{
  uint32_t gear;
  uint32_t channel_mask;
  uint32_t trigger_source;
  uint32_t trigger_mode;
  uint32_t trigger_level;
  uint32_t wave_frequency_hz;
} DAQ_StatusState;

typedef enum
{
  DAQ_STATUS_OK = 0,
  DAQ_STATUS_BAD_ARGUMENT,
  DAQ_STATUS_BUFFER_TOO_SMALL,
  DAQ_STATUS_PARAMETER_OUT_OF_RANGE
} DAQ_StatusBuildStatus;

/* V1.1 12-byte status frame. Validates all fields before writing any bytes.
 * Protocol ranges only: GEAR 0..4, CH_MASK 0..0x1F, TRIG_SRC 0..5,
 * TRIG_MODE 0..2, TRIG_LEVEL 0..255, WAVE_FREQ 100..10000 Hz.
 * No extra cross-field trigger restrictions are specified by V1.1.
 * On error the output is unchanged; success writes exactly 12 bytes. */
DAQ_StatusBuildStatus DAQ_BuildStatusFrame(const DAQ_StatusState *state,
                                         uint8_t *frame, size_t frame_capacity);

#ifdef __cplusplus
}
#endif

#endif
