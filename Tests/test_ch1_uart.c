/* Host-only tests of extracted firmware services, using real frame/CRC code
 * and a HAL UART stub. No serial port or development board is accessed. */
#include "daq_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch1_snapshot_type.inc"
#include "ch1_service_defines.inc"

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 }
  HAL_StatusTypeDef;
#ifdef UART_CONTROL_TEST
typedef struct { uint32_t error; FakeUartRegs *Instance; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
#else
typedef struct { uint32_t error; } UART_HandleTypeDef;
#endif
static UART_HandleTypeDef huart6;
static volatile uint32_t uart6_initialized = 1U;
static volatile uint32_t adc_snapshot_valid;
static volatile uint32_t adc_group_count;
static volatile uint32_t digital_input_d1;
static volatile uint32_t digital_input_d2;
static volatile uint32_t digital_input_d3;

#define GPIOG 1U
#define GPIOI 2U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_3 8U
#define GPIO_PIN_SET 1U
#define HAL_UART_ERROR_NONE 0U

static unsigned int checks;
#define CHECK(condition) do { checks++; if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static Full_Snapshot source;
static HAL_StatusTypeDef snapshot_status = HAL_OK;
static HAL_StatusTypeDef transmit_status = HAL_OK;
static uint32_t snapshot_calls;
static uint32_t transmit_calls;
static uint32_t fake_tick;
static uint32_t gpio_calls;
static uint32_t publish_during_send;
static uint32_t partial_bytes;
static uint32_t expected_timeout = 10U;
static uint8_t captured_frame[260];

static void barrier(void);
#ifdef UART_CONTROL_TEST
static void control_barrier(void);
#endif
#define __DMB() barrier()
static uint32_t HAL_GetTick(void) { return fake_tick; }
static uint32_t HAL_UART_GetError(UART_HandleTypeDef *uart) { return uart->error; }
static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
    const uint8_t *data, uint16_t size, uint32_t timeout);
static uint32_t HAL_GPIO_ReadPin(uint32_t port, uint32_t pin)
{
  gpio_calls++;
  CHECK((port == GPIOG && (pin == GPIO_PIN_6 || pin == GPIO_PIN_7)) ||
        (port == GPIOI && pin == GPIO_PIN_3));
  return (pin == GPIO_PIN_7) ? 0U : GPIO_PIN_SET;
}
static HAL_StatusTypeDef CH1_GetSnapshot(CH1_Snapshot *snapshot)
{
  (void)snapshot;
  CHECK(0); /* Production must not independently read only CH1. */
  return HAL_ERROR;
}
static HAL_StatusTypeDef Analog_GetSnapshot(Analog_Snapshot *snapshot)
{
  (void)snapshot;
  CHECK(0);
  return HAL_ERROR;
}
static HAL_StatusTypeDef Full_GetSnapshot(Full_Snapshot *snapshot)
{
  snapshot_calls++;
  if (snapshot_status == HAL_OK) { *snapshot = source; }
  return snapshot_status;
}

#include "ch1_preview_state.inc"
#include "ch1_uart_state.inc"
#include "analog_frame_builder.inc"
#include "ch1_uart_under_test.inc"

static void barrier(void)
{
  uint32_t i;
#ifdef UART_CONTROL_TEST
  control_barrier();
#endif
  /* An even UART snapshot sequence may become visible only after the prior
   * successful frame and all corresponding metadata have been copied. */
  if ((ch1_uart_last_snapshot_seq & 1U) != 0U && ch1_uart_last_valid != 0U &&
      ch1_uart_last_seq == (uint16_t)(captured_frame[3] | ((uint16_t)captured_frame[4] << 8U)))
  {
    for (i = 0U; i < 260U; i++) { CHECK(ch1_uart_last_frame[i] == captured_frame[i]); }
  }
}

static void set_group(uint32_t group, uint16_t value)
{
  uint32_t i;
  adc_snapshot_valid = 1U;
  adc_group_count = group;
  source.group_count = group;
  for (i = 0U; i < 50U; i++)
  {
    source.ch1[i] = value;
    source.ch2[i] = (uint16_t)(value ^ 0xFFFU);
    source.dig[i] = (uint8_t)(i & 7U);
  }
}

#ifdef UART_CONTROL_TEST
static HAL_StatusTypeDef control_transmit(UART_HandleTypeDef *uart,
    const uint8_t *data, uint16_t size, uint32_t timeout);
#endif
static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
    const uint8_t *data, uint16_t size, uint32_t timeout)
{
#ifdef UART_CONTROL_TEST
  return control_transmit(uart, data, size, timeout);
#else
  uint16_t crc;
  uint32_t i;
  CHECK(uart == &huart6);
  CHECK(size == 260U && timeout == expected_timeout);
  CHECK(data == ch1_preview_work);
  CHECK(data[0] == 0xAAU && data[1] == 0x55U && data[2] == 1U);
  CHECK(data[5] == 0U && data[6] == 0x1FU && data[7] == 50U);
  CHECK((uint16_t)(data[3] | ((uint16_t)data[4] << 8U)) == ch1_uart_next_seq);
  crc = DAQ_CRC16_CCITT_FALSE(data, 258U);
  CHECK(crc == (uint16_t)(data[258] | ((uint16_t)data[259] << 8U)));
  for (i = 0U; i < 50U; i++)
  {
    CHECK((uint16_t)(data[8U + 5U * i] |
          ((uint16_t)data[9U + 5U * i] << 8U)) == (uint16_t)(source.ch1[i] * 16U));
    CHECK((uint16_t)(data[10U + 5U * i] |
          ((uint16_t)data[11U + 5U * i] << 8U)) == (uint16_t)(source.ch2[i] * 16U));
    CHECK(data[12U + 5U * i] == source.dig[i]);
    CHECK((data[12U + 5U * i] & 0xF8U) == 0U);
  }
  transmit_calls++;
  memcpy(captured_frame, data, 260U);
  partial_bytes = (transmit_status == HAL_TIMEOUT) ? 17U : 260U;
  if (publish_during_send != 0U)
  {
    set_group(publish_during_send, 0x321U);
    publish_during_send = 0U;
  }
  /* Even if DMA publishes new data while HAL polls, the actual argument
   * buffer must remain byte-for-byte unchanged until the call returns. */
  CHECK(memcmp(captured_frame, data, 260U) == 0);
  fake_tick += (transmit_status == HAL_TIMEOUT) ? 11U : 3U;
  return transmit_status;
#endif
}

static void check_success_frame(uint32_t group, uint16_t sequence)
{
  uint32_t i;
  CHECK(ch1_uart_last_valid == 1U);
  CHECK((ch1_uart_last_snapshot_seq & 1U) == 0U);
  CHECK(ch1_uart_last_group == group && ch1_uart_last_seq == sequence);
  for (i = 0U; i < 260U; i++) { CHECK(ch1_uart_last_frame[i] == captured_frame[i]); }
  CHECK(ch1_uart_last_frame[3] == (uint8_t)sequence);
  CHECK(ch1_uart_last_frame[4] == (uint8_t)(sequence >> 8U));
}

int main(void)
{
  uint8_t saved_frame[260];
  uint32_t i;
  uint32_t calls;
  uint32_t failures;
  uint32_t snapshots;
  uint32_t builds;
  uint16_t crc_before;
  uint16_t held_sequence;

  CHECK(sizeof(ch1_uart_next_seq) == 2U && sizeof(ch1_uart_last_seq) == 2U);
  CHECK(CH1_FrameTransmitPoll() == 0U && transmit_calls == 0U && snapshot_calls == 0U);
  set_group(0U, 0xABCU);
  CHECK(CH1_FrameTransmitPoll() == 1U);
  CHECK(transmit_calls == 1U && ch1_uart_tx_count == 1U && ch1_uart_next_seq == 1U);
  CHECK(captured_frame[8] == 0xC0U && captured_frame[9] == 0xABU);
  CHECK(captured_frame[10] == 0x30U && captured_frame[11] == 0x54U);
  check_success_frame(0U, 0U);
  calls = transmit_calls;
  snapshots = snapshot_calls;
  builds = ch1_frame_build_count;
  for (i = 0U; i < 100U; i++) { CHECK(CH1_FrameTransmitPoll() == 0U); }
  CHECK(transmit_calls == calls && snapshot_calls == snapshots);
  CHECK(ch1_frame_build_count == builds); /* Cheap idle path, no repeated copies/builds. */
  crc_before = (uint16_t)(captured_frame[258] | ((uint16_t)captured_frame[259] << 8U));
  set_group(1U, 0xABCU); /* Same ADC values, a new SEQ must still get a fresh CRC. */
  CHECK(CH1_FrameTransmitPoll() == 1U);
  CHECK(ch1_uart_tx_count == 2U && ch1_uart_next_seq == 2U);
  CHECK(crc_before != (uint16_t)(captured_frame[258] | ((uint16_t)captured_frame[259] << 8U)));
  check_success_frame(1U, 1U);
  for (i = 0U; i < 260U; i++) { saved_frame[i] = ch1_uart_last_frame[i]; }

  /* All failure classes retain SEQ/last-success data and consume the failed
   * sampling group once. HAL error info is recorded separately from status. */
  for (i = (uint32_t)HAL_ERROR; i <= (uint32_t)HAL_TIMEOUT; i++)
  {
    transmit_status = (HAL_StatusTypeDef)i;
    huart6.error = 0x10U + i;
    set_group(i + 1U, 0x100U);
    failures = ch1_uart_tx_error_count;
    CHECK(CH1_FrameTransmitPoll() == 1U);
    CHECK(ch1_uart_last_tx_status == i && ch1_uart_hal_error == 0x10U + i);
    CHECK(ch1_uart_tx_error_count == failures + 1U && ch1_uart_tx_count == 2U);
    CHECK(ch1_uart_next_seq == 2U && ch1_uart_last_seq == 1U && ch1_uart_last_group == 1U);
    for (calls = 0U; calls < 260U; calls++) { CHECK(ch1_uart_last_frame[calls] == saved_frame[calls]); }
    calls = transmit_calls;
    CHECK(CH1_FrameTransmitPoll() == 0U && transmit_calls == calls);
  }
  CHECK(partial_bytes == 17U); /* Modeled partial timeout: old group is not retried. */
  transmit_status = HAL_OK;
  huart6.error = 0U;
  set_group(5U, 0x222U);
  publish_during_send = 8U; /* Groups 6/7 are superseded while sending group 5. */
  CHECK(CH1_FrameTransmitPoll() == 1U);
  check_success_frame(5U, 2U);
  CHECK(adc_group_count == 8U && ch1_uart_next_seq == 3U);
  CHECK(CH1_FrameTransmitPoll() == 1U);
  check_success_frame(8U, 3U);
  CHECK(ch1_skipped_group_count == 2U);

  set_group(9U, 0x100U);
  source.ch1[49] = 0xFFFFU;
  calls = transmit_calls;
  held_sequence = ch1_uart_next_seq;
  CHECK(CH1_FrameTransmitPoll() == 1U && transmit_calls == calls);
  CHECK(ch1_uart_next_seq == held_sequence && ch1_frame_error_count == 1U);
  CHECK(CH1_FrameTransmitPoll() == 0U);
  set_group(10U, 0x100U);
  snapshot_status = HAL_BUSY;
  CHECK(CH1_FrameTransmitPoll() == 0U && transmit_calls == calls);
  snapshot_status = HAL_OK;
  CHECK(CH1_FrameTransmitPoll() == 1U);
  check_success_frame(10U, held_sequence);

  ch1_uart_next_seq = 0xFFFFU;
  set_group(11U, 0x300U);
  CHECK(CH1_FrameTransmitPoll() == 1U && ch1_uart_next_seq == 0U);
  check_success_frame(11U, 0xFFFFU);
  set_group(12U, 0x300U);
  CHECK(CH1_FrameTransmitPoll() == 1U && ch1_uart_next_seq == 1U);
  check_success_frame(12U, 0U);

  uart6_initialized = 0U;
  set_group(13U, 0x400U);
  calls = transmit_calls;
  failures = ch1_uart_tx_error_count;
  CHECK(CH1_FrameTransmitPoll() == 1U && transmit_calls == calls);
  CHECK(ch1_uart_not_ready_count == 1U && ch1_uart_tx_error_count == failures);
  CHECK(ch1_frame_group == 13U && ch1_uart_next_seq == 1U);
  uart6_initialized = 1U;
  CHECK(CH1_FrameTransmitPoll() == 0U && transmit_calls == calls);
  set_group(14U, 0x400U);
  CHECK(CH1_FrameTransmitPoll() == 1U);
  check_success_frame(14U, 1U);
  /* An illegal CH2 alone must not publish or send a new frame. */
  calls = transmit_calls;
  set_group(15U, 0x400U);
  source.ch2[25] = 0xFFFFU;
  held_sequence = ch1_uart_next_seq;
  CHECK(CH1_FrameTransmitPoll() == 1U && transmit_calls == calls);
  CHECK(ch1_uart_next_seq == held_sequence && ch1_frame_error_count == 2U);
  CHECK(ch1_frame_group == 14U && ch1_uart_last_group == 14U);
  set_group(16U, 0x400U);
  source.dig[49] = 8U;
  CHECK(CH1_FrameTransmitPoll() == 1U && transmit_calls == calls);
  CHECK(ch1_uart_next_seq == held_sequence && ch1_frame_error_count == 3U);
  CHECK(ch1_frame_group == 14U && ch1_uart_last_group == 14U);

  ch1_preview_have_group = 0U; /* Model a consumer starting near group wrap. */
  ch1_skipped_group_count = 0U;
  set_group(UINT32_MAX, 0x500U);
  CHECK(CH1_FrameTransmitPoll() == 1U);
  set_group(0U, 0x500U);
  CHECK(CH1_FrameTransmitPoll() == 1U && ch1_skipped_group_count == 0U);
  check_success_frame(0U, 3U);

  /* GPIO polling remains ~10 ms and all tick/counter differences wrap safely. */
  fake_tick = 100U;
  background_gpio_tick = 90U;
  background_rate_tick = 100U;
  background_rate_count = ch1_uart_tx_count;
  CH1_BackgroundPoll();
  CHECK(gpio_calls == 3U && digital_input_d1 == 1U && digital_input_d2 == 0U && digital_input_d3 == 1U);
  fake_tick = 109U;
  CH1_BackgroundPoll();
  CHECK(gpio_calls == 3U);
  fake_tick = 110U;
  CH1_BackgroundPoll();
  CHECK(gpio_calls == 6U);
  background_gpio_tick = UINT32_MAX - 4U;
  fake_tick = 4U;
  CH1_BackgroundPoll();
  CHECK(gpio_calls == 6U);
  fake_tick = 5U;
  CH1_BackgroundPoll();
  CHECK(gpio_calls == 9U);
  background_rate_tick = UINT32_MAX - 499U;
  background_rate_count = UINT32_MAX - 1U;
  ch1_uart_tx_count = 1U;
  ch1_uart_rate_elapsed_ms = 0U;
  fake_tick = 499U;
  CH1_BackgroundPoll();
  CHECK(ch1_uart_rate_elapsed_ms == 0U);
  fake_tick = 500U;
  CH1_BackgroundPoll();
  CHECK(ch1_uart_rate_elapsed_ms == 1000U && ch1_uart_rate_frames == 3U);

  printf("PASS UART services (five channels): %u checks; 260-byte sends, success-only SEQ, "
         "CRC, failures/partial timeout, latest-only data, stable buffers, "
         "group/SEQ/tick wrap, GPIO/rate schedule, idle gating.\n", checks);
  return 0;
}
