/* Host-only: real dual reader/publisher (with simulated preemption), and
 * real protocol module. Never compiled into the firmware target. */
#include "daq_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch1_snapshot_type.inc"

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 }
  HAL_StatusTypeDef;
#define __root
#define CH1_GROUP_SAMPLES CH1_SNAPSHOT_SAMPLES
#define ADC_SCAN_CHANNELS 2U
static unsigned int checks;
#define CHECK(condition) do { checks++; if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static volatile uint16_t adc_samples[50];
static volatile uint16_t adc_ch2_samples[50];
static volatile uint16_t adc_dma_buffer[200];
static volatile uint32_t adc_snapshot_seq;
static volatile uint32_t adc_snapshot_valid;
static volatile uint32_t adc_group_count;
static volatile uint32_t adc_group_ready;
static volatile uint32_t adc_running;
static volatile uint32_t adc_value;
static volatile uint32_t adc_ch2_value;
static volatile uint32_t adc_group_period_ms;
static volatile uint32_t adc_group_cycles;
static volatile uint32_t adc_last_group_tick;
static uint32_t adc_previous_group_cycle;
static struct { uint32_t CYCCNT; } fake_dwt;
#define DWT (&fake_dwt)
static uint32_t HAL_GetTick(void) { return 100U; }
static uint32_t barrier_count;
static uint32_t inject_barrier;
static uint32_t inject_mid_copy;
static void test_barrier(void);
static void test_copy_hook(uint32_t index);
#define __DMB() test_barrier()
#define ANALOG_COPY_TEST_HOOK(index) test_copy_hook(index)
#include "analog_snapshot_under_test.inc"

static void test_barrier(void)
{
  barrier_count++;
  if (inject_barrier != 0U && barrier_count == inject_barrier)
  {
    inject_barrier = 0U;
    CH1_PublishGroup(100U);
  }
}

static void test_copy_hook(uint32_t index)
{
  if (inject_mid_copy != 0U && index == 25U)
  {
    inject_mid_copy = 0U;
    CH1_PublishGroup(0U); /* Update between one CH1 read and its CH2 read. */
  }
}

static void test_snapshots(void)
{
  Analog_Snapshot output;
  Analog_Snapshot saved;
  uint32_t i;
  uint32_t test;

  memset(&output, 0x5A, sizeof(output));
  memcpy(&saved, &output, sizeof(saved));
  CHECK(Analog_GetSnapshot(NULL) == HAL_ERROR);
  CHECK(Analog_GetSnapshot(&output) == HAL_BUSY);
  CHECK(memcmp(&output, &saved, sizeof(output)) == 0);
  adc_snapshot_seq = 1U;
  barrier_count = 0U;
  CHECK(Analog_GetSnapshot(&output) == HAL_BUSY && barrier_count == 0U);
  CHECK(memcmp(&output, &saved, sizeof(output)) == 0);
  adc_snapshot_seq = 0U;
  for (i = 0U; i < 100U; i++)
  {
    adc_dma_buffer[2U * i] = (uint16_t)(1000U + i); /* Rank 1 = logical CH2. */
    adc_dma_buffer[2U * i + 1U] = (uint16_t)(10U + i); /* Rank 2 = CH1. */
  }
  adc_running = 1U;
  CH1_PublishGroup(0U);
  CHECK(Analog_GetSnapshot(&output) == HAL_OK && output.group_count == 1U);
  for (i = 0U; i < 50U; i++)
  {
    CHECK(output.ch1[i] == 10U + i);
    CHECK(output.ch2[i] == 1000U + i);
  }

  /* Publications before copying, after copying, and half-way through the
   * interleaved copy must all reject the candidate without changing output. */
  for (test = 1U; test <= 3U; test++)
  {
    memcpy(&saved, &output, sizeof(saved));
    barrier_count = 0U;
    inject_barrier = (test < 3U) ? test : 0U;
    inject_mid_copy = (test == 3U) ? 1U : 0U;
    CHECK(Analog_GetSnapshot(&output) == HAL_BUSY);
    CHECK(memcmp(&output, &saved, sizeof(output)) == 0);
    CHECK(barrier_count == 4U); /* One bounded read plus one complete writer. */
  }
  CHECK(Analog_GetSnapshot(&output) == HAL_OK && output.group_count == 4U);
  for (i = 0U; i < 50U; i++)
  {
    CHECK(output.ch1[i] == 10U + i && output.ch2[i] == 1000U + i);
  }
  adc_group_ready = 0U;
  CHECK(Analog_GetSnapshot(&output) == HAL_OK);
  adc_snapshot_seq = UINT32_MAX - 1U;
  adc_group_count = UINT32_MAX;
  CH1_PublishGroup(100U);
  CHECK(adc_snapshot_seq == 0U && adc_group_count == 0U);
  CHECK(Analog_GetSnapshot(&output) == HAL_OK && output.group_count == 0U);
  for (i = 0U; i < 50U; i++)
  {
    CHECK(output.ch1[i] == 60U + i && output.ch2[i] == 1050U + i);
  }
}

/* Independent polynomial division oracle, not a copy of the CRC function. */
static uint16_t reference_crc(const uint8_t *bytes, size_t length)
{
  uint32_t remainder = 0xFFFFU;
  size_t i;
  int bit;
  for (i = 0U; i < length; i++)
  {
    remainder = (remainder << 8U) ^ ((uint32_t)bytes[i] << 16U);
    for (bit = 23; bit >= 16; bit--)
    {
      if ((remainder & ((uint32_t)1U << bit)) != 0U)
      {
        remainder ^= (uint32_t)0x11021U << (bit - 16);
      }
    }
  }
  return (uint16_t)remainder;
}

static void test_frames(void)
{
  static const uint16_t sequences[] = {0U, 1U, 0xFFU, 0x100U, 0x1234U, 0x8000U, 0xFFFEU, 0xFFFFU};
  static const uint16_t boundaries[] = {0U, 1U, 0xABCU, 0xFFFU};
  static const uint16_t invalid[] = {0x1000U, 0x8000U, 0xFFFFU};
  uint16_t ch1[50];
  uint16_t ch2[50];
  uint16_t *channel;
  uint8_t storage[264];
  uint8_t saved[264];
  uint8_t expected[260];
  uint8_t *frame = storage + 2;
  uint32_t i;
  uint32_t j;
  uint32_t k;
  uint32_t mode;
  uint32_t offset;
  uint32_t encoded;
  uint16_t crc;
  uint16_t old;

  CHECK(DAQ_ANALOG_SAMPLE_COUNT == 50U && DAQ_DATA_FRAME_SIZE == 260U);
  for (mode = 0U; mode < 17U; mode++)
  {
    for (i = 0U; i < 50U; i++)
    {
      ch1[i] = (mode == 0U) ? (uint16_t)((i * 61U + 0x14U) & 0xFFFU) :
          boundaries[(mode - 1U) / 4U];
      ch2[i] = (mode == 0U) ? (uint16_t)((0xFFFU - i * 47U) & 0xFFFU) :
          boundaries[(mode - 1U) % 4U];
    }
    if (mode == 0U) { ch1[0] = 0xABCU; ch2[0] = 0xDEFU; }
    for (j = 0U; j < 8U; j++)
    {
      memset(storage, 0xA5, sizeof(storage));
      CHECK(DAQ_BuildAnalogFrame(ch1, ch2, sequences[j], frame, 260U) == DAQ_FRAME_OK);
      CHECK(storage[0] == 0xA5U && storage[1] == 0xA5U);
      CHECK(storage[262] == 0xA5U && storage[263] == 0xA5U);
      memset(expected, 0, sizeof(expected));
      expected[0] = 0xAAU; expected[1] = 0x55U; expected[2] = 1U;
      expected[3] = (uint8_t)(sequences[j] % 256U);
      expected[4] = (uint8_t)(sequences[j] / 256U);
      expected[6] = 3U; expected[7] = 50U;
      for (i = 0U; i < 50U; i++)
      {
        offset = 8U + 5U * i;
        encoded = (uint32_t)ch1[i] * 16U;
        expected[offset] = (uint8_t)(encoded % 256U);
        expected[offset + 1U] = (uint8_t)(encoded / 256U);
        encoded = (uint32_t)ch2[i] * 16U;
        expected[offset + 2U] = (uint8_t)(encoded % 256U);
        expected[offset + 3U] = (uint8_t)(encoded / 256U);
        CHECK(frame[offset + 4U] == 0U);
      }
      crc = reference_crc(expected, 258U);
      expected[258] = (uint8_t)(crc % 256U);
      expected[259] = (uint8_t)(crc / 256U);
      CHECK(memcmp(expected, frame, 260U) == 0);
      if (mode == 0U)
      {
        CHECK(frame[8] == 0xC0U && frame[9] == 0xABU);
        CHECK(frame[10] == 0xF0U && frame[11] == 0xDEU);
      }
    }
  }
  crc = reference_crc(frame, 258U);
  for (i = 0U; i < 258U; i++)
  {
    for (j = 0U; j < 8U; j++)
    {
      frame[i] ^= (uint8_t)(1U << j);
      CHECK(DAQ_CRC16_CCITT_FALSE(frame, 258U) != crc);
      frame[i] ^= (uint8_t)(1U << j);
    }
  }
  frame[258] ^= 0xFFU; frame[259] ^= 0xFFU;
  CHECK(DAQ_CRC16_CCITT_FALSE(frame, 258U) == crc);
  memset(storage, 0x5A, sizeof(storage));
  memcpy(saved, storage, sizeof(storage));
  for (mode = 0U; mode < 2U; mode++)
  {
    channel = (mode == 0U) ? ch1 : ch2;
    for (i = 0U; i < 50U; i++)
    {
      old = channel[i];
      for (k = 0U; k < 3U; k++)
      {
        channel[i] = invalid[k];
        CHECK(DAQ_BuildAnalogFrame(ch1, ch2, 0U, frame, 260U) == DAQ_FRAME_ADC_OUT_OF_RANGE);
        CHECK(memcmp(storage, saved, sizeof(storage)) == 0);
      }
      channel[i] = old;
    }
  }
  CHECK(DAQ_BuildAnalogFrame(NULL, ch2, 0U, frame, 260U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildAnalogFrame(ch1, NULL, 0U, frame, 260U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildAnalogFrame(ch1, ch2, 0U, NULL, 260U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildAnalogFrame(ch1, ch2, 0U, frame, 259U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(memcmp(storage, saved, sizeof(storage)) == 0);
}

int main(void)
{
  test_snapshots();
  test_frames();
  printf("PASS dual analog: %u checks; coherent 2x50 snapshots, mid-copy IRQ, "
         "both DMA halves, group wrap, full frames, 16 boundary pairs, "
         "8 SEQ cases, 300 invalid inputs, CRC/body/byte order.\n", checks);
  return 0;
}
