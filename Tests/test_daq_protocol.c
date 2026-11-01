/* Host-only tests. This file is deliberately not part of the IAR target. */
#include "daq_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int checks;
#define CHECK(condition) do { checks++; if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

/* Independent byte-wide polynomial division, rather than the production
 * function's bit-at-a-time 16-bit shift register. */
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

int main(void)
{
  static const uint8_t vector[] = "123456789";
  static const uint16_t sequences[] = {
    0x0000U, 0x0001U, 0x00FFU, 0x0100U,
    0x1234U, 0x8000U, 0xFFFEU, 0xFFFFU
  };
  static const uint16_t invalid_values[] = {0x1000U, 0x8000U, 0xFFFFU};
  uint16_t samples[DAQ_CH1_SAMPLE_COUNT];
  uint16_t ch2[DAQ_CH1_SAMPLE_COUNT];
  uint8_t dig[DAQ_CH1_SAMPLE_COUNT];
  uint8_t storage[DAQ_DATA_FRAME_SIZE + 4U];
  uint8_t saved_storage[sizeof(storage)];
  uint8_t expected[DAQ_DATA_FRAME_SIZE];
  uint8_t *frame = &storage[2];
  uint8_t byte;
  uint16_t crc;
  uint16_t saved_sample;
  uint32_t encoded;
  size_t i;
  size_t j;
  size_t k;
  size_t offset;

  CHECK(DAQ_DATA_FRAME_SIZE == 260U);
  CHECK(DAQ_DATA_CRC_OFFSET == 258U);
  CHECK(DAQ_CH1_SAMPLE_COUNT == 50U);
  CHECK(DAQ_CRC16_CCITT_FALSE(vector, sizeof(vector) - 1U) == 0x29B1U);
  CHECK(reference_crc(vector, sizeof(vector) - 1U) == 0x29B1U);
  CHECK(DAQ_CRC16_CCITT_FALSE(NULL, 0U) == 0xFFFFU);
  for (i = 0U; i < 256U; i++)
  {
    byte = (uint8_t)i;
    CHECK(DAQ_CRC16_CCITT_FALSE(&byte, 1U) == reference_crc(&byte, 1U));
  }

  for (i = 0U; i < DAQ_CH1_SAMPLE_COUNT; i++)
  {
    samples[i] = (uint16_t)((i * 83U + 0x123U) & 0xFFFU);
    ch2[i] = (uint16_t)(0xFFFU - samples[i]);
    dig[i] = (uint8_t)(i & 7U);
  }
  samples[0] = 0U;
  samples[1] = 0xFFFU;
  samples[2] = 0xABCU;
  samples[49] = 0x135U;

  for (j = 0U; j < sizeof(sequences) / sizeof(sequences[0]); j++)
  {
    memset(storage, 0xA5, sizeof(storage));
    CHECK(DAQ_BuildCH1Frame(samples, sequences[j], frame,
                            DAQ_DATA_FRAME_SIZE) == DAQ_FRAME_OK);
    CHECK(storage[0] == 0xA5U && storage[1] == 0xA5U);
    CHECK(storage[262] == 0xA5U && storage[263] == 0xA5U);
    memset(expected, 0, sizeof(expected));
    expected[0] = 0xAAU;
    expected[1] = 0x55U;
    expected[2] = 1U;
    expected[3] = (uint8_t)(sequences[j] % 256U);
    expected[4] = (uint8_t)(sequences[j] / 256U);
    expected[6] = 1U;
    expected[7] = 50U;
    for (i = 0U; i < 50U; i++)
    {
      offset = 8U + 5U * i;
      encoded = (uint32_t)samples[i] * 16U;
      expected[offset] = (uint8_t)(encoded % 256U);
      expected[offset + 1U] = (uint8_t)(encoded / 256U);
      CHECK(frame[offset + 2U] == 0U && frame[offset + 3U] == 0U);
      CHECK(frame[offset + 4U] == 0U);
    }
    crc = reference_crc(expected, 258U);
    expected[258] = (uint8_t)(crc % 256U);
    expected[259] = (uint8_t)(crc / 256U);
    CHECK(memcmp(frame, expected, 260U) == 0);
    CHECK(frame[18] == 0xC0U && frame[19] == 0xABU);
    CHECK((uint16_t)(frame[258] | ((uint16_t)frame[259] << 8U)) == crc);
  }

  /* A retained CRC must reject every single-bit change in the 258-byte body,
   * including both header bytes and the final DIG byte. */
  crc = reference_crc(frame, 258U);
  for (i = 0U; i < 258U; i++)
  {
    for (k = 0U; k < 8U; k++)
    {
      frame[i] ^= (uint8_t)(1U << k);
      CHECK(DAQ_CRC16_CCITT_FALSE(frame, 258U) != crc);
      frame[i] ^= (uint8_t)(1U << k);
    }
  }
  /* CRC-field bytes are excluded from recomputation. */
  frame[258] ^= 0xFFU;
  frame[259] ^= 0xFFU;
  CHECK(DAQ_CRC16_CCITT_FALSE(frame, 258U) == crc);

  memset(storage, 0x5A, sizeof(storage));
  memcpy(saved_storage, storage, sizeof(storage));
  for (i = 0U; i < 50U; i++)
  {
    saved_sample = samples[i];
    for (j = 0U; j < sizeof(invalid_values) / sizeof(invalid_values[0]); j++)
    {
      samples[i] = invalid_values[j];
      CHECK(DAQ_BuildCH1Frame(samples, 0U, frame, 260U) ==
            DAQ_FRAME_ADC_OUT_OF_RANGE);
      CHECK(memcmp(saved_storage, storage, sizeof(storage)) == 0);
    }
    samples[i] = saved_sample;
  }
  CHECK(DAQ_BuildCH1Frame(samples, 0U, frame, 259U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildCH1Frame(samples, 0U, frame, 0U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildCH1Frame(NULL, 0U, frame, 260U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(DAQ_BuildCH1Frame(samples, 0U, NULL, 260U) == DAQ_FRAME_BAD_ARGUMENT);
  CHECK(memcmp(saved_storage, storage, sizeof(storage)) == 0);

  /* All production high-rate GEAR values use the same 260-byte V1.1 body.
   * GEAR2..4 must encode their own value rather than aliasing GEAR2. */
  for (j = 2U; j <= 4U; j++)
  {
    memset(storage, 0xA5, sizeof(storage));
    CHECK(DAQ_BuildSelectedFrameForGear(samples, ch2, dig, (uint32_t)j, 0x1FU,
          (uint16_t)(0x1200U + j), frame, DAQ_DATA_FRAME_SIZE) == DAQ_FRAME_OK);
    CHECK(frame[3] == (uint8_t)(0x1200U + j) && frame[4] == 0x12U);
    CHECK(frame[5] == (uint8_t)j && frame[6] == 0x1FU && frame[7] == 50U);
    CHECK(storage[0] == 0xA5U && storage[1] == 0xA5U);
    CHECK(storage[262] == 0xA5U && storage[263] == 0xA5U);
    CHECK(frame[DAQ_DATA_CRC_OFFSET] == (uint8_t)reference_crc(frame, 258U));
    CHECK(frame[DAQ_DATA_CRC_OFFSET + 1U] ==
          (uint8_t)(reference_crc(frame, 258U) >> 8U));
  }

  printf("PASS protocol: %u checks; CRC vector, full 260-byte frames, 8 SEQ "
         "boundaries, GEAR2..4, 2064 bit mutations, 150 invalid ADC inputs.\n",
         checks);
  return 0;
}
