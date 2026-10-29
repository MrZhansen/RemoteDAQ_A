/* Host-only integration test: the runner extracts the actual preview state
 * and poll function from main.c. Uses the real protocol/CRC implementation. */
#include "daq_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ch1_snapshot_type.inc"

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 }
  HAL_StatusTypeDef;

static unsigned int checks;
#define CHECK(condition) do { checks++; if (!(condition)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static CH1_Snapshot injected_snapshot;
static HAL_StatusTypeDef injected_status = HAL_BUSY;
static uint32_t barriers;
static void preview_barrier(void);
#define __DMB() preview_barrier()

static HAL_StatusTypeDef CH1_GetSnapshot(CH1_Snapshot *snapshot)
{
  if (injected_status == HAL_OK)
  {
    *snapshot = injected_snapshot;
  }
  return injected_status;
}

static HAL_StatusTypeDef Analog_GetSnapshot(Analog_Snapshot *snapshot)
{
  (void)snapshot;
  CHECK(0); /* Legacy CH1 preview must still use its original CH1 API. */
  return HAL_ERROR;
}

static HAL_StatusTypeDef Full_GetSnapshot(Full_Snapshot *snapshot)
{
  (void)snapshot;
  CHECK(0);
  return HAL_ERROR;
}

#include "ch1_preview_state.inc"
#include "ch1_preview_under_test.inc"

static void preview_barrier(void)
{
  uint32_t i;
  barriers++;
  CHECK((ch1_frame_preview_seq & 1U) != 0U);
  if (barriers == 2U)
  {
    CHECK(ch1_frame_preview_valid == 1U);
    CHECK(ch1_frame_group == injected_snapshot.group_count);
    CHECK(ch1_frame_preview_crc == DAQ_CRC16_CCITT_FALSE(ch1_preview_work, 258U));
    for (i = 0U; i < 260U; i++)
    {
      CHECK(ch1_frame_preview[i] == ch1_preview_work[i]);
    }
  }
}

static void set_group(uint32_t group, uint16_t seed)
{
  uint32_t i;
  injected_status = HAL_OK;
  injected_snapshot.group_count = group;
  for (i = 0U; i < 50U; i++)
  {
    injected_snapshot.samples[i] = (uint16_t)((seed + 13U * i) & 0xFFFU);
  }
}

static void poll_once(void)
{
  barriers = 0U;
  CH1_FramePreviewPoll();
}

static void check_frame(void)
{
  uint8_t copy[260];
  uint32_t i;
  uint32_t offset;
  uint16_t wire;
  CHECK(barriers == 2U);
  CHECK((ch1_frame_preview_seq & 1U) == 0U);
  CHECK(ch1_frame_preview_valid == 1U);
  for (i = 0U; i < 260U; i++)
  {
    copy[i] = ch1_frame_preview[i];
  }
  CHECK(copy[0] == 0xAAU && copy[1] == 0x55U && copy[2] == 1U);
  CHECK(copy[3] == 0U && copy[4] == 0U); /* Fixed preview SEQ, never advanced. */
  CHECK(copy[5] == 0U && copy[6] == 1U && copy[7] == 50U);
  for (i = 0U; i < 50U; i++)
  {
    offset = 8U + 5U * i;
    wire = (uint16_t)(copy[offset] | ((uint16_t)copy[offset + 1U] << 8U));
    CHECK(wire == (uint16_t)((uint32_t)injected_snapshot.samples[i] * 16U));
    CHECK(copy[offset + 2U] == 0U && copy[offset + 3U] == 0U);
    CHECK(copy[offset + 4U] == 0U);
  }
  CHECK(ch1_frame_preview_crc == DAQ_CRC16_CCITT_FALSE(copy, 258U));
  CHECK(ch1_frame_preview_crc == (uint32_t)(copy[258] | ((uint16_t)copy[259] << 8U)));
}

int main(void)
{
  uint8_t saved_frame[260];
  uint32_t saved_crc;
  uint32_t saved_seq;
  uint32_t i;

  CHECK(CH1_SNAPSHOT_SAMPLES == DAQ_CH1_SAMPLE_COUNT);
  poll_once(); /* No data published yet. */
  CHECK(ch1_snapshot_fail_count == 1U && ch1_snapshot_ok_count == 0U);
  CHECK(ch1_new_group_count == 0U && ch1_frame_build_count == 0U);
  CHECK(ch1_frame_preview_valid == 0U && barriers == 0U);
  CHECK(ch1_last_build_status == UINT32_MAX);

  set_group(0U, 0xABCU); /* First group ID zero is valid, not a duplicate. */
  poll_once();
  check_frame();
  CHECK(ch1_snapshot_ok_count == 1U && ch1_new_group_count == 1U);
  CHECK(ch1_frame_build_count == 1U && ch1_skipped_group_count == 0U);
  CHECK(ch1_frame_preview[8] == 0xC0U && ch1_frame_preview[9] == 0xABU);
  saved_crc = ch1_frame_preview_crc;
  saved_seq = ch1_frame_preview_seq;
  set_group(0U, 1U); /* Same group ID, deliberately different supplied data. */
  poll_once();
  CHECK(ch1_snapshot_ok_count == 2U && ch1_new_group_count == 1U);
  CHECK(ch1_frame_build_count == 1U && ch1_frame_preview_crc == saved_crc);
  CHECK(ch1_frame_preview_seq == saved_seq && barriers == 0U);

  injected_status = HAL_ERROR;
  poll_once();
  CHECK(ch1_snapshot_fail_count == 2U && ch1_new_group_count == 1U);
  CHECK(ch1_frame_build_count == 1U && ch1_frame_preview_seq == saved_seq);
  set_group(1U, 0x100U);
  poll_once();
  check_frame();
  CHECK(ch1_skipped_group_count == 0U);
  set_group(4U, 0x200U); /* Groups 2 and 3 were not observed. */
  poll_once();
  check_frame();
  CHECK(ch1_skipped_group_count == 2U && ch1_new_group_count == 3U);
  for (i = 0U; i < 260U; i++)
  {
    saved_frame[i] = ch1_frame_preview[i];
  }
  saved_crc = ch1_frame_preview_crc;
  saved_seq = ch1_frame_preview_seq;

  set_group(5U, 0x300U);
  injected_snapshot.samples[49] = 0xFFFFU;
  poll_once();
  CHECK(ch1_frame_error_count == 1U && ch1_frame_build_count == 3U);
  CHECK(ch1_new_group_count == 4U && ch1_skipped_group_count == 2U);
  CHECK(ch1_last_build_status == (uint32_t)DAQ_FRAME_ADC_OUT_OF_RANGE);
  CHECK(ch1_frame_group == 4U && ch1_frame_preview_crc == saved_crc);
  CHECK(ch1_frame_preview_seq == saved_seq && ch1_frame_preview_valid == 1U);
  for (i = 0U; i < 260U; i++)
  {
    CHECK(ch1_frame_preview[i] == saved_frame[i]);
  }
  set_group(5U, 0x300U); /* Failed group must not be attempted twice. */
  poll_once();
  CHECK(ch1_frame_error_count == 1U && ch1_new_group_count == 4U);
  CHECK(ch1_frame_build_count == 3U && barriers == 0U);
  injected_status = HAL_BUSY;
  poll_once(); /* Must not reuse the retained snapshot as fresh data. */
  CHECK(ch1_snapshot_fail_count == 3U && ch1_new_group_count == 4U);
  CHECK(ch1_frame_preview_seq == saved_seq);
  set_group(6U, 0x400U);
  poll_once();
  check_frame();
  CHECK(ch1_new_group_count == 5U && ch1_frame_build_count == 4U);
  CHECK(ch1_skipped_group_count == 2U && ch1_last_build_status == 0U);

  /* Model a fresh consumer starting near the publisher's 32-bit wrap. */
  ch1_preview_have_group = 0U;
  ch1_skipped_group_count = 0U;
  set_group(UINT32_MAX - 1U, 1U);
  poll_once();
  check_frame();
  CHECK(ch1_skipped_group_count == 0U); /* First group is only a baseline. */
  set_group(UINT32_MAX, 2U);
  poll_once();
  check_frame();
  set_group(0U, 3U);
  poll_once();
  check_frame();
  CHECK(ch1_skipped_group_count == 0U);
  set_group(2U, 4U);
  poll_once();
  check_frame();
  CHECK(ch1_skipped_group_count == 1U);
  ch1_skipped_group_count = UINT32_MAX;
  set_group(4U, 5U);
  poll_once();
  check_frame();
  CHECK(ch1_skipped_group_count == 0U); /* Diagnostic itself is modulo 2^32. */
  ch1_frame_preview_seq = UINT32_MAX - 1U;
  set_group(5U, 6U);
  poll_once();
  check_frame();
  CHECK(ch1_frame_preview_seq == 0U);
  CHECK(ch1_new_group_count == ch1_frame_build_count + ch1_frame_error_count);

  printf("PASS preview integration: %u checks; first ID zero, duplicate "
         "suppression, failed reads/builds, retained frame, gaps, wrap, "
         "fixed SEQ zero, guarded publication.\n", checks);
  return 0;
}
