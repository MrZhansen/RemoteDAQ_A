/* Host-only test: run_host_tests.ps1 extracts the ACTUAL firmware functions
 * and exported snapshot type into the includes below before compilation. */
#include <stdint.h>
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

static uint32_t barriers;
static uint32_t inject_at_barrier;
static uint32_t watch_publication;
static uint32_t expected_group;
static uint32_t expected_offset;
static void snapshot_barrier(void);
#define __DMB() snapshot_barrier()

#include "ch1_snapshot_under_test.inc"

static void snapshot_barrier(void)
{
  uint32_t i;
  barriers++;
  if (watch_publication != 0U)
  {
    CHECK((adc_snapshot_seq & 1U) != 0U);
    if (barriers == 2U)
    {
      CHECK(adc_group_count == expected_group);
      CHECK(adc_snapshot_valid == 1U);
      CHECK(adc_group_ready == 1U);
      for (i = 0U; i < 50U; i++)
      {
        CHECK(adc_samples[i] == adc_dma_buffer[expected_offset + 2U * i + 1U]);
        CHECK(adc_ch2_samples[i] == adc_dma_buffer[expected_offset + 2U * i]);
      }
    }
  }
  if ((inject_at_barrier != 0U) && (barriers == inject_at_barrier))
  {
    inject_at_barrier = 0U;
    CH1_PublishGroup(100U);  /* Simulate IRQ preemption, then return to reader. */
  }
}

int main(void)
{
  CH1_Snapshot output;
  CH1_Snapshot untouched;
  uint32_t i;

  CHECK(CH1_SNAPSHOT_SAMPLES == 50U);
  memset(&output, 0x5A, sizeof(output));
  memcpy(&untouched, &output, sizeof(output));
  CHECK(CH1_GetSnapshot(NULL) == HAL_ERROR);
  CHECK(barriers == 0U);
  CHECK(CH1_GetSnapshot(&output) == HAL_BUSY); /* Initial data is not valid. */
  CHECK(memcmp(&output, &untouched, sizeof(output)) == 0);
  adc_snapshot_seq = 1U;
  barriers = 0U;
  CHECK(CH1_GetSnapshot(&output) == HAL_BUSY);
  CHECK(barriers == 0U);
  CHECK(memcmp(&output, &untouched, sizeof(output)) == 0);
  adc_snapshot_seq = 0U;
  CH1_PublishGroup(0U); /* Stopped acquisition does not publish. */
  CHECK(adc_snapshot_valid == 0U && adc_group_count == 0U);

  for (i = 0U; i < 200U; i++)
  {
    adc_dma_buffer[i] = (uint16_t)(i + 100U);
  }
  adc_running = 1U;
  barriers = 0U;
  expected_group = 1U;
  expected_offset = 0U;
  watch_publication = 1U;
  CH1_PublishGroup(0U);
  watch_publication = 0U;
  CHECK(barriers == 2U);
  CHECK(adc_snapshot_seq == 2U);
  CHECK(adc_value == 199U && adc_ch2_value == 198U);
  CHECK(CH1_GetSnapshot(&output) == HAL_OK);
  CHECK(output.group_count == 1U);
  for (i = 0U; i < 50U; i++)
  {
    CHECK(output.samples[i] == 101U + 2U * i);
  }

  /* Complete a publication at each reader barrier. No inconsistent copy
   * may be accepted, and rejection must leave caller storage unchanged. */
  for (i = 1U; i <= 2U; i++)
  {
    memcpy(&untouched, &output, sizeof(output));
    barriers = 0U;
    inject_at_barrier = i;
    CHECK(CH1_GetSnapshot(&output) == HAL_BUSY);
    CHECK(memcmp(&output, &untouched, sizeof(output)) == 0);
    CHECK(barriers == 4U); /* One reader attempt, plus the injected writer. */
  }
  CHECK(CH1_GetSnapshot(&output) == HAL_OK);
  CHECK(output.group_count == 3U && output.samples[0] == 201U);
  CHECK(output.samples[49] == 299U);

  adc_group_ready = 0U; /* Validity cannot depend on a consumer-clearable flag. */
  CHECK(CH1_GetSnapshot(&output) == HAL_OK);
  CHECK(adc_group_ready == 0U);
  adc_snapshot_seq = UINT32_MAX - 1U;
  adc_group_count = UINT32_MAX;
  barriers = 0U;
  expected_group = 0U;
  expected_offset = 0U;
  watch_publication = 1U;
  CH1_PublishGroup(0U);
  watch_publication = 0U;
  CHECK(adc_snapshot_seq == 0U && adc_group_count == 0U);
  CHECK(CH1_GetSnapshot(&output) == HAL_OK);
  CHECK(output.group_count == 0U && output.samples[0] == 101U);

  printf("PASS snapshot: %u checks; initial/odd rejection, publication order, "
         "both DMA halves, IRQ injection, flag clearing, 32-bit wrap.\n", checks);
  return 0;
}
