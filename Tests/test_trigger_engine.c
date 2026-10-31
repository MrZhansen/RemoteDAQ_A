/* Pure trigger-window tests: real engine functions extracted from main.c.
 * ADC/DMA, HAL and networking are modeled only; no board is accessed. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CH1_SNAPSHOT_SAMPLES 50U
#define DAQ_TRIGGER_PRE_SAMPLES 24U
#define DAQ_TRIGGER_POST_SAMPLES 25U
#define DAQ_TRIGGER_SOURCE_OFF 0U
#define DAQ_TRIGGER_MODE_RISE 0U
#define DAQ_TRIGGER_MODE_FALL 1U
#define DAQ_TRIGGER_MODE_ANY 2U
#define ACQ_GEAR_INVALID UINT32_MAX
#define __DMB() do { } while (0)

typedef struct
{
  uint16_t ch1[CH1_SNAPSHOT_SAMPLES];
  uint16_t ch2[CH1_SNAPSHOT_SAMPLES];
  uint8_t dig[CH1_SNAPSHOT_SAMPLES];
  uint32_t group_count;
  uint32_t gear;
} Full_Snapshot;

static uint32_t ch1_preview_have_group;
static volatile uint32_t daq_trigger_source = DAQ_TRIGGER_SOURCE_OFF;
static volatile uint32_t daq_trigger_mode = DAQ_TRIGGER_MODE_RISE;
static volatile uint32_t daq_trigger_level;
static volatile uint32_t trigger_edge_count;
static volatile uint32_t trigger_frame_count;
static volatile uint32_t trigger_rearm_count;
static volatile uint32_t trigger_group_gap_count;

static unsigned checks;
#define CHECK(x) do { checks++; if (!(x)) { \
  fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); exit(1); \
} } while (0)

#include "trigger_engine_under_test.inc"

static void make_group(Full_Snapshot *snapshot, uint16_t ch1_value,
                       uint16_t ch2_value, uint8_t dig_value)
{
  uint32_t i;
  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  {
    snapshot->ch1[i] = (uint16_t)(ch1_value + (uint16_t)i);
    snapshot->ch2[i] = (uint16_t)(ch2_value + (uint16_t)i);
    snapshot->dig[i] = (uint8_t)(dig_value & 0x07U);
  }
  snapshot->gear = 0U;
}

static void configure(uint32_t source, uint32_t mode, uint32_t level)
{
  daq_trigger_source = source;
  daq_trigger_mode = mode;
  daq_trigger_level = level;
  Trigger_Reset();
}

static void verify_edge_case(uint32_t source, uint32_t mode,
                             uint32_t before, uint32_t after,
                             uint32_t group_count)
{
  Full_Snapshot snapshot;
  Full_Snapshot ready;
  TriggerPoint point;
  uint32_t i;
  uint8_t before_dig = 0U;
  uint8_t after_dig = 0U;

  if (source >= 3U)
  {
    before_dig = (uint8_t)(before << (source - 3U));
    after_dig = (uint8_t)(after << (source - 3U));
  }
  configure(source, mode, 128U);
  make_group(&snapshot, 0U, 0U, before_dig);
  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  {
    snapshot.ch1[i] = (source == 1U) ? (uint16_t)before : 0U;
    snapshot.ch2[i] = (source == 2U) ? (uint16_t)before : 0U;
    snapshot.dig[i] = before_dig;
  }
  snapshot.group_count = group_count;
  Trigger_ProcessSnapshot(&snapshot);

  make_group(&snapshot, 0U, 0U, after_dig);
  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  {
    snapshot.ch1[i] = (source == 1U) ? (uint16_t)after : 0U;
    snapshot.ch2[i] = (source == 2U) ? (uint16_t)after : 0U;
    snapshot.dig[i] = after_dig;
  }
  snapshot.group_count = group_count + 1U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U);
  CHECK(ready.group_count == group_count + 1U);

  point.ch1 = ready.ch1[DAQ_TRIGGER_PRE_SAMPLES];
  point.ch2 = ready.ch2[DAQ_TRIGGER_PRE_SAMPLES];
  point.dig = ready.dig[DAQ_TRIGGER_PRE_SAMPLES];
  CHECK(Trigger_PointValue(&point) == after);
}

int main(void)
{
  Full_Snapshot snapshot;
  Full_Snapshot ready;
  uint32_t i;

  /* CH1 rising at the first point of the second group. The previous group
   * primes a full 24-point history and the edge lands at frame index 24. */
  configure(1U, DAQ_TRIGGER_MODE_RISE, 128U);
  make_group(&snapshot, 1000U, 2000U, 0U);
  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  { snapshot.ch1[i] = 1000U; snapshot.ch2[i] = 2000U; snapshot.dig[i] = 0U; }
  snapshot.group_count = 10U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 0U);
  make_group(&snapshot, 1000U, 2000U, 0U);
  snapshot.group_count = 11U;
  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  {
    snapshot.ch1[i] = (i < 10U) ? 1000U : 3000U;
    snapshot.ch2[i] = (uint16_t)(snapshot.ch1[i] + 1000U);
    snapshot.dig[i] = 0U;
  }
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U);
  CHECK(trigger_edge_count == 1U && trigger_frame_count == 1U);
  CHECK(ready.group_count == 11U && ready.gear == 0U);
  for (i = 0U; i < DAQ_TRIGGER_PRE_SAMPLES; i++)
  { CHECK(ready.ch1[i] == 1000U); }
  CHECK(ready.ch1[24] == 3000U);
  CHECK(ready.ch1[25] == 3000U);
  CHECK(ready.ch1[49] == 3000U);
  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  { CHECK(ready.ch2[i] == (uint16_t)(ready.ch1[i] + 1000U)); }
  CHECK(ready.dig[0] == 0U && ready.dig[49] == 0U);

  /* Falling CH1 edge. */
  configure(1U, DAQ_TRIGGER_MODE_FALL, 128U);
  make_group(&snapshot, 3000U, 0U, 0U);
  snapshot.group_count = 20U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 1000U, 0U, 0U);
  snapshot.group_count = 21U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U);
  CHECK(ready.ch1[24] == 1000U);

  /* CH2 any-edge trigger. */
  configure(2U, DAQ_TRIGGER_MODE_ANY, 128U);
  make_group(&snapshot, 0U, 1000U, 0U);
  snapshot.group_count = 30U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 0U, 3000U, 0U);
  snapshot.group_count = 31U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U);
  CHECK(ready.ch2[24] == 3000U);
  CHECK(ready.ch1[24] == 0U);

  /* D1/D2/D3 rising edges use protocol bits 0/1/2. */
  for (i = 3U; i <= 5U; i++)
  {
    uint8_t bit = (uint8_t)(1U << (i - 3U));
    configure(i, DAQ_TRIGGER_MODE_RISE, 0U);
    make_group(&snapshot, 0U, 0U, 0U);
    snapshot.group_count = 40U + i;
    Trigger_ProcessSnapshot(&snapshot);
    make_group(&snapshot, 0U, 0U, bit);
    snapshot.group_count = 41U + i;
    Trigger_ProcessSnapshot(&snapshot);
    CHECK(Trigger_TakeReady(&ready) == 1U);
    CHECK(ready.dig[24] == bit);
  }

  /* D2 falling and D3 any-edge. */
  configure(4U, DAQ_TRIGGER_MODE_FALL, 0U);
  make_group(&snapshot, 0U, 0U, 0x02U);
  snapshot.group_count = 50U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 0U, 0U, 0U);
  snapshot.group_count = 51U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U && ready.dig[24] == 0U);

  configure(5U, DAQ_TRIGGER_MODE_ANY, 0U);
  make_group(&snapshot, 0U, 0U, 0x00U);
  snapshot.group_count = 60U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 0U, 0U, 0x04U);
  snapshot.group_count = 61U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U && (ready.dig[24] & 0x04U) != 0U);

  /* Every supported source works with every edge mode. Analog cases cross
   * the configured threshold; digital cases use the protocol's bit 0/1/2. */
  for (i = 1U; i <= 5U; i++)
  {
    uint32_t base = 200U + i * 10U;
    uint32_t before = (i <= 2U) ? 1000U : 0U;
    uint32_t after = (i <= 2U) ? 3000U : 1U;
    verify_edge_case(i, DAQ_TRIGGER_MODE_RISE, before, after, base);
    verify_edge_case(i, DAQ_TRIGGER_MODE_FALL, after, before, base + 2U);
    verify_edge_case(i, DAQ_TRIGGER_MODE_ANY, before, after, base + 4U);
  }

  /* No edge means no frame. */
  configure(3U, DAQ_TRIGGER_MODE_RISE, 0U);
  make_group(&snapshot, 0U, 0U, 0U);
  snapshot.group_count = 70U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 0U, 0U, 0U);
  snapshot.group_count = 71U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 0U);

  /* Take once, then re-arm and complete another rising window. */
  make_group(&snapshot, 0U, 0U, 0U);
  snapshot.group_count = 72U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 0U, 0U, 0x01U);
  snapshot.group_count = 73U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U);
  make_group(&snapshot, 0U, 0U, 0U);
  snapshot.group_count = 74U;
  Trigger_ProcessSnapshot(&snapshot);
  make_group(&snapshot, 0U, 0U, 0x01U);
  snapshot.group_count = 75U;
  Trigger_ProcessSnapshot(&snapshot);
  CHECK(Trigger_TakeReady(&ready) == 1U);
  CHECK(trigger_frame_count == 25U && trigger_rearm_count == 25U);

  /* Source 0 restores free-run behavior and clears any pending window. */
  configure(0U, DAQ_TRIGGER_MODE_RISE, 0U);
  CHECK(trigger_capture_active == 0U && Trigger_TakeReady(&ready) == 0U);

  printf("PASS trigger engine: %u checks; CH1/CH2, D1-D3, all 15 source/mode pairs, "
         "24/1/25 window, ring history, no-trigger and re-arm\n", checks);
  return 0;
}
