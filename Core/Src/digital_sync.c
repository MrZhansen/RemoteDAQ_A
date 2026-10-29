#include "digital_sync.h"
#include <stddef.h>
#if defined(__ICCARM__)
#include "stm32f7xx.h"
#define TIMING_BARRIER() __DMB()
#else
#include <stdatomic.h>
#if defined(DIGITAL_TIMING_TEST)
void DigitalTiming_TestHook(void);
#define TIMING_BARRIER() do { atomic_signal_fence(memory_order_seq_cst); DigitalTiming_TestHook(); } while (0)
#else
#define TIMING_BARRIER() atomic_signal_fence(memory_order_seq_cst)
#endif
#endif

static const DigitalSyncTimingConfig timing_profiles[] =
{
  {200U, DIGITAL_SYNC_MAX_PHASE_US, DIGITAL_SYNC_MAX_BODY_US},
  {100U, DIGITAL_SYNC_MAX_PHASE_US, DIGITAL_SYNC_MAX_BODY_US}
};

const DigitalSyncTimingConfig *DigitalSync_GetTimingConfig(uint32_t gear)
{
  if (gear >= (sizeof(timing_profiles) / sizeof(timing_profiles[0]))) { return NULL; }
  return &timing_profiles[gear];
}

/* No GEAR2 profile. The short deadline applies only to GEAR0/144 cycles. */
static const DigitalSyncTimingConfig adc144_timing = {200U, 4U, DIGITAL_SYNC_MAX_BODY_US};

const DigitalSyncTimingConfig *DigitalSync_GetTimingConfigForADC(uint32_t gear,
                                                               uint32_t sample_cycles)
{
  if (sample_cycles == 480U) { return DigitalSync_GetTimingConfig(gear); }
  if ((sample_cycles == 144U) && (gear == 0U)) { return &adc144_timing; }
  return NULL;
}

/* Candidate only: same <=4-count early-read guard as GEAR0/144.
 * A separate 4 us body budget is deliberately stricter than the old 20 us.
 * Neither limit is a measured hardware qualification. */
const DigitalSyncTimingConfig *DigitalSync_GetOfflineTimingConfig(void)
{
  static const DigitalSyncTimingConfig candidate = {20U, 4U, 4U};
  return &candidate;
}

/* Same conservative candidate deadlines; separate activation owners. */
const DigitalSyncTimingConfig *DigitalSync_GetUDPCandidateTimingConfig(void)
{
  return DigitalSync_GetOfflineTimingConfig();
}

void DigitalSync_Init(DigitalSync *state)
{
  uint32_t i;
  for (i = 0U; i < DIGITAL_SYNC_POINTS; i++) { state->points[i] = 0U; }
  for (i = 0U; i < 2U; i++)
  {
    state->half_ready[i] = 0U;
    state->half_group[i] = 0U;
  }
  state->next_index = 0U;
  state->completed_groups = 0U;
  state->error = 0U;
  state->error_count = 0U;
  state->sample_count = 0U;
  state->last_ndtr = 0U;
  state->last_phase_us = 0U;
  state->last_trigger = 0U;
  state->fault_detail = 0U;
  state->timing = timing_profiles[0];
}

void DigitalSync_FaultWithDetail(DigitalSync *state, uint32_t reason, uint32_t detail)
{
  if (state->error == 0U)
  {
    state->fault_detail = detail;
    state->error = reason;
    state->error_count++;
  }
}

void DigitalSync_Fault(DigitalSync *state, uint32_t reason)
{
  DigitalSync_FaultWithDetail(state, reason, 0U);
}

uint32_t DigitalSync_Record(DigitalSync *state, uint32_t trigger, uint32_t ndtr,
                            uint32_t phase, uint8_t bits)
{
  uint32_t index;
  uint32_t half;
  state->last_ndtr = ndtr;
  state->last_phase_us = phase;
  if (state->error != 0U) { return 0U; }
  if (phase > state->timing.max_phase_us)
  {
    DigitalSync_FaultWithDetail(state, DIGITAL_SYNC_ERROR_LATE, DIGITAL_SYNC_DETAIL_PHASE);
    return 0U;
  }
  index = state->next_index;
  if ((uint32_t)(trigger - state->last_trigger) != 1U)
  {
    DigitalSync_FaultWithDetail(state, DIGITAL_SYNC_ERROR_ALIGN, DIGITAL_SYNC_DETAIL_TRIGGER);
    return 0U;
  }
  if ((ndtr == 0U) || (ndtr > 200U))
  {
    DigitalSync_FaultWithDetail(state, DIGITAL_SYNC_ERROR_ALIGN, DIGITAL_SYNC_DETAIL_NDTR_RANGE);
    return 0U;
  }
  if (((200U - ndtr) / 2U) != index)
  {
    DigitalSync_FaultWithDetail(state, DIGITAL_SYNC_ERROR_ALIGN, DIGITAL_SYNC_DETAIL_NDTR_INDEX);
    return 0U;
  }
  if ((bits & 0xF8U) != 0U)
  {
    DigitalSync_FaultWithDetail(state, DIGITAL_SYNC_ERROR_ALIGN, DIGITAL_SYNC_DETAIL_BITS);
    return 0U;
  }
  half = index / DIGITAL_SYNC_HALF;
  if ((index % DIGITAL_SYNC_HALF) == 0U) { state->half_ready[half] = 0U; }
  state->points[index] = bits;
  state->last_trigger = trigger;
  state->sample_count++;
  index++;
  if ((index % DIGITAL_SYNC_HALF) == 0U)
  {
    state->completed_groups++;
    state->half_group[half] = state->completed_groups;
    state->half_ready[half] = 1U;
  }
  state->next_index = (index == DIGITAL_SYNC_POINTS) ? 0U : index;
  return 1U;
}

uint32_t DigitalSync_HalfMatches(const DigitalSync *state, uint32_t half,
                                uint32_t group, uint32_t ndtr)
{
  if ((half > 1U) || (ndtr == 0U) || (ndtr > 200U) ||
      (state->error != 0U) || (state->half_ready[half] == 0U) ||
      (state->half_group[half] != group)) { return 0U; }
  return (half == 0U) ? ((ndtr <= 100U) ? 1U : 0U) :
                       ((ndtr > 100U) ? 1U : 0U);
}

volatile DigitalTimingGroup *DigitalTiming_Group(
    volatile DigitalSyncTimingDiagnostics *diagnostics, uint32_t gear)
{
  if (gear == 2U) { return &diagnostics->offline; }
  return (gear < 2U) ? &diagnostics->gear[gear] : &diagnostics->inactive;
}

void DigitalTiming_Record(volatile DigitalTimingStat *stat, uint32_t value,
                         uint32_t limit, uint32_t flags, uint32_t context)
{
  uint32_t count;
  if (stat == NULL) { return; }
  stat->seq++;
  TIMING_BARRIER();
  count = stat->count + 1U;
  stat->count = count;
  if (count == 0U) { stat->count_wraps++; }
  stat->last = value;
  if ((flags & DIGITAL_TIMING_INVALID) == 0U)
  {
    if (value > stat->max) { stat->max = value; }
    if (value > limit)
    {
      stat->over_count++;
      flags |= DIGITAL_TIMING_OVER_BUDGET;
    }
  }
  if ((flags & ~(DIGITAL_TIMING_CYCLE_WRAP)) != 0U) { stat->anomaly_count++; }
  stat->limit = limit;
  stat->flags = flags;
  stat->sticky_flags |= flags;
  stat->context = context;
  stat->valid = 1U;
  TIMING_BARRIER();
  stat->seq++;
}

void DigitalTiming_RecordSpan(volatile DigitalTimingStat *stat, uint32_t start,
    uint32_t end, uint32_t limit, uint32_t flags, uint32_t context)
{
  if (end < start) { flags |= DIGITAL_TIMING_CYCLE_WRAP; }
  DigitalTiming_Record(stat, (uint32_t)(end - start), limit, flags, context);
}

#if defined(__ICCARM__)
__root
#endif
uint32_t DigitalTiming_Read(const volatile DigitalTimingStat *stat,
                          DigitalTimingStat *snapshot)
{
  DigitalTimingStat candidate;
  uint32_t before;
  if ((stat == NULL) || (snapshot == NULL)) { return 0U; }
  before = stat->seq;
  if ((before & 1U) != 0U) { return 0U; }
  TIMING_BARRIER();
  candidate.seq = before;
  candidate.valid = stat->valid;
  candidate.count = stat->count;
  candidate.count_wraps = stat->count_wraps;
  candidate.last = stat->last;
  candidate.max = stat->max;
  candidate.limit = stat->limit;
  candidate.flags = stat->flags;
  candidate.sticky_flags = stat->sticky_flags;
  candidate.over_count = stat->over_count;
  candidate.anomaly_count = stat->anomaly_count;
  candidate.context = stat->context;
  TIMING_BARRIER();
  if ((before != stat->seq) || (candidate.valid == 0U)) { return 0U; }
  *snapshot = candidate;
  return 1U;
}

void DigitalTiming_Response(volatile DigitalTimingStat *stat, uint32_t entry,
    uint32_t before, uint32_t gpio_after, uint32_t checks_after,
    uint32_t period, uint32_t deadline, uint32_t sync_error)
{
  uint32_t flags = 0U;
  uint32_t upper;
  if ((period == 0U) || (entry >= period) || (before >= period) ||
      (gpio_after >= period) || (checks_after >= period) || (deadline >= period))
  { flags |= DIGITAL_TIMING_INVALID; }
  if ((before < entry) || (gpio_after < before) || (checks_after < gpio_after))
  { flags |= DIGITAL_TIMING_INVALID | DIGITAL_TIMING_PHASE_WRAP; }
  if (checks_after > deadline) { flags |= DIGITAL_TIMING_LATE; }
  /* TIM5/NDTR faults invalidate a modulo-CNT latency inference, including an
   * IRQ delayed by a whole period. Preserve original detection semantics. */
  if (sync_error != 0U) { flags |= DIGITAL_TIMING_INVALID | DIGITAL_TIMING_SYNC_FAULT; }
  upper = ((flags & DIGITAL_TIMING_INVALID) == 0U) ? gpio_after + 1U : 0U;
  DigitalTiming_Record(stat, upper, deadline + 1U, flags,
                      (before & 0xFFFFU) | ((gpio_after & 0xFFFFU) << 16U));
}
