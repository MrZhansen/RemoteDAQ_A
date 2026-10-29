#ifndef DIGITAL_SYNC_H
#define DIGITAL_SYNC_H
#include <stdint.h>

#define DIGITAL_SYNC_POINTS 100U
#define DIGITAL_SYNC_HALF   50U
#define DIGITAL_SYNC_MAX_PHASE_US 20U
#define DIGITAL_SYNC_MAX_BODY_US  20U
#define DIGITAL_SYNC_ERROR_LATE   0x01U
#define DIGITAL_SYNC_ERROR_ALIGN  0x02U
#define DIGITAL_SYNC_ERROR_GROUP  0x04U
#define DIGITAL_SYNC_ERROR_REUSE  0x08U
#define DIGITAL_SYNC_ERROR_CONFIG 0x10U

/* Production profiles are 200/100 us. The offline candidate has a separate
 * accessor and is not evidence that a production GEAR2 is supported. */
typedef struct
{
  uint32_t sample_interval_us;
  uint32_t max_phase_us;
  uint32_t max_body_us;
} DigitalSyncTimingConfig;

#define DIGITAL_SYNC_DETAIL_PHASE       0x01U
#define DIGITAL_SYNC_DETAIL_TRIGGER     0x02U
#define DIGITAL_SYNC_DETAIL_NDTR_RANGE  0x04U
#define DIGITAL_SYNC_DETAIL_NDTR_INDEX  0x08U
#define DIGITAL_SYNC_DETAIL_BITS        0x10U
#define DIGITAL_SYNC_DETAIL_CNT_WRAP    0x20U
#define DIGITAL_SYNC_DETAIL_NOT_RUNNING 0x40U
#define DIGITAL_SYNC_DETAIL_BODY_BUDGET 0x80U
#define DIGITAL_SYNC_DETAIL_HALF        0x100U

/* First sync failure since reset; valid is written last. Phase fields are
 * from the most recent TIM2 read, not simultaneous with the live registers.
 * source: 1=TIM2, 2=DMA half callback, 3=DMA full callback. */
typedef struct
{
  uint32_t valid, source, error, detail, sample_interval_us;
  uint32_t irq_entry_phase_us, gpio_phase_us, checks_phase_us;
  uint32_t record_ndtr, record_trigger, next_index, last_trigger;
  uint32_t tim2_cnt, tim2_sr, tim2_dier, tim2_arr;
  uint32_t tim5_cnt, dma_ndtr, dma_cr;
} DigitalSyncFaultSnapshot;

/* Diagnostic flags only: these never relax the acquisition checks. */
#define DIGITAL_TIMING_ERROR       0x001U
#define DIGITAL_TIMING_EMPTY       0x002U
#define DIGITAL_TIMING_COALESCED   0x004U
#define DIGITAL_TIMING_INVALID     0x008U
#define DIGITAL_TIMING_CYCLE_WRAP  0x010U
#define DIGITAL_TIMING_OVER_BUDGET 0x020U
#define DIGITAL_TIMING_SYNC_FAULT  0x040U
#define DIGITAL_TIMING_LATE        0x080U
#define DIGITAL_TIMING_PHASE_WRAP  0x100U
#define DIGITAL_TIMING_NO_CALLBACK 0x200U
#define DIGITAL_TIMING_PATH_HALF   1U
#define DIGITAL_TIMING_PATH_FULL   2U

/* One IRQ writer per record. Readers use DigitalTiming_Read, never count as
 * a validity flag: both count and seq intentionally wrap modulo 2^32.
 * last/max/limit are cycles, except gpio_upper_us (microsecond bounds).
 * Invalid response bounds do not update max. Counters wrap, not saturate. */
typedef struct
{
  uint32_t seq, valid, count, count_wraps;
  uint32_t last, max, limit, flags, sticky_flags;
  uint32_t over_count, anomaly_count, context;
} DigitalTimingStat;

typedef struct
{
  DigitalTimingStat tim2_c, tim2_accounting;
  DigitalTimingStat dma_c, dma_accounting, dma_half_c, dma_full_c;
  DigitalTimingStat publish_half, publish_full;
  DigitalTimingStat gpio_upper_us;
} DigitalTimingGroup;

/* Maxima are cumulative since reset, across both gears. CYCCNT intervals
 * include any preemption, but exclude hardware exception entry/return. */
typedef struct
{
  uint32_t irq_entry_phase_us, gpio_phase_us, checks_phase_us;
  uint32_t irq_entry_phase_max_us, gpio_phase_max_us, checks_phase_max_us;
  uint32_t tim2_handler_max_cycles;
  uint32_t dma_half_callback_max_cycles, dma_full_callback_max_cycles;
  DigitalSyncFaultSnapshot fault;
  /* New records are separated by actual gear. inactive is NOT GEAR2. */
  DigitalTimingGroup gear[2], inactive;
  DigitalTimingGroup offline; /* Candidate-only, independently observable. */
  uint32_t dma_paths; /* DMA IRQ writer/callbacks only; not a Watch snapshot. */
} DigitalSyncTimingDiagnostics;

volatile DigitalTimingGroup *DigitalTiming_Group(
    volatile DigitalSyncTimingDiagnostics *diagnostics, uint32_t gear);
void DigitalTiming_Record(volatile DigitalTimingStat *stat, uint32_t value,
                         uint32_t limit, uint32_t flags, uint32_t context);
/* Subtraction handles CYCCNT rollover, provided a span is <2^32 cycles.
 * No software counter can infer multiple DWT wraps or debug-halt behavior. */
void DigitalTiming_RecordSpan(volatile DigitalTimingStat *stat, uint32_t start,
    uint32_t end, uint32_t limit, uint32_t flags, uint32_t context);
/* Single attempt, main-context reader; unchanged output on every failure.
 * Consistency is per record, not an atomic snapshot of all IRQ statistics. */
uint32_t DigitalTiming_Read(const volatile DigitalTimingStat *stat,
                          DigitalTimingStat *snapshot);
/* Uses existing CNT timestamps only, AFTER the critical GPIO/NDTR reads.
 * No new hardware sampling. A valid gpio_after count q means elapsed <q+1 us
 * at the configured 1 MHz timer tick, not an exact GPIO access timestamp. */
void DigitalTiming_Response(volatile DigitalTimingStat *stat, uint32_t entry,
    uint32_t before, uint32_t gpio_after, uint32_t checks_after,
    uint32_t period, uint32_t deadline, uint32_t sync_error);

typedef struct
{
  volatile uint8_t points[DIGITAL_SYNC_POINTS];
  volatile uint32_t next_index;
  volatile uint32_t completed_groups;
  volatile uint32_t half_ready[2];
  volatile uint32_t half_group[2];
  volatile uint32_t error;
  volatile uint32_t error_count;
  volatile uint32_t sample_count;
  volatile uint32_t last_ndtr;
  volatile uint32_t last_phase_us;
  volatile uint32_t last_trigger;
  volatile uint32_t fault_detail;
  DigitalSyncTimingConfig timing; /* Change only while acquisition is stopped. */
} DigitalSync;

/* NULL for every unsupported gear, including GEAR2. */
const DigitalSyncTimingConfig *DigitalSync_GetTimingConfig(uint32_t gear);
/* 144 cycles is a GEAR0-only experiment. CNT <= 4 means elapsed < 5 us
 * at the existing 1 MHz timer tick; this candidate still needs board testing. */
const DigitalSyncTimingConfig *DigitalSync_GetTimingConfigForADC(uint32_t gear,
                                                               uint32_t sample_cycles);
/* Internal 20 us/144-cycle candidate only; production lookups reject GEAR2. */
const DigitalSyncTimingConfig *DigitalSync_GetOfflineTimingConfig(void);
const DigitalSyncTimingConfig *DigitalSync_GetUDPCandidateTimingConfig(void);
void DigitalSync_Init(DigitalSync *state);
void DigitalSync_Fault(DigitalSync *state, uint32_t reason);
void DigitalSync_FaultWithDetail(DigitalSync *state, uint32_t reason, uint32_t detail);
/* One call per TIM2 update; trigger is the TIM5 hardware event count.
 * phase is after GPIO, DMA NDTR and TIM5 reads (the checked end phase).
 * 100/200 us gears, 200 halfword ADC DMA, two sequential ADC conversions.
 * At 27 MHz: 480-cycle pairs take 36.44 us (20 us guard); the experimental
 * 144-cycle pair takes 11.56 us (CNT <= 4, elapsed < 5 us guard).
 * IRQ priority and phase limit must ensure the current ADC pair is unfinished. */
uint32_t DigitalSync_Record(DigitalSync *state, uint32_t trigger, uint32_t ndtr,
                            uint32_t phase, uint8_t bits);
/* A completed digital half must match the analog group and remain outside
 * the DMA's currently filling half, both before and after group copying. */
uint32_t DigitalSync_HalfMatches(const DigitalSync *state, uint32_t half,
                                uint32_t group, uint32_t ndtr);
#endif
