#ifndef ACQUISITION_OFFLINE_H
#define ACQUISITION_OFFLINE_H
#include <stdint.h>

/* Internal experiment only; these are not protocol gears/commands. */
#define OFFLINE_IDLE       0U
#define OFFLINE_STARTING   1U
#define OFFLINE_RUNNING    2U
#define OFFLINE_RESTORING  3U
#define OFFLINE_DONE       4U
#define OFFLINE_FAULT      5U
#define OFFLINE_MAX_GROUPS 1000U
#define OFFLINE_CAPACITY   32U
#define OFFLINE_OK         0U
#define OFFLINE_BAD_REQUEST 1U
#define OFFLINE_NOT_READY  2U
#define OFFLINE_CLOCK      3U
#define OFFLINE_UART_PAUSE 4U
#define OFFLINE_START      5U
#define OFFLINE_RUNTIME    6U
#define OFFLINE_TIMEOUT    7U
#define OFFLINE_CANCELLED  8U
#define OFFLINE_STOP       9U
#define OFFLINE_RESTORE    10U
#define OFFLINE_UART_RESUME 11U

typedef struct
{
  uint32_t state, result, requested_groups;
  uint32_t accepted, rejected, start_ok, start_fail, restore_ok, restore_fail;
  uint32_t snapshot_ok, snapshot_busy, duplicate_reads, consumed, skipped;
  uint32_t published, analog_published, half_callbacks, full_callbacks, dma_irqs;
  uint32_t unconsumed, retained, retention_dropped;
  uint32_t elapsed_ms, elapsed_cycles, elapsed_us, nominal_rate_groups;
  uint32_t first_group, last_group, data_seq;
  uint32_t adc_error, adc_hal_error, dma_error, digital_error, digital_detail;
  uint32_t start_switch_result, stop_result, restore_switch_result;
  uint32_t uart_pause_status, uart_resume_status;
  uint32_t end_ndtr, end_trigger, end_next_index;
  uint32_t adc_smpr2_after_restore, adc_sqr3_after_restore;
  uint32_t adc_clock_hz, timer_clock_hz;
} AcquisitionOfflineDiagnostics;
#endif
