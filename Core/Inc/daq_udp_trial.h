#ifndef DAQ_UDP_TRIAL_H
#define DAQ_UDP_TRIAL_H
#include <stdint.h>
/* Internal qualification entry only. No CMD01 capability advertisement. */
#define UDP_TRIAL_ARM_MAGIC 0x47453255UL
#define UDP_TRIAL_IDLE 0U
#define UDP_TRIAL_STARTING 1U
#define UDP_TRIAL_RUNNING 2U
#define UDP_TRIAL_RESTORING 3U
#define UDP_TRIAL_DONE 4U
#define UDP_TRIAL_FAULT 5U
#define UDP_TRIAL_OWNS(s) ((((s)>=UDP_TRIAL_STARTING)&&((s)<=UDP_TRIAL_RESTORING))||((s)==UDP_TRIAL_FAULT))
typedef struct {
  uint32_t state, arm, request, result, rejected, start_ok, start_fail;
  uint32_t restore_ok, restore_fail, stop_result, switch_result;
  uint32_t adc_error, adc_hal_error, dma_error, digital_error, digital_detail;
  uint32_t base_published, base_analog, base_half, base_full;
  uint32_t base_consumed, base_skipped, base_submitted, start_tick;
  uint32_t published, analog_published, half_callbacks, full_callbacks;
  uint32_t consumed, skipped, submitted, elapsed_ms;
  uint32_t rate_groups, rate_frames, rate_elapsed_ms;
  uint32_t rate_tick, rate_base_groups, rate_base_frames;
  uint32_t restored_smpr2, restored_sqr3;
} DAQ_UDPTrialDiagnostics;
extern volatile DAQ_UDPTrialDiagnostics udp_trial;
#endif
