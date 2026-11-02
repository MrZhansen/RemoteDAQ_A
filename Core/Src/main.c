/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "daq_protocol.h"
#include "digital_sync.h"
#include "daq_control_stream.h"
#include "daq_network.h"
#include "daq_udp_trial.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  uint32_t tick;
  uint32_t tim_cr1;
  uint32_t tim_cr2;
  uint32_t tim_smcr;
  uint32_t tim_sr;
  uint32_t tim_cnt;
  uint32_t tim_psc;
  uint32_t tim_arr;
  uint32_t adc_sr;
  uint32_t adc_cr2;
  uint32_t adc_common_ccr;
  uint32_t dma_cr;
  uint32_t dma_ndtr;
  uint32_t dma_par;
  uint32_t dma_m0ar;
  uint32_t dma_fcr;
  uint32_t dma_lisr;
  uint32_t rcc_apb1enr;
  uint32_t rcc_apb2enr;
  uint32_t rcc_ahb1enr;
  uint32_t nvic_iser1;
  uint32_t nvic_ispr1;
  uint32_t primask_before;
  uint32_t basepri;
  uint32_t device_idcode;
  uint32_t valid;
} CH1_DebugSnapshot;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CH1_GROUP_SAMPLES       CH1_SNAPSHOT_SAMPLES
#define ADC_SCAN_CHANNELS       2U
#define ADC_DMA_HALF_SAMPLES    (ADC_SCAN_CHANNELS * CH1_GROUP_SAMPLES)
#define CH1_DMA_SAMPLES         (2U * ADC_DMA_HALF_SAMPLES)
#define CH1_STALL_TIMEOUT_MS    100U

#define ACQ_GEAR_INVALID        0xFFFFFFFFUL
#define ACQ_GEAR_COUNT          5U
#define ACQ_SWITCH_OK           0U
#define ACQ_SWITCH_BAD_GEAR     1U
#define ACQ_SWITCH_CLOCK        2U
#define ACQ_SWITCH_TIM2_STOP    3U
#define ACQ_SWITCH_ADC_STOP     4U
#define ACQ_SWITCH_TIM5_STOP    5U
#define ACQ_SWITCH_TIM2_CONFIG  6U
#define ACQ_SWITCH_ADC_START    7U
#define ACQ_SWITCH_TIM5_START   8U
#define ACQ_SWITCH_TIM2_START   9U
#define ACQ_SWITCH_LATCHED      10U
#define ACQ_SWITCH_ADC_CONFIG   11U
#define ACQ_SWITCH_ADC_CONFLICT 12U
#define ACQ_SWITCH_ADC_CYCLES   13U
#define ACQ_SWITCH_CLEANUP      14U
#define ACQ_SWITCH_UDP_REQUIRED 15U

/* Sticky application errors; inspect adc_hal_error and dma_error as well. */
#define ADC_ERROR_NONE          0x00U
#define ADC_ERROR_START         0x01U
#define ADC_ERROR_TIMER_START   0x02U
#define ADC_ERROR_RUNTIME       0x04U
#define ADC_ERROR_STALLED       0x08U

#define UART6_TEST_PERIOD_MS    100U
#define UART6_TEST_TIMEOUT_MS   10U
#define UART6_FRAME_TIMEOUT_MS  10U
#define UART6_STATUS_TIMEOUT_MS 2U
#define UART6_RX_BUFFER_SIZE    2048U
#define UART6_RX_COPY_SIZE      512U
#define UART6_STATUS_PENDING_LIMIT 16U
#define GPIO_INPUT_PERIOD_MS   10U
#define UART6_RATE_WINDOW_MS   1000U

#define SQUARE_WAVE_COUNTER_HZ  1000000U
/* Sticky PWM diagnostics, independent of ADC errors. */
#define SQUARE_WAVE_ERROR_INIT  0x01U
#define SQUARE_WAVE_ERROR_CLOCK 0x02U
#define SQUARE_WAVE_ERROR_START 0x04U
#define SQUARE_WAVE_ERROR_STOP  0x08U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* CH1 frame preview state: main-context only. */
/* CMD02 gates wire fields only; all five hardware inputs keep sampling. */
volatile uint32_t daq_active_channel_mask = 0x1FU;
volatile uint32_t uart6_mask_apply_count = 0U;
volatile uint32_t ch1_frame_channel_mask = 0x1FU;
volatile uint32_t ch1_snapshot_ok_count = 0U;
volatile uint32_t ch1_snapshot_fail_count = 0U;
volatile uint32_t ch1_new_group_count = 0U;
volatile uint32_t ch1_frame_build_count = 0U;
volatile uint32_t ch1_frame_error_count = 0U;
volatile uint32_t ch1_frame_group = 0U;
volatile uint32_t ch1_frame_gear = 0U;
volatile uint32_t ch1_skipped_group_count = 0U;
/* DAQ_FrameStatus; 0xFFFFFFFF means no build has been attempted. */
volatile uint32_t ch1_last_build_status = 0xFFFFFFFFUL;
/* Watch may read bytes only with valid=1 and the same even publication seq
 * before/after reading bytes, group ID, CRC, and successful build count. */
volatile uint8_t ch1_frame_preview[DAQ_DATA_FRAME_SIZE];
volatile uint32_t ch1_frame_preview_valid = 0U;
volatile uint32_t ch1_frame_preview_seq = 0U;
volatile uint32_t ch1_frame_preview_crc = 0U;
static CH1_Snapshot ch1_preview_snapshot;
static Analog_Snapshot analog_preview_snapshot;
static Full_Snapshot full_preview_snapshot;
static uint8_t ch1_preview_work[DAQ_DATA_FRAME_SIZE];
static uint32_t ch1_preview_have_group = 0U;
static uint32_t ch1_preview_last_group = 0U;
/* End CH1 frame preview state. */

/* CH1 UART transmit state: main-context only. */
volatile uint32_t ch1_uart_tx_count = 0U;
volatile uint32_t ch1_uart_tx_error_count = 0U;
volatile uint32_t ch1_uart_last_tx_status = 0xFFFFFFFFUL;
volatile uint32_t ch1_uart_hal_error = HAL_UART_ERROR_NONE;
/* First SEQ=0 is an implementation choice. Increment only after HAL_OK. */
volatile uint16_t ch1_uart_next_seq = 0U;
volatile uint16_t ch1_uart_last_seq = 0U;
volatile uint32_t ch1_uart_last_group = 0U;
volatile uint32_t ch1_uart_not_ready_count = 0U;
/* Last successful HAL transmission, retained even after later failures.
 * Read only when valid=1 and the same even snapshot_seq brackets the read. */
volatile uint8_t ch1_uart_last_frame[DAQ_DATA_FRAME_SIZE];
volatile uint32_t ch1_uart_last_valid = 0U;
volatile uint32_t ch1_uart_last_snapshot_seq = 0U;
/* Measured HAL_OK count over rate_elapsed_ms; not a receiver acknowledgement. */
volatile uint32_t ch1_uart_rate_frames = 0U;
volatile uint32_t ch1_uart_rate_elapsed_ms = 0U;
static uint32_t background_gpio_tick;
static uint32_t background_rate_tick;
static uint32_t background_rate_count;
static uint32_t uart6_transmit_busy = 0U;
/* End CH1 UART transmit state. */

/* Synchronous digital state. */
static DigitalSync digital_sync;
static TIM_HandleTypeDef htim5;
volatile uint32_t digital_counter_status = 0xFFFFFFFFUL;
static volatile Full_Snapshot full_published_snapshot;
volatile uint32_t full_snapshot_seq = 0U;
volatile uint32_t full_snapshot_valid = 0U;
volatile uint32_t digital_irq_max_cycles = 0U;
volatile uint32_t digital_read_phase_max_us = 0U;
volatile DigitalSyncTimingDiagnostics digital_timing;
/* End synchronous digital state. */

/* Acquisition gear state. */
typedef DigitalSyncTimingConfig AcquisitionGearProfile;

volatile uint32_t acquisition_active_gear = ACQ_GEAR_INVALID;
volatile uint32_t acquisition_requested_gear = 0U;
volatile uint32_t acquisition_request_pending = 0U;
volatile uint32_t acquisition_last_switch_result = ACQ_GEAR_INVALID;
volatile uint32_t acquisition_switch_ok_count = 0U;
volatile uint32_t acquisition_switch_fail_count = 0U;
volatile uint32_t acquisition_switching = 0U;
volatile uint32_t acquisition_fault_latched = 0U;
volatile uint32_t acquisition_sample_interval_us = 0U;
volatile uint32_t acquisition_timer_clock_hz = 0U;
/* Software feature permission, derived from the selected live UDP link.
 * This is not an ADC accuracy or external timing qualification flag. */
volatile uint32_t acquisition_udp_gear2_available = 0U;
volatile uint32_t acquisition_udp_gear2_active = 0U;
volatile uint32_t acquisition_udp_gear2_return_count = 0U;
volatile uint32_t acquisition_udp_gear2_fault_count = 0U;
static uint32_t acquisition_udp_gear2_starting = 0U;
volatile uint32_t acquisition_tim2_stop_status = ACQ_GEAR_INVALID;
volatile uint32_t acquisition_adc_stop_status = ACQ_GEAR_INVALID;
volatile uint32_t acquisition_tim5_stop_status = ACQ_GEAR_INVALID;
volatile uint32_t full_group_publish_count = 0U;
static uint32_t acquisition_last_request = 0U;
/* ADC sample time is independent of protocol GEAR. Active is invalid while
 * stopped/switching; requested values are ADC clock cycles, not HAL encodings. */
volatile uint32_t acquisition_adc_active_cycles = ACQ_GEAR_INVALID;
volatile uint32_t acquisition_adc_requested_cycles = 480U;
volatile uint32_t acquisition_adc_request_pending = 0U;
volatile uint32_t acquisition_adc_test_mode = 0U;
volatile uint32_t acquisition_adc_last_switch_result = ACQ_GEAR_INVALID;
volatile uint32_t acquisition_adc_switch_ok_count = 0U;
volatile uint32_t acquisition_adc_switch_fail_count = 0U;
volatile uint32_t acquisition_adc_rank1_status = 0xFFFFFFFFUL;
volatile uint32_t acquisition_adc_rank2_status = 0xFFFFFFFFUL;
static uint32_t acquisition_adc_last_request = 480U;
volatile uint32_t acquisition_fast_isr_active = 0U;
volatile uint32_t acquisition_fast_irq_last_cycles = 0U;
volatile uint32_t acquisition_fast_irq_max_cycles = 0U;
volatile uint32_t acquisition_fast_irq_budget_cycles = 0U;
volatile uint32_t acquisition_fast_phase_last_cycles = 0U;
volatile uint32_t acquisition_fast_phase_max_cycles = 0U;
volatile uint32_t acquisition_fast_budget_fail_count = 0U;
/* Finite offline experiment state; no IRQ writer and no UART protocol entry. */
volatile AcquisitionOfflineDiagnostics offline;
volatile uint32_t offline_request_groups = 0U;
volatile uint32_t offline_stop_request = 0U;
volatile Full_Snapshot offline_samples[OFFLINE_CAPACITY];
static uint32_t offline_base_publish, offline_base_analog, offline_base_half;
static uint32_t offline_base_full, offline_base_irq, offline_last_group;
static uint32_t offline_seen_publish, offline_start_tick, offline_start_cycle;
static uint32_t offline_end_tick, offline_end_cycle;
/* Independent continuous UDP qualification state; never the offline owner. */
volatile DAQ_UDPTrialDiagnostics udp_trial;
/* End acquisition gear state. */
/* Offline private state. */
static Full_Snapshot offline_candidate;
/* End offline private state. */
/* HAL status: 0=HAL_OK; 0xFFFFFFFF means the call has not run. */
volatile uint32_t uart6_initialized = 0U;
volatile uint32_t uart6_init_status = 0xFFFFFFFFUL;
volatile uint32_t uart6_last_tx_status = 0xFFFFFFFFUL;
volatile uint32_t uart6_tx_count = 0U;
volatile uint32_t uart6_tx_error_count = 0U;
volatile uint32_t uart6_hal_error = HAL_UART_ERROR_NONE;
static uint32_t uart6_last_tx_tick = 0U;
static const uint8_t uart6_test_message[] = "UART6_OK\r\n";

/* IAR absolute SRAM1 placement makes the DMA memory address explicit.
 * F746 DMA can also access DTCM via AHBS (AN4667); SRAM1 is used here.
 * Reserve this address for both channels; the linker checks absolute placement.
 * D-Cache is explicitly disabled before any CPU/DMA accesses below. */
#pragma location=0x20010000
#pragma data_alignment=32
__no_init volatile uint16_t adc_dma_buffer[CH1_DMA_SAMPLES];

/* CPU-only snapshots of 50 point pairs, guarded together by adc_snapshot_seq.
 * Keep adc_samples/adc_value as logical CH1 (PF10 / ADC3_IN8) for Watch.
 * Logical CH2 is PA0 / ADC3_IN0, the first rank in the DMA scan. */
volatile uint16_t adc_samples[CH1_GROUP_SAMPLES];
volatile uint32_t adc_value = 0U;
volatile uint16_t adc_ch2_samples[CH1_GROUP_SAMPLES];
volatile uint32_t adc_ch2_value = 0U;
volatile uint32_t adc_error = ADC_ERROR_NONE;
volatile uint32_t adc_hal_error = HAL_ADC_ERROR_NONE;
volatile uint32_t dma_error = HAL_DMA_ERROR_NONE;
volatile uint32_t adc_error_count = 0U;
volatile uint32_t adc_group_count = 0U;
volatile uint32_t adc_group_ready = 0U;
volatile uint32_t adc_snapshot_seq = 0U;
/* Separate from the consumer-clearable notification and wrapping group ID. */
static volatile uint32_t adc_snapshot_valid = 0U;
volatile uint32_t adc_last_group_tick = 0U;
volatile uint32_t adc_group_period_ms = 0U;
volatile uint32_t adc_group_cycles = 0U;
volatile uint32_t adc_dcache_enabled = 0U;
volatile uint32_t adc_running = 0U;
/* Startup stages: 1 cache entry, 2 cache done, 3 MPU/HAL done,
 * 4 clock done, 5 peripheral init done, 6 buffer done, 7 DMA armed, 8 running. */
volatile uint32_t adc_startup_stage = 0U;
volatile uint32_t adc_startup_ccr = 0U;
/* Fast-gear instruction-fetch evidence: 1 = Cortex-M7 L1 I-cache enabled,
 * FLASH->ACR shows LATENCY/PRFTEN/ARTEN after the clock switch. */
volatile uint32_t adc_startup_icache = 0U;
volatile uint32_t adc_startup_flash_acr = 0U;
/* HAL status: 0=HAL_OK; 0xFFFFFFFF means the call has not run. */
volatile uint32_t adc_dma_start_status = 0xFFFFFFFFUL;
volatile uint32_t tim2_start_status = 0xFFFFFFFFUL;
volatile uint32_t adc_half_callback_count = 0U;
volatile uint32_t adc_full_callback_count = 0U;
volatile CH1_DebugSnapshot adc_debug_start;
volatile CH1_DebugSnapshot adc_debug_timeout;
extern volatile uint32_t dma2_stream0_irq_count;
extern volatile uint32_t dma2_stream0_last_lisr;
static uint32_t adc_previous_group_cycle;

/* Slow level diagnostics only; not synchronized with ADC sample points.
 * Logical D1/D2/D3 map to Arduino D2/D4/D7, respectively. */
volatile uint32_t digital_input_d1 = 0U;
volatile uint32_t digital_input_d2 = 0U;
volatile uint32_t digital_input_d3 = 0U;

volatile uint32_t square_wave_running = 0U;
volatile uint32_t square_wave_error = 0U;
volatile uint32_t square_wave_frequency_hz = 0U;
/* Nominal frequency after timer quantization; not a hardware measurement. */
volatile uint32_t square_wave_actual_hz = 0U;
volatile uint32_t square_wave_timer_clock_hz = 0U;
static uint32_t square_wave_initialized = 0U;
/* CMD07 diagnostics. Frequencies are requests/results, not Watch controls. */
volatile uint32_t square_wave_last_request_hz = SQUARE_WAVE_DEFAULT_HZ;
volatile uint32_t square_wave_last_set_status = 0xFFFFFFFFUL;
volatile uint32_t square_wave_set_ok_count = 0U;
volatile uint32_t square_wave_set_error_count = 0U;
volatile uint32_t square_wave_restore_status = 0xFFFFFFFFUL;

/* CMD03/04/05 configuration. Source 0 keeps the legacy free-run path. */
volatile uint32_t daq_trigger_source = DAQ_TRIGGER_SOURCE_OFF;
volatile uint32_t daq_trigger_mode = DAQ_TRIGGER_MODE_RISE;
volatile uint32_t daq_trigger_level = 0U;
volatile uint32_t trigger_command_ok_count = 0U;
volatile uint32_t trigger_edge_count = 0U;
volatile uint32_t trigger_frame_count = 0U;
volatile uint32_t trigger_rearm_count = 0U;
volatile uint32_t trigger_group_gap_count = 0U;

/* USART6 control receive/status state. */
DMA_HandleTypeDef hdma_usart6_rx;
/* F746 DMA-accessible RAM; cache remains disabled by the existing startup. */
#pragma data_alignment=32
static volatile uint8_t uart6_rx_buffer[UART6_RX_BUFFER_SIZE];
static uint8_t uart6_rx_work[UART6_RX_COPY_SIZE];
static volatile uint32_t uart6_rx_wraps = 0U;
static uint32_t uart6_rx_consumed = 0U;
static uint32_t uart6_rx_observed = 0U;
static uint32_t uart6_control_started = 0U;
static DAQ_ControlStream uart6_control_stream;
volatile uint32_t uart6_rx_active = 0U;
volatile uint32_t uart6_rx_start_status = 0xFFFFFFFFUL;
volatile uint32_t uart6_rx_byte_count = 0U;
volatile uint32_t uart6_rx_half_count = 0U;
volatile uint32_t uart6_rx_full_count = 0U;
volatile uint32_t uart6_rx_idle_count = 0U;
volatile uint32_t uart6_rx_hardware_error_count = 0U;
volatile uint32_t uart6_rx_last_error = HAL_UART_ERROR_NONE;
volatile uint32_t uart6_rx_recovery_pending = 0U;
volatile uint32_t uart6_rx_recovery_count = 0U;
volatile uint32_t uart6_rx_recovery_error_count = 0U;
volatile uint32_t uart6_rx_overflow_count = 0U;
volatile uint32_t uart6_rx_dropped_bytes = 0U;
volatile uint32_t uart6_control_unsupported_count = 0U;
volatile uint32_t uart6_status_pending = 0U;
volatile uint32_t uart6_status_queue_overflow_count = 0U;
volatile uint32_t uart6_status_unavailable_count = 0U;
volatile uint32_t uart6_status_tx_count = 0U;
volatile uint32_t uart6_status_tx_error_count = 0U;
volatile uint32_t uart6_status_last_tx_status = 0xFFFFFFFFUL;
volatile uint32_t uart6_status_hal_error = HAL_UART_ERROR_NONE;
volatile uint32_t uart6_status_last_build_status = 0xFFFFFFFFUL;
volatile uint8_t uart6_status_last_frame[DAQ_STATUS_FRAME_SIZE];
volatile uint32_t uart6_status_last_valid = 0U;
volatile uint32_t uart6_status_snapshot_seq = 0U;
static uint8_t uart6_status_work[DAQ_STATUS_FRAME_SIZE];
/* UART6 gear command state: main-context only. */
enum
{
  UART6_GEAR_IDLE = 0U,
  UART6_GEAR_PENDING,
  UART6_GEAR_OK,
  UART6_GEAR_UNSUPPORTED,
  UART6_GEAR_REJECTED,
  UART6_GEAR_FAILED,
  UART6_GEAR_BUSY
};
volatile uint32_t uart6_gear_request_count = 0U;
volatile uint32_t uart6_gear_submit_count = 0U;
volatile uint32_t uart6_gear_success_count = 0U;
volatile uint32_t uart6_gear_error_count = 0U;
volatile uint32_t uart6_gear_unsupported_count = 0U;
volatile uint32_t uart6_gear_busy_count = 0U;
volatile uint32_t uart6_gear_pending = 0U;
volatile uint32_t uart6_gear_requested = 0xFFFFFFFFUL;
volatile uint32_t uart6_gear_last_request_status = 0xFFFFFFFFUL;
volatile uint32_t uart6_gear_last_result = UART6_GEAR_IDLE;
volatile uint32_t uart6_gear_last_switch_result = 0xFFFFFFFFUL;
/* End UART6 gear command state. */
/* End USART6 control receive/status state. */

/* Trigger engine state: main-context writer and reader.
 * The 24-point history contains only complete, coherent Full_Snapshot points.
 * A triggered frame stores 24 pre-trigger points, the edge point, then 25
 * post-trigger points. No dynamic allocation is used. */
typedef struct
{
  uint16_t ch1;
  uint16_t ch2;
  uint8_t dig;
} TriggerPoint;

static TriggerPoint trigger_history[DAQ_TRIGGER_PRE_SAMPLES];
static uint32_t trigger_history_head = 0U;
static uint32_t trigger_history_count = 0U;
static uint32_t trigger_previous_valid = 0U;
static TriggerPoint trigger_previous;
static TriggerPoint trigger_frame[CH1_SNAPSHOT_SAMPLES];
static uint32_t trigger_fill_count = 0U;
static uint32_t trigger_capture_gear = ACQ_GEAR_INVALID;
static uint32_t trigger_have_group = 0U;
static uint32_t trigger_last_group = 0U;
static volatile uint32_t trigger_frame_ready = 0U;
static volatile uint32_t trigger_capture_active = 0U;
static Full_Snapshot trigger_ready_snapshot;
/* End trigger engine state. */

/* Transport selection is local/preselected, not a new V1.1 command.
 * 0=UART; 1=UDP (boot default for the validated two-board setup).
 * Ordinary CMD01 still accepts 0/1.
 * The legacy ch1_uart_next_seq is the shared selected-transport wire SEQ;
 * UDP has separate submission diagnostics, never UART-success counters. */
volatile uint32_t daq_transport_request = 1U;
volatile uint32_t daq_transport_active = 0U;
volatile uint32_t daq_transport_result = 0U;
volatile uint32_t daq_transport_change_count = 0U;
volatile uint32_t daq_udp_data_error_count = 0U;
volatile uint32_t daq_udp_status_error_count = 0U;
volatile uint32_t daq_udp_last_seq = 0U;
volatile uint32_t daq_udp_last_group = 0U;
static uint8_t daq_udp_status_frame[DAQ_STATUS_FRAME_SIZE];
/* GEAR4 UDP throughput diagnostics: where the per-frame ~570 us goes.
 * Pure observation; no control path reads these values. */
volatile uint32_t daq_tx_build_last_cycles = 0U;
volatile uint32_t daq_tx_build_max_cycles = 0U;
volatile uint32_t daq_tx_net_last_cycles = 0U;
volatile uint32_t daq_tx_net_max_cycles = 0U;
volatile uint32_t daq_tx_frame_period_cycles = 0U;
volatile uint32_t daq_tx_frame_period_min_cycles = 0xFFFFFFFFUL;
volatile uint32_t daq_crc_bench_cycles = 0U;
volatile uint32_t daq_eth_maccr = 0U;
static uint32_t daq_tx_prev_cycle = 0U;
/* End-to-end send-rate evidence, computed inside the firmware so a stale
 * Live Watch display cannot distort the result. */
volatile uint32_t daq_tx_fps = 0U;
volatile uint32_t daq_tx_group_delta_last = 0U;
volatile uint32_t daq_tx_group_delta_max = 0U;
static uint32_t daq_tx_fps_count = 0U;
static uint32_t daq_tx_fps_start_cycle = 0U;
static uint32_t daq_tx_prev_group = 0U;
/* Main-loop cost split for the GEAR4 send-rate investigation. */
volatile uint32_t daq_loop_period_cycles = 0U;
volatile uint32_t daq_loop_period_min_cycles = 0xFFFFFFFFUL;
volatile uint32_t daq_transportpoll_cycles = 0U;
volatile uint32_t daq_transportpoll_max_cycles = 0U;
static uint32_t daq_loop_prev_cycle = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef UART6_TransmitFrame(const uint8_t *data,
                                           uint16_t length, uint32_t timeout);
static void UART6_ControlStart(void);
static uint32_t UART6_ServicePoll(void);
static void UART6_TestInit(void);
static void UART6_TestTransmitPoll(void);
static void CH1_FramePreviewPoll(void);
static uint32_t CH1_BuildFramePreview(uint16_t sequence);
static uint32_t Analog_BuildFramePreview(uint16_t sequence, uint32_t include_ch2);
static uint32_t CH1_FrameTransmitPoll(void);
static void CH1_BackgroundPoll(void);

static void CH1_SamplingStart(void);
static uint32_t Acquisition_OfflinePoll(void);
static void CH1_PublishGroup(uint32_t offset);
static void Full_PublishGroup(uint32_t offset);
static void DigitalSync_CaptureFault(uint32_t source, uint32_t ndtr, uint32_t trigger);
static HAL_StatusTypeDef DigitalCounter_Init(void);
static const AcquisitionGearProfile *Acquisition_GetGearProfile(uint32_t gear);
static void Acquisition_UDPPolicyPoll(void);
static HAL_StatusTypeDef SquareWave_ChangeFrequency(uint32_t frequency_hz);
static void Trigger_Reset(void);
static uint32_t Trigger_ProcessNewSnapshot(void);
static uint32_t Trigger_TakeReady(Full_Snapshot *snapshot);
static uint32_t Trigger_BuildFramePreview(uint16_t sequence,
                                          const Full_Snapshot *snapshot);
static void CH1_CaptureDebug(volatile CH1_DebugSnapshot *snapshot,
                             uint32_t primask_before);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Manual UART setup: keep failure diagnostics independent of ADC/PWM. */
static void UART6_TestInit(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  uart6_init_status = (uint32_t)HAL_UART_Init(&huart6);
  uart6_hal_error = HAL_UART_GetError(&huart6);
  uart6_initialized = (uart6_init_status == (uint32_t)HAL_OK) ? 1U : 0U;
  uart6_last_tx_tick = HAL_GetTick();
}

/* Retained for explicit diagnostics only; never scheduled in binary mode. */
__root static void UART6_TestTransmitPoll(void)
{
  uint32_t now = HAL_GetTick();

  /* Unsigned subtraction also works across the 32-bit tick wrap. */
  if ((uart6_initialized != 0U) &&
      ((uint32_t)(now - uart6_last_tx_tick) >= UART6_TEST_PERIOD_MS))
  {
    uart6_last_tx_tick = now;
    /* Polling only; interrupts remain enabled for ADC DMA and SysTick.
     * Exclude the string terminator and keep any failed attempt bounded. */
    uart6_last_tx_status = (uint32_t)UART6_TransmitFrame(
        uart6_test_message, (uint16_t)(sizeof(uart6_test_message) - 1U),
        UART6_TEST_TIMEOUT_MS);
    uart6_hal_error = HAL_UART_GetError(&huart6);
    if (uart6_last_tx_status == (uint32_t)HAL_OK)
    {
      uart6_tx_count++;
    }
    else
    {
      uart6_tx_error_count++;
    }
  }
}

/* Shared build path: one stable snapshot and one protocol build per group. */
static uint32_t Analog_BuildFramePreview(uint16_t sequence, uint32_t include_ch2)
{
  HAL_StatusTypeDef snapshot_status;
  DAQ_FrameStatus build_status;
  uint32_t group_delta;
  uint32_t group;
  uint32_t i;

  uint32_t channel_mask;

  /* include_ch2 selects the legacy preview path or the full snapshot. */
  if (include_ch2 == 2U)
  {
    snapshot_status = Full_GetSnapshot(&full_preview_snapshot);
  }
  else if (include_ch2 != 0U)
  {
    snapshot_status = Analog_GetSnapshot(&analog_preview_snapshot);
  }
  else
  {
    snapshot_status = CH1_GetSnapshot(&ch1_preview_snapshot);
  }
  if (snapshot_status != HAL_OK)
  {
    ch1_snapshot_fail_count++;
    return 0U;
  }
  group = (include_ch2 == 2U) ? full_preview_snapshot.group_count :
      ((include_ch2 != 0U) ? analog_preview_snapshot.group_count :
                            ch1_preview_snapshot.group_count);
  ch1_snapshot_ok_count++;
  if (ch1_preview_have_group != 0U)
  {
    group_delta = (uint32_t)(group -
                             ch1_preview_last_group);
    if (group_delta == 0U)
    {
      return 0U;
    }
    /* Modulo-2^32 difference handles UINT32_MAX -> 0. The first observed
     * group establishes a baseline; startup groups are not inferred lost. */
    ch1_skipped_group_count += group_delta - 1U;
  }
  /* Mark as seen BEFORE building: even a failed group is attempted once.
   * Build failures have their own counter and are not counted as skipped. */
  ch1_preview_have_group = 1U;
  ch1_preview_last_group = group;
  ch1_new_group_count++;

  /* Only the main-context dispatcher updates this mask. Capture it once for
   * the complete frame; RX IRQs only buffer bytes during a blocking send. */
  channel_mask = (include_ch2 == 2U) ? daq_active_channel_mask :
      ((include_ch2 != 0U) ? 0x03U : 0x01U);
  if (include_ch2 == 2U)
  {
    if (full_preview_snapshot.gear == 2U)
    {
      build_status = DAQ_BuildUDPGEAR2CandidateFrame(full_preview_snapshot.ch1,
          full_preview_snapshot.ch2, full_preview_snapshot.dig, channel_mask,
          sequence, ch1_preview_work, sizeof(ch1_preview_work));
    }
    else
    {
    build_status = DAQ_BuildSelectedFrameForGear(full_preview_snapshot.ch1,
        full_preview_snapshot.ch2, full_preview_snapshot.dig, full_preview_snapshot.gear,
        channel_mask, sequence, ch1_preview_work, sizeof(ch1_preview_work));
    }
  }
  else if (include_ch2 != 0U)
  {
    build_status = DAQ_BuildAnalogFrame(analog_preview_snapshot.ch1,
        analog_preview_snapshot.ch2, sequence, ch1_preview_work,
        sizeof(ch1_preview_work));
  }
  else
  {
    build_status = DAQ_BuildCH1Frame(ch1_preview_snapshot.samples, sequence,
                                    ch1_preview_work, sizeof(ch1_preview_work));
  }
  ch1_last_build_status = (uint32_t)build_status;
  if (build_status != DAQ_FRAME_OK)
  {
    ch1_frame_error_count++;
    return 0U;  /* Keep the last valid frame and its metadata intact. */
  }

  ch1_frame_preview_seq++;          /* Odd: Watch publication is in progress. */
  __DMB();
  for (i = 0U; i < DAQ_DATA_FRAME_SIZE; i++)
  {
    ch1_frame_preview[i] = ch1_preview_work[i];
  }
  ch1_frame_group = group;
  ch1_frame_channel_mask = channel_mask;
  ch1_frame_gear = (include_ch2 == 2U) ? full_preview_snapshot.gear : 0U;
  ch1_frame_preview_crc = (uint32_t)ch1_preview_work[DAQ_DATA_CRC_OFFSET] |
      ((uint32_t)ch1_preview_work[DAQ_DATA_CRC_OFFSET + 1U] << 8U);
  ch1_frame_build_count++;
  ch1_frame_preview_valid = 1U;
  __DMB();
  ch1_frame_preview_seq++;          /* Even: complete frame and metadata. */
  return 1U;
}

/* Backward-compatible CH1-only preview; production uses the selected mask. */
static uint32_t CH1_BuildFramePreview(uint16_t sequence)
{
  return Analog_BuildFramePreview(sequence, 0U);
}

/* Legacy preview-only entry retained for tests/explicit diagnostics.
 * Production calls the shared builder with the real UART SEQ instead. */
__root static void CH1_FramePreviewPoll(void)
{
  (void)CH1_BuildFramePreview((uint16_t)0U);
}

/* Returns 1 after processing a new group, 0 when no work is ready.
 * No callback/IRQ may call this service or mutate ch1_preview_work. */
static uint32_t CH1_FrameTransmitPoll(void)
{
  uint32_t new_groups_before;
  uint32_t i;
  uint16_t sequence;
  Full_Snapshot trigger_snapshot;
  HAL_StatusTypeDef status;

  if (daq_trigger_source != DAQ_TRIGGER_SOURCE_OFF)
  {
    uint32_t trigger_work = Trigger_ProcessNewSnapshot();
    sequence = ch1_uart_next_seq;
    if ((Trigger_TakeReady(&trigger_snapshot) == 0U) ||
        (Trigger_BuildFramePreview(sequence, &trigger_snapshot) == 0U))
    {
      return trigger_work;
    }
    goto TriggerFrameReady;
  }

  /* Cheap idle gate only. Full_GetSnapshot performs the authoritative
   * data/group consistency check inside the shared builder. */
  if ((adc_snapshot_valid == 0U) ||
      ((ch1_preview_have_group != 0U) &&
       (adc_group_count == ch1_preview_last_group)))
  {
    return 0U;
  }
  new_groups_before = ch1_new_group_count;
  sequence = ch1_uart_next_seq;
  if (Analog_BuildFramePreview(sequence, 2U) == 0U)
  {
    return (ch1_new_group_count != new_groups_before) ? 1U : 0U;
  }

TriggerFrameReady:
  if ((uart6_initialized == 0U) || (ch1_frame_gear > 1U))
  {
    ch1_uart_not_ready_count++;
    return 1U; /* Preview remains available; no HAL call or false HAL result. */
  }

  /* The shared builder already produced SEQ and its CRC in this private,
   * main-only buffer. Polling is bounded, and ADC DMA IRQs remain enabled. */
  status = UART6_TransmitFrame(ch1_preview_work,
                            (uint16_t)DAQ_DATA_FRAME_SIZE, UART6_FRAME_TIMEOUT_MS);
  ch1_uart_last_tx_status = (uint32_t)status;
  ch1_uart_hal_error = HAL_UART_GetError(&huart6);
  if (status != HAL_OK)
  {
    ch1_uart_tx_error_count++;
    /* This group is already consumed. No replay or history queue, including
     * after a partial timeout. The next new group reuses the unadvanced SEQ. */
    return 1U;
  }

  ch1_uart_last_snapshot_seq++;
  __DMB();
  for (i = 0U; i < DAQ_DATA_FRAME_SIZE; i++)
  {
    ch1_uart_last_frame[i] = ch1_preview_work[i];
  }
  ch1_uart_last_seq = sequence;
  ch1_uart_last_group = ch1_frame_group;
  ch1_uart_tx_count++;
  ch1_uart_next_seq = (uint16_t)((uint32_t)sequence + 1U);
  ch1_uart_last_valid = 1U;
  __DMB();
  ch1_uart_last_snapshot_seq++;
  return 1U;
}

static void CH1_BackgroundPoll(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed;
  uint32_t successes;

  if ((uint32_t)(now - background_gpio_tick) >= GPIO_INPUT_PERIOD_MS)
  {
    background_gpio_tick = now;
    digital_input_d1 = (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;
    digital_input_d2 = (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
    digital_input_d3 = (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_3) == GPIO_PIN_SET) ? 1U : 0U;
  }
  elapsed = (uint32_t)(now - background_rate_tick);
  if (elapsed >= UART6_RATE_WINDOW_MS)
  {
    successes = ch1_uart_tx_count;
    ch1_uart_rate_frames = (uint32_t)(successes - background_rate_count);
    ch1_uart_rate_elapsed_ms = elapsed;
    background_rate_count = successes;
    background_rate_tick = now;
  }
}

/* Sole HAL transmit call site; all callers are main-context only. */
static HAL_StatusTypeDef UART6_TransmitFrame(const uint8_t *data,
                                           uint16_t length, uint32_t timeout)
{
  HAL_StatusTypeDef status;
  if (uart6_transmit_busy != 0U) { return HAL_BUSY; }
  uart6_transmit_busy = 1U;
  status = HAL_UART_Transmit(&huart6, data, length, timeout);
  uart6_transmit_busy = 0U;
  return status;
}

static void UART6_QueueStatus(void)
{
  if (uart6_status_pending < UART6_STATUS_PENDING_LIMIT)
  {
    uart6_status_pending++;
  }
  else { uart6_status_queue_overflow_count++; }
}

/* DMA and USART IRQs share priority 2, below TIM2=0 and ADC DMA=1.
 * No stream parsing, status construction or transmission in these callbacks. */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart6) { uart6_rx_half_count++; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart6)
  {
    uart6_rx_wraps++;
    uart6_rx_full_count++;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart6)
  {
    uart6_rx_last_error = HAL_UART_GetError(uart);
    uart6_rx_hardware_error_count++;
    uart6_rx_active = 0U;
    uart6_rx_recovery_pending = 1U;
  }
}

void UART6_ReceiveIRQ(void)
{
  /* Manual IDLE notification with standard circular Receive_DMA (not ToIdle).
   * Clear it here: HAL's standard reception handler does not service IDLE. */
  if ((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) &&
      (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    uart6_rx_idle_count++;
  }
  HAL_UART_IRQHandler(&huart6);
}

static HAL_StatusTypeDef UART6_ReceiveArm(void)
{
  HAL_StatusTypeDef status;
  uart6_rx_wraps = 0U;
  uart6_rx_consumed = 0U;
  uart6_rx_observed = 0U;
  DAQ_ControlStreamReset(&uart6_control_stream);
  status = HAL_UART_Receive_DMA(&huart6,
      (uint8_t *)uart6_rx_buffer, UART6_RX_BUFFER_SIZE);
  uart6_rx_start_status = (uint32_t)status;
  uart6_rx_active = (status == HAL_OK) ? 1U : 0U;
  if (status == HAL_OK)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  }
  return status;
}

static void UART6_ControlStart(void)
{
  if (uart6_control_started != 0U) { return; }
  uart6_control_started = 1U;
  DAQ_ControlStreamInit(&uart6_control_stream);
  UART6_QueueStatus(); /* One boot response request, never periodically repeated. */
  if (uart6_initialized == 0U) { return; }
  __HAL_RCC_DMA2_CLK_ENABLE();
  hdma_usart6_rx.Instance = DMA2_Stream1;
  hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
  hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  uart6_rx_start_status = (uint32_t)HAL_DMA_Init(&hdma_usart6_rx);
  if (uart6_rx_start_status != (uint32_t)HAL_OK) { return; }
  __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 2U, 0U);
  HAL_NVIC_SetPriority(USART6_IRQn, 2U, 0U);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART6_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  (void)UART6_ReceiveArm();
}

static void UART6_ReceiveRecover(void)
{
  HAL_StatusTypeDef status;
  if (uart6_rx_recovery_pending == 0U) { return; }
  HAL_NVIC_DisableIRQ(USART6_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
  __HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
  /* Bounded HAL aborts with SysTick and all acquisition IRQs still enabled. */
  status = HAL_UART_AbortReceive(&huart6);
  if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) != 0U)
  {
    if (HAL_DMA_Abort(&hdma_usart6_rx) != HAL_OK) { status = HAL_ERROR; }
  }
  uart6_rx_recovery_pending = 0U;
  DAQ_ControlStreamReset(&uart6_control_stream);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART6_IRQn);
  if (status == HAL_OK) { status = UART6_ReceiveArm(); }
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
  if (status == HAL_OK) { uart6_rx_recovery_count++; }
  else
  {
    uart6_rx_active = 0U;
    uart6_rx_start_status = (uint32_t)status;
    uart6_rx_recovery_error_count++;
    /* No endless restart loop. A failed recovery remains visible in Watch. */
  }
}

/* A pending transfer-complete IRQ makes wrap/NDTR ambiguous: defer the read.
 * IRQs stay enabled and normally resolve this before the next main iteration. */
static uint32_t UART6_ReceivePosition(uint32_t *position)
{
  uint32_t attempt;
  for (attempt = 0U; attempt < 2U; attempt++)
  {
    uint32_t wraps = uart6_rx_wraps;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
    uint32_t pending = __HAL_DMA_GET_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);
    __DMB();
    if ((wraps == uart6_rx_wraps) && (pending == 0U) &&
        (remaining > 0U) && (remaining <= UART6_RX_BUFFER_SIZE))
    {
      *position = wraps * UART6_RX_BUFFER_SIZE + UART6_RX_BUFFER_SIZE - remaining;
      return 1U;
    }
  }
  return 0U;
}

static void UART6_DispatchControl(const DAQ_ControlCommand *command)
{
  HAL_StatusTypeDef status;
  if (command->command == DAQ_CMD_REQUEST_STATUS) { UART6_QueueStatus(); }
  else if (command->command == DAQ_CMD_SET_WAVE_FREQUENCY)
  {
    /* UART and UDP both dispatch here in main, between complete TX calls. */
    if (SquareWave_ChangeFrequency(command->parameter) == HAL_OK)
    { UART6_QueueStatus(); }
  }
  else if (command->command == DAQ_CMD_SET_TRIGGER_SOURCE)
  {
    daq_trigger_source = command->parameter;
    Trigger_Reset();
    trigger_command_ok_count++;
    UART6_QueueStatus();
  }
  else if (command->command == DAQ_CMD_SET_TRIGGER_MODE)
  {
    daq_trigger_mode = command->parameter;
    Trigger_Reset();
    trigger_command_ok_count++;
    UART6_QueueStatus();
  }
  else if (command->command == DAQ_CMD_SET_TRIGGER_LEVEL)
  {
    daq_trigger_level = command->parameter;
    Trigger_Reset();
    trigger_command_ok_count++;
    UART6_QueueStatus();
  }
  else if (command->command == DAQ_CMD_SET_CHANNEL_MASK)
  {
    /* Parser already validated 0..0x1F and PARAM_H/reserved/CRC. This runs
     * between complete UART frames, never in an IRQ or inside a transmit. */
    daq_active_channel_mask = command->parameter;
    uart6_mask_apply_count++;
    UART6_QueueStatus(); /* Existing TX arbiter sends actual state first. */
  }
  else if (command->command == DAQ_CMD_SET_GEAR)
  {
    uart6_gear_request_count++;
    /* GEAR2..4 are UDP-only production modes. UART6 cannot carry their
     * high-rate data stream, so it never submits these requests. */
    if ((command->parameter >= ACQ_GEAR_COUNT) ||
        ((command->parameter >= 2U) && (acquisition_udp_gear2_available == 0U)))
    {
      uart6_control_unsupported_count++;
      uart6_gear_unsupported_count++;
      uart6_gear_last_result = UART6_GEAR_UNSUPPORTED;
      UART6_QueueStatus(); /* Actual state only; not a success acknowledgement. */
      return;
    }
    if (uart6_gear_pending != 0U)
    {
      uart6_gear_busy_count++;
      UART6_QueueStatus(); /* Deferred until the existing request is resolved. */
      return; /* No overwrite, queue or automatic retry of this request. */
    }
    uart6_gear_requested = command->parameter;
    status = Acquisition_RequestGear(command->parameter);
    uart6_gear_last_request_status = (uint32_t)status;
    if (status != HAL_OK)
    {
      uart6_gear_last_switch_result = acquisition_last_switch_result;
      uart6_gear_last_result = UART6_GEAR_REJECTED;
      uart6_gear_error_count++;
      UART6_QueueStatus(); /* For example, GEAR1 conflicts with ADC144 mode. */
      return;
    }
    uart6_gear_submit_count++;
    uart6_gear_pending = 1U;
    uart6_gear_last_result = UART6_GEAR_PENDING;
    /* The next main-loop Acquisition_GearPoll performs the hardware change. */
  }
  else { uart6_control_unsupported_count++; }
}

static uint32_t UART6_GearControlPoll(void)
{
  uint32_t requested;
  if (uart6_gear_pending == 0U) { return 0U; }
  if ((acquisition_request_pending != 0U) || (acquisition_switching != 0U))
  { return 1U; } /* Do not publish a response or an old data frame yet. */
  uart6_gear_pending = 0U;
  uart6_gear_last_switch_result = acquisition_last_switch_result;
  requested = uart6_gear_requested;
  if ((acquisition_last_switch_result == ACQ_SWITCH_OK) &&
      (acquisition_active_gear == requested) &&
      (adc_running != 0U) && (acquisition_fault_latched == 0U) &&
      (adc_error == 0U) && (dma_error == 0U) && (digital_sync.error == 0U))
  {
    uart6_gear_last_result = UART6_GEAR_OK;
    uart6_gear_success_count++;
  }
  else
  {
    uart6_gear_last_result = UART6_GEAR_FAILED;
    uart6_gear_error_count++;
  }
  /* The existing status getter refuses stopped/faulted acquisition. V1.1 has
   * no failure field: never manufacture a healthy status on switch failure. */
  UART6_QueueStatus();
  return 1U;
}

static uint32_t UART6_ReceivePoll(void)
{
  uint32_t before;
  uint32_t after;
  uint32_t available;
  uint32_t count;
  uint32_t i;
  DAQ_ControlCommand command;
  UART6_ReceiveRecover();
  DAQ_ControlStreamExpire(&uart6_control_stream, HAL_GetTick());
  if ((uart6_rx_active == 0U) || (UART6_ReceivePosition(&before) == 0U)) { return 0U; }
  uart6_rx_byte_count += (uint32_t)(before - uart6_rx_observed);
  uart6_rx_observed = before;
  available = (uint32_t)(before - uart6_rx_consumed);
  if (available == 0U) { return 0U; }
  if (available > UART6_RX_BUFFER_SIZE)
  {
    uart6_rx_overflow_count++;
    uart6_rx_dropped_bytes += available;
    uart6_rx_consumed = before;
    DAQ_ControlStreamReset(&uart6_control_stream);
    return 1U;
  }
  count = (available < UART6_RX_COPY_SIZE) ? available : UART6_RX_COPY_SIZE;
  for (i = 0U; i < count; i++)
  {
    uart6_rx_work[i] = uart6_rx_buffer[(uart6_rx_consumed + i) % UART6_RX_BUFFER_SIZE];
  }
  __DMB();
  if ((uart6_rx_recovery_pending != 0U) ||
      (UART6_ReceivePosition(&after) == 0U)) { return 1U; }
  if ((uint32_t)(after - uart6_rx_consumed) > UART6_RX_BUFFER_SIZE)
  {
    uart6_rx_overflow_count++;
    uart6_rx_dropped_bytes += (uint32_t)(after - uart6_rx_consumed);
    uart6_rx_consumed = after;
    DAQ_ControlStreamReset(&uart6_control_stream);
    return 1U;
  }
  uart6_rx_consumed += count;
  for (i = 0U; (i < count) && (uart6_rx_recovery_pending == 0U); i++)
  {
    if (DAQ_ControlStreamFeed(&uart6_control_stream, uart6_rx_work[i],
                             HAL_GetTick(), &command) != 0U)
    {
      UART6_DispatchControl(&command);
    }
  }
  return 1U;
}

static uint32_t UART6_GetStatus(DAQ_StatusState *state)
{
  /* Hardware always samples all five inputs; CMD02 gates transmitted fields.
   * Refuse an unavailable source instead of inventing a valid running state. */
  if ((adc_running == 0U) || (acquisition_switching != 0U) ||
      (acquisition_active_gear > 1U) || (acquisition_fault_latched != 0U) ||
      (digital_sync.error != 0U) || (square_wave_running == 0U) ||
      (square_wave_actual_hz < 100U) || (square_wave_actual_hz > 10000U))
  { return 0U; }
  state->gear = acquisition_active_gear;
  state->channel_mask = daq_active_channel_mask;
  state->trigger_source = daq_trigger_source;
  state->trigger_mode = daq_trigger_mode;
  state->trigger_level = daq_trigger_level;
  state->wave_frequency_hz = square_wave_actual_hz;
  return 1U;
}

static uint32_t UART6_StatusTransmitPoll(void)
{
  DAQ_StatusState state;
  HAL_StatusTypeDef status;
  uint32_t i;
  if (uart6_status_pending == 0U) { return 0U; }
  uart6_status_pending--;
  if ((uart6_initialized == 0U) || (UART6_GetStatus(&state) == 0U))
  {
    uart6_status_unavailable_count++;
    return 1U;
  }
  uart6_status_last_build_status = (uint32_t)DAQ_BuildStatusFrame(
      &state, uart6_status_work, sizeof(uart6_status_work));
  if (uart6_status_last_build_status != (uint32_t)DAQ_STATUS_OK)
  {
    uart6_status_unavailable_count++;
    return 1U;
  }
  status = UART6_TransmitFrame(uart6_status_work, DAQ_STATUS_FRAME_SIZE,
                             UART6_STATUS_TIMEOUT_MS);
  uart6_status_last_tx_status = (uint32_t)status;
  uart6_status_hal_error = HAL_UART_GetError(&huart6);
  if (status == HAL_OK)
  {
    uart6_status_snapshot_seq++;
    __DMB();
    uart6_status_last_valid = 0U;
    for (i = 0U; i < DAQ_STATUS_FRAME_SIZE; i++)
    { uart6_status_last_frame[i] = uart6_status_work[i]; }
    uart6_status_tx_count++;
    uart6_status_last_valid = 1U;
    __DMB();
    uart6_status_snapshot_seq++;
  }
  else { uart6_status_tx_error_count++; }
  return 1U;
}

/* One automatic TX manager: one whole status frame, then one whole data frame.
 * Sustained command floods are bounded; queue overflow is reported, not hidden. */
static uint32_t UART6_ServicePoll(void)
{
  uint32_t work;
  if (uart6_transmit_busy != 0U) { return 0U; }
  work = UART6_ReceivePoll();
  work |= UART6_GearControlPoll();
  if (uart6_gear_pending != 0U) { return 1U; }
  work |= UART6_StatusTransmitPoll();
  work |= CH1_FrameTransmitPoll();
  return work;
}

/* TIM2 update IRQ only: two port reads, O(1) ring bookkeeping, no frame work. */
/* TIM5 ITR0 = TIM2 TRGO (RM0385 table 130). It counts every hardware event,
 * including ones whose TIM2 IRQ was missed; no pin, DMA, or TIM5 IRQ is used. */
static HAL_StatusTypeDef DigitalCounter_Init(void)
{
  TIM_SlaveConfigTypeDef slave = {0};
  __HAL_RCC_TIM5_CLK_ENABLE();
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0U;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFFUL;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) { return HAL_ERROR; }
  slave.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  slave.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &slave) != HAL_OK) { return HAL_ERROR; }
  __HAL_TIM_SET_COUNTER(&htim5, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
  __HAL_DBGMCU_FREEZE_TIM5();
  return HAL_TIM_Base_Start(&htim5);
}

/* All gear transitions execute in main context, never from an ISR. */
static void Acquisition_Invalidate(void)
{
  adc_snapshot_seq++;
  __DMB();
  adc_snapshot_valid = 0U;
  adc_group_ready = 0U;
  __DMB();
  adc_snapshot_seq++;
  full_snapshot_seq++;
  __DMB();
  full_snapshot_valid = 0U;
  __DMB();
  full_snapshot_seq++;
  ch1_frame_preview_seq++;
  __DMB();
  ch1_frame_preview_valid = 0U;
  __DMB();
  ch1_frame_preview_seq++;
}

static uint32_t Acquisition_Stop(void)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t reason = ACQ_SWITCH_OK;
  /* Only the short hardware gate is IRQ-masked. HAL stop/abort waits below
   * run with SysTick enabled. No callback can publish after this gate. */
  __disable_irq();
  acquisition_switching = 1U;
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
  CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_DisableIRQ(ADC_IRQn);
  adc_running = 0U;
  acquisition_fast_isr_active = 0U;
  __DSB();
  if (offline.state == OFFLINE_RESTORING)
  {
    offline_end_cycle = DWT->CYCCNT;
    offline_end_tick = HAL_GetTick();
  }
  __set_PRIMASK(primask);
  acquisition_active_gear = ACQ_GEAR_INVALID;
  acquisition_sample_interval_us = 0U;
  acquisition_adc_active_cycles = ACQ_GEAR_INVALID;
  acquisition_adc_test_mode = 0U;
  Acquisition_Invalidate();

  acquisition_tim2_stop_status = (uint32_t)HAL_TIM_Base_Stop_IT(&htim2);
  acquisition_adc_stop_status = (uint32_t)HAL_ADC_Stop_DMA(&hadc3);
  acquisition_tim5_stop_status = (htim5.Instance == TIM5) ?
      (uint32_t)HAL_TIM_Base_Stop(&htim5) : (uint32_t)HAL_OK;
  if ((acquisition_tim2_stop_status != (uint32_t)HAL_OK) ||
      ((TIM2->CR1 & TIM_CR1_CEN) != 0U)) { reason = ACQ_SWITCH_TIM2_STOP; }
  else if ((acquisition_adc_stop_status != (uint32_t)HAL_OK) ||
      ((hadc3.Instance->CR2 & ADC_CR2_ADON) != 0U) ||
      ((hdma_adc3.Instance->CR & DMA_SxCR_EN) != 0U)) { reason = ACQ_SWITCH_ADC_STOP; }
  else if ((acquisition_tim5_stop_status != (uint32_t)HAL_OK) ||
      ((htim5.Instance == TIM5) && ((TIM5->CR1 & TIM_CR1_CEN) != 0U)))
  { reason = ACQ_SWITCH_TIM5_STOP; }
  adc_hal_error |= HAL_ADC_GetError(&hadc3);
  dma_error |= HAL_DMA_GetError(&hdma_adc3);
  return reason;
}

static HAL_StatusTypeDef Acquisition_Fail(uint32_t reason)
{
  /* A failed arm can leave HAL partly configured; force the trigger gate
   * shut and try bounded cleanup. Never resume automatically after failure. */
  (void)Acquisition_Stop();
  DigitalSync_Fault(&digital_sync, DIGITAL_SYNC_ERROR_CONFIG);
  acquisition_fault_latched = 1U;
  acquisition_last_switch_result = reason;
  acquisition_switching = 0U;
  return HAL_ERROR;
}

/* Configure the complete regular pair only with trigger, ADC and DMA stopped.
 * On partial HAL failure the caller latches a fault and never starts sampling. */
static HAL_StatusTypeDef Acquisition_ConfigureADC(uint32_t sample_cycles)
{
  ADC_ChannelConfTypeDef channel = {0};
  HAL_StatusTypeDef status;
  uint32_t sampling;
  uint32_t smpr2;
  uint32_t sqr3;
  acquisition_adc_rank1_status = 0xFFFFFFFFUL;
  acquisition_adc_rank2_status = 0xFFFFFFFFUL;
  if ((adc_running != 0U) || ((TIM2->CR1 & TIM_CR1_CEN) != 0U) ||
      ((hadc3.Instance->CR2 & ADC_CR2_ADON) != 0U) ||
      ((hdma_adc3.Instance->CR & DMA_SxCR_EN) != 0U)) { return HAL_BUSY; }
  if (sample_cycles == 480U) { sampling = ADC_SAMPLETIME_480CYCLES; }
  else if (sample_cycles == 144U) { sampling = ADC_SAMPLETIME_144CYCLES; }
  else if (sample_cycles == 28U) { sampling = ADC_SAMPLETIME_28CYCLES; }
  else if (sample_cycles == 3U) { sampling = ADC_SAMPLETIME_3CYCLES; }
  else { return HAL_ERROR; }
  channel.Channel = ADC_CHANNEL_0;
  channel.Rank = ADC_REGULAR_RANK_1;
  channel.SamplingTime = sampling;
  status = HAL_ADC_ConfigChannel(&hadc3, &channel);
  acquisition_adc_rank1_status = (uint32_t)status;
  if (status != HAL_OK) { return status; }
  channel.Channel = ADC_CHANNEL_8;
  channel.Rank = ADC_REGULAR_RANK_2;
  status = HAL_ADC_ConfigChannel(&hadc3, &channel);
  acquisition_adc_rank2_status = (uint32_t)status;
  if (status != HAL_OK) { return status; }
  /* Verify both sample fields and both ranks before arming either channel. */
  smpr2 = hadc3.Instance->SMPR2;
  sqr3 = hadc3.Instance->SQR3;
  if (((smpr2 & ADC_SMPR2_SMP0) != sampling) ||
      ((smpr2 & ADC_SMPR2_SMP8) != (sampling << 24U)) ||
      ((sqr3 & ADC_SQR3_SQ1) != ADC_CHANNEL_0) ||
      ((sqr3 & ADC_SQR3_SQ2) != (ADC_CHANNEL_8 << 5U))) { return HAL_ERROR; }
  return HAL_OK;
}

static HAL_StatusTypeDef Acquisition_ApplyConfiguration(uint32_t gear, uint32_t sample_cycles)
{
  uint32_t reason;
  uint32_t i;
  uint32_t pclk;
  uint32_t hclk;
  uint32_t timer_clock;
  uint32_t old_samples;
  uint32_t old_errors;
  HAL_StatusTypeDef status;
  const AcquisitionGearProfile *profile = Acquisition_GetGearProfile(gear);
  const DigitalSyncTimingConfig *timing = DigitalSync_GetTimingConfigForADC(gear, sample_cycles);
  /* Reject a mismatched pair before stopping healthy acquisition. GEAR0/144
   * is the sole non-default ADC experiment; all other pairs come from the
   * single production table. */
  if ((profile == NULL) || (timing == NULL) ||
      ((sample_cycles != profile->adc_sample_cycles) &&
       !((gear == 0U) && (sample_cycles == 144U))) ||
      (timing->sample_interval_us != profile->sample_interval_us) ||
      (!((gear == 0U) && (sample_cycles == 144U)) &&
       ((timing->max_phase_cycles != profile->max_phase_cycles) ||
        (timing->max_body_cycles != profile->max_body_cycles))))
  { return HAL_ERROR; }
  if ((gear >= 2U) &&
      !(((gear == 2U) && (offline.state == OFFLINE_STARTING)) ||
        ((gear == 2U) && (udp_trial.state == UDP_TRIAL_STARTING)) ||
        ((acquisition_udp_gear2_starting != 0U) &&
         (acquisition_udp_gear2_available != 0U))))
  { return HAL_ERROR; }
  reason = Acquisition_Stop();
  if (reason != ACQ_SWITCH_OK)
  {
    DigitalSync_Fault(&digital_sync, DIGITAL_SYNC_ERROR_CONFIG);
    acquisition_fault_latched = 1U;
    acquisition_last_switch_result = reason;
    acquisition_switching = 0U;
    return HAL_ERROR;
  }

  if ((gear == 2U) && (offline.state == OFFLINE_STARTING))
  {
    /* Counters are now frozen; no old baseline IRQ can enter this window. */
    offline_base_publish = full_group_publish_count;
    offline_base_analog = adc_group_count;
    offline_base_half = adc_half_callback_count;
    offline_base_full = adc_full_callback_count;
    offline_base_irq = dma2_stream0_irq_count;
    offline_last_group = adc_group_count;
    offline_seen_publish = full_group_publish_count;
  }
  if (udp_trial.state == UDP_TRIAL_STARTING)
  {
    udp_trial.base_published = full_group_publish_count;
    udp_trial.base_analog = adc_group_count;
    udp_trial.base_half = adc_half_callback_count;
    udp_trial.base_full = adc_full_callback_count;
  }
  status = Acquisition_ConfigureADC(sample_cycles);
  if (status != HAL_OK) { return Acquisition_Fail(ACQ_SWITCH_ADC_CONFIG); }

  pclk = HAL_RCC_GetPCLK1Freq();
  hclk = HAL_RCC_GetHCLKFreq();
  if ((RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) == 0U)
  { timer_clock = (pclk == hclk) ? pclk : (2U * pclk); }
  else
  { timer_clock = (pclk >= (hclk / 4U)) ? hclk : (4U * pclk); }
  acquisition_timer_clock_hz = timer_clock;
  if ((timer_clock < 1000000U) || ((timer_clock % 1000000U) != 0U) ||
      ((timer_clock / 1000000U) > 65536U))
  { return Acquisition_Fail(ACQ_SWITCH_CLOCK); }

  /* TIM_Base_SetConfig issues UG even with URS set. MMS=ENABLE with CEN=0
   * holds TRGO low; ADC and TIM5 are also stopped. MMS=RESET is NOT a gate:
   * in that mode UG itself is routed to TRGO. Restore UPDATE only after UG. */
  MODIFY_REG(TIM2->CR2, TIM_CR2_MMS, TIM_TRGO_ENABLE);
  htim2.Init.Prescaler = timer_clock / 1000000U - 1U;
  htim2.Init.Period = timing->sample_interval_us - 1U;
  status = HAL_TIM_Base_Init(&htim2);
  if (status != HAL_OK) { return Acquisition_Fail(ACQ_SWITCH_TIM2_CONFIG); }
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  MODIFY_REG(TIM2->CR2, TIM_CR2_MMS, TIM_TRGO_UPDATE);
  if ((TIM2->PSC != htim2.Init.Prescaler) ||
      (TIM2->ARR != htim2.Init.Period) || ((TIM2->CR1 & TIM_CR1_CEN) != 0U))
  { return Acquisition_Fail(ACQ_SWITCH_TIM2_CONFIG); }

  /* Drop the old partial group. Group IDs and protocol SEQ never reset. */
  old_samples = digital_sync.sample_count;
  old_errors = digital_sync.error_count;
  for (i = 0U; i < CH1_DMA_SAMPLES; i++) { adc_dma_buffer[i] = 0xFFFFU; }
  DigitalSync_Init(&digital_sync);
  digital_sync.timing = *timing; /* IRQs and acquisition are stopped. */
  digital_sync.completed_groups = adc_group_count;
  digital_sync.sample_count = old_samples;
  digital_sync.error_count = old_errors;
  __HAL_DMA_CLEAR_FLAG(&hdma_adc3, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_adc3) |
      __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_adc3) | __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_adc3) |
      __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_adc3) | __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_adc3));
  __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOC | ADC_FLAG_OVR | ADC_FLAG_STRT);
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
  __DSB();
  if (((offline.state == OFFLINE_STARTING) || (offline.state == OFFLINE_RESTORING)) &&
      (((DMA2->LISR & (DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 |
                      DMA_LISR_HTIF0 | DMA_LISR_TCIF0)) != 0U) ||
       ((hadc3.Instance->SR & (ADC_FLAG_EOC | ADC_FLAG_OVR | ADC_FLAG_STRT)) != 0U) ||
       ((TIM2->SR & TIM_FLAG_UPDATE) != 0U)))
  { return Acquisition_Fail(ACQ_SWITCH_CLEANUP); }
  adc_previous_group_cycle = DWT->CYCCNT;
  adc_last_group_tick = HAL_GetTick();
  adc_startup_stage = 6U;
  status = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)(void *)adc_dma_buffer, CH1_DMA_SAMPLES);
  adc_dma_start_status = (uint32_t)status;
  if ((status != HAL_OK) || (HAL_ADC_GetError(&hadc3) != HAL_ADC_ERROR_NONE) ||
      ((hdma_adc3.Instance->CR & DMA_SxCR_EN) == 0U) ||
      (hdma_adc3.Instance->NDTR != CH1_DMA_SAMPLES))
  {
    adc_error |= ADC_ERROR_START;
    adc_error_count++;
    return Acquisition_Fail(ACQ_SWITCH_ADC_START);
  }
  digital_counter_status = (uint32_t)DigitalCounter_Init();
  if (digital_counter_status != (uint32_t)HAL_OK)
  { return Acquisition_Fail(ACQ_SWITCH_TIM5_START); }

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  acquisition_active_gear = gear;
  acquisition_adc_active_cycles = sample_cycles;
  acquisition_adc_test_mode = (sample_cycles == 144U) ? 1U : 0U;
  acquisition_sample_interval_us = timing->sample_interval_us;
  acquisition_fast_isr_active = profile->fast_isr;
  acquisition_fast_irq_last_cycles = 0U;
  acquisition_fast_irq_max_cycles = 0U;
  acquisition_fast_irq_budget_cycles = profile->max_body_cycles;
  acquisition_fast_phase_last_cycles = 0U;
  acquisition_fast_phase_max_cycles = 0U;
  acquisition_switching = 0U;
  adc_startup_stage = 7U;
  adc_running = 1U;
  __DMB();
  if (offline.state == OFFLINE_STARTING)
  {
    offline_start_tick = HAL_GetTick();
    offline_start_cycle = DWT->CYCCNT;
  }
  if (udp_trial.state == UDP_TRIAL_STARTING) { udp_trial.start_tick = HAL_GetTick(); }
  status = HAL_TIM_Base_Start_IT(&htim2); /* First point: the first overflow. */
  tim2_start_status = (uint32_t)status;
  if (status != HAL_OK)
  {
    adc_error |= ADC_ERROR_TIMER_START;
    adc_error_count++;
    return Acquisition_Fail(ACQ_SWITCH_TIM2_START);
  }
  adc_startup_stage = 8U;
  acquisition_last_switch_result = ACQ_SWITCH_OK;
  CH1_CaptureDebug(&adc_debug_start, __get_PRIMASK());
  return HAL_OK;
}

static const AcquisitionGearProfile *Acquisition_GetGearProfile(uint32_t gear)
{
  return DigitalSync_GetTimingConfig(gear);
}

static HAL_StatusTypeDef Acquisition_ApplyGear(uint32_t gear)
{
  const AcquisitionGearProfile *profile = Acquisition_GetGearProfile(gear);
  if (profile == NULL) { return HAL_ERROR; }
  return Acquisition_ApplyConfiguration(gear, profile->adc_sample_cycles);
}

__root HAL_StatusTypeDef Acquisition_RequestGear(uint32_t gear)
{
  if (UDP_TRIAL_OWNS(udp_trial.state)) { return HAL_BUSY; }
  if ((offline.state == OFFLINE_STARTING) || (offline.state == OFFLINE_RUNNING) ||
      (offline.state == OFFLINE_RESTORING) || (offline.state == OFFLINE_FAULT))
  { return HAL_BUSY; }
  if (gear >= ACQ_GEAR_COUNT)
  {
    acquisition_last_switch_result = ACQ_SWITCH_BAD_GEAR;
    acquisition_switch_fail_count++;
    return HAL_ERROR;
  }
  if ((gear >= 2U) && (acquisition_udp_gear2_available == 0U))
  {
    acquisition_last_switch_result = ACQ_SWITCH_UDP_REQUIRED;
    acquisition_switch_fail_count++;
    return HAL_ERROR;
  }
  /* GEAR0/144 is the only ADC experiment that may not be mixed with a high
   * gear. A healthy high gear legitimately runs at 28/3 cycles, so 480 is not
   * a valid precondition when its UDP ownership bit is being re-established. */
  if ((gear >= 2U) && (acquisition_udp_gear2_active == 0U) &&
      ((acquisition_adc_active_cycles == 144U) ||
       ((acquisition_adc_request_pending != 0U) &&
        (acquisition_adc_requested_cycles == 144U))))
  {
    acquisition_last_switch_result = ACQ_SWITCH_ADC_CONFLICT;
    acquisition_switch_fail_count++;
    return HAL_ERROR;
  }
  if ((gear == 1U) && (acquisition_udp_gear2_active == 0U) &&
      ((acquisition_adc_active_cycles == 144U) ||
      ((acquisition_adc_request_pending != 0U) && (acquisition_adc_requested_cycles == 144U))))
  {
    acquisition_last_switch_result = ACQ_SWITCH_ADC_CONFLICT;
    acquisition_switch_fail_count++;
    return HAL_ERROR;
  }
  acquisition_requested_gear = gear;
  __DMB();
  acquisition_request_pending = 1U;
  return HAL_OK;
}

/* Request acceptance is not hardware completion. Rejected combinations leave
 * the running acquisition untouched. Watch may also edit requested_cycles. */
__root HAL_StatusTypeDef Acquisition_RequestADCSampling(uint32_t sample_cycles)
{
  if (UDP_TRIAL_OWNS(udp_trial.state)) { return HAL_BUSY; }
  if ((acquisition_udp_gear2_active != 0U) ||
      (acquisition_udp_gear2_starting != 0U)) { return HAL_BUSY; }
  if ((offline.state == OFFLINE_STARTING) || (offline.state == OFFLINE_RUNNING) ||
      (offline.state == OFFLINE_RESTORING) || (offline.state == OFFLINE_FAULT))
  { return HAL_BUSY; }
  if ((sample_cycles != 480U) && (sample_cycles != 144U))
  {
    acquisition_adc_last_switch_result = ACQ_SWITCH_ADC_CYCLES;
    acquisition_adc_switch_fail_count++;
    return HAL_ERROR;
  }
  if ((sample_cycles == 144U) && ((acquisition_active_gear != 0U) ||
      (acquisition_requested_gear != 0U)))
  {
    acquisition_adc_last_switch_result = ACQ_SWITCH_ADC_CONFLICT;
    acquisition_adc_switch_fail_count++;
    return HAL_ERROR;
  }
  acquisition_adc_requested_cycles = sample_cycles;
  __DMB();
  acquisition_adc_request_pending = 1U;
  return HAL_OK;
}

static void Acquisition_ADCSamplingPoll(void)
{
  if ((acquisition_udp_gear2_active != 0U) ||
      (acquisition_udp_gear2_starting != 0U)) { return; }
  uint32_t request = acquisition_adc_requested_cycles;
  if ((request == acquisition_adc_last_request) && (acquisition_adc_request_pending == 0U))
  { return; }
  acquisition_adc_last_request = request;
  acquisition_adc_request_pending = 0U;
  if ((request != 480U) && (request != 144U))
  {
    acquisition_adc_last_switch_result = ACQ_SWITCH_ADC_CYCLES;
    acquisition_adc_switch_fail_count++;
    return;
  }
  if ((request == 144U) && ((acquisition_active_gear != 0U) ||
      (acquisition_requested_gear != 0U)))
  {
    acquisition_adc_last_switch_result = ACQ_SWITCH_ADC_CONFLICT;
    acquisition_adc_switch_fail_count++;
    return;
  }
  if ((acquisition_fault_latched != 0U) || (adc_error != 0U) ||
      (dma_error != 0U) || (digital_sync.error != 0U))
  {
    acquisition_adc_last_switch_result = ACQ_SWITCH_LATCHED;
    acquisition_adc_switch_fail_count++;
    return;
  }
  if ((request == acquisition_adc_active_cycles) && (adc_running != 0U))
  { acquisition_adc_last_switch_result = ACQ_SWITCH_OK; return; }
  if (Acquisition_ApplyConfiguration(acquisition_active_gear, request) == HAL_OK)
  {
    acquisition_adc_last_switch_result = ACQ_SWITCH_OK;
    acquisition_adc_switch_ok_count++;
  }
  else
  {
    acquisition_adc_last_switch_result = acquisition_last_switch_result;
    acquisition_adc_switch_fail_count++;
  }
}

static void Acquisition_GearPoll(void)
{
  uint32_t request;
  uint32_t leaving_udp_gear2;
  HAL_StatusTypeDef status;
  if (UDP_TRIAL_OWNS(udp_trial.state)) { return; }
  Acquisition_ADCSamplingPoll();
  request = acquisition_requested_gear;
  if ((request == acquisition_last_request) && (acquisition_request_pending == 0U))
  { return; }
  acquisition_last_request = request;
  acquisition_request_pending = 0U;
  if (request >= ACQ_GEAR_COUNT)
  {
    acquisition_last_switch_result = ACQ_SWITCH_BAD_GEAR;
    acquisition_switch_fail_count++;
    return; /* Invalid requests must not interrupt healthy acquisition. */
  }
  if ((request >= 2U) && (acquisition_udp_gear2_available == 0U))
  {
    acquisition_last_switch_result = ACQ_SWITCH_UDP_REQUIRED;
    acquisition_switch_fail_count++;
    return;
  }
  if ((request >= 2U) && (acquisition_udp_gear2_active == 0U) &&
      ((acquisition_adc_active_cycles == 144U) ||
       ((acquisition_adc_request_pending != 0U) &&
        (acquisition_adc_requested_cycles == 144U))))
  {
    acquisition_last_switch_result = ACQ_SWITCH_ADC_CONFLICT;
    acquisition_switch_fail_count++;
    return;
  }
  if ((acquisition_fault_latched != 0U) || (adc_error != 0U) ||
      (dma_error != 0U) || (digital_sync.error != 0U))
  {
    acquisition_last_switch_result = ACQ_SWITCH_LATCHED;
    acquisition_switch_fail_count++;
    return;
  }
  if ((request == 1U) && (acquisition_udp_gear2_active == 0U) &&
      (acquisition_adc_active_cycles == 144U))
  {
    acquisition_last_switch_result = ACQ_SWITCH_ADC_CONFLICT;
    acquisition_switch_fail_count++;
    return;
  }
  if ((request == acquisition_active_gear) && (adc_running != 0U))
  { acquisition_last_switch_result = ACQ_SWITCH_OK; return; }
  leaving_udp_gear2 = acquisition_udp_gear2_active;
  acquisition_udp_gear2_starting = (request >= 2U) ? 1U : 0U;
  status = Acquisition_ApplyGear(request);
  acquisition_udp_gear2_starting = 0U;
  if (status == HAL_OK)
  {
    acquisition_udp_gear2_active = (request >= 2U) ? 1U : 0U;
    if ((leaving_udp_gear2 != 0U) && (request < 2U))
    {
      acquisition_adc_requested_cycles = 480U;
      acquisition_adc_last_request = 480U;
      acquisition_adc_request_pending = 0U;
    }
    acquisition_switch_ok_count++;
  }
  else
  {
    acquisition_udp_gear2_active = 0U;
    acquisition_switch_fail_count++;
  }
}

/* Main only. Refresh before gear execution and after network polling. Link
 * failure requests a safe G0/480 return before any UART transport change.
 * A sampling fault is stopped/latched, never hidden by automatic restart. */
static void Acquisition_UDPPolicyPoll(void)
{
  uint32_t available = ((daq_transport_active == 1U) &&
      (daq_transport_request == 1U) && (DAQ_NetReady() != 0U) &&
      (daq_net.identity_status == (uint32_t)DAQ_NET_ID_ES0290_LISTED) &&
      (adc_running != 0U) && (acquisition_fault_latched == 0U) &&
      (adc_error == 0U) && (adc_hal_error == 0U) && (dma_error == 0U) &&
      (digital_sync.error == 0U)) ? 1U : 0U;
  /* Link-mode evidence for the GEAR4 throughput review (FES bit14 = 100 Mb,
   * DM bit11 = full duplex). Observation only. */
  daq_eth_maccr = ETH->MACCR;
  if ((available != 0U) && (acquisition_requested_gear >= 2U) &&
      (acquisition_requested_gear < ACQ_GEAR_COUNT) &&
      (acquisition_udp_gear2_active == 0U))
  {
    /* Same entry clock constraints as the already exercised UDP trial;
     * do not repeat the relatively expensive RCC queries at 50 kHz. */
    if ((SystemCoreClock != 216000000U) ||
        (HAL_RCC_GetHCLKFreq() != 216000000U) ||
        (HAL_RCC_GetPCLK1Freq() != 54000000U) ||
        (HAL_RCC_GetPCLK2Freq() != 108000000U) ||
        ((RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) != 0U) ||
        ((ADC->CCR & ADC_CCR_ADCPRE) != ADC_CLOCK_SYNC_PCLK_DIV4) ||
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)) { available = 0U; }
  }
  acquisition_udp_gear2_available = available;
  if (acquisition_udp_gear2_active == 0U) { return; }
  if ((adc_running == 0U) || (acquisition_fault_latched != 0U) ||
      (adc_error != 0U) || (adc_hal_error != 0U) ||
      (dma_error != 0U) || (digital_sync.error != 0U))
  {
    acquisition_udp_gear2_active = 0U;
    acquisition_udp_gear2_available = 0U;
    acquisition_udp_gear2_fault_count++;
    (void)Acquisition_Fail(ACQ_SWITCH_LATCHED);
    return;
  }
  if ((available == 0U) && ((acquisition_requested_gear != 0U) ||
                           (acquisition_request_pending == 0U)))
  {
    acquisition_requested_gear = 0U;
    acquisition_request_pending = 1U;
    acquisition_udp_gear2_return_count++;
  }
}

/* Main context, between complete TX calls. RX bytes during isolation are
 * intentionally discarded, never replayed as commands after restoration. */
static HAL_StatusTypeDef Offline_UARTPause(void)
{
  HAL_StatusTypeDef status;
  if ((uart6_transmit_busy != 0U) || (huart6.hdmarx != &hdma_usart6_rx))
  { return HAL_BUSY; }
  HAL_NVIC_DisableIRQ(USART6_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
  __HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
  uart6_rx_active = 0U;
  status = HAL_UART_AbortReceive(&huart6);
  if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) != 0U)
  {
    if (HAL_DMA_Abort(&hdma_usart6_rx) != HAL_OK) { status = HAL_ERROR; }
  }
  if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) != 0U) { status = HAL_ERROR; }
  uart6_rx_recovery_pending = 0U;
  uart6_status_pending = 0U;
  DAQ_ControlStreamReset(&uart6_control_stream);
  HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
  HAL_NVIC_ClearPendingIRQ(USART6_IRQn);
  return status;
}

static HAL_StatusTypeDef Offline_UARTResume(void)
{
  HAL_StatusTypeDef status = UART6_ReceiveArm();
  if ((status == HAL_OK) && ((hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) != 0U))
  {
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART6_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    return HAL_OK;
  }
  (void)Offline_UARTPause();
  return HAL_ERROR;
}

/* Explicit internal request only, not a UART command. Watch uses the same
 * validation through offline_request_groups. No automatic power-on request. */
__root HAL_StatusTypeDef Acquisition_RequestOfflineGEAR2(uint32_t groups)
{
  if (UDP_TRIAL_OWNS(udp_trial.state)) { return HAL_BUSY; }
  uint32_t state = offline.state;
  uint32_t actual_gear = acquisition_active_gear;
  if ((groups == 0U) || (groups > OFFLINE_MAX_GROUPS))
  { offline.rejected++; return HAL_ERROR; }
  if (((state != OFFLINE_IDLE) && (state != OFFLINE_DONE)) ||
      (offline_request_groups != 0U) || (adc_running == 0U) ||
      (acquisition_active_gear > 1U) || (acquisition_adc_active_cycles != 480U) ||
      (acquisition_fault_latched != 0U) || (adc_error != 0U) ||
      (dma_error != 0U) || (digital_sync.error != 0U) ||
      (acquisition_switching != 0U) || (acquisition_request_pending != 0U) ||
      (acquisition_adc_request_pending != 0U) || (uart6_gear_pending != 0U) ||
      (acquisition_requested_gear != actual_gear) ||
      (acquisition_adc_requested_cycles != 480U) || (uart6_initialized == 0U) ||
      (uart6_rx_active == 0U) || (uart6_transmit_busy != 0U))
  { offline.rejected++; return HAL_BUSY; }
  offline_request_groups = groups;
  return HAL_OK;
}

/* Nominal configuration/readback evidence, not a physical clock measurement. */
static uint32_t Offline_ClockReady(void)
{
  uint32_t pclk = HAL_RCC_GetPCLK1Freq();
  uint32_t hclk = HAL_RCC_GetHCLKFreq();
  uint32_t timer_clock;
  if ((RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) == 0U)
  { timer_clock = (pclk == hclk) ? pclk : (2U * pclk); }
  else { timer_clock = (pclk >= hclk / 4U) ? hclk : (4U * pclk); }
  offline.timer_clock_hz = timer_clock;
  offline.adc_clock_hz = HAL_RCC_GetPCLK2Freq() / 4U;
  return ((SystemCoreClock == 216000000U) && (hclk == 216000000U) &&
      (timer_clock == 108000000U) && (HAL_RCC_GetPCLK2Freq() == 108000000U) &&
      ((ADC->CCR & ADC_CCR_ADCPRE) == ADC_CLOCK_SYNC_PCLK_DIV4) &&
      ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)) ? 1U : 0U;
}

/* Called with the trigger and acquisition IRQs gated off. */
static void Offline_CaptureEnd(void)
{
  offline.published = (uint32_t)(full_group_publish_count - offline_base_publish);
  offline.analog_published = (uint32_t)(adc_group_count - offline_base_analog);
  offline.half_callbacks = (uint32_t)(adc_half_callback_count - offline_base_half);
  offline.full_callbacks = (uint32_t)(adc_full_callback_count - offline_base_full);
  offline.dma_irqs = (uint32_t)(dma2_stream0_irq_count - offline_base_irq);
  {
    uint32_t published = offline.published;
    uint32_t consumed = offline.consumed;
    offline.unconsumed = (uint32_t)(published - consumed);
  }
  offline.elapsed_ms = (uint32_t)(offline_end_tick - offline_start_tick);
  offline.elapsed_cycles = (uint32_t)(offline_end_cycle - offline_start_cycle);
  offline.elapsed_us = offline.elapsed_cycles / (SystemCoreClock / 1000000U);
  /* Sequence volatile reads explicitly (IAR Pa082). */
  {
    uint32_t published = offline.published;
    uint32_t duration = offline.elapsed_us;
    offline.nominal_rate_groups = (duration == 0U) ? 0U :
        (published * 1000000U / duration);
  }
  { uint32_t error = adc_error; offline.adc_error |= error; }
  { uint32_t error = adc_hal_error; offline.adc_hal_error |= error; }
  { uint32_t error = dma_error; offline.dma_error |= error; }
  { uint32_t error = digital_sync.error; offline.digital_error |= error; }
  { uint32_t error = digital_sync.fault_detail; offline.digital_detail |= error; }
  offline.end_ndtr = hdma_adc3.Instance->NDTR;
  offline.end_trigger = TIM5->CNT;
  offline.end_next_index = digital_sync.next_index;
}

static void Offline_LatchFailure(uint32_t result)
{
  /* One final bounded cleanup; later polls never retry. */
  (void)Acquisition_Stop();
  acquisition_fault_latched = 1U;
  DigitalSync_Fault(&digital_sync, DIGITAL_SYNC_ERROR_CONFIG);
  acquisition_switching = 0U;
  offline.result = result;
  offline.restore_fail++;
  offline.state = OFFLINE_FAULT;
}

static void Offline_Finish(uint32_t result)
{
  HAL_StatusTypeDef status;
  uint32_t previous_state = offline.state;
  offline.state = OFFLINE_RESTORING;
  offline.stop_result = Acquisition_Stop();
  if (previous_state == OFFLINE_RUNNING) { Offline_CaptureEnd(); }
  else
  {
    { uint32_t error = adc_error; offline.adc_error |= error; }
    { uint32_t error = adc_hal_error; offline.adc_hal_error |= error; }
    { uint32_t error = dma_error; offline.dma_error |= error; }
    { uint32_t error = digital_sync.error; offline.digital_error |= error; }
    { uint32_t error = digital_sync.fault_detail; offline.digital_detail |= error; }
  }
  offline.result = result;
  if ((offline.stop_result != ACQ_SWITCH_OK) ||
      ((offline.start_switch_result >= ACQ_SWITCH_TIM2_STOP) &&
       (offline.start_switch_result <= ACQ_SWITCH_TIM5_STOP)))
  { Offline_LatchFailure(OFFLINE_STOP); return; }

  /* Candidate-only faults were saved above. Explicitly restore both ranks;
   * ApplyGear alone would inherit the previous 144-cycle setting. */
  adc_error = 0U;
  adc_hal_error = HAL_ADC_ERROR_NONE;
  dma_error = HAL_DMA_ERROR_NONE;
  acquisition_requested_gear = 0U;
  acquisition_last_request = 0U;
  acquisition_request_pending = 0U;
  acquisition_adc_requested_cycles = 480U;
  acquisition_adc_last_request = 480U;
  acquisition_adc_request_pending = 0U;
  status = Acquisition_ApplyConfiguration(0U, 480U);
  offline.restore_switch_result = acquisition_last_switch_result;
  offline.adc_smpr2_after_restore = hadc3.Instance->SMPR2;
  offline.adc_sqr3_after_restore = hadc3.Instance->SQR3;
  if ((status != HAL_OK) || (acquisition_active_gear != 0U) ||
      (acquisition_adc_active_cycles != 480U) || (adc_running == 0U) ||
      (TIM2->PSC != 107U) || (TIM2->ARR != 199U) ||
      (digital_sync.timing.sample_interval_us != 200U) ||
      (digital_sync.error != 0U) || (adc_error != 0U) || (dma_error != 0U))
  { Offline_LatchFailure(OFFLINE_RESTORE); return; }
  acquisition_fault_latched = 0U;
  offline.uart_resume_status = (uint32_t)Offline_UARTResume();
  if (offline.uart_resume_status != (uint32_t)HAL_OK)
  { Offline_LatchFailure(OFFLINE_UART_RESUME); return; }
  offline.restore_ok++;
  offline_stop_request = 0U;
  offline.state = OFFLINE_DONE;
}

static void Offline_Begin(uint32_t groups)
{
  offline.requested_groups = groups;
  offline.result = OFFLINE_OK;
  offline.snapshot_ok = 0U; offline.snapshot_busy = 0U;
  offline.duplicate_reads = 0U; offline.consumed = 0U; offline.skipped = 0U;
  offline.published = 0U; offline.analog_published = 0U;
  offline.half_callbacks = 0U; offline.full_callbacks = 0U; offline.dma_irqs = 0U;
  offline.unconsumed = 0U; offline.retention_dropped = 0U;
  offline.elapsed_ms = 0U; offline.elapsed_cycles = 0U; offline.elapsed_us = 0U;
  offline.nominal_rate_groups = 0U; offline.first_group = 0U; offline.last_group = 0U;
  offline.adc_error = 0U; offline.adc_hal_error = 0U; offline.dma_error = 0U;
  offline.digital_error = 0U; offline.digital_detail = 0U;
  offline.start_switch_result = ACQ_GEAR_INVALID;
  offline.stop_result = ACQ_GEAR_INVALID; offline.restore_switch_result = ACQ_GEAR_INVALID;
  offline.uart_resume_status = 0xFFFFFFFFUL;
  offline.data_seq++; __DMB(); offline.retained = 0U; __DMB(); offline.data_seq++;
  offline_stop_request = 0U;
  if (Offline_ClockReady() == 0U)
  {
    offline.start_fail++; offline.result = OFFLINE_CLOCK;
    offline.state = OFFLINE_DONE; return; /* Production hardware untouched. */
  }
  offline.state = OFFLINE_STARTING;
  offline.accepted++;
  offline.uart_pause_status = (uint32_t)Offline_UARTPause();
  if (offline.uart_pause_status != (uint32_t)HAL_OK)
  {
    offline.start_fail++;
    Offline_LatchFailure(OFFLINE_UART_PAUSE); return;
  }
  if (Acquisition_ApplyConfiguration(2U, 144U) != HAL_OK)
  {
    offline.start_fail++; offline.start_switch_result = acquisition_last_switch_result;
    Offline_Finish(OFFLINE_START); return;
  }
  offline.start_switch_result = ACQ_SWITCH_OK;
  offline.start_ok++;
  offline.state = OFFLINE_RUNNING;
}

/* Nonzero means this iteration belongs to the isolated experiment. A single
 * coherent read attempt per observed publication; no ISR protocol work. */
static uint32_t Acquisition_OfflinePoll(void)
{
  uint32_t request = offline_request_groups;
  uint32_t target = offline.requested_groups;
  if (request != 0U)
  {
    offline_request_groups = 0U;
    if (Acquisition_RequestOfflineGEAR2(request) == HAL_OK)
    {
      offline_request_groups = 0U;
      Offline_Begin(request);
      return 1U;
    }
  }
  if (offline.state == OFFLINE_FAULT) { return 1U; }
  if (offline.state != OFFLINE_RUNNING) { return 0U; }
  if ((adc_running == 0U) || (acquisition_fault_latched != 0U) ||
      (acquisition_active_gear != 2U) || (acquisition_adc_active_cycles != 144U) ||
      (adc_error != 0U) || (dma_error != 0U) || (digital_sync.error != 0U))
  { Offline_Finish(OFFLINE_RUNTIME); return 1U; }
  if (full_group_publish_count != offline_seen_publish)
  {
    if (Full_GetSnapshot(&offline_candidate) == HAL_OK)
    {
      uint32_t delta = (uint32_t)(offline_candidate.group_count - offline_last_group);
      offline.snapshot_ok++;
      if (offline_candidate.gear != 2U)
      { Offline_Finish(OFFLINE_RUNTIME); return 1U; }
      if (delta == 0U) { offline.duplicate_reads++; }
      else
      {
        if (offline.consumed == 0U) { offline.first_group = offline_candidate.group_count; }
        offline.skipped += delta - 1U;
        offline.last_group = offline_candidate.group_count;
        offline_last_group = offline_candidate.group_count;
        offline.consumed++;
        if (offline.retained < OFFLINE_CAPACITY)
        {
          offline.data_seq++; __DMB();
          offline_samples[offline.retained] = offline_candidate;
          __DMB(); offline.retained++; __DMB(); offline.data_seq++;
        }
        else { offline.retention_dropped++; }
      }
      /* Derive the copied publication, not an ISR counter that can advance
       * after Full_GetSnapshot has completed. */
      offline_seen_publish = offline_base_publish +
          (uint32_t)(offline_candidate.group_count - offline_base_analog);
    }
    else { offline.snapshot_busy++; }
  }
  if (offline_stop_request != 0U) { Offline_Finish(OFFLINE_CANCELLED); }
  else if ((uint32_t)(full_group_publish_count - offline_base_publish) >= target)
  { Offline_Finish(OFFLINE_OK); }
  else if ((uint32_t)(HAL_GetTick() - offline_start_tick) >= (offline.requested_groups + 250U))
  { Offline_Finish(OFFLINE_TIMEOUT); }
  return 1U;
}

/* Failure-only path: UIE is disabled before taking the first snapshot.
 * Register reads are sequential, not an atomic hardware capture. */
DIGITAL_SYNC_FAST_CODE
static void DigitalSync_CaptureFault(uint32_t source, uint32_t ndtr, uint32_t trigger)
{
  if (digital_timing.fault.valid != 0U) { return; }
  digital_timing.fault.source = source;
  digital_timing.fault.error = digital_sync.error;
  digital_timing.fault.detail = digital_sync.fault_detail;
  digital_timing.fault.sample_interval_us = digital_sync.timing.sample_interval_us;
  digital_timing.fault.irq_entry_phase_us = digital_timing.irq_entry_phase_us;
  digital_timing.fault.gpio_phase_us = digital_timing.gpio_phase_us;
  digital_timing.fault.checks_phase_us = digital_timing.checks_phase_us;
  digital_timing.fault.record_ndtr = ndtr;
  digital_timing.fault.record_trigger = trigger;
  digital_timing.fault.next_index = digital_sync.next_index;
  digital_timing.fault.last_trigger = digital_sync.last_trigger;
  digital_timing.fault.tim2_cnt = TIM2->CNT;
  digital_timing.fault.tim2_sr = TIM2->SR;
  digital_timing.fault.tim2_dier = TIM2->DIER;
  digital_timing.fault.tim2_arr = TIM2->ARR;
  digital_timing.fault.tim5_cnt = TIM5->CNT;
  digital_timing.fault.dma_ndtr = hdma_adc3.Instance->NDTR;
  digital_timing.fault.dma_cr = hdma_adc3.Instance->CR;
  __DMB();
  digital_timing.fault.valid = 1U;
}

void Acquisition_TIM2Update(uint32_t irq_entry_phase)
{
  uint32_t cycles = DWT->CYCCNT;
  uint32_t phase_before = TIM2->CNT;
  uint32_t g = GPIOG->IDR;
  uint32_t port_i = GPIOI->IDR;
  uint32_t gpio_phase = TIM2->CNT;
  uint32_t ndtr = hdma_adc3.Instance->NDTR;
  uint32_t trigger = TIM5->CNT;
  uint32_t phase_after = TIM2->CNT;
  uint32_t elapsed;
  uint8_t bits = (uint8_t)(((g >> 6U) & 3U) | (((port_i >> 3U) & 1U) << 2U));

  /* All diagnostic stores follow the time-critical GPIO/NDTR/TIM5 reads. */
  digital_timing.irq_entry_phase_us = irq_entry_phase;
  digital_timing.gpio_phase_us = gpio_phase;
  digital_timing.checks_phase_us = phase_after;
  if (irq_entry_phase > digital_timing.irq_entry_phase_max_us)
  { digital_timing.irq_entry_phase_max_us = irq_entry_phase; }
  if (gpio_phase > digital_timing.gpio_phase_max_us)
  { digital_timing.gpio_phase_max_us = gpio_phase; }
  if (phase_after > digital_timing.checks_phase_max_us)
  { digital_timing.checks_phase_max_us = phase_after; }
  if (phase_before > digital_read_phase_max_us) { digital_read_phase_max_us = phase_before; }
  if (phase_after > digital_read_phase_max_us) { digital_read_phase_max_us = phase_after; }
  if (adc_running == 0U)
  {
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_LATE, DIGITAL_SYNC_DETAIL_NOT_RUNNING);
  }
  if ((phase_before < irq_entry_phase) || (gpio_phase < phase_before) ||
      (phase_after < gpio_phase))
  {
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_LATE, DIGITAL_SYNC_DETAIL_CNT_WRAP);
  }
  if (phase_before > digital_sync.timing.max_phase_us)
  {
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_LATE, DIGITAL_SYNC_DETAIL_PHASE);
  }
  if (DigitalSync_Record(&digital_sync, trigger, ndtr, phase_after, bits) == 0U)
  {
    /* UIE is independent of Update TRGO: stop digital IRQs, keep ADC running. */
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
  }
  __DMB();
  elapsed = DWT->CYCCNT - cycles;
  if (elapsed > digital_irq_max_cycles) { digital_irq_max_cycles = elapsed; }
  /* At the existing 216 MHz this is the unchanged 4320-cycle/20 us limit.
   * Body measurement excludes wrapper and final fault-snapshot stores. */
  if (elapsed > ((SystemCoreClock / 1000000U) * digital_sync.timing.max_body_us))
  {
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_LATE, DIGITAL_SYNC_DETAIL_BODY_BUDGET);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
  }
  if (digital_sync.error != 0U) { DigitalSync_CaptureFault(1U, ndtr, trigger); }
  /* Observation only, after all original reads/checks/fault capture. */
  {
    volatile DigitalTimingGroup *timing = DigitalTiming_Group(&digital_timing,
                                                            acquisition_active_gear);
    uint32_t sync_error = digital_sync.error;
    DigitalTiming_Response(&timing->gpio_upper_us, irq_entry_phase, phase_before,
        gpio_phase, phase_after, digital_sync.timing.sample_interval_us,
        digital_sync.timing.max_phase_us, sync_error);
  }
}

DIGITAL_SYNC_FAST_CODE
void Acquisition_TIM2FastUpdate(uint32_t irq_entry_cycles, uint32_t entry_ndtr)
{
  uint32_t g = GPIOG->IDR;
  uint32_t port_i = GPIOI->IDR;
  /* Captured in TIM2_IRQHandler before the C prologue: the GEAR4 ADC pair
   * finishes 1.11 us after the trigger, so the write pointer must be sampled
   * as close to the trigger as possible instead of after the GPIO reads. */
  uint32_t ndtr = entry_ndtr;
  uint32_t trigger = TIM5->CNT;
  uint32_t phase_cycles = DWT->CYCCNT - irq_entry_cycles;
  uint32_t elapsed;
  uint8_t bits = (uint8_t)(((g >> 6U) & 3U) | (((port_i >> 3U) & 1U) << 2U));

  acquisition_fast_phase_last_cycles = phase_cycles;
  if (phase_cycles > acquisition_fast_phase_max_cycles)
  { acquisition_fast_phase_max_cycles = phase_cycles; }
  if (adc_running == 0U)
  {
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_LATE,
                                DIGITAL_SYNC_DETAIL_NOT_RUNNING);
  }
  else if (DigitalSync_RecordFast(&digital_sync, trigger, ndtr, phase_cycles, bits) == 0U)
  {
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
  }

  elapsed = DWT->CYCCNT - irq_entry_cycles;
  acquisition_fast_irq_last_cycles = elapsed;
  if (elapsed > acquisition_fast_irq_max_cycles)
  { acquisition_fast_irq_max_cycles = elapsed; }
  if ((acquisition_fast_irq_budget_cycles != 0U) &&
      (elapsed >= acquisition_fast_irq_budget_cycles))
  {
    acquisition_fast_budget_fail_count++;
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_LATE,
                                DIGITAL_SYNC_DETAIL_BODY_BUDGET);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
  }
  if (digital_sync.error != 0U) { DigitalSync_CaptureFault(1U, ndtr, trigger); }
}

/* Continuous UDP qualification owner. Disabled at reset; no protocol command
 * arms it. It shares stopped reconfiguration and all original sync guards. */
static uint32_t UDPTrial_Delta(volatile uint32_t *current, volatile uint32_t *base)
{
  uint32_t value = *current;
  uint32_t initial = *base;
  return (uint32_t)(value - initial);
}
static void UDPTrial_Latch(void)
{
  (void)Acquisition_Stop();
  acquisition_fault_latched = 1U;
  udp_trial.restore_fail++;
  udp_trial.state = UDP_TRIAL_FAULT;
  udp_trial.request = 0U; udp_trial.arm = 0U;
}
static void UDPTrial_Restore(uint32_t result)
{
  HAL_StatusTypeDef status;
  uint32_t error;
  udp_trial.state = UDP_TRIAL_RESTORING;
  error = adc_error; udp_trial.adc_error |= error;
  error = adc_hal_error; udp_trial.adc_hal_error |= error;
  error = dma_error; udp_trial.dma_error |= error;
  error = digital_sync.error; udp_trial.digital_error |= error;
  error = digital_sync.fault_detail; udp_trial.digital_detail |= error;
  udp_trial.result = result;
  udp_trial.stop_result = Acquisition_Stop();
  if (udp_trial.stop_result != ACQ_SWITCH_OK)
  { udp_trial.result = 6U; UDPTrial_Latch(); return; }
  adc_error = 0U; adc_hal_error = HAL_ADC_ERROR_NONE; dma_error = HAL_DMA_ERROR_NONE;
  acquisition_requested_gear = 0U; acquisition_last_request = 0U;
  acquisition_request_pending = 0U;
  acquisition_adc_requested_cycles = 480U; acquisition_adc_last_request = 480U;
  acquisition_adc_request_pending = 0U;
  status = Acquisition_ApplyConfiguration(0U,480U);
  udp_trial.switch_result = acquisition_last_switch_result;
  udp_trial.restored_smpr2 = hadc3.Instance->SMPR2;
  udp_trial.restored_sqr3 = hadc3.Instance->SQR3;
  if ((status != HAL_OK) || (adc_running == 0U) ||
      (acquisition_active_gear != 0U) || (acquisition_adc_active_cycles != 480U) ||
      (TIM2->PSC != 107U) || (TIM2->ARR != 199U) ||
      (digital_sync.timing.sample_interval_us != 200U) ||
      ((hadc3.Instance->SMPR2 & ADC_SMPR2_SMP0) != ADC_SAMPLETIME_480CYCLES) ||
      (((hadc3.Instance->SMPR2 & ADC_SMPR2_SMP8) >> 24U) != ADC_SAMPLETIME_480CYCLES) ||
      (digital_sync.error != 0U) || (adc_error != 0U) || (dma_error != 0U))
  { udp_trial.result = 5U; UDPTrial_Latch(); return; }
  acquisition_fault_latched = 0U;
  udp_trial.restore_ok++;
  udp_trial.request = 0U; udp_trial.arm = 0U;
  udp_trial.state = UDP_TRIAL_DONE;
  UART6_QueueStatus();
}
static void DAQ_UDPTrialPoll(void)
{
  uint32_t request = udp_trial.request;
  uint32_t now = HAL_GetTick(), elapsed, groups, frames;
  if (udp_trial.state == UDP_TRIAL_FAULT) { return; }
  if (udp_trial.state == UDP_TRIAL_RUNNING)
  {
    udp_trial.published = UDPTrial_Delta(&full_group_publish_count,&udp_trial.base_published);
    udp_trial.analog_published = UDPTrial_Delta(&adc_group_count,&udp_trial.base_analog);
    udp_trial.half_callbacks = UDPTrial_Delta(&adc_half_callback_count,&udp_trial.base_half);
    udp_trial.full_callbacks = UDPTrial_Delta(&adc_full_callback_count,&udp_trial.base_full);
    udp_trial.consumed = UDPTrial_Delta(&ch1_new_group_count,&udp_trial.base_consumed);
    udp_trial.skipped = UDPTrial_Delta(&ch1_skipped_group_count,&udp_trial.base_skipped);
    udp_trial.submitted = UDPTrial_Delta(&daq_net.data_submitted,&udp_trial.base_submitted);
    elapsed = udp_trial.start_tick; udp_trial.elapsed_ms = (uint32_t)(now-elapsed);
    elapsed = udp_trial.rate_tick; elapsed = (uint32_t)(now-elapsed);
    if (elapsed >= 1000U)
    {
      groups = udp_trial.published; frames = udp_trial.submitted;
      udp_trial.rate_groups = UDPTrial_Delta(&udp_trial.published,&udp_trial.rate_base_groups);
      udp_trial.rate_frames = UDPTrial_Delta(&udp_trial.submitted,&udp_trial.rate_base_frames);
      udp_trial.rate_elapsed_ms = elapsed;
      udp_trial.rate_base_groups = groups; udp_trial.rate_base_frames = frames;
      udp_trial.rate_tick = now;
    }
    if (request != 1U) { UDPTrial_Restore((request == 0U) ? 7U : 1U); }
    else if ((adc_running == 0U) || (acquisition_fault_latched != 0U) ||
        (adc_error != 0U) || (dma_error != 0U) || (digital_sync.error != 0U) ||
        (acquisition_active_gear != 2U) || (acquisition_adc_active_cycles != 144U))
    { UDPTrial_Restore(4U); }
    else if ((daq_transport_request != 1U) || (DAQ_NetReady() == 0U))
    { UDPTrial_Restore(8U); }
    return;
  }
  if (request == 0U) { return; }
  if ((request != 1U) || (udp_trial.arm != UDP_TRIAL_ARM_MAGIC) ||
      (daq_transport_active != 1U) || (daq_transport_request != 1U) ||
      (DAQ_NetReady() == 0U) || (acquisition_active_gear > 1U) ||
      (acquisition_adc_active_cycles != 480U) || (adc_running == 0U) ||
      (acquisition_fault_latched != 0U) || (adc_error != 0U) || (dma_error != 0U) ||
      (digital_sync.error != 0U) || (uart6_gear_pending != 0U) ||
      (acquisition_request_pending != 0U) || (acquisition_adc_request_pending != 0U) ||
      (acquisition_requested_gear > 1U) || (acquisition_adc_requested_cycles != 480U) ||
      ((offline.state != OFFLINE_IDLE) && (offline.state != OFFLINE_DONE)) ||
      (offline_request_groups != 0U) || (uart6_rx_active != 0U) ||
      (SystemCoreClock != 216000000U) || (HAL_RCC_GetHCLKFreq() != 216000000U) ||
      (HAL_RCC_GetPCLK1Freq() != 54000000U) || (HAL_RCC_GetPCLK2Freq() != 108000000U) ||
      ((RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) != 0U) ||
      ((ADC->CCR & ADC_CCR_ADCPRE) != ADC_CLOCK_SYNC_PCLK_DIV4) ||
      ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U))
  {
    udp_trial.request = 0U; udp_trial.arm = 0U;
    udp_trial.result = 2U; udp_trial.rejected++; return;
  }
  udp_trial.state = UDP_TRIAL_STARTING;
  udp_trial.base_consumed = ch1_new_group_count;
  udp_trial.base_skipped = ch1_skipped_group_count;
  udp_trial.base_submitted = daq_net.data_submitted;
  if (Acquisition_ApplyConfiguration(2U,144U) != HAL_OK)
  {
    uint32_t reason = acquisition_last_switch_result;
    udp_trial.start_fail++; udp_trial.switch_result = reason;
    if ((reason >= ACQ_SWITCH_TIM2_STOP) && (reason <= ACQ_SWITCH_TIM5_STOP))
    { udp_trial.result = 6U; UDPTrial_Latch(); }
    else { UDPTrial_Restore(3U); }
    return;
  }
  udp_trial.published = 0U; udp_trial.analog_published = 0U;
  udp_trial.half_callbacks = 0U; udp_trial.full_callbacks = 0U;
  udp_trial.consumed = 0U; udp_trial.skipped = 0U; udp_trial.submitted = 0U;
  udp_trial.rate_groups = 0U; udp_trial.rate_frames = 0U;
  udp_trial.rate_base_groups = 0U; udp_trial.rate_base_frames = 0U;
  udp_trial.elapsed_ms = 0U; udp_trial.rate_elapsed_ms = 0U;
  udp_trial.rate_tick = udp_trial.start_tick;
  udp_trial.start_ok++; udp_trial.result = 0U;
  udp_trial.state = UDP_TRIAL_RUNNING;
  UART6_QueueStatus();
}
static uint32_t DAQ_TransportGetStatus(DAQ_StatusState *state)
{
  const DigitalSyncTimingConfig *profile;
  if (acquisition_active_gear < 2U) { return UART6_GetStatus(state); }
  profile = DigitalSync_GetTimingConfig(acquisition_active_gear);
  if (((udp_trial.state != UDP_TRIAL_RUNNING) &&
       (acquisition_udp_gear2_active == 0U)) || (daq_transport_active != 1U) ||
      (profile == NULL) || (acquisition_adc_active_cycles != profile->adc_sample_cycles) ||
      (adc_running == 0U) || (acquisition_fault_latched != 0U) ||
      (adc_error != 0U) || (dma_error != 0U) || (digital_sync.error != 0U) ||
      (square_wave_running == 0U) || (square_wave_actual_hz < 100U) ||
      (square_wave_actual_hz > 10000U) ||
      ((profile->fast_isr != 0U) &&
       ((acquisition_fast_isr_active == 0U) ||
        (acquisition_fast_irq_budget_cycles != profile->max_body_cycles) ||
        (acquisition_fast_budget_fail_count != 0U)))) { return 0U; }
  state->gear = acquisition_active_gear; state->channel_mask = daq_active_channel_mask;
  state->trigger_source = daq_trigger_source;
  state->trigger_mode = daq_trigger_mode;
  state->trigger_level = daq_trigger_level;
  state->wave_frequency_hz = square_wave_actual_hz;
  return 1U;
}

/* One selected transport service in main. UART/UDP never transmit together.
 * Existing parser/dispatcher/gear completion/status source are shared. */
static uint32_t DAQ_TransportPoll(void)
{
  DAQ_ControlCommand command;
  DAQ_StatusState state;
  Full_Snapshot trigger_snapshot;
  uint32_t work = 0U;
  uint32_t requested = daq_transport_request;
  uint32_t active = daq_transport_active;
  if (requested != active)
  {
    if ((requested > 1U) || (acquisition_active_gear > 1U) ||
        (acquisition_switching != 0U) || (uart6_gear_pending != 0U) ||
        (acquisition_request_pending != 0U) || (uart6_transmit_busy != 0U))
    { daq_transport_result = 1U; daq_transport_request = daq_transport_active; return 1U; }
    if (requested == 1U)
    {
      if (DAQ_NetSetEnabled(1U) == 0U)
      { daq_transport_result = 2U; daq_transport_request = 0U; return 1U; }
      if ((huart6.hdmarx == &hdma_usart6_rx) && (Offline_UARTPause() != HAL_OK))
      {
        (void)DAQ_NetSetEnabled(0U);
        (void)Offline_UARTResume();
        daq_transport_result = 3U; daq_transport_request = 0U; return 1U;
      }
    }
    else
    {
      uint32_t stopped = DAQ_NetSetEnabled(0U);
      if (stopped == 2U)
      { daq_transport_result = 6U; return 0U; } /* WFI, then bounded retry. */
      if (stopped != 1U)
      { daq_transport_result = 4U; daq_transport_request = 1U; return 1U; }
      if (Offline_UARTResume() != HAL_OK)
      { daq_transport_result = 5U; daq_transport_request = 1U; return 1U; }
    }
    daq_transport_active = requested;
    daq_transport_result = 0U; daq_transport_change_count++;
    UART6_QueueStatus();
    return 1U;
  }
  if (daq_transport_active == 0U) { return 0U; }
  work = DAQ_NetPoll();
  Acquisition_UDPPolicyPoll();
  /* At most one command per service iteration; no command executes in ISR. */
  if (DAQ_NetTakeControl(&command) != 0U)
  { UART6_DispatchControl(&command); work = 1U; }
  work |= UART6_GearControlPoll();
  if ((uart6_gear_pending != 0U) || (DAQ_NetReady() == 0U)) { return work; }
  if (uart6_status_pending != 0U)
  {
    uart6_status_pending--;
    if ((DAQ_TransportGetStatus(&state) == 0U) ||
        (DAQ_BuildStatusFrame(&state, daq_udp_status_frame,
                             sizeof(daq_udp_status_frame)) != DAQ_STATUS_OK) ||
        (DAQ_NetSendFrame(daq_udp_status_frame, DAQ_STATUS_FRAME_SIZE) == 0U))
    { daq_udp_status_error_count++; }
    work = 1U;
  }
  if (daq_trigger_source != DAQ_TRIGGER_SOURCE_OFF)
  {
    uint32_t trigger_work = Trigger_ProcessNewSnapshot();
    if ((Trigger_TakeReady(&trigger_snapshot) != 0U) &&
        (Trigger_BuildFramePreview(ch1_uart_next_seq, &trigger_snapshot) != 0U))
    {
      if (DAQ_NetSendFrame(ch1_preview_work, DAQ_DATA_FRAME_SIZE) != 0U)
      {
        daq_udp_last_seq = ch1_uart_next_seq;
        daq_udp_last_group = ch1_frame_group;
        ch1_uart_next_seq = (uint16_t)((uint32_t)ch1_uart_next_seq + 1U);
      }
      else { daq_udp_data_error_count++; }
    }
    work |= trigger_work;
  }
  else if ((adc_snapshot_valid != 0U) &&
           ((ch1_preview_have_group == 0U) ||
            (adc_group_count != ch1_preview_last_group)))
  {
    uint16_t sequence = ch1_uart_next_seq;
    uint32_t build_start = DWT->CYCCNT;
    if (Analog_BuildFramePreview(sequence, 2U) != 0U)
    {
      {
        uint32_t build_cycles = DWT->CYCCNT - build_start;
        uint32_t net_start;
        daq_tx_build_last_cycles = build_cycles;
        if (build_cycles > daq_tx_build_max_cycles)
        { daq_tx_build_max_cycles = build_cycles; }
        net_start = DWT->CYCCNT;
        if (DAQ_NetSendFrame(ch1_preview_work, DAQ_DATA_FRAME_SIZE) != 0U)
        {
          uint32_t net_end = DWT->CYCCNT;
          uint32_t net_cycles = net_end - net_start;
          daq_tx_net_last_cycles = net_cycles;
          if (net_cycles > daq_tx_net_max_cycles)
          { daq_tx_net_max_cycles = net_cycles; }
          if (daq_tx_prev_cycle != 0U)
          {
            uint32_t period = net_end - daq_tx_prev_cycle;
            daq_tx_frame_period_cycles = period;
            if (period < daq_tx_frame_period_min_cycles)
            { daq_tx_frame_period_min_cycles = period; }
          }
          daq_tx_prev_cycle = net_end;
          if (daq_tx_fps_start_cycle == 0U) { daq_tx_fps_start_cycle = net_end; }
          daq_tx_fps_count++;
          {
            uint32_t window = net_end - daq_tx_fps_start_cycle;
            uint32_t window_ms = window / 216000UL;
            if (window_ms >= 1000U)
            {
              daq_tx_fps = (daq_tx_fps_count * 1000UL) / window_ms;
              daq_tx_fps_count = 0U;
              daq_tx_fps_start_cycle = net_end;
            }
          }
          if (daq_tx_prev_group != 0U)
          {
            uint32_t group_delta = ch1_frame_group - daq_tx_prev_group;
            daq_tx_group_delta_last = group_delta;
            if (group_delta > daq_tx_group_delta_max)
            { daq_tx_group_delta_max = group_delta; }
          }
          daq_tx_prev_group = ch1_frame_group;
          daq_udp_last_seq = sequence; daq_udp_last_group = ch1_frame_group;
          ch1_uart_next_seq = (uint16_t)((uint32_t)sequence + 1U);
        }
        else { daq_udp_data_error_count++; }
      }
    }
    work = 1U;
  }
  return work;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  adc_startup_stage = 1U;
  adc_startup_ccr = SCB->CCR;
  /* Normal reset leaves D-Cache off. The local CMSIS disable routine also
   * cleans every set/way, so do not call it on an unused/uninitialized cache.
   * Retain the disable operation only for entry with D-Cache already enabled. */
  if ((adc_startup_ccr & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_DisableDCache();
  }
  __DSB();
  __ISB();
  /* Cortex-M7 L1 instruction cache. GEAR3/4 run the TIM2 fast path from
   * flash at 7 wait states; the 972/390-cycle body budgets are unreachable
   * without it, and ART prefetch alone does not accelerate the AXIM alias at
   * 0x08000000. D-Cache stays disabled so DMA buffers remain coherent. */
  SCB_EnableICache();
  __DSB();
  __ISB();
  adc_startup_icache = ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) ? 1U : 0U;
  adc_startup_stage = 2U;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  adc_startup_stage = 3U;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  adc_startup_flash_acr = FLASH->ACR; /* LATENCY + PRFTEN + ARTEN evidence */
  adc_startup_stage = 4U;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  adc_startup_stage = 5U;
  /* ES0290 Rev 7, 2.2.1: TIM2 TRGO can miss ADC triggers when
   * TIM2CLK=4*ADCCLK. Enable DAC clock only; no DAC channel or pin is used. */
  __HAL_RCC_DAC_CLK_ENABLE();
  CH1_SamplingStart();

  /* PWM runs independently; a PWM API error must not stop ADC acquisition. */
  if (SquareWave_Init(SQUARE_WAVE_DEFAULT_HZ) == HAL_OK)
  {
    (void)SquareWave_Start();
  }

  /* UART failure must not stop the already started acquisition or PWM. */
  UART6_TestInit();
  background_gpio_tick = HAL_GetTick() - GPIO_INPUT_PERIOD_MS;
  background_rate_tick = HAL_GetTick();
  background_rate_count = ch1_uart_tx_count;
  UART6_ControlStart();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* No per-sample software trigger; TIM2 and DMA run independently. */
    uint32_t primask;
    uint32_t loop_start = DWT->CYCCNT;
    uint32_t transport_work;
    uint32_t transport_start;
    if (Acquisition_OfflinePoll() != 0U)
    {
      /* Offline owns acquisition and suppresses ALL production UART service.
       * TIM2/SysTick wake the finite collector; no polling frame construction. */
      __WFI();
      continue;
    }
    DAQ_UDPTrialPoll();
    if (udp_trial.state == UDP_TRIAL_FAULT) { __WFI(); continue; }
    primask = __get_PRIMASK();
    __disable_irq();
    if ((adc_running != 0U) &&
        ((uint32_t)(HAL_GetTick() - adc_last_group_tick) > CH1_STALL_TIMEOUT_MS))
    {
      /* Preserve hardware evidence BEFORE stopping TIM2. PRIMASK is
       * temporarily set here; record the mask value from loop entry. */
      CH1_CaptureDebug(&adc_debug_timeout, primask);
      __HAL_TIM_DISABLE(&htim2);
      adc_running = 0U;
      adc_error |= ADC_ERROR_STALLED;
      adc_error_count++;
    }
    __set_PRIMASK(primask);
    Acquisition_UDPPolicyPoll();
    Acquisition_GearPoll();
    CH1_BackgroundPoll();
    transport_start = DWT->CYCCNT;
    transport_work = DAQ_TransportPoll();
    daq_transportpoll_cycles = DWT->CYCCNT - transport_start;
    {
      uint32_t poll_cycles = daq_transportpoll_cycles;
      if (poll_cycles > daq_transportpoll_max_cycles)
      { daq_transportpoll_max_cycles = poll_cycles; }
    }
    if (daq_loop_prev_cycle != 0U)
    {
      uint32_t loop_period = loop_start - daq_loop_prev_cycle;
      daq_loop_period_cycles = loop_period;
      if (loop_period < daq_loop_period_min_cycles)
      { daq_loop_period_min_cycles = loop_period; }
    }
    daq_loop_prev_cycle = loop_start;
    if ((transport_work == 0U) &&
        ((daq_transport_active != 0U) || (UART6_ServicePoll() == 0U)))
    {
      /* SysTick (1 ms) bounds the check-to-sleep race; DMA also wakes us.
       * After a processed group, loop immediately to pick up newer data. */
      __WFI();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 107;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 107;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin : PI3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PG7 PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Configure only TIM3, after CubeMX peripheral initialization and while stopped.
 * API calls belong in main context. The duty cycle remains exactly 50% by
 * rounding the period to an even number of 1 MHz counter ticks. */
HAL_StatusTypeDef SquareWave_Init(uint32_t frequency_hz)
{
  uint32_t pclk1_hz;
  uint32_t hclk_hz;
  uint32_t timer_clock_hz;
  uint32_t prescaler_div;
  uint32_t period_ticks;
  HAL_StatusTypeDef status;

  if (square_wave_running != 0U)
  {
    return HAL_BUSY;
  }
  if ((frequency_hz < SQUARE_WAVE_MIN_HZ) ||
      (frequency_hz > SQUARE_WAVE_MAX_HZ) ||
      (htim3.Instance != TIM3) || (htim3.State == HAL_TIM_STATE_RESET))
  {
    square_wave_error |= SQUARE_WAVE_ERROR_INIT;
    return HAL_ERROR;
  }

  /* APB1 timer clock rules include TIMPRE; never assume the CPU frequency. */
  pclk1_hz = HAL_RCC_GetPCLK1Freq();
  hclk_hz = HAL_RCC_GetHCLKFreq();
  if ((RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE) == 0U)
  {
    timer_clock_hz = (hclk_hz == pclk1_hz) ? pclk1_hz : (2U * pclk1_hz);
  }
  else
  {
    timer_clock_hz = (pclk1_hz >= (hclk_hz / 4U)) ? hclk_hz : (4U * pclk1_hz);
  }
  square_wave_timer_clock_hz = timer_clock_hz;
  if ((timer_clock_hz < SQUARE_WAVE_COUNTER_HZ) ||
      ((timer_clock_hz % SQUARE_WAVE_COUNTER_HZ) != 0U))
  {
    square_wave_error |= SQUARE_WAVE_ERROR_CLOCK;
    return HAL_ERROR;
  }
  prescaler_div = timer_clock_hz / SQUARE_WAVE_COUNTER_HZ;
  if (prescaler_div > 65536U)
  {
    square_wave_error |= SQUARE_WAVE_ERROR_CLOCK;
    return HAL_ERROR;
  }
  period_ticks = 2U * ((SQUARE_WAVE_COUNTER_HZ + frequency_hz) / (2U * frequency_hz));

  square_wave_initialized = 0U;
  htim3.Init.Prescaler = prescaler_div - 1U;
  __HAL_TIM_SET_PRESCALER(&htim3, prescaler_div - 1U);
  __HAL_TIM_SET_AUTORELOAD(&htim3, period_ticks - 1U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period_ticks / 2U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  /* Load PSC/ARR/CCR preloads before the first output cycle. No TIM3 IRQ/DMA. */
  status = HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
  if (status != HAL_OK)
  {
    square_wave_error |= SQUARE_WAVE_ERROR_INIT;
    return status;
  }
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  square_wave_frequency_hz = frequency_hz;
  square_wave_actual_hz = (SQUARE_WAVE_COUNTER_HZ + (period_ticks / 2U)) / period_ticks;
  square_wave_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef SquareWave_Start(void)
{
  HAL_StatusTypeDef status;

  if (square_wave_initialized == 0U)
  {
    square_wave_error |= SQUARE_WAVE_ERROR_INIT;
    return HAL_ERROR;
  }
  if (square_wave_running != 0U)
  {
    return HAL_OK;
  }
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  if (status == HAL_OK)
  {
    square_wave_running = 1U;
  }
  else
  {
    square_wave_error |= SQUARE_WAVE_ERROR_START;
  }
  return status;
}

/* Shared by the CMD07 stop/configure/start sequence. */
__root HAL_StatusTypeDef SquareWave_Stop(void)
{
  HAL_StatusTypeDef status;

  if (square_wave_running == 0U)
  {
    return HAL_OK;
  }
  status = HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  if (status == HAL_OK)
  {
    square_wave_running = 0U;
  }
  else
  {
    square_wave_error |= SQUARE_WAVE_ERROR_STOP;
  }
  return status;
}

/* Main context only. Validate before stopping a valid output; a HAL failure
 * gets one bounded attempt to restore the previous running frequency. */
static HAL_StatusTypeDef SquareWave_ChangeFrequency(uint32_t frequency_hz)
{
  uint32_t previous_hz = square_wave_frequency_hz;
  uint32_t was_running = square_wave_running;
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef restore;
  square_wave_last_request_hz = frequency_hz;
  square_wave_restore_status = 0xFFFFFFFFUL;
  if ((frequency_hz < SQUARE_WAVE_MIN_HZ) ||
      (frequency_hz > SQUARE_WAVE_MAX_HZ))
  {
    square_wave_last_set_status = (uint32_t)HAL_ERROR;
    square_wave_set_error_count++;
    return HAL_ERROR;
  }
  if ((frequency_hz == previous_hz) && (was_running != 0U))
  {
    square_wave_last_set_status = (uint32_t)HAL_OK;
    square_wave_set_ok_count++;
    return HAL_OK;
  }
  status = SquareWave_Stop();
  if (status == HAL_OK) { status = SquareWave_Init(frequency_hz); }
  if (status == HAL_OK) { status = SquareWave_Start(); }
  square_wave_last_set_status = (uint32_t)status;
  if (status == HAL_OK)
  {
    square_wave_set_ok_count++;
  }
  else
  {
    square_wave_set_error_count++;
    if ((was_running != 0U) && (square_wave_running == 0U) &&
        (previous_hz >= SQUARE_WAVE_MIN_HZ) && (previous_hz <= SQUARE_WAVE_MAX_HZ))
    {
      restore = SquareWave_Stop();
      if (restore == HAL_OK) { restore = SquareWave_Init(previous_hz); }
      if (restore == HAL_OK) { restore = SquareWave_Start(); }
      square_wave_restore_status = (uint32_t)restore;
    }
  }
  return status; /* A successful rollback does not acknowledge the request. */
}

/* Read-only, single-pass snapshots: never read ADC DR (which clears EOC),
 * never clear status flags, and never stop a peripheral from this helper.
 * Hardware keeps running, so fields are not an atomic peripheral snapshot. */
static void CH1_CaptureDebug(volatile CH1_DebugSnapshot *snapshot,
                             uint32_t primask_before)
{
  snapshot->valid = 0U;
  snapshot->tick = HAL_GetTick();
  snapshot->tim_cr1 = TIM2->CR1;
  snapshot->tim_cr2 = TIM2->CR2;
  snapshot->tim_smcr = TIM2->SMCR;
  snapshot->tim_sr = TIM2->SR;
  snapshot->tim_cnt = TIM2->CNT;
  snapshot->tim_psc = TIM2->PSC;
  snapshot->tim_arr = TIM2->ARR;
  snapshot->adc_sr = ADC3->SR;
  snapshot->adc_cr2 = ADC3->CR2;
  snapshot->adc_common_ccr = ADC->CCR;
  snapshot->dma_cr = DMA2_Stream0->CR;
  snapshot->dma_ndtr = DMA2_Stream0->NDTR;
  snapshot->dma_par = DMA2_Stream0->PAR;
  snapshot->dma_m0ar = DMA2_Stream0->M0AR;
  snapshot->dma_fcr = DMA2_Stream0->FCR;
  snapshot->dma_lisr = DMA2->LISR;
  snapshot->rcc_apb1enr = RCC->APB1ENR;
  snapshot->rcc_apb2enr = RCC->APB2ENR;
  snapshot->rcc_ahb1enr = RCC->AHB1ENR;
  snapshot->nvic_iser1 = NVIC->ISER[1];
  snapshot->nvic_ispr1 = NVIC->ISPR[1];
  snapshot->primask_before = primask_before;
  snapshot->basepri = __get_BASEPRI();
  snapshot->device_idcode = DBGMCU->IDCODE;
  __DMB();
  snapshot->valid = 1U;
}

static void CH1_SamplingStart(void)
{
  adc_dcache_enabled = ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) ? 1U : 0U;
  __HAL_DBGMCU_FREEZE_TIM2();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55UL;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* GEAR4 throughput diagnostic: cost of the 258-byte bitwise CRC that every
   * data frame pays. Best of 100 runs, observation only. */
  {
    uint32_t n;
    uint32_t best = 0xFFFFFFFFUL;
    for (n = 0U; n < 100U; n++)
    {
      uint32_t crc_start = DWT->CYCCNT;
      (void)DAQ_CRC16_CCITT_FALSE(ch1_preview_work, DAQ_DATA_FRAME_SIZE - 2U);
      crc_start = DWT->CYCCNT - crc_start;
      if (crc_start < best) { best = crc_start; }
    }
    daq_crc_bench_cycles = best;
  }
  (void)Acquisition_ApplyGear(0U); /* Power-on default; not a switch count. */
}

/* ADC callback context (priority 1); TIM2 (priority 0) can preempt copying.
 * Preserve the legacy analog publication, then create a separately guarded
 * full snapshot. Half/group/NDTR checks bracket all three channel copies. */
static void Full_PublishGroup(uint32_t offset)
{
  uint32_t half = offset / ADC_DMA_HALF_SAMPLES;
  uint32_t i;
  uint32_t group;
  uint32_t valid;
  if (adc_running == 0U) { return; }
  CH1_PublishGroup(offset);
  group = adc_group_count;
  full_snapshot_seq++;
  __DMB();
  full_snapshot_valid = 0U;
  valid = DigitalSync_HalfMatches(&digital_sync, half, group,
                                  hdma_adc3.Instance->NDTR);
  if (valid != 0U)
  {
    for (i = 0U; i < CH1_GROUP_SAMPLES; i++)
    {
      full_published_snapshot.ch1[i] = adc_samples[i];
      full_published_snapshot.ch2[i] = adc_ch2_samples[i];
      full_published_snapshot.dig[i] = digital_sync.points[half * DIGITAL_SYNC_HALF + i];
    }
    __DMB();
    valid = DigitalSync_HalfMatches(&digital_sync, half, group,
                                    hdma_adc3.Instance->NDTR);
  }
  if (valid != 0U)
  {
    full_published_snapshot.group_count = group;
    full_published_snapshot.gear = acquisition_active_gear;
    full_group_publish_count++;
    full_snapshot_valid = 1U;
  }
  else
  {
    DigitalSync_FaultWithDetail(&digital_sync, DIGITAL_SYNC_ERROR_GROUP | DIGITAL_SYNC_ERROR_REUSE,
                                DIGITAL_SYNC_DETAIL_HALF);
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    /* Sequence volatile register reads explicitly for IAR Pa082. */
    {
      uint32_t fault_ndtr = hdma_adc3.Instance->NDTR;
      uint32_t fault_trigger = TIM5->CNT;
      DigitalSync_CaptureFault((half == 0U) ? 2U : 3U, fault_ndtr, fault_trigger);
    }
  }
  __DMB();
  full_snapshot_seq++;
}

__root HAL_StatusTypeDef Full_GetSnapshot(Full_Snapshot *snapshot)
{
  Full_Snapshot candidate;
  uint32_t before;
  uint32_t i;
  if (snapshot == NULL) { return HAL_ERROR; }
  before = full_snapshot_seq;
  if ((before & 1U) != 0U) { return HAL_BUSY; }
  __DMB();
  if ((full_snapshot_valid == 0U) || (digital_sync.error != 0U)) { return HAL_BUSY; }
  for (i = 0U; i < CH1_GROUP_SAMPLES; i++)
  {
    candidate.ch1[i] = full_published_snapshot.ch1[i];
    candidate.ch2[i] = full_published_snapshot.ch2[i];
    candidate.dig[i] = full_published_snapshot.dig[i];
  }
  candidate.group_count = full_published_snapshot.group_count;
  candidate.gear = full_published_snapshot.gear;
  __DMB();
  if ((before != full_snapshot_seq) || (digital_sync.error != 0U)) { return HAL_BUSY; }
  *snapshot = candidate;
  return HAL_OK;
}

static void Trigger_Reset(void)
{
  uint32_t i;

  trigger_history_head = 0U;
  trigger_history_count = 0U;
  trigger_previous_valid = 0U;
  trigger_fill_count = 0U;
  trigger_capture_gear = ACQ_GEAR_INVALID;
  trigger_have_group = 0U;
  trigger_last_group = 0U;
  trigger_frame_ready = 0U;
  trigger_capture_active = (daq_trigger_source != DAQ_TRIGGER_SOURCE_OFF) ? 1U : 0U;
  trigger_previous.ch1 = 0U;
  trigger_previous.ch2 = 0U;
  trigger_previous.dig = 0U;
  for (i = 0U; i < DAQ_TRIGGER_PRE_SAMPLES; i++)
  {
    trigger_history[i].ch1 = 0U;
    trigger_history[i].ch2 = 0U;
    trigger_history[i].dig = 0U;
  }
  /* Trigger mode intentionally suppresses free-run frames. Restart sequence
   * bookkeeping so returning to source 0 does not treat that suppression as
   * acquisition loss. */
  ch1_preview_have_group = 0U;
}

/* CMD05 is an 8-bit protocol value. Map it linearly to the 12-bit ADC range,
 * rounding to the nearest raw count. Digital sources ignore this threshold. */
static uint32_t Trigger_AnalogThreshold(void)
{
  return ((daq_trigger_level * 4095U) + 127U) / 255U;
}

static uint32_t Trigger_PointValue(const TriggerPoint *point)
{
  switch (daq_trigger_source)
  {
    case 1U: return point->ch1;
    case 2U: return point->ch2;
    case 3U: return (uint32_t)(point->dig & 0x01U);
    case 4U: return (uint32_t)((point->dig >> 1U) & 0x01U);
    case 5U: return (uint32_t)((point->dig >> 2U) & 0x01U);
    default: return 0U;
  }
}

static uint32_t Trigger_IsEdge(const TriggerPoint *previous,
                               const TriggerPoint *current)
{
  uint32_t before;
  uint32_t after;

  if ((daq_trigger_source == DAQ_TRIGGER_SOURCE_OFF) ||
      (daq_trigger_mode > DAQ_TRIGGER_MODE_ANY))
  {
    return 0U;
  }
  before = Trigger_PointValue(previous);
  after = Trigger_PointValue(current);
  if (daq_trigger_source <= 2U)
  {
    uint32_t threshold = Trigger_AnalogThreshold();
    if (daq_trigger_mode == DAQ_TRIGGER_MODE_RISE)
    { return (before <= threshold) && (after > threshold); }
    if (daq_trigger_mode == DAQ_TRIGGER_MODE_FALL)
    { return (before >= threshold) && (after < threshold); }
    return (((before <= threshold) && (after > threshold)) ||
            ((before >= threshold) && (after < threshold)));
  }
  if (daq_trigger_mode == DAQ_TRIGGER_MODE_RISE)
  { return (before == 0U) && (after != 0U); }
  if (daq_trigger_mode == DAQ_TRIGGER_MODE_FALL)
  { return (before != 0U) && (after == 0U); }
  return before != after;
}

static void Trigger_HistoryPush(const TriggerPoint *point)
{
  trigger_history[trigger_history_head] = *point;
  trigger_history_head = (trigger_history_head + 1U) % DAQ_TRIGGER_PRE_SAMPLES;
  if (trigger_history_count < DAQ_TRIGGER_PRE_SAMPLES)
  {
    trigger_history_count++;
  }
}

/* Pure main-context trigger core. Every point comes from one Full_Snapshot,
 * so ch1/ch2/dig always carry the same acquisition time index. */
static void Trigger_ProcessSnapshot(const Full_Snapshot *snapshot)
{
  TriggerPoint current;
  uint32_t i;
  uint32_t j;

  if ((snapshot == NULL) || (daq_trigger_source == DAQ_TRIGGER_SOURCE_OFF))
  {
    return;
  }
  if ((trigger_capture_active != 0U) && (trigger_capture_gear != snapshot->gear))
  {
    Trigger_Reset();
  }
  trigger_capture_gear = snapshot->gear;
  if (trigger_capture_active == 0U)
  {
    trigger_capture_active = 1U;
  }

  for (i = 0U; i < CH1_SNAPSHOT_SAMPLES; i++)
  {
    current.ch1 = snapshot->ch1[i];
    current.ch2 = snapshot->ch2[i];
    current.dig = snapshot->dig[i];

    /* Do not overwrite a completed window before the main transport has
     * consumed it. The next group re-arms after Trigger_TakeReady(). */
    if (trigger_frame_ready != 0U)
    {
      continue;
    }

    if (trigger_fill_count != 0U)
    {
      trigger_frame[trigger_fill_count] = current;
      trigger_fill_count++;
      if (trigger_fill_count == CH1_SNAPSHOT_SAMPLES)
      {
        for (j = 0U; j < CH1_SNAPSHOT_SAMPLES; j++)
        {
          trigger_ready_snapshot.ch1[j] = trigger_frame[j].ch1;
          trigger_ready_snapshot.ch2[j] = trigger_frame[j].ch2;
          trigger_ready_snapshot.dig[j] = trigger_frame[j].dig;
        }
        trigger_ready_snapshot.group_count = snapshot->group_count;
        trigger_ready_snapshot.gear = snapshot->gear;
        __DMB();
        trigger_frame_ready = 1U;
        trigger_frame_count++;
        trigger_rearm_count++;
        trigger_fill_count = 0U;
        trigger_capture_active = 0U;
        trigger_previous_valid = 0U;
        trigger_history_head = 0U;
        trigger_history_count = 0U;
      }
      continue;
    }

    if ((trigger_previous_valid != 0U) &&
        (trigger_history_count == DAQ_TRIGGER_PRE_SAMPLES) &&
        (Trigger_IsEdge(&trigger_previous, &current) != 0U))
    {
      uint32_t oldest = trigger_history_head;
      for (j = 0U; j < DAQ_TRIGGER_PRE_SAMPLES; j++)
      {
        trigger_frame[j] = trigger_history[(oldest + j) %
                                           DAQ_TRIGGER_PRE_SAMPLES];
      }
      trigger_frame[DAQ_TRIGGER_PRE_SAMPLES] = current;
      trigger_fill_count = DAQ_TRIGGER_PRE_SAMPLES + 1U;
      trigger_edge_count++;
      trigger_previous_valid = 0U;
      continue;
    }

    trigger_previous = current;
    trigger_previous_valid = 1U;
    Trigger_HistoryPush(&current);
  }
}

static uint32_t Trigger_ProcessNewSnapshot(void)
{
  Full_Snapshot snapshot;
  uint32_t group;
  uint32_t delta;

  if (daq_trigger_source == DAQ_TRIGGER_SOURCE_OFF)
  {
    return 0U;
  }
  if ((trigger_have_group != 0U) && (adc_group_count == trigger_last_group))
  {
    return 0U;
  }
  if (Full_GetSnapshot(&snapshot) != HAL_OK)
  {
    return 0U;
  }
  group = snapshot.group_count;
  if (trigger_have_group != 0U)
  {
    delta = group - trigger_last_group;
    if (delta > 1U)
    {
      trigger_group_gap_count += delta - 1U;
    }
  }
  trigger_have_group = 1U;
  trigger_last_group = group;
  Trigger_ProcessSnapshot(&snapshot);
  return 1U;
}

static uint32_t Trigger_TakeReady(Full_Snapshot *snapshot)
{
  if ((snapshot == NULL) || (trigger_frame_ready == 0U))
  {
    return 0U;
  }
  *snapshot = trigger_ready_snapshot;
  __DMB();
  trigger_frame_ready = 0U;
  return 1U;
}

/* Build a protocol frame from a completed triggered window. This reuses the
 * normal protocol builders and sequence ownership; it does not bypass GEAR2
 * or channel-mask validation. */
static uint32_t Trigger_BuildFramePreview(uint16_t sequence,
                                          const Full_Snapshot *snapshot)
{
  DAQ_FrameStatus build_status;
  uint32_t i;
  uint32_t channel_mask;

  if (snapshot == NULL)
  {
    return 0U;
  }
  channel_mask = daq_active_channel_mask;
  if (snapshot->gear == 2U)
  {
    build_status = DAQ_BuildUDPGEAR2CandidateFrame(snapshot->ch1,
        snapshot->ch2, snapshot->dig, channel_mask, sequence,
        ch1_preview_work, sizeof(ch1_preview_work));
  }
  else
  {
    build_status = DAQ_BuildSelectedFrameForGear(snapshot->ch1,
        snapshot->ch2, snapshot->dig, snapshot->gear, channel_mask, sequence,
        ch1_preview_work, sizeof(ch1_preview_work));
  }
  ch1_last_build_status = (uint32_t)build_status;
  if (build_status != DAQ_FRAME_OK)
  {
    ch1_frame_error_count++;
    return 0U;
  }

  ch1_frame_preview_seq++;
  __DMB();
  for (i = 0U; i < DAQ_DATA_FRAME_SIZE; i++)
  {
    ch1_frame_preview[i] = ch1_preview_work[i];
  }
  ch1_frame_group = snapshot->group_count;
  ch1_frame_channel_mask = channel_mask;
  ch1_frame_gear = snapshot->gear;
  ch1_preview_have_group = 1U;
  ch1_preview_last_group = snapshot->group_count;
  ch1_new_group_count++;
  ch1_frame_preview_crc = (uint32_t)ch1_preview_work[DAQ_DATA_CRC_OFFSET] |
      ((uint32_t)ch1_preview_work[DAQ_DATA_CRC_OFFSET + 1U] << 8U);
  ch1_frame_build_count++;
  ch1_frame_preview_valid = 1U;
  __DMB();
  ch1_frame_preview_seq++;
  return 1U;
}

/* Called in the DMA IRQ: each half contains 50 [CH2, CH1] point pairs.
 * Deinterleave the completed half while DMA fills the other. At 5 kHz
 * the half-buffer copy budget is 5/10 ms; no blocking work belongs here. */
static void CH1_PublishGroup(uint32_t offset)
{
  uint32_t i;
  uint32_t now_tick = HAL_GetTick();
  uint32_t now_cycle = DWT->CYCCNT;

  if (adc_running == 0U)
  {
    return;
  }

  adc_snapshot_seq++;             /* Odd: snapshot is being updated. */
  __DMB();
  for (i = 0U; i < CH1_GROUP_SAMPLES; i++)
  {
    uint32_t sample_offset = offset + (ADC_SCAN_CHANNELS * i);
    adc_ch2_samples[i] = adc_dma_buffer[sample_offset];
    adc_samples[i] = adc_dma_buffer[sample_offset + 1U];
  }
  adc_value = adc_samples[CH1_GROUP_SAMPLES - 1U];
  adc_ch2_value = adc_ch2_samples[CH1_GROUP_SAMPLES - 1U];
  adc_group_period_ms = now_tick - adc_last_group_tick;
  adc_group_cycles = now_cycle - adc_previous_group_cycle;
  adc_last_group_tick = now_tick;
  adc_previous_group_cycle = now_cycle;
  adc_group_count++;
  adc_group_ready = 1U;           /* Sticky notification; consumer may clear. */
  adc_snapshot_valid = 1U;
  __DMB();
  adc_snapshot_seq++;             /* Even: data AND group ID are published. */
}

/* Single writer: DMA IRQ. Call only from main context, with private output.
 * One bounded attempt: never disable IRQs or spin while the writer is active. */
__root HAL_StatusTypeDef CH1_GetSnapshot(CH1_Snapshot *snapshot)
{
  CH1_Snapshot candidate;
  uint32_t sequence_before;
  uint32_t i;

  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }
  sequence_before = adc_snapshot_seq;
  if ((sequence_before & 1U) != 0U)
  {
    return HAL_BUSY;
  }
  __DMB();
  if (adc_snapshot_valid == 0U)
  {
    return HAL_BUSY;
  }
  for (i = 0U; i < CH1_GROUP_SAMPLES; i++)
  {
    candidate.samples[i] = adc_samples[i];
  }
  candidate.group_count = adc_group_count;
  __DMB();
  if (sequence_before != adc_snapshot_seq)
  {
    return HAL_BUSY;
  }
  *snapshot = candidate;
  return HAL_OK;
}

/* Both arrays are copied under ONE sequence check. No IRQ masking/retry. */
__root HAL_StatusTypeDef Analog_GetSnapshot(Analog_Snapshot *snapshot)
{
  Analog_Snapshot candidate;
  uint32_t sequence_before;
  uint32_t i;

  if (snapshot == NULL)
  {
    return HAL_ERROR;
  }
  sequence_before = adc_snapshot_seq;
  if ((sequence_before & 1U) != 0U)
  {
    return HAL_BUSY;
  }
  __DMB();
  if (adc_snapshot_valid == 0U)
  {
    return HAL_BUSY;
  }
  for (i = 0U; i < CH1_GROUP_SAMPLES; i++)
  {
    candidate.ch1[i] = adc_samples[i];
    candidate.ch2[i] = adc_ch2_samples[i];
  }
  candidate.group_count = adc_group_count;
  __DMB();
  if (sequence_before != adc_snapshot_seq)
  {
    return HAL_BUSY;
  }
  *snapshot = candidate;
  return HAL_OK;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    uint32_t start = DWT->CYCCNT;
    uint32_t elapsed;
    digital_timing.dma_paths |= DIGITAL_TIMING_PATH_HALF;
    adc_half_callback_count++;
    Full_PublishGroup(0U);
    elapsed = DWT->CYCCNT - start;
    if (elapsed > digital_timing.dma_half_callback_max_cycles)
    { digital_timing.dma_half_callback_max_cycles = elapsed; }
    /* Callback publication span includes its small counter/path bookkeeping;
     * HAL dispatch and the IRQ wrapper are measured separately. */
    {
      volatile DigitalTimingGroup *timing = DigitalTiming_Group(&digital_timing,
                                                              acquisition_active_gear);
      uint32_t budget = (SystemCoreClock / 1000000U) * (TIM2->ARR + 1U);
      uint32_t sync_error = digital_sync.error;
      uint32_t running = adc_running;
      uint32_t flags = (sync_error != 0U) ? DIGITAL_TIMING_ERROR :
          ((running == 0U) ? DIGITAL_TIMING_EMPTY : 0U);
      DigitalTiming_RecordSpan(&timing->publish_half, start, start + elapsed,
                              budget, flags, 0U);
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    uint32_t start = DWT->CYCCNT;
    uint32_t elapsed;
    digital_timing.dma_paths |= DIGITAL_TIMING_PATH_FULL;
    adc_full_callback_count++;
    Full_PublishGroup(ADC_DMA_HALF_SAMPLES);
    elapsed = DWT->CYCCNT - start;
    if (elapsed > digital_timing.dma_full_callback_max_cycles)
    { digital_timing.dma_full_callback_max_cycles = elapsed; }
    /* Callback publication span includes its small counter/path bookkeeping;
     * HAL dispatch and the IRQ wrapper are measured separately. */
    {
      volatile DigitalTimingGroup *timing = DigitalTiming_Group(&digital_timing,
                                                              acquisition_active_gear);
      uint32_t budget = (SystemCoreClock / 1000000U) * (TIM2->ARR + 1U);
      uint32_t sync_error = digital_sync.error;
      uint32_t running = adc_running;
      uint32_t flags = (sync_error != 0U) ? DIGITAL_TIMING_ERROR :
          ((running == 0U) ? DIGITAL_TIMING_EMPTY : 0U);
      DigitalTiming_RecordSpan(&timing->publish_full, start, start + elapsed,
                              budget, flags, ADC_DMA_HALF_SAMPLES);
    }
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3)
  {
    /* Stop new triggers and retain the failure for Watch; reset to retry. */
    __HAL_TIM_DISABLE(&htim2);
    adc_running = 0U;
    adc_error |= ADC_ERROR_RUNTIME;
    adc_hal_error |= HAL_ADC_GetError(hadc);
    dma_error |= HAL_DMA_GetError(hadc->DMA_Handle);
    adc_error_count++;
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
