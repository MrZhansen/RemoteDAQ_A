/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "digital_sync.h"
#include "acquisition_offline.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define CH1_SNAPSHOT_SAMPLES 50U

typedef struct
{
  uint16_t samples[CH1_SNAPSHOT_SAMPLES];
  uint32_t group_count;
} CH1_Snapshot;

/* One publication generation covers both logical analog channels. */
typedef struct
{
  uint16_t ch1[CH1_SNAPSHOT_SAMPLES];
  uint16_t ch2[CH1_SNAPSHOT_SAMPLES];
  uint32_t group_count;
} Analog_Snapshot;

typedef struct
{
  uint16_t ch1[CH1_SNAPSHOT_SAMPLES];
  uint16_t ch2[CH1_SNAPSHOT_SAMPLES];
  uint8_t dig[CH1_SNAPSHOT_SAMPLES];
  uint32_t group_count;
  uint32_t gear; /* Actual gear of this complete publication, not the request. */
} Full_Snapshot;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define SQUARE_WAVE_MIN_HZ      100U
#define SQUARE_WAVE_MAX_HZ      10000U
#define SQUARE_WAVE_DEFAULT_HZ  1000U

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* Main-context, single-attempt copy into private caller-owned storage.
 * HAL_OK: complete snapshot; HAL_BUSY: not yet published/concurrent update;
 * HAL_ERROR: NULL output. Output is unchanged unless HAL_OK is returned.
 * Does not consume group_ready or track duplicates. Group number may wrap. */
HAL_StatusTypeDef CH1_GetSnapshot(CH1_Snapshot *snapshot);

/* Same single-attempt/status/output-preservation contract as CH1_GetSnapshot.
 * Both channel arrays and group_count belong to one validated publication. */
HAL_StatusTypeDef Analog_GetSnapshot(Analog_Snapshot *snapshot);
HAL_StatusTypeDef Full_GetSnapshot(Full_Snapshot *snapshot);
extern volatile DigitalSyncTimingDiagnostics digital_timing;
extern volatile uint32_t acquisition_active_gear;
void Acquisition_TIM2Update(uint32_t irq_entry_phase);
/* Main-context request only; execution occurs in the acquisition service.
 * GEAR 0/1 on either transport; GEAR2 only under the live UDP policy.
 * This is not a UART command receiver. */
HAL_StatusTypeDef Acquisition_RequestGear(uint32_t gear);
/* Main-context request: 480 for normal mode; 144 only while GEAR0 is active
 * and no GEAR1 request is pending. Hardware changes run in the service. */
HAL_StatusTypeDef Acquisition_RequestADCSampling(uint32_t sample_cycles);

/* Explicit internal, finite experiment. Not available through CMD01.
 * Request 1..1000 groups only from a healthy GEAR0/1 + 480-cycle baseline.
 * Watch: set offline_request_groups to request; offline_stop_request cancels.
 * DONE means production is usable (restored or an untouched clock rejection).
 * Check result and restore_ok; DONE alone does not qualify the experiment.
 * FAULT blocks production service until reset; there is no automatic retry. */
HAL_StatusTypeDef Acquisition_RequestOfflineGEAR2(uint32_t groups);
extern volatile uint32_t offline_request_groups, offline_stop_request;
extern volatile AcquisitionOfflineDiagnostics offline;
/* Append-only first 32 coherent groups; data_seq brackets retained updates. */
extern volatile Full_Snapshot offline_samples[OFFLINE_CAPACITY];

/* Call Init after MX_TIM3_Init, while stopped. Stop -> Init(hz) -> Start
 * changes frequency without adding a command parser or a timer interrupt. */
HAL_StatusTypeDef SquareWave_Init(uint32_t frequency_hz);
HAL_StatusTypeDef SquareWave_Start(void);
HAL_StatusTypeDef SquareWave_Stop(void);

/* USART6 IRQ bridge; parsing and transmission remain in main context. */
void UART6_ReceiveIRQ(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
