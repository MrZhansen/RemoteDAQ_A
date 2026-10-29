/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

volatile uint32_t dma2_stream0_irq_count = 0U;
volatile uint32_t dma2_stream0_last_lisr = 0U;

/* Read-only fault-register snapshot. BFAR/MMFAR require their VALID bits.
 * An IMPRECISERR does not identify the offending instruction or address. */
volatile struct
{
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t abfsr;
  uint32_t bfar;
  uint32_t mmfar;
  uint32_t ccr;
  uint32_t shcsr;
  uint32_t mpu_ctrl;
  uint32_t valid;
} hardfault_info;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc3;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  hardfault_info.cfsr = SCB->CFSR;
  hardfault_info.hfsr = SCB->HFSR;
  hardfault_info.abfsr = SCB->ABFSR;
  hardfault_info.bfar = SCB->BFAR;
  hardfault_info.mmfar = SCB->MMFAR;
  hardfault_info.ccr = SCB->CCR;
  hardfault_info.shcsr = SCB->SHCSR;
  hardfault_info.mpu_ctrl = MPU->CTRL;
  hardfault_info.valid = 1U;

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
  uint32_t start = DWT->CYCCNT;
  uint32_t end;
  uint32_t flags = 0U;
  uint32_t paths;
  uint32_t budget = (SystemCoreClock / 1000000U) * (TIM2->ARR + 1U);
  uint32_t lisr = DMA2->LISR;
  volatile DigitalTimingGroup *timing = DigitalTiming_Group(&digital_timing,
                                                          acquisition_active_gear);
  dma2_stream0_irq_count++;
  dma2_stream0_last_lisr = lisr; /* Before HAL clears IRQ flags. */
  digital_timing.dma_paths = 0U;

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */
  end = DWT->CYCCNT;
  paths = digital_timing.dma_paths;
  if ((lisr & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) != 0U)
  { flags |= DIGITAL_TIMING_ERROR; }
  if (paths == 0U) { flags |= DIGITAL_TIMING_NO_CALLBACK; }
  if (paths == (DIGITAL_TIMING_PATH_HALF | DIGITAL_TIMING_PATH_FULL))
  { flags |= DIGITAL_TIMING_COALESCED; }
  /* C span: first timestamp through HAL dispatch/all callbacks. Excludes
   * exception entry/return, compiler prologue/epilogue and accounting below.
   * If both callbacks ran, both path records describe THIS SAME IRQ span. */
  DigitalTiming_RecordSpan(&timing->dma_c, start, end, budget, flags, lisr);
  if ((paths & DIGITAL_TIMING_PATH_HALF) != 0U)
  { DigitalTiming_RecordSpan(&timing->dma_half_c, start, end, budget, flags, lisr); }
  if ((paths & DIGITAL_TIMING_PATH_FULL) != 0U)
  { DigitalTiming_RecordSpan(&timing->dma_full_c, start, end, budget, flags, lisr); }
  DigitalTiming_RecordSpan(&timing->dma_accounting, end, DWT->CYCCNT,
                          budget, flags, lisr);
  /* The accounting record's own update is outside its measurement. */
  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void TIM2_IRQHandler(void)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t entry_phase = TIM2->CNT;
  uint32_t elapsed;
  uint32_t end;
  uint32_t flags = 0U;
  uint32_t budget;
  volatile DigitalTimingGroup *timing;
  if (((TIM2->SR & TIM_SR_UIF) != 0U) &&
      ((TIM2->DIER & TIM_DIER_UIE) != 0U))
  {
    TIM2->SR = (uint32_t)~TIM_SR_UIF; /* rc_w0: clear UIF only. */
    Acquisition_TIM2Update(entry_phase);
  }
  else { flags |= DIGITAL_TIMING_EMPTY; }
  end = DWT->CYCCNT;
  elapsed = end - start;
  if ((flags == 0U) && (elapsed > digital_timing.tim2_handler_max_cycles))
  { digital_timing.tim2_handler_max_cycles = elapsed; }
  /* First C timestamp through dispatch/callee; excludes exception/prologue/
   * epilogue and the accounting below. Not a complete hardware ISR duration. */
  timing = DigitalTiming_Group(&digital_timing, acquisition_active_gear);
  budget = (SystemCoreClock / 1000000U) * (TIM2->ARR + 1U);
  DigitalTiming_RecordSpan(&timing->tim2_c, start, end, budget, flags, entry_phase);
  DigitalTiming_RecordSpan(&timing->tim2_accounting, end, DWT->CYCCNT,
                          budget, flags, entry_phase);
  /* The accounting record's own update is outside its measurement. */
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void USART6_IRQHandler(void)
{
  UART6_ReceiveIRQ();
}

/* USER CODE END 1 */
