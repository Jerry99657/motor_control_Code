/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
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
#include "stm32h7xx_it.h"
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
typedef struct
{
  uint32_t valid;
  uint32_t stacked_r0;
  uint32_t stacked_r1;
  uint32_t stacked_r2;
  uint32_t stacked_r3;
  uint32_t stacked_r12;
  uint32_t stacked_lr;
  uint32_t stacked_pc;
  uint32_t stacked_psr;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t dfsr;
  uint32_t afsr;
  uint32_t bfar;
  uint32_t mmfar;
  uint32_t fault_type;
} HardFaultSnapshot_t;

volatile HardFaultSnapshot_t g_hardfault_snapshot;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void HardFault_HandlerC(uint32_t *stacked_sp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HardFault_HandlerC(uint32_t *stacked_sp)
{
  __DSB();
  __ISB();

  g_hardfault_snapshot.valid = 0x48464C54UL; /* "HFLT" */
  g_hardfault_snapshot.fault_type = 1U;
  g_hardfault_snapshot.stacked_r0 = stacked_sp[0];
  g_hardfault_snapshot.stacked_r1 = stacked_sp[1];
  g_hardfault_snapshot.stacked_r2 = stacked_sp[2];
  g_hardfault_snapshot.stacked_r3 = stacked_sp[3];
  g_hardfault_snapshot.stacked_r12 = stacked_sp[4];
  g_hardfault_snapshot.stacked_lr = stacked_sp[5];
  g_hardfault_snapshot.stacked_pc = stacked_sp[6];
  g_hardfault_snapshot.stacked_psr = stacked_sp[7];
  g_hardfault_snapshot.cfsr = SCB->CFSR;
  g_hardfault_snapshot.hfsr = SCB->HFSR;
  g_hardfault_snapshot.dfsr = SCB->DFSR;
  g_hardfault_snapshot.afsr = SCB->AFSR;
  g_hardfault_snapshot.bfar = SCB->BFAR;
  g_hardfault_snapshot.mmfar = SCB->MMFAR;

  __DSB();
  __ISB();

  while (1)
  {
  }
}

void MemManage_HandlerC(uint32_t *stacked_sp)
{
  __DSB();
  __ISB();

  g_hardfault_snapshot.valid = 0x48464C54UL; /* "HFLT" */
  g_hardfault_snapshot.fault_type = 2U;
  g_hardfault_snapshot.stacked_r0 = stacked_sp[0];
  g_hardfault_snapshot.stacked_r1 = stacked_sp[1];
  g_hardfault_snapshot.stacked_r2 = stacked_sp[2];
  g_hardfault_snapshot.stacked_r3 = stacked_sp[3];
  g_hardfault_snapshot.stacked_r12 = stacked_sp[4];
  g_hardfault_snapshot.stacked_lr = stacked_sp[5];
  g_hardfault_snapshot.stacked_pc = stacked_sp[6];
  g_hardfault_snapshot.stacked_psr = stacked_sp[7];
  g_hardfault_snapshot.cfsr = SCB->CFSR;
  g_hardfault_snapshot.hfsr = SCB->HFSR;
  g_hardfault_snapshot.dfsr = SCB->DFSR;
  g_hardfault_snapshot.afsr = SCB->AFSR;
  g_hardfault_snapshot.bfar = SCB->BFAR;
  g_hardfault_snapshot.mmfar = SCB->MMFAR;

  while (1)
  {
  }
}

void BusFault_HandlerC(uint32_t *stacked_sp)
{
  __DSB();
  __ISB();

  g_hardfault_snapshot.valid = 0x48464C54UL; /* "HFLT" */
  g_hardfault_snapshot.fault_type = 3U;
  g_hardfault_snapshot.stacked_r0 = stacked_sp[0];
  g_hardfault_snapshot.stacked_r1 = stacked_sp[1];
  g_hardfault_snapshot.stacked_r2 = stacked_sp[2];
  g_hardfault_snapshot.stacked_r3 = stacked_sp[3];
  g_hardfault_snapshot.stacked_r12 = stacked_sp[4];
  g_hardfault_snapshot.stacked_lr = stacked_sp[5];
  g_hardfault_snapshot.stacked_pc = stacked_sp[6];
  g_hardfault_snapshot.stacked_psr = stacked_sp[7];
  g_hardfault_snapshot.cfsr = SCB->CFSR;
  g_hardfault_snapshot.hfsr = SCB->HFSR;
  g_hardfault_snapshot.dfsr = SCB->DFSR;
  g_hardfault_snapshot.afsr = SCB->AFSR;
  g_hardfault_snapshot.bfar = SCB->BFAR;
  g_hardfault_snapshot.mmfar = SCB->MMFAR;

  while (1)
  {
  }
}

void UsageFault_HandlerC(uint32_t *stacked_sp)
{
  __DSB();
  __ISB();

  g_hardfault_snapshot.valid = 0x48464C54UL; /* "HFLT" */
  g_hardfault_snapshot.fault_type = 4U;
  g_hardfault_snapshot.stacked_r0 = stacked_sp[0];
  g_hardfault_snapshot.stacked_r1 = stacked_sp[1];
  g_hardfault_snapshot.stacked_r2 = stacked_sp[2];
  g_hardfault_snapshot.stacked_r3 = stacked_sp[3];
  g_hardfault_snapshot.stacked_r12 = stacked_sp[4];
  g_hardfault_snapshot.stacked_lr = stacked_sp[5];
  g_hardfault_snapshot.stacked_pc = stacked_sp[6];
  g_hardfault_snapshot.stacked_psr = stacked_sp[7];
  g_hardfault_snapshot.cfsr = SCB->CFSR;
  g_hardfault_snapshot.hfsr = SCB->HFSR;
  g_hardfault_snapshot.dfsr = SCB->DFSR;
  g_hardfault_snapshot.afsr = SCB->AFSR;
  g_hardfault_snapshot.bfar = SCB->BFAR;
  g_hardfault_snapshot.mmfar = SCB->MMFAR;

  while (1)
  {
  }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DAC_HandleTypeDef hdac1;
extern DMA2D_HandleTypeDef hdma2d;
extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
extern JPEG_HandleTypeDef hjpeg;
extern SD_HandleTypeDef hsd1;
extern DMA_HandleTypeDef hdma_spi6_tx;
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim13;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
  __asm volatile
  (
    "tst lr, #4       \n"
    "ite eq           \n"
    "mrseq r0, msp    \n"
    "mrsne r0, psp    \n"
    "b HardFault_HandlerC \n"
  );
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

  __asm volatile
  (
    "tst lr, #4       \n"
    "ite eq           \n"
    "mrseq r0, msp    \n"
    "mrsne r0, psp    \n"
    "b MemManage_HandlerC \n"
  );

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

  __asm volatile
  (
    "tst lr, #4       \n"
    "ite eq           \n"
    "mrseq r0, msp    \n"
    "mrsne r0, psp    \n"
    "b BusFault_HandlerC \n"
  );

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

  __asm volatile
  (
    "tst lr, #4       \n"
    "ite eq           \n"
    "mrseq r0, msp    \n"
    "mrsne r0, psp    \n"
    "b UsageFault_HandlerC \n"
  );

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
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles SDMMC1 global interrupt.
  */
void SDMMC1_IRQHandler(void)
{
  /* USER CODE BEGIN SDMMC1_IRQn 0 */

  /* USER CODE END SDMMC1_IRQn 0 */
  HAL_SD_IRQHandler(&hsd1);
  /* USER CODE BEGIN SDMMC1_IRQn 1 */

  /* USER CODE END SDMMC1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac1);
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles SPI6 global interrupt.
  */
void SPI6_IRQHandler(void)
{
  /* USER CODE BEGIN SPI6_IRQn 0 */

  /* USER CODE END SPI6_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi6);
  /* USER CODE BEGIN SPI6_IRQn 1 */

  /* USER CODE END SPI6_IRQn 1 */
}

/**
  * @brief This function handles DMA2D global interrupt.
  */
void DMA2D_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2D_IRQn 0 */

  /* USER CODE END DMA2D_IRQn 0 */
  HAL_DMA2D_IRQHandler(&hdma2d);
  /* USER CODE BEGIN DMA2D_IRQn 1 */

  /* USER CODE END DMA2D_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles JPEG global interrupt.
  */
void JPEG_IRQHandler(void)
{
  /* USER CODE BEGIN JPEG_IRQn 0 */

  /* USER CODE END JPEG_IRQn 0 */
  HAL_JPEG_IRQHandler(&hjpeg);
  /* USER CODE BEGIN JPEG_IRQn 1 */

  /* USER CODE END JPEG_IRQn 1 */
}

/**
  * @brief This function handles MDMA global interrupt.
  */
void MDMA_IRQHandler(void)
{
  /* USER CODE BEGIN MDMA_IRQn 0 */

  /* USER CODE END MDMA_IRQn 0 */
  HAL_MDMA_IRQHandler(&hmdma_jpeg_infifo_th);
  HAL_MDMA_IRQHandler(&hmdma_jpeg_outfifo_th);
  /* USER CODE BEGIN MDMA_IRQn 1 */

  /* USER CODE END MDMA_IRQn 1 */
}

/**
  * @brief This function handles BDMA channel0 global interrupt.
  */
void BDMA_Channel0_IRQHandler(void)
{
  /* USER CODE BEGIN BDMA_Channel0_IRQn 0 */

  /* USER CODE END BDMA_Channel0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi6_tx);
  /* USER CODE BEGIN BDMA_Channel0_IRQn 1 */

  /* USER CODE END BDMA_Channel0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
