/**
 ******************************************************************************
 * @file    stm32f1xx_hal_conf.h
 * @brief   HAL configuration. CubeMX generates this automatically based on
 *          which peripherals you enable in the .ioc -- this is what it
 *          should look like once ADC1, TIM1-4, USART1 and DMA are on.
 ******************************************************************************
 */
#ifndef STM32F1xx_HAL_CONF_H
#define STM32F1xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Module Selection ---------------------------------------------------*/
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ---- Oscillator values --------------------------------------------------
 * Blue Pill board: 8 MHz HSE crystal, PLL x9 -> 72 MHz SYSCLK. */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    (8000000U)
#endif
#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    (100U)
#endif
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    (8000000U)
#endif
#if !defined  (LSI_VALUE)
  #define LSI_VALUE  (40000U)
#endif
#if !defined  (LSE_VALUE)
  #define LSE_VALUE  (32768U)
#endif
#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    (5000U)
#endif

#define  VDD_VALUE                    (3300U)
#define  TICK_INT_PRIORITY            (0x0FU)
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE               1U

/* ---- Ethernet/USB related (unused on F103, kept 0) ---------------------- */
#define USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define USE_HAL_UART_REGISTER_CALLBACKS       0U
#define USE_HAL_TIM_REGISTER_CALLBACKS        0U

#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_exti.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_pwr.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_def.h"

#ifdef  USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t *file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F1xx_HAL_CONF_H */
