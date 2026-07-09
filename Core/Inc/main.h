/**
 ******************************************************************************
 * @file    main.h
 * @brief   This file matches what STM32CubeMX generates once you set the
 *          GPIO "User Labels" exactly as listed in README.md section 2.
 *          If you let CubeMX generate its own main.h, just paste the
 *          "Private defines" and "extern" block below into the matching
 *          USER CODE sections instead of replacing the whole file.
 ******************************************************************************
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* Exported handles (defined in main.c; CubeMX makes these global by
 * default, this just lets every module see them via main.h). */
extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern UART_HandleTypeDef huart1;

void Error_Handler(void);

/* ---- GPIO User Label defines -------------------------------------------
 * Set these exact "User Label" strings on the matching pins in the CubeMX
 * Pinout view (click the pin -> type the label) and it will generate
 * these macros for you automatically.
 * ------------------------------------------------------------------------*/
#define TRIG_FRONT_Pin        GPIO_PIN_0
#define TRIG_FRONT_GPIO_Port  GPIOB
#define ECHO_FRONT_Pin        GPIO_PIN_1
#define ECHO_FRONT_GPIO_Port  GPIOB
#define TRIG_LEFT_Pin         GPIO_PIN_2
#define TRIG_LEFT_GPIO_Port   GPIOB
#define ECHO_LEFT_Pin         GPIO_PIN_3
#define ECHO_LEFT_GPIO_Port   GPIOB
#define TRIG_RIGHT_Pin        GPIO_PIN_4
#define TRIG_RIGHT_GPIO_Port  GPIOB
#define ECHO_RIGHT_Pin        GPIO_PIN_5
#define ECHO_RIGHT_GPIO_Port  GPIOB

#define LED_COLLISION_Pin       GPIO_PIN_8
#define LED_COLLISION_GPIO_Port GPIOB
#define LED_BLINDSPOT_L_Pin       GPIO_PIN_9
#define LED_BLINDSPOT_L_GPIO_Port GPIOB
#define LED_BLINDSPOT_R_Pin       GPIO_PIN_10
#define LED_BLINDSPOT_R_GPIO_Port GPIOB
#define LED_FAULT_Pin           GPIO_PIN_11
#define LED_FAULT_GPIO_Port     GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
