/**
 ******************************************************************************
 * @file    stm32f1xx_hal_msp.c
 * @brief   MSP (MCU Support Package) callbacks: clock enables, GPIO
 *          alternate-function muxing, and NVIC priorities for every
 *          peripheral used by this project. CubeMX generates this file
 *          automatically from your .ioc configuration -- shown here so you
 *          can verify your own output matches.
 ******************************************************************************
 */
#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

/* ---- ADC1 : PA0-PA3 analog inputs --------------------------------------*/
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    }
}

/* ---- TIM1 (20 kHz motor PWM, PA8) / TIM2 (1 MHz free-run) /
 *      TIM3 (100 ms scheduler tick) / TIM4 (buzzer PWM, PB6) -------------*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
    else if (htim_base->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    else if (htim_base->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
    else if (htim_base->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}

/* Called by HAL_TIM_MspPostInit() for the channel-to-GPIO (AF) mapping of
 * TIM1_CH1 (PA8, motor PWM) and TIM4_CH1 (PB6, buzzer PWM). CubeMX emits
 * this as a separate weak-override function distinct from the base MSP
 * init above. */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin   = GPIO_PIN_8;               /* TIM1_CH1 = PA8 */
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM4)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin   = GPIO_PIN_6;                /* TIM4_CH1 = PB6 */
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ---- USART1 : PA9 TX (blocking) / PA10 RX (IT) --------------------------
 * TX deliberately does NOT use DMA here -- see the comment on SendFrame()
 * in uart_shell.c for why (PICSimLab's DMA support is unreliable). */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 = USART1_TX (AF push-pull) */
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* PA10 = USART1_RX (floating input) */
        GPIO_InitStruct.Pin  = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
}
