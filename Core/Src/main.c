/**
 ******************************************************************************
 * @file    main.c
 * @brief   EV ADAS main scheduler.
 *
 * The MX_xxx_Init() bodies below are exactly what STM32CubeMX generates once
 * you configure the peripherals as described in README.md section 2 -- they
 * are included here so you can verify your own generated project matches,
 * not so you type them by hand. Everything inside a
 * "USER CODE BEGIN n / USER CODE END n" pair is the part CubeMX will never
 * overwrite, and is exactly what you should paste into your own generated
 * main.c.
 *
 * Scheduler structure (cooperative, SysTick-driven -- see README.md section 4
 * for why TIM1 is dedicated to the 20 kHz motor PWM instead of also trying
 * to serve as the 10 ms base tick):
 *   - every  10 ms : ADC scaling, EV_Update(), Fault_Update(), motor PWM +
 *                     LED/buzzer output, state-change telemetry check
 *   - every 100 ms : ADAS_Update() (this is what actually polls the three
 *                     HC-SR04 sensors and may block up to ~90 ms), plus the
 *                     $EV/$AD telemetry frames
 *   - every loop   : UART_Shell_Process() (drains the RX ring buffer)
 ******************************************************************************
 */
#include "main.h"

/* USER CODE BEGIN Includes */
#include "ultrasonic.h"
#include "ev_control.h"
#include "adas.h"
#include "fault.h"
#include "uart_shell.h"
#include <math.h>
/* USER CODE END Includes */

ADC_HandleTypeDef  hadc1;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim4;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile uint8_t tim3_100ms_flag = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Implemented in stm32f1xx_hal_msp.c; CubeMX declares this the same way
 * whenever a timer channel needs GPIO AF muxing after the base Init. */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static uint8_t ScalePct(uint32_t raw12bit);
static float   ScaleRange(uint32_t raw12bit, float out_min, float out_max);
static uint16_t ReadAdcChannel(uint32_t channel);
static void    Buzzer_SetTone(uint16_t freq_hz);
static void    UpdateOutputs(const EvStatus_t *ev, const AdasStatus_t *adas);
/* USER CODE END PFP */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    Ultrasonic_Init();
    EV_Init();
    ADAS_Init();
    Fault_Init();
    UART_Shell_Init();

    HAL_TIM_Base_Start(&htim2);                    /* 1 MHz free-run for HC-SR04 timing */
    HAL_TIM_Base_Start_IT(&htim3);                  /* 100 ms scheduler tick             */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       /* motor PWM, starts at 0% duty      */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);       /* buzzer PWM, starts silent         */
    /* No HAL_ADC_Start_DMA() here on purpose -- see ReadAdcChannel() below:
     * PICSimLab's ADC model only implements single-conversion mode, so each
     * channel is started/read/stopped individually instead of running a
     * free-running scan in the background. */

    uint32_t last_ev_tick = HAL_GetTick();
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN WHILE */

        /* ---- 10 ms EV control tick (cooperative, SysTick-based) -------- */
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_ev_tick) >= 10U)
        {
            float dt_s = (float)(now - last_ev_tick) / 1000.0f;
            last_ev_tick = now;

            uint8_t accel_pct = ScalePct(ReadAdcChannel(ADC_CHANNEL_0));
            uint8_t brake_pct = ScalePct(ReadAdcChannel(ADC_CHANNEL_1));
            float   soc_pot_pct = ScaleRange(ReadAdcChannel(ADC_CHANNEL_2), 0.0f, 100.0f);
            float   motor_temp  = ScaleRange(ReadAdcChannel(ADC_CHANNEL_3), 25.0f, 120.0f);

            /* SOC potentiometer is a live manual override: touching it
             * takes over from the computed energy-integration model,
             * exactly like the "soc set" shell command. Small deadband
             * avoids injecting on ADC noise alone. */
            static float last_soc_pot_pct = -100.0f;
            if (fabsf(soc_pot_pct - last_soc_pot_pct) > 1.0f)
            {
                EV_InjectSOC(soc_pot_pct);
                last_soc_pot_pct = soc_pot_pct;
            }

            float temp_override;
            if (UART_Shell_GetTempOverride(&temp_override)) motor_temp = temp_override;

            EV_Update(accel_pct, brake_pct, motor_temp, dt_s);

            const EvStatus_t *ev = EV_GetStatus();
            uint8_t motor_ot = (ev->motor_temp_c > 90.0f) ? 1U : 0U;
            uint8_t soc_crit = (ev->soc_pct < 2.0f) ? 1U : 0U;

            const AdasStatus_t *adas = ADAS_GetStatus();
            uint8_t collision_crit = (adas->collision_lvl == COLLISION_CRITICAL) ? 1U : 0U;
            uint8_t sensor_fault   = ADAS_SensorFault();

            Fault_Update(motor_ot, soc_crit, collision_crit, sensor_fault,
                         0U /* comm timeout: test-injected only, see uart_shell.c */,
                         accel_pct, brake_pct);

            uint16_t duty = 0U;
            if (Fault_MotorPWMAllowed())
            {
                float torque_frac = fabsf(ev->torque_nm) / EV_MAX_DRIVE_TORQUE_NM;
                if (torque_frac > 1.0f) torque_frac = 1.0f;
                duty = (uint16_t)(torque_frac * (float)__HAL_TIM_GET_AUTORELOAD(&htim1));
            }
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

            UpdateOutputs(ev, adas);
            UART_Shell_CheckStateChange();
        }

        /* ---- 100 ms ADAS/sensor + telemetry tick (flagged by TIM3 ISR) - */
        if (tim3_100ms_flag)
        {
            tim3_100ms_flag = 0U;
            ADAS_Update(EV_GetStatus()->speed_kmh);
            UART_Shell_SendEvFrame();
            UART_Shell_SendAdasFrame();
        }

        UART_Shell_Process();
        /* USER CODE END WHILE */
    }
}

/* USER CODE BEGIN 4 */

/**
 * PICSimLab's ADC peripheral model only implements single-conversion mode --
 * configuring Continuous/Scan+DMA (the "efficient" real-hardware approach)
 * makes the simulator abort with "hardware error: Mode Single conversion is
 * only implemented". This reconfigures the channel and runs one blocking
 * start/poll/stop cycle per call instead. Four of these per 10 ms tick cost
 * a few microseconds each on real silicon; even with simulator overhead this
 * stays well inside the 10 ms budget.
 */
static uint16_t ReadAdcChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { return 0U; }

    uint16_t value = 0U;
    if (HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 50U) == HAL_OK)
        {
            value = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    return value;
}

/** 12-bit ADC raw -> 0..100 %, clamped, rounded. */
static uint8_t ScalePct(uint32_t raw12bit)
{
    float pct = ((float)raw12bit / 4095.0f) * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(pct + 0.5f);
}

/** 12-bit ADC raw -> [out_min, out_max], linear. */
static float ScaleRange(uint32_t raw12bit, float out_min, float out_max)
{
    float frac = (float)raw12bit / 4095.0f;
    return out_min + frac * (out_max - out_min);
}

/** Recomputes TIM4's period for a new buzzer tone; TIM4 runs at a 1 MHz
 *  base (Prescaler = 71), so ARR = 1e6 / freq_hz - 1. */
static void Buzzer_SetTone(uint16_t freq_hz)
{
    if (freq_hz == 0U) freq_hz = 1U;
    uint32_t arr = (1000000UL / freq_hz);
    if (arr > 0U) arr -= 1U;
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
}

/** Drives the four status LEDs and the buzzer from the latest ADAS/fault
 *  status. Kept in main.c on purpose: adas.c/fault.c stay pure logic
 *  modules with no direct hardware access, which makes them easy to test
 *  in isolation (e.g. on a host PC) and easy to reason about. */
static void UpdateOutputs(const EvStatus_t *ev, const AdasStatus_t *adas)
{
    (void)ev;

    /* Collision LED (PB8): rapid flash on CRITICAL, steady on WARNING. */
    GPIO_PinState collision_led = GPIO_PIN_RESET;
    if (adas->collision_lvl == COLLISION_CRITICAL)
        collision_led = ((HAL_GetTick() / 150U) % 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    else if (adas->collision_lvl == COLLISION_WARNING)
        collision_led = GPIO_PIN_SET;
    HAL_GPIO_WritePin(LED_COLLISION_GPIO_Port, LED_COLLISION_Pin, collision_led);

    HAL_GPIO_WritePin(LED_BLINDSPOT_L_GPIO_Port, LED_BLINDSPOT_L_Pin,
                       adas->blindspot_l ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_BLINDSPOT_R_GPIO_Port, LED_BLINDSPOT_R_Pin,
                       adas->blindspot_r ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin,
                       (Fault_GetState() == STATE_FAULT) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Buzzer (TIM4_CH1 / PB6): tone + on/off pattern mapped to alarm level. */
    static AlarmLevel_t   prev_alarm         = ALARM_NONE;
    static uint32_t        advisory_beep_until = 0U;

    if (adas->alarm_lvl == ALARM_ADVISORY && prev_alarm != ALARM_ADVISORY)
        advisory_beep_until = HAL_GetTick() + 200U;   /* one-shot beep on new event */
    prev_alarm = adas->alarm_lvl;

    uint8_t buzzer_on = 0U;
    switch (adas->alarm_lvl)
    {
        case ALARM_CRITICAL:
            Buzzer_SetTone(2500U);
            buzzer_on = ((HAL_GetTick() / 150U) % 2U) ? 1U : 0U;
            break;
        case ALARM_WARNING:
            Buzzer_SetTone(1200U);
            buzzer_on = 1U;
            break;
        case ALARM_ADVISORY:
            Buzzer_SetTone(1800U);
            buzzer_on = (HAL_GetTick() < advisory_beep_until) ? 1U : 0U;
            break;
        default:
            buzzer_on = 0U;
            break;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, buzzer_on ? (__HAL_TIM_GET_AUTORELOAD(&htim4) / 2U) : 0U);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        tim3_100ms_flag = 1U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        UART_Shell_RxCpltCallback();
    }
}
/* USER CODE END 4 */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* Blue Pill: 8 MHz HSE crystal x PLL9 = 72 MHz SYSCLK. */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* 36 MHz -> x2 for TIM2-4 = 72 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 72 MHz -> TIM1 = 72 MHz          */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;   /* 72/6 = 12 MHz, within 14 MHz max */
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, TRIG_FRONT_Pin | TRIG_LEFT_Pin | TRIG_RIGHT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_COLLISION_Pin | LED_BLINDSPOT_L_Pin | LED_BLINDSPOT_R_Pin | LED_FAULT_Pin, GPIO_PIN_RESET);

    /* TRIG outputs: PB0, PB2, PB4 */
    GPIO_InitStruct.Pin   = TRIG_FRONT_Pin | TRIG_LEFT_Pin | TRIG_RIGHT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ECHO inputs: PB1, PB3, PB5 (pulled down so a disconnected/idle line
     * reads a defined LOW rather than floating). */
    GPIO_InitStruct.Pin  = ECHO_FRONT_Pin | ECHO_LEFT_Pin | ECHO_RIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LED outputs: PB8-PB11 */
    GPIO_InitStruct.Pin   = LED_COLLISION_Pin | LED_BLINDSPOT_L_Pin | LED_BLINDSPOT_R_Pin | LED_FAULT_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    /* Only Channel 0 / Rank 1 is pre-configured here; ReadAdcChannel() in
     * USER CODE 4 reconfigures Channel/Rank immediately before every single
     * conversion, since that's the only mode PICSimLab's ADC model runs
     * (see the long comment on ReadAdcChannel() for why). */
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* 20 kHz PWM, Channel 1 -> PA8 (motor drive output) */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 3599;   /* 72 MHz / 3600 = 20 kHz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode      = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime          = 0;
    sBreakDeadTimeConfig.BreakState        = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim1);
}

/* Free-running 1 MHz counter used as the HC-SR04 microsecond timebase. */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 71;    /* 72 MHz / 72 = 1 MHz -> 1 us/tick */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/* 100 ms base tick (update interrupt) driving the ADAS/sensor scheduler. */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 7199;   /* 72 MHz / 7200 = 10 kHz */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;    /* 10 kHz / 1000 = 10 Hz -> 100 ms */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

/* Buzzer PWM, Channel 1 -> PB6. Base 1 MHz timebase; tone changed at
 * runtime by rewriting ARR (see Buzzer_SetTone() in USER CODE 4). */
static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 71;     /* 72 MHz / 72 = 1 MHz base */
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 499;    /* default ~2 kHz, overwritten by Buzzer_SetTone() */
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse       = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    HAL_TIM_MspPostInit(&htim4);
}

/* 115200-8-N-1, TX blocking, RX via byte interrupt (see uart_shell.c). */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
