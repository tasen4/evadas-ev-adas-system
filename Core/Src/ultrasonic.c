/**
 ******************************************************************************
 * @file    ultrasonic.c
 * @brief   HC-SR04 driver implementation. Blocking trigger/echo polling built
 *          on TIM2 as a free-running 1 MHz counter (see ultrasonic.h).
 ******************************************************************************
 */
#include "ultrasonic.h"

#define TRIG_PULSE_US        (10U)
#define ECHO_TIMEOUT_US      (30000U)   /* 30 ms — matches NFR-02 sensor budget */
#define INTER_SENSOR_GAP_US  (50U)
#define SPEED_OF_SOUND_CM_US (0.0343f)

static Sensor_t sensors[SENSOR_COUNT];

/* --------------------------------------------------------------------------
 * Microsecond timebase helpers, built on TIM2's free-running 16-bit counter.
 * Casting every difference back down to uint16_t makes the subtraction
 * correct modulo 65536 even across a counter wraparound, without needing to
 * special-case the overflow.
 * ------------------------------------------------------------------------*/
static inline uint16_t Now_us(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static void DelayUs(uint16_t us)
{
    uint16_t start = Now_us();
    while ((uint16_t)(Now_us() - start) < us)
    {
        /* busy-wait: fine here, HC-SR04 timing is inherently a polling task */
    }
}

void Ultrasonic_Init(void)
{
    sensors[SENSOR_FRONT].trig_port = TRIG_FRONT_GPIO_Port;
    sensors[SENSOR_FRONT].trig_pin  = TRIG_FRONT_Pin;
    sensors[SENSOR_FRONT].echo_port = ECHO_FRONT_GPIO_Port;
    sensors[SENSOR_FRONT].echo_pin  = ECHO_FRONT_Pin;

    sensors[SENSOR_LEFT].trig_port = TRIG_LEFT_GPIO_Port;
    sensors[SENSOR_LEFT].trig_pin  = TRIG_LEFT_Pin;
    sensors[SENSOR_LEFT].echo_port = ECHO_LEFT_GPIO_Port;
    sensors[SENSOR_LEFT].echo_pin  = ECHO_LEFT_Pin;

    sensors[SENSOR_RIGHT].trig_port = TRIG_RIGHT_GPIO_Port;
    sensors[SENSOR_RIGHT].trig_pin  = TRIG_RIGHT_Pin;
    sensors[SENSOR_RIGHT].echo_port = ECHO_RIGHT_GPIO_Port;
    sensors[SENSOR_RIGHT].echo_pin  = ECHO_RIGHT_Pin;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        sensors[i].distance_cm = ULTRASONIC_DIST_MAX_CM;
        sensors[i].valid       = 0;
        HAL_GPIO_WritePin(sensors[i].trig_port, sensors[i].trig_pin, GPIO_PIN_RESET);
    }
}

static void Ultrasonic_ReadOne(Sensor_t *s)
{
    /* 1. Send a 10 us TRIG pulse. */
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
    DelayUs(TRIG_PULSE_US);
    HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

    /* 2. Wait for ECHO to rise (start of the return pulse). */
    uint16_t wait_start = Now_us();
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET)
    {
        if ((uint16_t)(Now_us() - wait_start) > ECHO_TIMEOUT_US)
        {
            s->distance_cm = ULTRASONIC_DIST_MAX_CM;
            s->valid = 0;
            return; /* TC-14: sensor timeout -> distance = 400 cm, valid = 0 */
        }
    }

    /* 3. Measure how long ECHO stays high. */
    uint16_t pulse_start = Now_us();
    while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET)
    {
        if ((uint16_t)(Now_us() - pulse_start) > ECHO_TIMEOUT_US)
        {
            s->distance_cm = ULTRASONIC_DIST_MAX_CM;
            s->valid = 0;
            return;
        }
    }
    uint16_t pulse_us = (uint16_t)(Now_us() - pulse_start);

    /* 4. Convert to distance and clamp to the sensor's rated range. */
    float dist = ((float)pulse_us * SPEED_OF_SOUND_CM_US) / 2.0f;
    if (dist < ULTRASONIC_DIST_MIN_CM) dist = ULTRASONIC_DIST_MIN_CM;
    if (dist > ULTRASONIC_DIST_MAX_CM) dist = ULTRASONIC_DIST_MAX_CM;

    s->distance_cm = dist;
    s->valid       = 1;
}

void Ultrasonic_ReadAll(void)
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Ultrasonic_ReadOne(&sensors[i]);
        DelayUs(INTER_SENSOR_GAP_US);
    }
}

float Ultrasonic_GetDistance(SensorId_t id)
{
    if (id >= SENSOR_COUNT) return ULTRASONIC_DIST_MAX_CM;
    return sensors[id].distance_cm;
}

uint8_t Ultrasonic_IsValid(SensorId_t id)
{
    if (id >= SENSOR_COUNT) return 0;
    return sensors[id].valid;
}
