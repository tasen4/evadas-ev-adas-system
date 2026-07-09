/**
 ******************************************************************************
 * @file    ultrasonic.h
 * @brief   HC-SR04 ultrasonic driver — Front/Left/Right, TIM2-based (1 us
 *          resolution) polling measurement.
 *
 * Hardware mapping (per EV ADAS Requirements & Design Doc, section 6.1):
 *   SENSOR_FRONT : TRIG = PB0   ECHO = PB1
 *   SENSOR_LEFT  : TRIG = PB2   ECHO = PB3
 *   SENSOR_RIGHT : TRIG = PB4   ECHO = PB5
 *
 * TIM2 must be configured in CubeMX as a simple time-base, no interrupt,
 * Prescaler = 71, Period (Counter Period) = 65535, giving a 1 MHz
 * (1 us/tick) free-running 16-bit counter. Start it once from main() with
 * HAL_TIM_Base_Start(&htim2) before calling Ultrasonic_Init().
 ******************************************************************************
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Shared physical constants — also used by adas.c, so they live here as the
 * single source of truth for "what does the sensor's range mean". */
#define ULTRASONIC_DIST_MIN_CM   (2.0f)
#define ULTRASONIC_DIST_MAX_CM   (400.0f)   /* also the "no echo / timeout" value */

typedef enum
{
    SENSOR_FRONT = 0,
    SENSOR_LEFT  = 1,
    SENSOR_RIGHT = 2,
    SENSOR_COUNT
} SensorId_t;

/* One HC-SR04 channel's pin mapping + last reading. */
typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
    float         distance_cm;   /* clamped to [DIST_MIN_CM, DIST_MAX_CM]      */
    uint8_t       valid;         /* 1 = echo received in time, 0 = timed out    */
} Sensor_t;

/** Must be called once after HAL_TIM_Base_Start(&htim2). */
void Ultrasonic_Init(void);

/**
 * Sequentially triggers and reads Front, then Left, then Right, with a
 * 50 us guard interval between sensors (per project spec). This function
 * BLOCKS for up to ~3 x 30 ms = 90 ms in the worst case (all three sensors
 * timing out) — call it only from the 100 ms scheduler context, never from
 * the 10 ms EV control tick or from any ISR.
 */
void Ultrasonic_ReadAll(void);

float   Ultrasonic_GetDistance(SensorId_t id);
uint8_t Ultrasonic_IsValid(SensorId_t id);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */
