/**
 ******************************************************************************
 * @file    adas.h
 * @brief   ADAS engine: TTC, forward-collision warning/critical (with
 *          hysteresis), blind-spot detection, and alarm priority (P0-P3).
 ******************************************************************************
 */
#ifndef ADAS_H
#define ADAS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    COLLISION_NONE     = 0,
    COLLISION_WARNING  = 1,
    COLLISION_CRITICAL = 2
} CollisionLevel_t;

typedef enum
{
    ALARM_NONE     = 0,
    ALARM_ADVISORY = 1,
    ALARM_WARNING  = 2,
    ALARM_CRITICAL = 3
} AlarmLevel_t;

typedef struct
{
    float            front_cm;
    float            left_cm;
    float            right_cm;
    float            ttc_sec;        /* <= 0 means "not applicable"           */
    CollisionLevel_t collision_lvl;  /* hysteresis-protected                  */
    uint8_t          blindspot_l;
    uint8_t          blindspot_r;
    AlarmLevel_t     alarm_lvl;
} AdasStatus_t;

void                ADAS_Init(void);

/**
 * Triggers Ultrasonic_ReadAll() (blocking, up to ~90 ms worst case) and
 * recomputes TTC/collision/blind-spot/alarm state. Call only from the
 * 100 ms scheduler context, passing the EV controller's current speed.
 */
void                ADAS_Update(float speed_kmh);

const AdasStatus_t *ADAS_GetStatus(void);

/** 1 if any of the three ultrasonic sensors timed out on the last read. */
uint8_t             ADAS_SensorFault(void);

/* Test-injection hooks used by the UART shell. */
void ADAS_InjectFront(float cm);
void ADAS_InjectBlindSpot(uint8_t left_on, uint8_t right_on);
void ADAS_ClearInjection(void);

#ifdef __cplusplus
}
#endif

#endif /* ADAS_H */
