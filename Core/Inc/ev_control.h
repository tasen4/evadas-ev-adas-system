/**
 ******************************************************************************
 * @file    ev_control.h
 * @brief   EV physics/state module: speed (inertia model), SOC, torque,
 *          power, range, drive modes, and regen braking.
 ******************************************************************************
 */
#ifndef EV_CONTROL_H
#define EV_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MODE_ECO    = 0,
    MODE_NORMAL = 1,
    MODE_SPORT  = 2
} DriveMode_t;

typedef struct
{
    float       speed_kmh;      /* 0..200                                    */
    float       soc_pct;        /* 0..100                                    */
    float       torque_nm;      /* -80 (max regen) .. +250 (max drive)       */
    float       power_kw;       /* negative during regen                     */
    float       range_km;
    float       motor_temp_c;   /* last value passed into EV_Update()        */
    DriveMode_t mode;
    uint8_t     accel_pct;      /* 0..100, last value passed in              */
    uint8_t     brake_pct;      /* 0..100, last value passed in              */
} EvStatus_t;

/* Public so main.c can scale the TIM1 PWM duty cycle without a duplicated
 * magic number. */
#define EV_MAX_DRIVE_TORQUE_NM   (250.0f)
#define EV_MAX_REGEN_TORQUE_NM   (80.0f)
#define EV_MAX_SPEED_KMH         (200.0f)

void EV_Init(void);

/**
 * Advances the EV physics model by dt_s seconds. Call every ~10 ms from the
 * cooperative scheduler with the ACTUAL elapsed time (not a hardcoded
 * constant) so the integration stays correct even if a cycle was delayed
 * by the blocking HC-SR04 read.
 */
void EV_Update(uint8_t accel_pct, uint8_t brake_pct, float motor_temp_c, float dt_s);

const EvStatus_t *EV_GetStatus(void);

void EV_SetMode(DriveMode_t mode);

/* Test-injection hooks used by the UART shell ("speed set"/"soc set").
 * Once injected, the corresponding quantity stops being computed by the
 * model until EV_ClearInjection() is called (e.g. via "fault clear"). */
void EV_InjectSpeed(float kmh);
void EV_InjectSOC(float pct);
void EV_ClearInjection(void);

#ifdef __cplusplus
}
#endif

#endif /* EV_CONTROL_H */
