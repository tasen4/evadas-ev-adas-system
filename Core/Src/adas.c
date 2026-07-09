/**
 ******************************************************************************
 * @file    adas.c
 * @brief   ADAS engine implementation.
 *
 * Alarm hysteresis (doc section 7.6): escalation to a WORSE collision level
 * is immediate (a real obstacle must never be masked by a filter), but
 * de-escalation to a BETTER level requires the better condition to hold for
 * 3 consecutive 100 ms sensor cycles (300 ms) before it is accepted. This
 * applies uniformly whether stepping CRITICAL -> WARNING or WARNING -> NONE.
 ******************************************************************************
 */
#include "adas.h"
#include "ultrasonic.h"

#define FCW_WARN_CM        (50.0f)
#define FCW_CRIT_CM        (20.0f)
#define TTC_WARN_S         (3.0f)
#define TTC_CRIT_S         (1.5f)
#define BSD_RANGE_CM       (30.0f)
#define BSD_MIN_SPEED_KMH  (20.0f)
#define OVERSPEED_KMH      (120.0f)
#define HYSTERESIS_SAMPLES (3U)   /* 3 x 100 ms = 300 ms */

static AdasStatus_t status;
static CollisionLevel_t asserted_lvl;
static uint8_t           settle_counter;

static uint8_t inject_front_active = 0;
static float   inject_front_cm     = ULTRASONIC_DIST_MAX_CM;
static uint8_t inject_bl = 0, inject_br = 0;

void ADAS_Init(void)
{
    status.front_cm      = ULTRASONIC_DIST_MAX_CM;
    status.left_cm       = ULTRASONIC_DIST_MAX_CM;
    status.right_cm      = ULTRASONIC_DIST_MAX_CM;
    status.ttc_sec       = -1.0f;
    status.collision_lvl = COLLISION_NONE;
    status.blindspot_l   = 0;
    status.blindspot_r   = 0;
    status.alarm_lvl     = ALARM_NONE;

    asserted_lvl   = COLLISION_NONE;
    settle_counter = 0;

    inject_front_active = 0;
    inject_bl = 0;
    inject_br = 0;
}

void ADAS_InjectFront(float cm)                     { inject_front_active = 1; inject_front_cm = cm; }
void ADAS_InjectBlindSpot(uint8_t left_on, uint8_t right_on) { inject_bl = left_on ? 1 : 0; inject_br = right_on ? 1 : 0; }
void ADAS_ClearInjection(void)                        { inject_front_active = 0; inject_bl = 0; inject_br = 0; }

uint8_t ADAS_SensorFault(void)
{
    return !(Ultrasonic_IsValid(SENSOR_FRONT) &&
             Ultrasonic_IsValid(SENSOR_LEFT)  &&
             Ultrasonic_IsValid(SENSOR_RIGHT));
}

void ADAS_Update(float speed_kmh)
{
    Ultrasonic_ReadAll();

    status.front_cm = inject_front_active ? inject_front_cm : Ultrasonic_GetDistance(SENSOR_FRONT);
    status.left_cm  = Ultrasonic_GetDistance(SENSOR_LEFT);
    status.right_cm = Ultrasonic_GetDistance(SENSOR_RIGHT);

    /* ---- Time-to-collision ------------------------------------------- */
    float speed_cm_s = (speed_kmh * 100000.0f) / 3600.0f;
    if (speed_cm_s > 1.0f && status.front_cm < (ULTRASONIC_DIST_MAX_CM - 0.01f))
        status.ttc_sec = status.front_cm / speed_cm_s;
    else
        status.ttc_sec = -1.0f;   /* stationary or no obstacle in range */

    /* ---- Instantaneous collision evaluation --------------------------- */
    uint8_t crit_now = (status.front_cm < FCW_CRIT_CM) ||
                        (status.ttc_sec > 0.0f && status.ttc_sec < TTC_CRIT_S);
    uint8_t warn_now = (status.front_cm < FCW_WARN_CM) ||
                        (status.ttc_sec > 0.0f && status.ttc_sec < TTC_WARN_S);

    CollisionLevel_t instant_lvl = crit_now ? COLLISION_CRITICAL
                                 : warn_now ? COLLISION_WARNING
                                            : COLLISION_NONE;

    /* ---- Hysteresis ladder: instant escalation, filtered de-escalation - */
    if (instant_lvl > asserted_lvl)
    {
        asserted_lvl   = instant_lvl;
        settle_counter = 0;
    }
    else if (instant_lvl < asserted_lvl)
    {
        settle_counter++;
        if (settle_counter >= HYSTERESIS_SAMPLES)
        {
            asserted_lvl   = instant_lvl;
            settle_counter = 0;
        }
    }
    else
    {
        settle_counter = 0;
    }
    status.collision_lvl = asserted_lvl;

    /* ---- Blind-spot detection (active above 20 km/h) ------------------ */
    if (speed_kmh > BSD_MIN_SPEED_KMH)
    {
        status.blindspot_l = (inject_bl || (status.left_cm  < BSD_RANGE_CM)) ? 1 : 0;
        status.blindspot_r = (inject_br || (status.right_cm < BSD_RANGE_CM)) ? 1 : 0;
    }
    else
    {
        status.blindspot_l = 0;
        status.blindspot_r = 0;
    }

    /* ---- Alarm priority (P0..P3) --------------------------------------- */
    if (status.collision_lvl == COLLISION_CRITICAL)
        status.alarm_lvl = ALARM_CRITICAL;
    else if (status.collision_lvl == COLLISION_WARNING)
        status.alarm_lvl = ALARM_WARNING;
    else if (status.blindspot_l || status.blindspot_r || (speed_kmh > OVERSPEED_KMH))
        status.alarm_lvl = ALARM_ADVISORY;
    else
        status.alarm_lvl = ALARM_NONE;
}

const AdasStatus_t *ADAS_GetStatus(void)
{
    return &status;
}
