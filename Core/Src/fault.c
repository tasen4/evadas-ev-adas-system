/**
 ******************************************************************************
 * @file    fault.c
 * @brief   Fault manager + vehicle FSM implementation.
 *
 * State table (design doc section 5.3), with two gaps filled by explicit,
 * documented judgement calls where the table left the transition unlisted:
 *   PARKED  -> READY    : accel > 2%
 *   READY   -> DRIVING  : accel > 5%
 *   READY   -> PARKED   : [added] accel back to <= 2%
 *   DRIVING -> REGEN    : brake > 10%
 *   REGEN   -> DRIVING  : [added] brake < 5% AND accel > 5%
 *   REGEN   -> READY    : [added] brake < 5% AND accel <= 5%
 *   any     -> FAULT    : hard-trip condition (see fault.h)
 *   FAULT   -> PARKED   : explicit "fault clear" only (UC-05)
 ******************************************************************************
 */
#include "fault.h"

static uint8_t         fault_flags    = 0;
static uint8_t         injected_flags = 0;
static VehicleState_t  state          = STATE_PARKED;

void Fault_Init(void)
{
    fault_flags    = 0;
    injected_flags = 0;
    state          = STATE_PARKED;
}

void Fault_InjectRaw(uint8_t mask)
{
    injected_flags |= mask;
}

void Fault_Clear(void)
{
    fault_flags    = 0;
    injected_flags = 0;
    state          = STATE_PARKED;
}

void Fault_Update(uint8_t motor_overtemp, uint8_t soc_critical,
                   uint8_t collision_critical, uint8_t sensor_fault,
                   uint8_t comm_timeout, uint8_t accel_pct, uint8_t brake_pct)
{
    fault_flags = injected_flags;
    if (motor_overtemp)     fault_flags |= FAULT_OT;
    if (soc_critical)       fault_flags |= FAULT_SOC;
    if (collision_critical) fault_flags |= FAULT_COL;
    if (sensor_fault)       fault_flags |= FAULT_SEN;
    if (comm_timeout)       fault_flags |= FAULT_COM;

    /* Only OT/SOC/COL (or ANY manually-injected test fault) constitute a
     * hard trip. SEN/COM arising naturally are reported but do not by
     * themselves cut torque -- "degrade gracefully" per the NFR table. */
    uint8_t hard_trip = (injected_flags != 0) ||
                         motor_overtemp || soc_critical || collision_critical;

    if (hard_trip)
    {
        state = STATE_FAULT;
        return;
    }

    if (state == STATE_FAULT)
    {
        /* Underlying condition cleared on its own, but we still wait for an
         * explicit "fault clear" command before leaving FAULT (UC-05). */
        return;
    }

    switch (state)
    {
        case STATE_PARKED:
            if (accel_pct > 2U) state = STATE_READY;
            break;

        case STATE_READY:
            if (accel_pct > 5U)       state = STATE_DRIVING;
            else if (accel_pct <= 2U) state = STATE_PARKED;
            break;

        case STATE_DRIVING:
            if (brake_pct > 10U) state = STATE_REGEN;
            break;

        case STATE_REGEN:
            if (brake_pct < 5U)
                state = (accel_pct > 5U) ? STATE_DRIVING : STATE_READY;
            break;

        default:
            state = STATE_PARKED;
            break;
    }
}

uint8_t Fault_GetFlags(void)
{
    return fault_flags;
}

VehicleState_t Fault_GetState(void)
{
    return state;
}

uint8_t Fault_MotorPWMAllowed(void)
{
    return (state != STATE_FAULT) ? 1U : 0U;
}
