/**
 ******************************************************************************
 * @file    fault.h
 * @brief   Fault bit-register + 5-state vehicle FSM
 *          (PARKED -> READY -> DRIVING <-> REGEN, with FAULT reachable from
 *          any state and requiring an explicit "fault clear" to leave).
 ******************************************************************************
 */
#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    STATE_PARKED  = 0,
    STATE_READY   = 1,
    STATE_DRIVING = 2,
    STATE_REGEN   = 3,
    STATE_FAULT   = 4
} VehicleState_t;

#define FAULT_OT   (0x01U)   /* motor over-temperature (>90 C)      -- hard trip */
#define FAULT_SOC  (0x02U)   /* battery SOC critical (<2%)           -- hard trip */
#define FAULT_COL  (0x04U)   /* collision CRITICAL                   -- hard trip */
#define FAULT_SEN  (0x08U)   /* ultrasonic sensor timeout             -- logged only */
#define FAULT_COM  (0x10U)   /* UART comm timeout (test-injectable)   -- logged only */

void Fault_Init(void);

/**
 * Call every 10 ms with the latest evaluated conditions. OT/SOC/COL force a
 * hard trip into STATE_FAULT (and cut motor PWM via Fault_MotorPWMAllowed());
 * SEN/COM are recorded in the fault byte for telemetry/diagnostics but let
 * the vehicle degrade gracefully rather than stranding it, per the design
 * doc's NFR table. A manually-injected fault of ANY type (see
 * Fault_InjectRaw) always forces STATE_FAULT, since that command exists
 * specifically to exercise the fault + recovery workflow end to end.
 */
void Fault_Update(uint8_t motor_overtemp, uint8_t soc_critical,
                   uint8_t collision_critical, uint8_t sensor_fault,
                   uint8_t comm_timeout, uint8_t accel_pct, uint8_t brake_pct);

void            Fault_InjectRaw(uint8_t fault_mask);
void            Fault_Clear(void);
uint8_t         Fault_GetFlags(void);
VehicleState_t  Fault_GetState(void);
uint8_t         Fault_MotorPWMAllowed(void);   /* 0 once latched into FAULT */

#ifdef __cplusplus
}
#endif

#endif /* FAULT_H */
