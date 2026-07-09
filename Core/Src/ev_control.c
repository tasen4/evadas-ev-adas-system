/**
 ******************************************************************************
 * @file    ev_control.c
 * @brief   Simple point-mass longitudinal dynamics model for the simulated
 *          EV, plus SOC energy-integration and range estimation.
 *
 * This is a *teaching-grade* physics model: a single-gear reduction drive,
 * constant rolling resistance, and a quadratic aerodynamic drag term. The
 * constants below are documented assumptions, not measured vehicle data —
 * tune them freely if you want different accel/braking "feel" in the demo.
 ******************************************************************************
 */
#include "ev_control.h"
#include <math.h>

/* ---- Vehicle/physics constants (tunable) -------------------------------- */
#define VEHICLE_MASS_KG        (1500.0f)
#define WHEEL_RADIUS_M         (0.30f)
#define GEAR_RATIO             (8.0f)
#define ROLLING_RESIST_N       (150.0f)
#define DRAG_COEFF             (0.35f)     /* lumped 0.5*rho*Cd*A              */
#define BATTERY_CAPACITY_KWH   (20.0f)
#define REGEN_BRAKE_THRESH_PCT (5U)        /* torque-model regen onset         */

/* ECO / NORMAL / SPORT, indexed by DriveMode_t */
static const float MODE_TORQUE_SCALE[3]   = { 0.6f, 1.0f, 1.3f };
static const float MODE_EFFICIENCY_WHKM[3] = { 25.0f, 30.0f, 35.0f }; /* NORMAL interpolated */

static EvStatus_t ev;
static float      speed_ms;              /* higher-resolution internal state */

static uint8_t inject_speed_active = 0;
static uint8_t inject_soc_active   = 0;

void EV_Init(void)
{
    ev.speed_kmh    = 0.0f;
    ev.soc_pct      = 100.0f;
    ev.torque_nm    = 0.0f;
    ev.power_kw     = 0.0f;
    ev.motor_temp_c = 25.0f;
    ev.mode         = MODE_NORMAL;
    ev.accel_pct    = 0;
    ev.brake_pct    = 0;
    ev.range_km     = (BATTERY_CAPACITY_KWH * 1000.0f) / MODE_EFFICIENCY_WHKM[MODE_NORMAL];

    speed_ms            = 0.0f;
    inject_speed_active = 0;
    inject_soc_active    = 0;
}

void EV_SetMode(DriveMode_t mode)
{
    if (mode == MODE_ECO || mode == MODE_NORMAL || mode == MODE_SPORT)
        ev.mode = mode;
}

void EV_InjectSpeed(float kmh)
{
    inject_speed_active = 1;
    ev.speed_kmh = kmh;
    speed_ms     = kmh / 3.6f;
}

void EV_InjectSOC(float pct)
{
    inject_soc_active = 1;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    ev.soc_pct = pct;
}

void EV_ClearInjection(void)
{
    inject_speed_active = 0;
    inject_soc_active    = 0;
}

void EV_Update(uint8_t accel_pct, uint8_t brake_pct, float motor_temp_c, float dt_s)
{
    ev.accel_pct    = accel_pct;
    ev.brake_pct    = brake_pct;
    ev.motor_temp_c = motor_temp_c;

    /* ---- Torque command: positive drive, or negative regen ------------- */
    if (brake_pct > REGEN_BRAKE_THRESH_PCT)
    {
        float brake_frac = (float)brake_pct / 100.0f;
        if (brake_frac > 1.0f) brake_frac = 1.0f;
        ev.torque_nm = -EV_MAX_REGEN_TORQUE_NM * brake_frac;
    }
    else
    {
        float accel_frac = (float)accel_pct / 100.0f;
        if (accel_frac > 1.0f) accel_frac = 1.0f;
        ev.torque_nm = EV_MAX_DRIVE_TORQUE_NM * accel_frac * MODE_TORQUE_SCALE[ev.mode];
    }

    /* ---- Longitudinal dynamics (skipped while a test speed is injected) - */
    if (!inject_speed_active)
    {
        float f_drive = (ev.torque_nm * GEAR_RATIO) / WHEEL_RADIUS_M;
        float f_drag  = DRAG_COEFF * speed_ms * speed_ms;   /* always opposes motion */
        float f_roll  = (speed_ms > 0.01f) ? ROLLING_RESIST_N : 0.0f;

        float f_net       = f_drive - f_roll - f_drag;
        float accel_mps2  = f_net / VEHICLE_MASS_KG;

        speed_ms += accel_mps2 * dt_s;
        if (speed_ms < 0.0f) speed_ms = 0.0f;

        float max_speed_ms = EV_MAX_SPEED_KMH / 3.6f;
        if (speed_ms > max_speed_ms) speed_ms = max_speed_ms;

        ev.speed_kmh = speed_ms * 3.6f;
    }

    /* ---- Electrical power (kW): P = torque x motor angular speed --------- */
    float omega_wheel_rad_s = speed_ms / WHEEL_RADIUS_M;
    float omega_motor_rad_s = omega_wheel_rad_s * GEAR_RATIO;
    ev.power_kw = (ev.torque_nm * omega_motor_rad_s) / 1000.0f;

    /* ---- SOC integration: energy = power x time (skipped while injected) - */
    if (!inject_soc_active)
    {
        float dt_hours  = dt_s / 3600.0f;
        float delta_pct = (ev.power_kw * dt_hours / BATTERY_CAPACITY_KWH) * 100.0f;
        ev.soc_pct -= delta_pct;   /* regen (power_kw < 0) increases SOC */
        if (ev.soc_pct > 100.0f) ev.soc_pct = 100.0f;
        if (ev.soc_pct < 0.0f)   ev.soc_pct = 0.0f;
    }

    /* ---- Range estimate from remaining energy and mode efficiency -------- */
    float remaining_kwh = (ev.soc_pct / 100.0f) * BATTERY_CAPACITY_KWH;
    ev.range_km = (remaining_kwh * 1000.0f) / MODE_EFFICIENCY_WHKM[ev.mode];
}

const EvStatus_t *EV_GetStatus(void)
{
    return &ev;
}
