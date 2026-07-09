/**
 ******************************************************************************
 * @file    uart_shell.c
 * @brief   USART1 command shell + telemetry framer implementation.
 *
 * WIRE PROTOCOL
 * --------------
 * Every frame is human-readable ASCII, wrapped as:
 *     $TAG,field:value,field:value,...*CCCC\r\n
 * where CCCC is a 4-hex-digit CRC-16/CCITT (poly 0x1021, init 0xFFFF)
 * computed over the bytes between '$' and '*' (exclusive). An ASCII
 * envelope was chosen over a raw binary packet so that (a) PICSimLab's
 * built-in UART Terminal can display it directly while debugging, and
 * (b) the Python dashboard can parse it with a single regex — while the
 * CRC still gives the same "packet integrity" guarantee a binary CRC16
 * packet would (see TC-13 in the test plan).
 *
 * Frame tags (matching the four packet types in the design doc's UART
 * Packet Topics table, section 9.1):
 *   $EV   EV metrics    (0x01 equivalent) - 10 Hz
 *   $AD   ADAS alerts   (0x02 equivalent) - 10 Hz
 *   $ST   State change  (0x03 equivalent) - on change only
 *   $ACK  Status/ack    (0x04 equivalent) - on "status" command
 *
 * SHELL COMMANDS (case-sensitive, lowercase, one per line, newline-terminated)
 *   mode <eco|normal|sport>     Set drive mode
 *   speed set <kmh>             Override speed (bypasses the physics model)
 *   speed <kmh>                 Shorthand for "speed set <kmh>"
 *   speed auto                  Return speed to the computed model
 *   soc set <pct>                Override SOC
 *   soc <pct>                    Shorthand for "soc set <pct>"
 *   soc auto                     Return SOC to the computed model
 *   obstacle <cm>                Inject a front-obstacle distance reading
 *   blindspot <on|off>           Force both blind-spot flags
 *   fault inject <motor|soc|col|sen|com>   Force that fault bit (-> FAULT state)
 *   fault clear                  Clear all faults/injections -> PARKED
 *   reset                        Alias for "fault clear"
 *   status                       Request one $ACK frame
 *   stream <on|off>               Enable/disable the 10 Hz telemetry frames
 *   temp <C>                      Override motor temperature (bonus, non-core)
 ******************************************************************************
 */
#include "uart_shell.h"
#include "main.h"
#include "ev_control.h"
#include "adas.h"
#include "fault.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- RX ring buffer ------------------------------------------------------ */
#define RING_SIZE     (128U)
#define LINE_BUF_SIZE (96U)

static uint8_t          rx_byte;
static uint8_t          ring[RING_SIZE];
static volatile uint16_t ring_head = 0;
static volatile uint16_t ring_tail = 0;

static char    line_buf[LINE_BUF_SIZE];
static uint8_t line_len = 0;

/* ---- TX --------------------------------------------------------------
 * A blocking HAL_UART_Transmit() is used on purpose instead of DMA/IT:
 * PICSimLab's simulated peripherals are known to be incomplete for some
 * modes (its ADC only implements single-conversion, for instance), and
 * DMA is one of the more complex peripherals to emulate correctly. A
 * ~100-byte frame at 115200 baud takes ~9 ms to send; since frames only
 * go out from the 100 ms scheduler tick (not the time-critical 10 ms EV
 * loop), and that loop already uses variable-dt integration to absorb
 * exactly this kind of occasional stall (see main.c), blocking here costs
 * nothing functionally and removes an entire category of simulator risk. */
static char tx_frame[176];
static uint8_t stream_enabled = 1;

/* ---- "temp <C>" bonus override -------------------------------------------- */
static uint8_t temp_override_active = 0;
static float   temp_override_c      = 25.0f;

/* -------------------------------------------------------------------------
 * CRC-16/CCITT (poly 0x1021, init 0xFFFF, no reflect) -- mirrored exactly
 * in dashboard.py's crc16_ccitt() so both sides agree byte-for-byte.
 * ---------------------------------------------------------------------- */
static uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8U; b++)
        {
            if (crc & 0x8000U) crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* -------------------------------------------------------------------------
 * Fixed-point float formatting that does NOT depend on newlib's printf
 * float support. STM32CubeIDE links newlib-nano by default, which needs
 * "-u _printf_float" added manually before "%f" works in printf/snprintf --
 * a very common first-timer gotcha. Avoiding it here also saves several KB
 * of flash on a 64 KB part.
 * ---------------------------------------------------------------------- */
static void FormatFixed(char *out, float value, uint8_t decimals)
{
    if (value < 0.0f)
    {
        *out++ = '-';
        value = -value;
    }

    uint32_t scale = 1U;
    for (uint8_t i = 0; i < decimals; i++) scale *= 10U;

    uint32_t scaled = (uint32_t)(value * (float)scale + 0.5f);
    uint32_t whole  = scaled / scale;
    uint32_t frac   = scaled % scale;

    out += sprintf(out, "%lu", (unsigned long)whole);

    if (decimals > 0U)
    {
        char fbuf[12];
        sprintf(fbuf, "%lu", (unsigned long)frac);
        uint8_t fLen = (uint8_t)strlen(fbuf);

        *out++ = '.';
        for (uint8_t i = fLen; i < decimals; i++) *out++ = '0';  /* zero-pad */
        strcpy(out, fbuf);
    }
}

/* -------------------------------------------------------------------------
 * Frame transmission
 * ---------------------------------------------------------------------- */
static void SendFrame(const char *tag, const char *payload)
{
    uint16_t plen = (uint16_t)strlen(payload);
    uint16_t crc  = CRC16_CCITT((const uint8_t *)payload, plen);

    int n = snprintf(tx_frame, sizeof(tx_frame), "$%s,%s*%04X\r\n", tag, payload, (unsigned int)crc);
    if (n <= 0) return;
    if ((uint32_t)n >= sizeof(tx_frame)) n = (int)sizeof(tx_frame) - 1;

    HAL_UART_Transmit(&huart1, (uint8_t *)tx_frame, (uint16_t)n, 50U);
}

void UART_Shell_SendEvFrame(void)
{
    if (!stream_enabled) return;

    const EvStatus_t *ev = EV_GetStatus();
    char s1[12], s2[12], s3[12], s4[12], s5[12], s6[12];
    FormatFixed(s1, ev->speed_kmh,    1);
    FormatFixed(s2, ev->soc_pct,      1);
    FormatFixed(s3, ev->torque_nm,    1);
    FormatFixed(s4, ev->power_kw,     2);
    FormatFixed(s5, ev->range_km,     0);
    FormatFixed(s6, ev->motor_temp_c, 1);

    char payload[128];
    snprintf(payload, sizeof(payload),
             "SPD:%s,SOC:%s,TRQ:%s,PWR:%s,RNG:%s,TMP:%s,MODE:%d,ACC:%u,BRK:%u,UP:%lu",
             s1, s2, s3, s4, s5, s6,
             (int)ev->mode, (unsigned)ev->accel_pct, (unsigned)ev->brake_pct,
             (unsigned long)HAL_GetTick());

    SendFrame("EV", payload);
}

void UART_Shell_SendAdasFrame(void)
{
    if (!stream_enabled) return;

    const AdasStatus_t *a = ADAS_GetStatus();
    char sF[10], sL[10], sR[10], sT[10];
    FormatFixed(sF, a->front_cm, 0);
    FormatFixed(sL, a->left_cm,  0);
    FormatFixed(sR, a->right_cm, 0);

    char payload[128];
    if (a->ttc_sec > 0.0f)
    {
        FormatFixed(sT, a->ttc_sec, 1);
        snprintf(payload, sizeof(payload),
                 "F:%s,L:%s,R:%s,TTC:%s,COL:%d,BSD:%d%d,ALM:%d,FLT:%02X",
                 sF, sL, sR, sT,
                 (int)a->collision_lvl, a->blindspot_l, a->blindspot_r,
                 (int)a->alarm_lvl, Fault_GetFlags());
    }
    else
    {
        snprintf(payload, sizeof(payload),
                 "F:%s,L:%s,R:%s,TTC:--,COL:%d,BSD:%d%d,ALM:%d,FLT:%02X",
                 sF, sL, sR,
                 (int)a->collision_lvl, a->blindspot_l, a->blindspot_r,
                 (int)a->alarm_lvl, Fault_GetFlags());
    }

    SendFrame("AD", payload);
}

static void SendStateFrame(void)
{
    char payload[48];
    snprintf(payload, sizeof(payload), "STATE:%d,MODE:%d,FLT:%02X",
             (int)Fault_GetState(), (int)EV_GetStatus()->mode, Fault_GetFlags());
    SendFrame("ST", payload);
}

void UART_Shell_CheckStateChange(void)
{
    static VehicleState_t last_state = STATE_PARKED;
    VehicleState_t s = Fault_GetState();
    if (s != last_state)
    {
        last_state = s;
        SendStateFrame();
    }
}

static void SendAckFrame(void)
{
    char payload[48];
    snprintf(payload, sizeof(payload), "UP:%lu,VER:1.0", (unsigned long)HAL_GetTick());
    SendFrame("ACK", payload);
}

uint8_t UART_Shell_GetTempOverride(float *out_c)
{
    if (temp_override_active && out_c != NULL)
    {
        *out_c = temp_override_c;
        return 1U;
    }
    return 0U;
}

/* -------------------------------------------------------------------------
 * Command dispatch (all tokens are case-sensitive, lowercase by design --
 * see the file header comment for the full command list).
 * ---------------------------------------------------------------------- */
static void DispatchCommand(char *line)
{
    char *cmd = strtok(line, " ");
    if (cmd == NULL) return;

    if (strcmp(cmd, "mode") == 0)
    {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) return;
        if      (strcmp(arg, "eco")    == 0) EV_SetMode(MODE_ECO);
        else if (strcmp(arg, "normal") == 0) EV_SetMode(MODE_NORMAL);
        else if (strcmp(arg, "sport")  == 0) EV_SetMode(MODE_SPORT);
    }
    else if (strcmp(cmd, "speed") == 0)
    {
        char *arg1 = strtok(NULL, " ");
        if (arg1 == NULL) return;
        if      (strcmp(arg1, "set")  == 0) { char *a2 = strtok(NULL, " "); if (a2) EV_InjectSpeed((float)atof(a2)); }
        else if (strcmp(arg1, "auto") == 0) { EV_ClearInjection(); }
        else                                 { EV_InjectSpeed((float)atof(arg1)); }
    }
    else if (strcmp(cmd, "soc") == 0)
    {
        char *arg1 = strtok(NULL, " ");
        if (arg1 == NULL) return;
        if      (strcmp(arg1, "set")  == 0) { char *a2 = strtok(NULL, " "); if (a2) EV_InjectSOC((float)atof(a2)); }
        else if (strcmp(arg1, "auto") == 0) { EV_ClearInjection(); }
        else                                 { EV_InjectSOC((float)atof(arg1)); }
    }
    else if (strcmp(cmd, "obstacle") == 0)
    {
        char *arg = strtok(NULL, " ");
        if (arg) ADAS_InjectFront((float)atof(arg));
    }
    else if (strcmp(cmd, "blindspot") == 0)
    {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) return;
        uint8_t on = (strcmp(arg, "on") == 0) ? 1U : 0U;
        ADAS_InjectBlindSpot(on, on);
    }
    else if (strcmp(cmd, "fault") == 0)
    {
        char *arg1 = strtok(NULL, " ");
        if (arg1 == NULL) return;
        if (strcmp(arg1, "clear") == 0)
        {
            Fault_Clear();
            ADAS_ClearInjection();
            EV_ClearInjection();
            temp_override_active = 0;
        }
        else if (strcmp(arg1, "inject") == 0)
        {
            char *type = strtok(NULL, " ");
            if (type == NULL) return;
            if      (strcmp(type, "motor") == 0) Fault_InjectRaw(FAULT_OT);
            else if (strcmp(type, "soc")   == 0) Fault_InjectRaw(FAULT_SOC);
            else if (strcmp(type, "col")   == 0) Fault_InjectRaw(FAULT_COL);
            else if (strcmp(type, "sen")   == 0) Fault_InjectRaw(FAULT_SEN);
            else if (strcmp(type, "com")   == 0) Fault_InjectRaw(FAULT_COM);
        }
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        Fault_Clear();
        ADAS_ClearInjection();
        EV_ClearInjection();
        temp_override_active = 0;
    }
    else if (strcmp(cmd, "status") == 0)
    {
        SendAckFrame();
    }
    else if (strcmp(cmd, "stream") == 0)
    {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) return;
        stream_enabled = (strcmp(arg, "on") == 0) ? 1U : 0U;
    }
    else if (strcmp(cmd, "temp") == 0)
    {
        char *arg = strtok(NULL, " ");
        if (arg)
        {
            temp_override_active = 1;
            temp_override_c = (float)atof(arg);
        }
    }
    /* Unknown command: silently ignored, matching a permissive debug shell.
     * If you'd like an explicit "ERR" reply, add an else-branch calling
     * SendFrame("ERR", "UNKNOWN_CMD") here. */
}

/* -------------------------------------------------------------------------
 * RX ring buffer + line assembly
 * ---------------------------------------------------------------------- */
void UART_Shell_Init(void)
{
    ring_head = 0;
    ring_tail = 0;
    line_len  = 0;
    stream_enabled = 1;
    temp_override_active = 0;

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void UART_Shell_RxCpltCallback(void)
{
    uint16_t next = (uint16_t)((ring_head + 1U) % RING_SIZE);
    if (next != ring_tail)              /* drop the byte if the ring is full */
    {
        ring[ring_head] = rx_byte;
        ring_head = next;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* re-arm for the next byte */
}

void UART_Shell_Process(void)
{
    while (ring_tail != ring_head)
    {
        uint8_t c = ring[ring_tail];
        ring_tail = (uint16_t)((ring_tail + 1U) % RING_SIZE);

        if (c == '\r')
        {
            continue;
        }
        else if (c == '\n')
        {
            line_buf[line_len] = '\0';
            if (line_len > 0U) DispatchCommand(line_buf);
            line_len = 0U;
        }
        else if (line_len < (LINE_BUF_SIZE - 1U))
        {
            line_buf[line_len++] = (char)c;
        }
        /* else: line too long, silently truncated until the next '\n' */
    }
}
