/**
 ******************************************************************************
 * @file    uart_shell.h
 * @brief   USART1 command shell (IT-driven RX ring buffer) and telemetry
 *          framer (blocking TX -- see the note in uart_shell.c on why DMA
 *          is deliberately avoided for the PICSimLab target). See
 *          README.md for the full wire-protocol spec and shell command list.
 ******************************************************************************
 */
#ifndef UART_SHELL_H
#define UART_SHELL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** One-time init: clears buffers and arms the first byte-interrupt RX. */
void UART_Shell_Init(void);

/** Drains the RX ring buffer and dispatches any complete command lines.
 *  Call every main-loop iteration; it never blocks. */
void UART_Shell_Process(void);

/** Sends the $EV,... telemetry frame (EV metrics). Call at 10 Hz. Blocks
 *  for ~9 ms while the frame is transmitted. */
void UART_Shell_SendEvFrame(void);

/** Sends the $AD,... telemetry frame (ADAS alerts). Call at 10 Hz. Blocks
 *  for ~9 ms while the frame is transmitted. */
void UART_Shell_SendAdasFrame(void);

/** Sends $ST,... only if the vehicle state changed since the last call.
 *  Cheap to call every 10 ms (no-op most calls). */
void UART_Shell_CheckStateChange(void);

/** If a "temp <C>" override is active, writes it to *out_c and returns 1. */
uint8_t UART_Shell_GetTempOverride(float *out_c);

/* Wire this up from your HAL callback in stm32f1xx_it.c / main.c:
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *   { if (huart->Instance == USART1) UART_Shell_RxCpltCallback(); }
 */
void UART_Shell_RxCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_SHELL_H */
