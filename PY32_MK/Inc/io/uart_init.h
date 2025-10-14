#ifndef UART_INIT_H
#define UART_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#define PY32F003x8
#include "main.h"
#include "py32f0xx_hal.h"
#include "py32f0xx_hal_uart.h"

/** UART Handle used for communication */
extern UART_HandleTypeDef UartHandle;

/**
 * @brief Initializes UART peripheral for the project.
 * 
 * Configures UART2 with:
 * - PA0 as TX, PA1 as RX
 * - 115200 baud
 * - 8 data bits
 * - 1 stop bit
 * - No parity
 * - No flow control
 */
void UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif // UART_INIT_H
