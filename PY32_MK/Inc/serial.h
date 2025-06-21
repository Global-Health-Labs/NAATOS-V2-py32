/*
*   File: serial.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include "py32f0xx_hal.h"

extern char outputStr[192];
extern UART_HandleTypeDef UartHandle;

void UART_Init(void);
void UART_IdleCallback(UART_HandleTypeDef *huart);
void Process_UARTRxData(void);

#endif
