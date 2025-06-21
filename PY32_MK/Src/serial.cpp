/*
*   File: adc.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "app_data.h"
#include "alarm.h"
#include "timers.h"
#include "pid.h"


UART_HandleTypeDef UartHandle;
#define UART_RX_BUFFER_SIZE 64
#define MAX_RX_COMMAND_SIZE 64
volatile uint8_t Uart_RxQ[UART_RX_BUFFER_SIZE+1];  // Buffer to store received data
volatile uint8_t Uart_rxChar = 0;                       // Buffer to store received character
DMA_HandleTypeDef hdma_usart2_rx;       // The DMA handle for USART2

uint16_t Uart_RxQ_wrPtr = 0;
uint16_t Uart_RxQ_rdPtr = 0;
uint16_t Uart_RxQ_size = 0;
uint8_t  rxCommand[MAX_RX_COMMAND_SIZE];
uint16_t rxCommandSize = 0;

char outputStr[192];

#ifdef UART_RX_ENABLED
const char *host_commands[NUM_HOST_COMMANDS] = {
    [CMD_START] = "CMD_START",
    [CMD_STOP] = "CMD_STOP",
    [CMD_STATUS] = "CMD_STATUS",
    [CMD_STAGE1_TIME_MINUTES] = "CMD_S1T",
    [CMD_STAGE2_TIME_MINUTES] = "CMD_S2T",
    [CMD_STAGE3_TIME_MINUTES] = "CMD_S3T",
    [CMD_DETECT_TIME_MINUTES] = "CMD_DETT",
    [CMD_HEATER1_STAGE1_SETPOINT] = "CMD_H1S1",
    [CMD_HEATER2_STAGE1_SETPOINT] = "CMD_H2S1",
    [CMD_HEATER3_STAGE1_SETPOINT] = "CMD_H3S1",
    [CMD_HEATER4_STAGE1_SETPOINT] = "CMD_H4S1",
    [CMD_HEATER1_STAGE2_SETPOINT] = "CMD_H1S2",
    [CMD_HEATER2_STAGE2_SETPOINT] = "CMD_H2S2",
    [CMD_HEATER3_STAGE2_SETPOINT] = "CMD_H3S2",
    [CMD_HEATER4_STAGE2_SETPOINT] = "CMD_H4S2",
    [CMD_HEATER1_STAGE3_SETPOINT] = "CMD_H1S3",
    [CMD_HEATER2_STAGE3_SETPOINT] = "CMD_H2S3",
    [CMD_HEATER3_STAGE3_SETPOINT] = "CMD_H3S3",
    [CMD_HEATER4_STAGE3_SETPOINT] = "CMD_H4S3"    
};
#endif

void UART_Init(void)
{
	//Configure UART2
	//PA0 is TX
	//PA1 is RX (PF2 on MK7)
	
    UartHandle.Instance          = USART2;
    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }

    // Start UART reception:
    // 
    Uart_RxQ_wrPtr = 0;
    Uart_RxQ_rdPtr = 0;
    Uart_RxQ_size = 0;
    flags.flagNewUARTData = false;

#ifdef UART_RX_ENABLED
    // enable USART2 interrupts:
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    HAL_UART_Receive_IT(&UartHandle, (uint8_t *) &Uart_rxChar, 1);
#endif
}

// The interrupt handler must be defined with extern "C" to properly override the default handler.
extern "C" void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandle);
    
    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&UartHandle); // Clear the idle flag
        //UART_IdleCallback(&UartHandle);         // Custom processing
    }
    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE); // Clear the rxne flag
    }
}

void print_settings(void)
{
#ifndef DEBUG_REDUCE_MEMORY
    sprintf(outputStr, "Stage 1,2,3,detect times: %d %d %d %d\r\n", (int) data.stage1_time_min, (int)data.stage2_time_min, (int)data.stage3_time_min, (int)data.detection_time_min);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
    sprintf(outputStr, "Stage 1 heater setpoints: %d %d %d %d\r\n", (int)H1_pid_control[1].setpoint, (int)H2_pid_control[1].setpoint, (int)H3_pid_control[1].setpoint, (int)H4_pid_control[1].setpoint);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
    sprintf(outputStr, "Stage 2 heater setpoints: %d %d %d %d\r\n", (int)H1_pid_control[2].setpoint, (int)H2_pid_control[2].setpoint, (int)H3_pid_control[2].setpoint, (int)H4_pid_control[2].setpoint);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
    sprintf(outputStr, "Stage 3 heater setpoints: %d %d %d %d\r\n", (int)H1_pid_control[3].setpoint, (int)H2_pid_control[3].setpoint, (int)H3_pid_control[3].setpoint, (int)H4_pid_control[3].setpoint);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
#endif
}

void Process_UARTRxData(void)
{
#ifdef UART_RX_ENABLED
    uint8_t c;
    bool rxErr = false;
    bool rxCommandValid = false;
    int qSize;
    int i;
    int command_found;
    char command_key[MAX_RX_COMMAND_SIZE];
    int command_value;
    int num_parameters_found;
    
    while (Uart_RxQ_size > 0) {
        command_found = -1;
        __disable_irq();
        qSize = Uart_RxQ_size;
        if ((qSize + rxCommandSize) >= MAX_RX_COMMAND_SIZE-1 || qSize >= UART_RX_BUFFER_SIZE) {
            rxErr = true;
            rxCommandSize = 0;
            Uart_RxQ_size = 0;
            Uart_RxQ_wrPtr = 0;
            Uart_RxQ_rdPtr = 0;
        } else {
            for (i=0; i < qSize; i++) {
                c = Uart_RxQ[Uart_RxQ_rdPtr++];
                if (Uart_RxQ_rdPtr >=UART_RX_BUFFER_SIZE) Uart_RxQ_rdPtr = 0;
                Uart_RxQ_size--;
                if (rxCommandSize == 0 && c == '{') {       // commands start with the '{' delimeter
                    rxCommand[rxCommandSize] = c;
                    rxCommandSize++;
                } else if (rxCommandSize > 0) {
                    if (c == '}') {                         // commands end with the '}' delimeter
                        rxCommand[rxCommandSize] = c;
                        rxCommandSize++;
                        rxCommandValid = true;
                        break;                              // stop receiving any more bytes and process the command
                    } else {
                        rxCommand[rxCommandSize] = c;
                        rxCommandSize++;
                    }
                }
            }
        }
        __enable_irq();
        
        if (rxErr) {
            sprintf(outputStr, "UartRx err: %d %d %d %d\r\n", rxCommandSize, Uart_RxQ_size, Uart_RxQ_wrPtr, Uart_RxQ_rdPtr);		
            HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
        } else if (rxCommandValid) {
            rxCommand[rxCommandSize] = 0;     // add a null char after the last byte
            num_parameters_found = sscanf((const char *) rxCommand, "{\"%10[^\"]\":%d}", command_key, &command_value); 
            if (num_parameters_found > 0) {
                for (i=0; i<NUM_HOST_COMMANDS; i++) {
                    if (strncmp((const char *) command_key, host_commands[i], strlen(host_commands[i])) == 0) {
                        command_found = i;
                        break;
                    }
                }
            }
            
            if (command_found >=0 && num_parameters_found == 1) {
                sprintf(outputStr, "rxCommand: %d %s %d\r\n", num_parameters_found, command_key, command_value);		
                HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);
                switch (command_found) {
                    case CMD_START: 
                        if (!data.test_active) {
                            start_naat_delay_sequence();              
                        }                
                        break;
                    case CMD_STOP: 
                        if (data.test_active) {
                            stop_naat_test();
                        }                
                        break;
                    case CMD_STATUS: 
                        print_settings();
                        break;
                }
            } else if (command_found >=0 && num_parameters_found == 2 && command_value >= 0 && command_value <= OVERTEMP_ERR_C) {
                sprintf(outputStr, "rxCommand: %d %s %d\r\n", num_parameters_found, command_key, command_value);		
                HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);

                switch (command_found) {
                    case CMD_STAGE1_TIME_MINUTES:
                        data.stage1_time_min = command_value;
                        break;
                    case CMD_STAGE2_TIME_MINUTES:
                        data.stage2_time_min = command_value;
                        break;
                    case CMD_STAGE3_TIME_MINUTES:
                        data.stage3_time_min = command_value;
                        break;
                    case CMD_DETECT_TIME_MINUTES:
                        data.detection_time_min = command_value;
                        break;
                    case CMD_HEATER1_STAGE1_SETPOINT:
                        H1_pid_control[1].setpoint = command_value;
                        break;
                    case CMD_HEATER2_STAGE1_SETPOINT:
                        H2_pid_control[1].setpoint = command_value;
                        break;
                    case CMD_HEATER3_STAGE1_SETPOINT:
                        H3_pid_control[1].setpoint = command_value;
                        break;
                    case CMD_HEATER4_STAGE1_SETPOINT:
                        H4_pid_control[1].setpoint = command_value;
                        break;
                    case CMD_HEATER1_STAGE2_SETPOINT:
                        H1_pid_control[2].setpoint = command_value;
                        break;
                    case CMD_HEATER2_STAGE2_SETPOINT:
                        H2_pid_control[2].setpoint = command_value;
                        break;
                    case CMD_HEATER3_STAGE2_SETPOINT:
                        H3_pid_control[2].setpoint = command_value;
                        break;
                    case CMD_HEATER4_STAGE2_SETPOINT:
                        H4_pid_control[2].setpoint = command_value;
                        break;
                    case CMD_HEATER1_STAGE3_SETPOINT:
                        H1_pid_control[3].setpoint = command_value;
                        break;
                    case CMD_HEATER2_STAGE3_SETPOINT:
                        H2_pid_control[3].setpoint = command_value;
                        break;
                    case CMD_HEATER3_STAGE3_SETPOINT:
                        H3_pid_control[3].setpoint = command_value;
                        break;
                    case CMD_HEATER4_STAGE3_SETPOINT:
                        H4_pid_control[3].setpoint = command_value;
                        break;
                }
            } else {
                sprintf(outputStr, "rxCommand err: %d %s\r\n", command_found, rxCommand);		
                HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);                
            }
            
            rxCommandSize = 0;
            rxCommandValid = false;
        }
    }
#endif
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // put the data in the RxBuffer
    Uart_RxQ[Uart_RxQ_wrPtr++] = Uart_rxChar;
    if (Uart_RxQ_wrPtr >= UART_RX_BUFFER_SIZE) Uart_RxQ_wrPtr = 0;
    Uart_RxQ_size++;

    HAL_UART_Receive_IT(&UartHandle, (uint8_t *) &Uart_rxChar, 1);
    
    // Set flag to process the received data in user space
    flags.flagNewUARTData = true;
}
