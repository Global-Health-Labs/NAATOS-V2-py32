#include "uart_init.h"
#include "alarm.h"
#include "main.h"

UART_HandleTypeDef UartHandle;

/**
 * @brief Initializes UART2 for TX/RX at 115200 baud.
 *
 * This function configures the UART peripheral and associated GPIO pins.
 */
void UART_Init(void)
{
    // Example: Enable USART2 clock
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure PA0 (TX) and PA1 (RX) for USART2
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin                     = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode                    = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull                    = GPIO_PULLUP;
    GPIO_InitStruct.Speed                   = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate               = GPIO_AF3_USART2;//GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure UART
    UartHandle.Instance                     = USART2;
    UartHandle.Init.BaudRate                = 115200;
    UartHandle.Init.WordLength              = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits                = UART_STOPBITS_1;
    UartHandle.Init.Parity                  = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl               = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode                    = UART_MODE_TX_RX;

    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        // Handle error
        APP_ErrorHandler(ERR_FIRMWARE_CONFIG);
    }
}
