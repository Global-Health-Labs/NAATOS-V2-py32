
#include "error_handler.h"
#include <string.h>


#define LOOP_TIMER_COUNT_100MSEC    0x17000

// This will delay by roughly 100msec * delay_count. It is not very accurate.
void sw_delay_100msec(uint32_t delay_count) {
    for (uint32_t i=0; i < (delay_count * LOOP_TIMER_COUNT_100MSEC); i++);
}

// APP_ErrorHandler will shut down all heaters, then blink the error code number
void APP_ErrorHandler(uint8_t errnum)
{
    char outputStr[256];

    __disable_irq(); // Disable all interrupts
    
    // Make sure all heaters are disabled
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL1, Pins.GPIO_Pin_AMP_CTRL1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_AMP_CTRL2, Pins.GPIO_Pin_AMP_CTRL2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL1, Pins.GPIO_Pin_VALVE_CTRL1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Pins.GPIOx_VALVE_CTRL2, Pins.GPIO_Pin_VALVE_CTRL2, GPIO_PIN_RESET);
    
    sprintf(outputStr, "APP_ErrorHandler: %d. Halting system.\r\n", errnum);		
    HAL_UART_Transmit(&UartHandle, (uint8_t *)outputStr, strlen(outputStr), 1000);	
    
    HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(Pins.GPIOx_LED2, Pins.GPIO_Pin_LED2, GPIO_PIN_RESET); 
    
    while (1)
    {
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);   // LED on for 1 sec
        sw_delay_100msec(10);
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);   // LED off for 1 sec        
        sw_delay_100msec(10);

        for (int i = 0; i < errnum; i++) {          // blink the error code
            HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_RESET);   // LED on for 200 msec
            sw_delay_100msec(2);
            HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);   // LED off for 200 msec        
            sw_delay_100msec(2);
        }
        HAL_GPIO_WritePin(Pins.GPIOx_LED1, Pins.GPIO_Pin_LED1, GPIO_PIN_SET);   // LED off for 1 sec        
        sw_delay_100msec(10);
    }
}
