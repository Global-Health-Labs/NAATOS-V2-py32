
#include "error_handler.h"
#include <string.h>


/** 
 * @brief Approximate delay count for 100ms software delay.
 * @note This value is calibrated for the current clock settings and may need
 *       adjustment if system clock frequency changes.
 */
#define LOOP_TIMER_COUNT_100MSEC    0x17000

/**
 * @brief Provides a rough software delay in 100ms increments.
 * @param delay_count Number of 100ms intervals to delay.
 * 
 * This function implements a simple busy-wait delay using a software loop.
 * The delay is approximate and not intended for precise timing. It should
 * only be used in error handling or other non-critical timing scenarios.
 * 
 * @note This is not an accurate delay mechanism and should not be used for
 *       precise timing requirements. The actual delay may vary based on
 *       system clock frequency and optimization settings.
 */
void sw_delay_100msec(uint32_t delay_count) {
    for (uint32_t i=0; i < (delay_count * LOOP_TIMER_COUNT_100MSEC); i++);
}

/**
 * @brief System error handler for critical failures.
 * @param errnum Error code number to display via LED blinks.
 * 
 * This function handles critical system errors by:
 * 1. Disabling all interrupts to prevent further operation
 * 2. Safely shutting down all heater outputs
 * 3. Sending error message via UART
 * 4. Entering infinite loop with visual error code display:
 *    - Initial 1-second LED ON
 *    - 1-second LED OFF
 *    - Blinks LED number of times corresponding to error code (200ms intervals)
 *    - 1-second pause before repeating
 * 
 * LED Pattern Example for errnum = 3:
 * ON(1s)->OFF(1s)->[ON(200ms)->OFF(200ms)]x3->OFF(1s)->repeat
 * 
 * @note This function never returns and requires a system reset to recover.
 */
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
