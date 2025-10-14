#include "gpio_init.h"
#include "py32f0xx_hal.h"
#include "py32f0xx_hal_gpio.h"

GPIO_PinAssignments_t Pins;

/**
 * @brief Initializes all GPIO pins for the project.
 *
 * This function enables clocks and configures all required pins for input, output, analog, and alternate functions:
 * - Configures ADC pins as analog inputs with no pull-ups
 * - Sets up UART pins for TX/RX with alternate function
 * - Configures LED outputs with pull-ups
 * - Sets up control pins for amplifier and valve with pull-downs
 * - Initializes pushbutton input with pull-up if enabled
 */
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitTypeDef AdcPinStruct;

    //Configure GPIO for ADC, UART, PWM outputs

    // Devkit (benchtop prototype) pins:
    //  PIN 19, PA0 is TX
    //  PIN 20, PA1 is RX
    //  PIN 1, PA2 is ADC in: AMP_TEMP_V
    //  PIN 2, PA3 is ADC in: VALVE_TEMP_V
    //  PIN 3, PA4 is ADC_SPARE 
    //  PIN 5, PA6 is ADC in: V_BATT_SENSE
    //  PIN 8, PA12 is the pushbutton input
    //  PIN 12, PB5 is the LED
    //  PIN 5, PB6 is PWM output1
    //  PIN 6, PB7 is PWM output2
    //  PIN 15, PF4 is BOOT0 jumper
    //  PIN 16, PF0 is OSCIN
    //  PIN 17, PF1 is OSCOUT
    
    
    // MK5 and MK6 pins:
    //  PIN 19, PA0 is TX
    //  PIN 20, PA1 is RX
    //  PIN 1, PA2 is ADC in: VALVE_TEMP_V
    //  PIN 2, PA3 is ADC in: AMP_TEMP_V
    //  PIN 8, PA4 is ADC in: USB_CC2
    //  PIN 4, PA5 is LED1
    //  PIN 12, PB5 is LED2
    //  PIN 5, PA6 is ADC in: V_BATT_SENSE
    //  PIN 6, PA7 is ADC in: USB_CC1
    //  PIN 13, PB6 is PWM output1: AMP_CTRL1
    //  PIN 14, PB7 is PWM output2: AMP_CTRL2
    //  PIN 16, PF0 is PWM output3: VALVE_CTRL1
    //  PIN 17, PF1 is PWM output4: VALVE_CTRL2
    //  PIN 18, PF2 is NRESET pushbutton (not used, pulled up)
    //  PIN 15, PF4 is BOOT0 (not used, pulled down)
    
    
    // Enable peripheral clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    
    // Initialize UART pins
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //Pin settings for ADC inputs
    __HAL_RCC_ADC_CLK_ENABLE();
    
    // Device pin assignments:
#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F)
    Pins.GPIOx_AMP_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_AMP_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_VALVE_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_VALVE_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_VALVE_TEMP_V = ADC_CHANNEL_3;    
    Pins.GPIOx_AMP_V_BATT_SENSE = GPIOA;
    Pins.GPIO_Pin_V_BATT_SENSE = GPIO_PIN_6;
    Pins.ADC_CHANNEL_V_BATT_SENSE = ADC_CHANNEL_6;    
    Pins.GPIOx_USB_CC1 = GPIOA;
    Pins.GPIO_Pin_USB_CC1 = GPIO_PIN_7;
    Pins.ADC_CHANNEL_USB_CC1 = ADC_CHANNEL_7;    
    Pins.GPIOx_USB_CC2 = GPIOA;
    Pins.GPIO_Pin_USB_CC2 = GPIO_PIN_4;
    Pins.ADC_CHANNEL_USB_CC2 = ADC_CHANNEL_4;    
    Pins.GPIOx_LED1 = GPIOA;
    Pins.GPIO_Pin_LED1 = GPIO_PIN_5;
    Pins.GPIOx_LED2 = GPIOB;
    Pins.GPIO_Pin_LED2 = GPIO_PIN_5;
    Pins.GPIOx_AMP_CTRL1 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL1 = GPIO_PIN_6;
    Pins.GPIOx_AMP_CTRL2 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL2 = GPIO_PIN_7;
    Pins.GPIOx_VALVE_CTRL1 = GPIOF;
    Pins.GPIO_Pin_VALVE_CTRL1 = GPIO_PIN_0;
    Pins.GPIOx_VALVE_CTRL2 = GPIOF;
    Pins.GPIO_Pin_VALVE_CTRL2 = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOF;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_2;
#elif defined(BOARDCONFIG_MK8)
    Pins.GPIOx_AMP_VALVE_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_VALVE_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_VALVE_TEMP_V = ADC_CHANNEL_2; 
    
    Pins.GPIOx_AMP_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_AMP_TEMP_V = ADC_CHANNEL_3;    
    
    Pins.GPIOx_AMP_V_BATT_SENSE = GPIOA;
    Pins.GPIO_Pin_V_BATT_SENSE = GPIO_PIN_6;
    Pins.ADC_CHANNEL_V_BATT_SENSE = ADC_CHANNEL_6;    
    
    Pins.GPIOx_USB_CC1 = GPIOA;
    Pins.GPIO_Pin_USB_CC1 = GPIO_PIN_7;
    Pins.ADC_CHANNEL_USB_CC1 = ADC_CHANNEL_7;    
    
    Pins.GPIOx_USB_CC2 = GPIOA;
    Pins.GPIO_Pin_USB_CC2 = GPIO_PIN_4;
    Pins.ADC_CHANNEL_USB_CC2 = ADC_CHANNEL_4;
    
    Pins.GPIOx_LED1 = GPIOA;
    Pins.GPIO_Pin_LED1 = GPIO_PIN_5;
    Pins.GPIOx_LED2 = GPIOB;
    Pins.GPIO_Pin_LED2 = GPIO_PIN_5;
    Pins.GPIOx_AMP_CTRL1 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL1 = GPIO_PIN_6;
    Pins.GPIOx_AMP_CTRL2 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL2 = GPIO_PIN_7;
    Pins.GPIOx_VALVE_CTRL1 = GPIOF;
    Pins.GPIO_Pin_VALVE_CTRL1 = GPIO_PIN_0;
    Pins.GPIOx_VALVE_CTRL2 = GPIOF;
    Pins.GPIO_Pin_VALVE_CTRL2 = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOF;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_2;
#else    
    Pins.GPIOx_AMP_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_TEMP_V = GPIO_PIN_2;
    Pins.ADC_CHANNEL_AMP_TEMP_V = ADC_CHANNEL_2;    
    Pins.GPIOx_AMP_VALVE_TEMP_V = GPIOA;
    Pins.GPIO_Pin_AMP_VALVE_TEMP_V = GPIO_PIN_3;
    Pins.ADC_CHANNEL_VALVE_TEMP_V = ADC_CHANNEL_3;    
    Pins.GPIOx_AMP_V_BATT_SENSE = GPIOA;
    Pins.GPIO_Pin_V_BATT_SENSE = GPIO_PIN_6;
    Pins.ADC_CHANNEL_V_BATT_SENSE = ADC_CHANNEL_6;    
    //Pins.GPIOx_USB_CC1 = GPIOA;
    //Pins.GPIO_Pin_USB_CC1 = GPIO_PIN_7;
    //Pins.ADC_CHANNEL_USB_CC1 = ADC_CHANNEL_7;    
    //Pins.GPIOx_USB_CC2 = GPIOA;
    //Pins.GPIO_Pin_USB_CC2 = GPIO_PIN_12;
    //Pins.ADC_CHANNEL_USB_CC2 = ADC_CHANNEL_12;    
    Pins.GPIOx_LED1 = GPIOB;
    Pins.GPIO_Pin_LED1 = GPIO_PIN_5;
    //Pins.GPIOx_LED2 = GPIOB;
    //Pins.GPIO_Pin_LED2 = GPIO_PIN_5;
    Pins.GPIOx_AMP_CTRL1 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL1 = GPIO_PIN_6;
    Pins.GPIOx_AMP_CTRL2 = GPIOB;
    Pins.GPIO_Pin_AMP_CTRL2 = GPIO_PIN_7;
    //Pins.GPIOx_VALVE_CTRL1 = GPIOF;
    //Pins.GPIO_Pin_VALVE_CTRL1 = GPIO_PIN_0;
    //Pins.GPIOx_VALVE_CTRL2 = GPIOF;
    //Pins.GPIO_Pin_VALVE_CTRL2 = GPIO_PIN_1;
    Pins.GPIOx_PUSHBUTTON = GPIOA;
    Pins.GPIO_Pin_PUSHBUTTON = GPIO_PIN_12;
#endif
    
    //AdcPinStruct.Pin = GPIO_PIN_1;              // PA1 / UART_RX alternative function
    //AdcPinStruct.Mode = GPIO_MODE_ANALOG;
    //AdcPinStruct.Pull = GPIO_NOPULL;
    //HAL_GPIO_Init(GPIOA, &AdcPinStruct);    

    AdcPinStruct.Pin = Pins.GPIO_Pin_AMP_TEMP_V;            // AMP_TEMP_V
    AdcPinStruct.Mode = GPIO_MODE_ANALOG;
    AdcPinStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Pins.GPIOx_AMP_TEMP_V, &AdcPinStruct);    

    AdcPinStruct.Pin = Pins.GPIO_Pin_AMP_VALVE_TEMP_V;      // VALVE_TEMP_V
    AdcPinStruct.Mode = GPIO_MODE_ANALOG;
    AdcPinStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Pins.GPIOx_AMP_VALVE_TEMP_V, &AdcPinStruct);    
    
    AdcPinStruct.Pin = Pins.GPIO_Pin_V_BATT_SENSE;          // V_BATT_SENSE
    AdcPinStruct.Mode = GPIO_MODE_ANALOG;
    AdcPinStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Pins.GPIOx_AMP_V_BATT_SENSE, &AdcPinStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_LED1;               // LED1_N
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_LED1, &GPIO_InitStruct);    

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F) || defined(BOARDCONFIG_MK8)
    AdcPinStruct.Pin = Pins.GPIO_Pin_USB_CC1;               // USB_CC1
    AdcPinStruct.Mode = GPIO_MODE_ANALOG;
    AdcPinStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Pins.GPIOx_USB_CC1, &AdcPinStruct);    
    
    AdcPinStruct.Pin = Pins.GPIO_Pin_USB_CC2;               // USB_CC2
    AdcPinStruct.Mode = GPIO_MODE_ANALOG;
    AdcPinStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Pins.GPIOx_USB_CC2, &AdcPinStruct);    
    
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_LED2;               // LED2_N
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Pins.GPIOx_LED2, &GPIO_InitStruct);    

#endif

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_AMP_CTRL1;          // AMP_CTRL1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_AMP_CTRL1, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_AMP_CTRL2;          // AMP_CTRL2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_AMP_CTRL2, &GPIO_InitStruct);    

#if defined(BOARDCONFIG_MK5C) || defined(BOARDCONFIG_MK6C) || defined(BOARDCONFIG_MK5AA) || defined(BOARDCONFIG_MK6AA) || defined(BOARDCONFIG_MK6F) || defined(BOARDCONFIG_MK8)
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_VALVE_CTRL1;        // VALVE_CTRL1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_VALVE_CTRL1, &GPIO_InitStruct);    

    GPIO_InitStruct.Pin = Pins.GPIO_Pin_VALVE_CTRL2;        // VALVE_CTRL2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_VALVE_CTRL2, &GPIO_InitStruct);    
#endif

#ifdef PUSHBUTTON_UI_ENABLED
    GPIO_InitStruct.Pin = Pins.GPIO_Pin_PUSHBUTTON;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Pins.GPIOx_PUSHBUTTON, &GPIO_InitStruct);    
#endif
       
}

