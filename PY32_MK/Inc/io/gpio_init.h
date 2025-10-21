#ifndef GPIO_INIT_H
#define GPIO_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

//#define PY32F003x8
#include "main.h"
#include "py32f0xx_hal.h"
#include "py32f0xx_hal_gpio.h"
#include "py32f0xx_hal_def.h"
#include "py32f0xx.h"

/**
 * @brief Initializes all GPIO pins for the project.
 *
 * This function configures:
 * - ADC input pins for temperature and voltage sensing
 * - UART pins for communication
 * - Control pins for amplifier and valve drivers
 * - LED pins for status indication
 * - Pushbutton input for user interface
 */
void GPIO_Init(void);

/**
 * @brief Structure to store all GPIO pin assignments and configurations
 */
typedef struct GPIO_PinAssignments {
    GPIO_TypeDef    *GPIOx_AMP_TEMP_V;
    uint16_t        GPIO_Pin_AMP_TEMP_V;
    uint32_t        ADC_CHANNEL_AMP_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_AMP_VALVE_TEMP_V;
    uint16_t        GPIO_Pin_AMP_VALVE_TEMP_V;
    uint32_t        ADC_CHANNEL_VALVE_TEMP_V;
    
    GPIO_TypeDef    *GPIOx_AMP_V_BATT_SENSE;
    uint16_t        GPIO_Pin_V_BATT_SENSE;
    uint32_t        ADC_CHANNEL_V_BATT_SENSE;

    GPIO_TypeDef    *GPIOx_USB_CC1;
    uint16_t        GPIO_Pin_USB_CC1;
    uint32_t        ADC_CHANNEL_USB_CC1;
    
    GPIO_TypeDef    *GPIOx_USB_CC2;
    uint16_t        GPIO_Pin_USB_CC2;
    uint32_t        ADC_CHANNEL_USB_CC2;
    
    GPIO_TypeDef    *GPIOx_LED1;
    uint16_t        GPIO_Pin_LED1;
    GPIO_TypeDef    *GPIOx_LED2;
    uint16_t        GPIO_Pin_LED2;
    GPIO_TypeDef    *GPIOx_ADC_SPARE;
    uint16_t        GPIO_Pin_ADC_SPARE;
    GPIO_TypeDef    *GPIOx_AMP_CTRL1;
    uint16_t        GPIO_Pin_AMP_CTRL1;
    GPIO_TypeDef    *GPIOx_AMP_CTRL2;
    uint16_t        GPIO_Pin_AMP_CTRL2;
    GPIO_TypeDef    *GPIOx_VALVE_CTRL1;
    uint16_t        GPIO_Pin_VALVE_CTRL1;
    GPIO_TypeDef    *GPIOx_VALVE_CTRL2;
    uint16_t        GPIO_Pin_VALVE_CTRL2;
    GPIO_TypeDef    *GPIOx_PUSHBUTTON;
    uint16_t        GPIO_Pin_PUSHBUTTON;
} GPIO_PinAssignments_t;
extern GPIO_PinAssignments_t Pins;

#ifdef __cplusplus
}
#endif

#endif // GPIO_INIT_H
