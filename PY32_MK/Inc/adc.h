/*
*   File: adc.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "py32f0xx_hal.h"

#define V_BATT_SENSE_MULTIPLIER   11        // V_BATT_SENSE comes out of a 100k / 10k resistive divider.

/* USB-C CC Pins:
NAATOS (UFP) has 5.1k pull-down resistors on both CC pins.
USB DFP has pull-up resistors on both CC pins
    The USB cable will only wire one of these from end to end.
    Default USB power (500mA)       56k ohm pull-down +/- 20%       V = 0.41v nominal
    1.5A                            22k ohm pull-down +/- 20%       V = 0.94v nominal
    3.0A                            10k ohm pull-down +/- 20%       V = 1.69v nominal

*/

#define USB_CC_THRESHOLD_500MA      0.25
#define USB_CC_THRESHOLD_1500MA     0.75
#define USB_CC_THRESHOLD_3000MA     1.25

void ADC_Read(void);
float PY32_ADC_Temp_to_degC(uint32_t adc_temp);
float TMP235_V_to_degC(float vin);
uint32_t ADC_to_thermistor_resistance(uint32_t raw_adc);
float thermistor_r_to_temperature(uint32_t resistance);

#ifdef __cplusplus
}
#endif

#endif