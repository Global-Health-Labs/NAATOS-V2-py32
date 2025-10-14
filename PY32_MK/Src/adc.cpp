/*
*   File: adc.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include <math.h>
#include "adc.h"
#include "app_data.h"
#include "alarm.h"
#include "io/adc_init.h"
#include "io/gpio_init.h"
#include "py32f0xx_hal.h"

extern "C" {

#define VREFINT_V 1.20



float ADC_Vrefint_to_Vcc(uint32_t adc_val)
{
    float f;    
    f = (float) 4096.0 / (float) adc_val;
    return VREFINT_V * f;    
}

float ADC_to_Volts(uint32_t adc_val, float Vcc)
{
    float f;    
    f = (float) adc_val / (float) 4096.0;
    return Vcc * f;        
}

void ADC_Read(void)
{
/**
 * @brief Reads all configured ADC channels and updates application data.
 *
 * This function performs sequential ADC conversions for each relevant channel,
 * including sample temperature, valve temperature, battery sense, USB CC1/CC2 (if enabled),
 * MCU internal temperature sensor, and reference voltage. The results are stored in the
 * global data structure, and derived voltages and temperatures are calculated.
 * It also updates maximum temperature records and checks for overtemperature conditions,
 * triggering error handling if necessary.
 *
 * The function should be called periodically to keep sensor and system data up to date.
 * 
 * ADC values must be read in logical pin order (PA0, then PA1, etc)
 */
    float mcu_vcc;
            
    //Sample with ADC in polling mode
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[0] = HAL_ADC_GetValue(&AdcHandle); // Sample temperature
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[1] = HAL_ADC_GetValue(&AdcHandle); // Valve temperature

    if (data.usb_cc_adc_read_enabled) {
        HAL_ADC_Start(&AdcHandle);
        HAL_ADC_PollForConversion(&AdcHandle, 1000);
        data.adcReading[4] = HAL_ADC_GetValue(&AdcHandle); // USB_CC2               
    } 
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[2] = HAL_ADC_GetValue(&AdcHandle); // V_BATT_SENSE (system_input_voltage)
    
    if (data.usb_cc_adc_read_enabled) {
        HAL_ADC_Start(&AdcHandle);
        HAL_ADC_PollForConversion(&AdcHandle, 1000);
        data.adcReading[3] = HAL_ADC_GetValue(&AdcHandle); // USB_CC1        
    } 
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[5] = HAL_ADC_GetValue(&AdcHandle); // MCU Temp sensor
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[6] = HAL_ADC_GetValue(&AdcHandle); // MCU VrefInt

    mcu_vcc = ADC_Vrefint_to_Vcc(data.adcReading[6]);
    data.vcc_mcu_voltage = mcu_vcc;
    data.py32_temperature_c = PY32_ADC_Temp_to_degC(data.adcReading[5]);
    data.adcVoltage[0] = ADC_to_Volts (data.adcReading[0], mcu_vcc);
    data.adcVoltage[1] = ADC_to_Volts (data.adcReading[1], mcu_vcc);
    data.adcVoltage[2] = ADC_to_Volts (data.adcReading[2], mcu_vcc);

    if (data.usb_cc_adc_read_enabled) {
        data.adcVoltage[3] = ADC_to_Volts (data.adcReading[3], mcu_vcc);
        data.adcVoltage[4] = ADC_to_Volts (data.adcReading[4], mcu_vcc);
        data.usb_cc1_voltage = data.adcVoltage[3];
        data.usb_cc2_voltage = data.adcVoltage[4];
    } else {
        data.usb_cc1_voltage = -1.0;
        data.usb_cc2_voltage = -1.0;
    }
    
    data.sample_thermistor_v = data.adcVoltage[0];
    //data.sample_temperature_c = TMP235_V_to_degC(data.adcVoltage[0]);
    data.sample_thermistor_r = ADC_to_thermistor_resistance(data.adcReading[0]);
    data.sample_temperature_c = thermistor_r_to_temperature(data.sample_thermistor_r);
    
    data.valve_thermistor_v = data.adcVoltage[1];
    //data.valve_temperature_c = TMP235_V_to_degC(data.adcVoltage[1]);    
    data.valve_thermistor_r = ADC_to_thermistor_resistance(data.adcReading[1]);
    data.valve_temperature_c = thermistor_r_to_temperature(data.valve_thermistor_r);
    
    data.system_input_voltage = data.adcVoltage[2] * V_BATT_SENSE_MULTIPLIER;    
    
    if (data.sample_temperature_c > data.sample_max_temperature_c) {
        data.sample_max_temperature_c = data.sample_temperature_c;
    }
    
    if (data.valve_temperature_c > data.valve_max_temperature_c) {
        data.valve_max_temperature_c = data.valve_temperature_c;
    }
    
    if (data.sample_temperature_c >= OVERTEMP_ERR_C) {
		APP_ErrorHandler(ERR_OVERTEMP_SHUTDOWN);
    } else if (data.valve_temperature_c >= OVERTEMP_ERR_C) {
		APP_ErrorHandler(ERR_OVERTEMP_SHUTDOWN);
    }
		
    data.sh_pwm_during_adc_meas = data.sample_heater_pwm_value;    
    data.vh_pwm_during_adc_meas = data.valve_heater_pwm_value;    
    
}


// PY32 internal temperature conversion formula:
// temp_c = ((85c - 30c) / (HAL_ADC_TSCAL2 - HAL_ADC_TSCAL1)) * (ADC_TEMP - HAL_ADC_TSCAL1) - 30
//      The first part of this equation is pre-calculated and stored in: temperature_cal
float PY32_ADC_Temp_to_degC(uint32_t adc_temp)
{
    float temp_c;    
    temp_c = temperature_cal * ((float) adc_temp - (float) HAL_ADC_TSCAL1) + 30.0f;
    return temp_c;
}

// TMP235: temperature is 10 mv per deg_c with a 500 mv offset
//  (deg_c) = (v_in - 0.5) / .01
float TMP235_V_to_degC(float vin)
{
    float temp_c;
    temp_c = (vin - 0.5) / 0.01;
    return temp_c;
}

#define THERMISTOR_NOMINAL	((float) 100000)	/* 100k */
#define TEMPERATURE_NOMINAL	((float) 25)		/* 25 degrees C */
#define B_CONSTANT			((float) 4250)	    /* B-constant (K) 25/50 degrees C */

#define BRIDGE_RESISTOR		((float) 100000)	/* 100k */
#define ADC_RESOLUTION		4095	            /* 12bit */


uint32_t ADC_to_thermistor_resistance(uint32_t raw_adc)
{
	float r;					
	r = ((ADC_RESOLUTION - raw_adc) * BRIDGE_RESISTOR) / (raw_adc);
	return (uint32_t) r;				
}    
float thermistor_r_to_temperature(uint32_t resistance)
{
	float temp;

	temp = (float) resistance / (float) THERMISTOR_NOMINAL;	// (R/Ro)
	temp = logf(temp);			// ln(R/Ro)
	temp /= B_CONSTANT;			// 1/B * ln(R/Ro)
	temp += 1.0 / (TEMPERATURE_NOMINAL + 273.15);	// + (1/To)
	temp = 1.0 / temp;			// Invert
	temp -= 273.15;				// convert to C
    
    // round the temperature to 1 decimal place:
    temp = roundf(temp * 10.0f) / 10.0f;

	return temp;
}

} // extern "C"